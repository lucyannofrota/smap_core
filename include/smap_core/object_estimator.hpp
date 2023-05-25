#ifndef SMAP_CORE__OBJECT_ESTIMATOR_HPP_
#define SMAP_CORE__OBJECT_ESTIMATOR_HPP_

#include "visibility_control.h"
#include "../../include/smap_core/macros.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp_components/register_node_macro.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>

#include <pcl/segmentation/extract_clusters.h>


// SMAP
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/smap_detections.hpp"
#include "smap_interfaces/msg/bounding_box2_d.hpp"

// ImGui
#include "../../imgui/imgui.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <mutex>

// TODO: Convert this node into a ROS component

using namespace std::chrono_literals;

// TODO: Reject unregistered requests

inline bool timeout(const std::tuple<std::shared_ptr<std::future<void>>,std::chrono::_V2::system_clock::time_point,float> &element){
  if(std::get<0>(element)){
    bool ret = (std::get<0>(element)->wait_for(0ms) == std::future_status::ready) || (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now()-std::get<1>(element)
    )).count() >= std::get<2>(element);
    if(ret){
      // printf("timeout (%f)|%i\n",std::get<2>(element),(std::get<0>(element)->wait_for(0ms) == std::future_status::ready));
      std::get<0>(element).get(); // Finalize the thread
    }
    return ret;
  }
  // printf("\tf /timeout\n");
  return true;
  // return false;
}

class thread_queue : public std::vector<std::tuple<std::shared_ptr<std::future<void>>,std::chrono::_V2::system_clock::time_point,float>>
{
  using element_type = std::tuple<std::shared_ptr<std::future<void>>,std::chrono::_V2::system_clock::time_point,float>;
  private:
  const size_t _max_size;

  public:
  thread_queue(size_t max_size) : std::vector<element_type>(), _max_size(max_size){}

  inline bool available(void){ // Return true if the thread queue is not full
    std::vector<element_type>::erase(
      std::remove_if(std::vector<element_type>::begin(),std::vector<element_type>::end(),timeout),
      std::vector<element_type>::end()
    );

    return std::vector<element_type>::size() < this->_max_size;
  }

  inline float __compute_timeout(const size_t active_threads) const{
    const float tau = 2.5;
    double timeout = (double) this->_max_size-1;
    if(active_threads > 0) timeout = (double) this->_max_size*(exp(-(active_threads*1.0)/(tau)));
    return timeout*100;
  }

  inline bool push_back(const std::shared_ptr<std::future<void>> &element){

    // Add a new thread if possible
    if(thread_queue::available()){
      std::vector<element_type>::push_back(
        element_type({
          element,
          std::chrono::high_resolution_clock::now(),
          this->__compute_timeout(std::vector<element_type>::size())
        })
      );
      return true;
    }
    else{
      return false;
    } 
  }
};

class plot_vec : public std::list<int>
{
  private:
  const size_t _max_size = 128;
  float average;

  public:

  inline void push_back(const int &element){
    if(std::list<int>::size() >= _max_size) std::list<int>::pop_front();
    std::list<int>::push_back(element);
  }

  inline void get_float_arr_av(float *ptr){
    auto it = this->begin();
    this->average = 0;
    for(int i = 0; it != this->end(); ++it, i++){
      ptr[i] = *it;
      this->average+=*it;
    }
    this->average /= this->size();
  }

  inline float get_average(void){ return this->average; }
};

class count_time{

  private:
  std::chrono::_V2::system_clock::time_point start, stop;

  public:
  count_time(void){
    this->start = std::chrono::high_resolution_clock::now();  
  }

  int get_time(void){
    stop = std::chrono::high_resolution_clock::now();
    return (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count();
  }

  void get_time(const rclcpp::Logger& logger, const char* str, plot_vec& vec){
    stop = std::chrono::high_resolution_clock::now();

    vec.push_back(
      (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count()
    );

    RCLCPP_DEBUG(logger,"%s %ims",
      str,
      (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count()
    );
  }

};

plot_vec total_thread_time;
plot_vec box_filter_plot, roi_filter_plot, voxelization_plot, sof_filter_plot, euclidean_clustering_plot, total_filter_plot;
plot_vec centroid_plot, transform_plot, total_estimation_plot;

namespace smap
{

  using cloud_point_t = pcl::PointXYZRGB;
  using cloud_t = pcl::PointCloud<cloud_point_t>;

class object_estimator : public rclcpp::Node
{
private:
  const size_t max_threads = 8; // Should be grathen than 1 because of the function "__compute_timeout"
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
  //   "pcl", 10);

  rclcpp::Subscription<smap_interfaces::msg::SmapDetections>::SharedPtr smap_detections_sub = this->create_subscription<smap_interfaces::msg::SmapDetections>(
    std::string(this->get_namespace())+std::string("/perception/predictions"),10,std::bind(
      &smap::object_estimator::detections_callback, this, std::placeholders::_1)
  );


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_object_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(std::string(this->get_namespace())+std::string("/object_estimator/debug/object_pcl"),10);
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr object_bb_pub = this->create_publisher<visualization_msgs::msg::Marker>(std::string(this->get_namespace())+std::string("/object_estimator/debug/object_bb"),10);
  rclcpp::Publisher<smap_interfaces::msg::SmapObject>::SharedPtr object_pub = this->create_publisher<smap_interfaces::msg::SmapObject>(std::string(this->get_namespace())+std::string("/object_estimator/objects"),10);
  std::mutex object_bb_pub_mutex, object_pub_mutex, debug_object_pcl_pub_mutex;

  std::shared_ptr<thread_queue> thread_ctl = std::make_shared<thread_queue>(thread_queue(this->max_threads));

public:
  // TODO: move to private
  std::shared_ptr<
    std::pair<float,float>
  > pcl_lims = std::make_shared<std::pair<float,float>>(0.4,4); // TODO: Create parameter

  float leaf_size = 0.02f; // 0.00 <= leaf_size <= 0.05 | 0.03

  int mean_k = 10;
  float mu = 0.3f;

  float ClusterTolerance = 0.021f;
  int minClusterSize = 100;
  int maxClusterSize = 25000;

  bool roi_filt = true, voxelization = true, sof = true, euclidean_clust = true, pcl_lock = false;

  // pcl::visualization::PCLVisualizer visu;
  // std::list<int> box_filter_times;
  // Constructor/Destructor
  inline object_estimator()
  : Node("object_estimator")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing object_estimator");
    // this->viewer = std::make_shared<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  }

  inline object_estimator(const rclcpp::NodeOptions& options)
  : Node("object_estimator", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing object_estimator");
    // this->viewer = std::make_shared<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  }

  inline ~object_estimator()
  {
  }

  void box_filter(
    const pcl::shared_ptr<cloud_t>& input_cloud,
    const pcl::shared_ptr<cloud_t>& cloud_segment,
    const smap_interfaces::msg::SmapObject::SharedPtr& obj
  ) const;

  void roi_filter(const pcl::shared_ptr<cloud_t>& point_cloud) const;

  void pcl_voxelization(const pcl::shared_ptr<cloud_t>& point_cloud) const;

  void statistical_outlier_filter(const pcl::shared_ptr<cloud_t>& cloud_segment) const;

  void euclidean_clustering(
    const pcl::shared_ptr<cloud_t>& cloud_segment,
    const pcl::shared_ptr<cloud_t>& object_cloud
  ) const;

  void estimate_object_3D_AABB(
    const pcl::shared_ptr<cloud_t>& object_cloud,
    const smap_interfaces::msg::SmapObject::SharedPtr& obj
  ) const;

  void transform_object_param(
    const smap_interfaces::msg::SmapObject::SharedPtr& obj,
    const std::shared_ptr<geometry_msgs::msg::TransformStamped> &transform
  ) const;

  inline void puiblish_bb(int32_t id, const smap_interfaces::msg::SmapObject::SharedPtr& obj){
    visualization_msgs::msg::Marker bbx_marker;
    bbx_marker.id = id;
    bbx_marker.header.frame_id = "map";
    bbx_marker.header.stamp = this->get_clock()->now();
    bbx_marker.type = visualization_msgs::msg::Marker::CUBE;
    bbx_marker.action = visualization_msgs::msg::Marker::ADD;
    bbx_marker.pose.position = obj->obj_pose.pose.position;
    bbx_marker.pose.orientation.x = 0;
    bbx_marker.pose.orientation.y = 0;
    bbx_marker.pose.orientation.z = 0;
    bbx_marker.pose.orientation.w = 1;
    bbx_marker.scale.x = (obj->aabb.max.point.x - obj->aabb.min.point.x);
    bbx_marker.scale.y = (obj->aabb.max.point.y - obj->aabb.min.point.y);
    bbx_marker.scale.z = (obj->aabb.max.point.z - obj->aabb.min.point.z);
    bbx_marker.color.b = 0;
    bbx_marker.color.g = 0;
    bbx_marker.color.r = 255;
    bbx_marker.color.a = 0.5;
    this->object_bb_pub->publish(bbx_marker);
  }

  /*inline void estimate_object_3D_OBB(const pcl::shared_ptr<cloud_t>& object_cloud, smap_interfaces::msg::SmapObject::SharedPtr obj){
    count_time timer;
    pcl::MomentOfInertiaEstimation <cloud_point_t> feature_extractor;
    feature_extractor.setInputCloud (object_cloud);
    feature_extractor.compute ();
    cloud_point_t min_point_AABB;
    cloud_point_t max_point_AABB;
    cloud_point_t min_point_OBB;
    cloud_point_t max_point_OBB;
    cloud_point_t position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    obj->aabb.min.x = min_point_AABB.x; obj->aabb.min.y = min_point_AABB.y; obj->aabb.min.z = min_point_AABB.z;
    obj->aabb.max.x = max_point_AABB.x; obj->aabb.max.y = max_point_AABB.y; obj->aabb.max.z = max_point_AABB.z;

    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    obj->obb.keypoint_1[0] = min_point_OBB.x; obj->obb.keypoint_1[1] = min_point_OBB.y; obj->obb.keypoint_1[2] = min_point_OBB.z;
    obj->obb.keypoint_2[0] = max_point_OBB.x; obj->obb.keypoint_2[1] = max_point_OBB.y; obj->obb.keypoint_2[2] = max_point_OBB.z;


    Eigen::Quaternionf quat (rotational_matrix_OBB);
    cloud_point_t pos, pos0;

    
    pos.x = position_OBB.x;
    pos.y = position_OBB.y;
    pos.z = position_OBB.z;

    this->puiblish_bb(
      0, pos, min_point_OBB, max_point_OBB, quat
    );

    pos0.x = 0;
    pos0.y = 0;
    pos0.z = 0;

    this->puiblish_bb(
      1, pos0, min_point_OBB, max_point_OBB, quat
    );

    this->puiblish_bb(
      2, pos, min_point_OBB, max_point_OBB, Eigen::Quaternionf::Identity()
    );

    const char str[] = "object_parameters_estimation";
    timer.get_time(this->get_logger(), str, this->centroid_plot);
  }*/

  void object_estimation_thread(
    const pcl::shared_ptr<cloud_t>& point_cloud,
    const std::shared_ptr<geometry_msgs::msg::TransformStamped>& transform,
    const smap_interfaces::msg::SmapObject::SharedPtr& obj
  );
 
  void detections_callback(
    const smap_interfaces::msg::SmapDetections::SharedPtr input_msg
  );

  inline void on_process(void){// Pooling

  } 

private:

};

}  // namespace smap

// RCLCPP_COMPONENTS_REGISTER_NODE(smap::object_estimator)

#endif  // SMAP_CORE__OBJECT__ESTIMATOR_HPP_