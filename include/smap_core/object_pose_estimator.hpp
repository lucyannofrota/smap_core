#ifndef SMAP_CORE__OBJECT_POSE_ESTIMATOR_HPP_
#define SMAP_CORE__OBJECT_POSE_ESTIMATOR_HPP_

#include "visibility_control.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "../include/smap_core/macros.hpp"
#include "visualization_msgs/msg/marker.hpp"


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
#include <pcl/features/gasd.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/moment_of_inertia_estimation.h>
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

    RCLCPP_WARN(logger,"%s %ims",
      str,
      (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count()
    );
  }

};

namespace smap
{

// struct object_parameters{
//   pcl::PointXYZ centroid = {0,0,0};
//   std::pair<pcl::PointXYZ,pcl::PointXYZ> boundaries = {{0,0,0},{0,0,0}}; // first -> min, second -> max
// };

  using cloud_point_t = pcl::PointXYZRGB;
  using cloud_t = pcl::PointCloud<cloud_point_t>;

class object_pose_estimator : public rclcpp::Node
{
private:
  const size_t max_threads = 8; // Should be grathen than 1 because of the function "__compute_timeout"
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
    "pcl", 10);

  rclcpp::Subscription<smap_interfaces::msg::SmapDetections>::SharedPtr smap_detections_sub = this->create_subscription<smap_interfaces::msg::SmapDetections>(
    "/smap_core/perception/modules/predictions",10,std::bind(
      &smap::object_pose_estimator::detections_callback, this, std::placeholders::_1)
  );

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/smap_core/perception/cpp",10);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pcl_pub_unfilt = this->create_publisher<sensor_msgs::msg::PointCloud2>("pci_unfilt",10);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pcl_pub_filt = this->create_publisher<sensor_msgs::msg::PointCloud2>("pci_filt",10);

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/smap_core/pose_estimation/marker",10);

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

  plot_vec total_thread_time;
  plot_vec box_filter_plot, roi_filter_plot, voxelization_plot, sof_filter_plot, euclidean_clustering_plot, total_filter_plot;
  plot_vec centroid_plot, boundaries_plot, total_estimation_plot;

  // pcl::visualization::PCLVisualizer visu;
  // std::list<int> box_filter_times;
  // Constructor/Destructor
  inline object_pose_estimator()
  : Node("smap_object_pose_estimator")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing smap_object_pose_estimator");
    // this->viewer = std::make_shared<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  }
  inline ~object_pose_estimator()
  {
  }

  inline void box_filter(
    const pcl::shared_ptr<cloud_t>& input_cloud,
    const pcl::shared_ptr<cloud_t>& cloud_segment,
    const smap_interfaces::msg::SmapObject::SharedPtr& obj)
  {
    count_time timer;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    for(size_t h = obj->bounding_box_2d.keypoint_1[1]; h <= obj->bounding_box_2d.keypoint_2[1]; h++){
      for(size_t w = obj->bounding_box_2d.keypoint_1[0]; w <= obj->bounding_box_2d.keypoint_2[0]; w++){
        inliers->indices.push_back(
          (w)+input_cloud->width*(h)-1
        );
      }
    }

    pcl::ExtractIndices<cloud_point_t> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);

    extract.filter(*cloud_segment);


    const char str[] = "box_filter";
    timer.get_time(this->get_logger(), str, this->box_filter_plot);
  }

  inline void roi_filter(
    const pcl::shared_ptr<cloud_t>& point_cloud)
  {
    count_time timer;


    point_cloud->erase(
      std::remove_if(point_cloud->begin(), point_cloud->end(), 
        [this](const cloud_point_t& point) {
          return (
            ((point.getVector3fMap()).norm() < this->pcl_lims->first) ||
            ((point.getVector3fMap()).norm() > this->pcl_lims->second)
          );
        }
      ),
      point_cloud->end()
    );

    const char str[] = "roi_filter";
    timer.get_time(this->get_logger(), str, this->roi_filter_plot);
  }

  inline void pcl_voxelization(const pcl::shared_ptr<cloud_t>& point_cloud)
  {
    count_time timer;
    pcl::VoxelGrid<cloud_point_t> vox_grid;

    vox_grid.setInputCloud(point_cloud);
    // vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
    vox_grid.setLeafSize (this->leaf_size, this->leaf_size, this->leaf_size);
    vox_grid.setDownsampleAllData(true);
    vox_grid.filter(*point_cloud);

    const char str[] = "pcl_vox";
    timer.get_time(this->get_logger(), str, this->voxelization_plot);
  }

  inline void statistical_outlier_filter(const pcl::shared_ptr<cloud_t>& cloud_segment)
  {
    count_time timer;
    pcl::StatisticalOutlierRemoval<cloud_point_t> sor;
    sor.setInputCloud(cloud_segment);
    sor.setMeanK(this->mean_k); // Greather values take more time to compute
    sor.setStddevMulThresh(this->mu);

    sor.filter(*cloud_segment);


    const char str[] = "statistical_outlier_filter";
    timer.get_time(this->get_logger(), str, this->sof_filter_plot);
  }

  inline void euclidean_clustering(const pcl::shared_ptr<cloud_t>& cloud_segment, const pcl::shared_ptr<cloud_t>& object_cloud)
  {
    count_time timer;

    pcl::search::KdTree<cloud_point_t>::Ptr tree (new pcl::search::KdTree<cloud_point_t>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_segment,*cloud_segment,indices);
    tree->setInputCloud (cloud_segment);

    std::vector<pcl::PointIndices> cluster_idx;
    pcl::EuclideanClusterExtraction<cloud_point_t> ece;
    ece.setClusterTolerance(this->ClusterTolerance); // cm
    ece.setMinClusterSize(25);
    // ece.setMaxClusterSize(this->max_threads);
    ece.setSearchMethod(tree);
    ece.setInputCloud(cloud_segment);
    ece.extract(cluster_idx);

    // Idxs of the biggest cluster
    if(cluster_idx.size() >= 1){
      for (const auto& idx : cluster_idx[0].indices) object_cloud->push_back((*cloud_segment)[idx]);
    }


    const char str[] = "euclidean_clustering";
    timer.get_time(this->get_logger(), str, this->euclidean_clustering_plot);
  }

  inline void estimate_object_3D_AABB(const pcl::shared_ptr<cloud_t>& object_cloud, const smap_interfaces::msg::SmapObject::SharedPtr& obj){
    count_time timer;

    obj->aabb.min.x = object_cloud->points[0].x;
    obj->aabb.min.y = object_cloud->points[0].y;
    obj->aabb.min.z = object_cloud->points[0].z;

    obj->aabb.max.x = object_cloud->points[0].x;
    obj->aabb.max.y = object_cloud->points[0].y;
    obj->aabb.max.z = object_cloud->points[0].z;

    for(cloud_point_t& point : *object_cloud){
      if(point.x < obj->aabb.min.x) obj->aabb.min.x = point.x;
      if(point.y < obj->aabb.min.y) obj->aabb.min.y = point.y;
      if(point.z < obj->aabb.min.y) obj->aabb.min.z = point.z;

      if(point.x > obj->aabb.max.x) obj->aabb.max.x = point.x;
      if(point.y > obj->aabb.max.y) obj->aabb.max.y = point.y;
      if(point.z > obj->aabb.max.z) obj->aabb.max.z = point.z;
    }


    obj->obj_pose.position.x = (obj->aabb.min.x + obj->aabb.max.x)/2;
    obj->obj_pose.position.y = (obj->aabb.min.y + obj->aabb.max.y)/2;
    obj->obj_pose.position.z = (obj->aabb.min.z + obj->aabb.max.z)/2;

    this->puiblish_bb(0,obj);

    const char str[] = "3D_AABB";
    timer.get_time(this->get_logger(), str, this->centroid_plot);
  }

  inline void puiblish_bb(int32_t id, const smap_interfaces::msg::SmapObject::SharedPtr& obj){
    visualization_msgs::msg::Marker bbx_marker;
    bbx_marker.id = id;
    bbx_marker.header.frame_id = "map";
    bbx_marker.header.stamp = this->get_clock()->now();
    bbx_marker.type = visualization_msgs::msg::Marker::CUBE;
    bbx_marker.action = visualization_msgs::msg::Marker::ADD;
    bbx_marker.pose.position = obj->obj_pose.position;
    bbx_marker.pose.orientation.x = 0;
    bbx_marker.pose.orientation.y = 0;
    bbx_marker.pose.orientation.z = 0;
    bbx_marker.pose.orientation.w = 1;
    bbx_marker.scale.x = (obj->aabb.max.x - obj->aabb.min.x);
    bbx_marker.scale.y = (obj->aabb.max.y - obj->aabb.min.y);
    bbx_marker.scale.z = (obj->aabb.max.z - obj->aabb.min.z);
    bbx_marker.color.b = 0;
    bbx_marker.color.g = 0;
    bbx_marker.color.r = 255;
    bbx_marker.color.a = 0.5;
    this->marker_pub->publish(bbx_marker);
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
    const smap_interfaces::msg::SmapObject::SharedPtr& obj
  );
 
  void detections_callback(const smap_interfaces::msg::SmapDetections::SharedPtr input_msg);

  inline void on_process(void){// Pooling

  } 

private:

};

}  // namespace smap

#endif  // SMAP_CORE__OBJECT_POSE_ESTIMATOR_HPP_
