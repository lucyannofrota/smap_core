#ifndef SMAP_CORE__OBJECT_POSE_ESTIMATOR_HPP_
#define SMAP_CORE__OBJECT_POSE_ESTIMATOR_HPP_

#include "visibility_control.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "../include/smap_core/macros.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// SMAP
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/smap_detections.hpp"
#include "smap_interfaces/msg/bounding_box2_d.hpp"


using namespace std::chrono_literals;

// TODO: Remove negatives



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
  size_t _max_size;

  public:
  thread_queue(size_t max_size) : std::vector<element_type>(){
    this->_max_size = max_size;
  }

  inline bool available(void){ // Return true if the thread queue is not full
    std::vector<element_type>::erase(
      std::remove_if(std::vector<element_type>::begin(),std::vector<element_type>::end(),timeout),
      std::vector<element_type>::end()
    );
    // printf("vec size: %i\n",(int)std::vector<element_type>::size());
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
    // printf("push\n");
    if(thread_queue::available()){
      std::vector<element_type>::push_back(
        element_type({
          element,
          std::chrono::high_resolution_clock::now(),
          this->__compute_timeout(std::vector<element_type>::size())
          // std::get<1>(element)
        })
      );
      // printf("/push T\n");
      return true;
    }
    else{
      // printf("/push F\n");
      return false;
    } 
  }
};



namespace smap
{

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

  std::shared_ptr<thread_queue> thread_ctl;

  std::shared_ptr<
    std::pair<float,float>
  > pcl_lims; // TODO: Create parameter
 
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr x_range_condition;
  // std::shared_ptr<pcl::ConditionAnd<pcl::PointXYZRGB>> x_range_condition;


public:
  // Constructor/Destructor
  inline object_pose_estimator()
  : Node("smap_object_pose_estimator")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing smap_object_pose_estimator");
    this->thread_ctl = std::make_shared<thread_queue>(thread_queue(this->max_threads));
    this->pcl_lims = std::make_shared<std::pair<float,float>>(0.4,4);

    this->x_range_condition.reset(new pcl::ConditionAnd<pcl::PointXYZRGB>());
      // ;
    x_range_condition->addComparison(
      pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB>(
        "x", pcl::ComparisonOps::GE, this->pcl_lims->first
    )));
    x_range_condition->addComparison(
      pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB>(
        "x", pcl::ComparisonOps::LE, this->pcl_lims->second
    )));
  }
  inline ~object_pose_estimator()
  {
  }

  inline void box_filter(
    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> input_cloud,
    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_segment,
    const smap_interfaces::msg::SmapObject::SharedPtr obj) const
  {
    std::chrono::_V2::system_clock::time_point start, stop;
    size_t outliers = input_cloud->size();
    start = std::chrono::high_resolution_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    inliers->header = input_cloud->header;


    for(size_t h = obj->bounding_box_2d.keypoint_1[1]; h <= obj->bounding_box_2d.keypoint_2[1]; h++){
      for(size_t w = obj->bounding_box_2d.keypoint_1[0]; w <= obj->bounding_box_2d.keypoint_2[0]; w++){
        inliers->indices.push_back(
          (w)+input_cloud->width*(h)-1
        );
      }
    }

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);


    // extract.setNegative(false);
    extract.filter(*cloud_segment);

    // // Negatives
    // extract.setNegative(true);
    // extract.filter(*cloud_segment_neg);

    stop = std::chrono::high_resolution_clock::now();
    RCLCPP_WARN(this->get_logger(),"box_filter time %ims | %i points removed (%5.2f%%)",
      (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count(),
      outliers-cloud_segment->size(),
      ((outliers-cloud_segment->size())*100.0/outliers)
    );
  }

  inline void roi_filter(
    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud) const
  {
    std::chrono::_V2::system_clock::time_point start, stop;
    size_t outliers = point_cloud->size();
    start = std::chrono::high_resolution_clock::now();


    // this->pcl_lims->first <= z <= this->pcl_lims->second
    

    pcl::ConditionalRemoval<pcl::PointXYZRGB> cond_filter;
    // cond_filter.setIndices()
    cond_filter.setCondition(this->x_range_condition);
    cond_filter.setInputCloud(point_cloud);

    cond_filter.filter(*point_cloud);


    stop = std::chrono::high_resolution_clock::now();
    RCLCPP_WARN(this->get_logger(),"roi_filter time %ims | %i points removed (%5.2f%%)",
      (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count(),
      outliers-point_cloud->size(),
      ((outliers-point_cloud->size())*100.0/outliers)
    );
  }

  inline void pcl_voxelization(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud) const
  {
    std::chrono::_V2::system_clock::time_point start, stop;
    size_t outliers = point_cloud->size();
    start = std::chrono::high_resolution_clock::now();
    pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;

    vox_grid.setInputCloud(point_cloud);
    // vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
    vox_grid.setLeafSize (0.02f, 0.02f, 0.02f);
    vox_grid.setDownsampleAllData(true);
    vox_grid.filter(*point_cloud);

    stop = std::chrono::high_resolution_clock::now();
    RCLCPP_WARN(this->get_logger(),"pcl_vox time %ims | %i points removed (%5.2f%%)",
      (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count(),
      outliers-point_cloud->size(),
      ((outliers-point_cloud->size())*100.0/outliers)
    );
  }

  inline void statistical_outlier_filter(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_segment) const
  {
    std::chrono::_V2::system_clock::time_point start, stop;
    size_t outliers = cloud_segment->size();
    start = std::chrono::high_resolution_clock::now();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_segment);
    sor.setMeanK(5); // Greather values take more time to compute
    sor.setStddevMulThresh(0.5);
    // sor.setNegative(false);
    sor.filter(*cloud_segment);

    // Negatives
    // sor.setNegative(true);
    // sor.filter(*cloud_segment_neg);

    stop = std::chrono::high_resolution_clock::now();
    RCLCPP_WARN(this->get_logger(),"statistical_outlier_filter time %ims | %i points removed (%5.2f%%)",
      (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count(),
      outliers-cloud_segment->size(),
      ((outliers-cloud_segment->size())*100.0/outliers)
    );
  }

  void object_estimation_thread(
    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud,
    const smap_interfaces::msg::SmapObject::SharedPtr obj
  );
 
  void detections_callback(const smap_interfaces::msg::SmapDetections::SharedPtr input_msg);

  void on_process(void); // Pooling

private:

};

}  // namespace smap

#endif  // SMAP_CORE__OBJECT_POSE_ESTIMATOR_HPP_
