#include "../include/smap_core/object_pose_estimator.hpp"

using namespace std::chrono_literals;

namespace smap
{

void object_pose_estimator::object_filtering_thread(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud, const smap_interfaces::msg::SmapObject::SharedPtr obj){
  std::chrono::_V2::system_clock::time_point start, stop;
  size_t outliers = point_cloud->size();
  start = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(this->get_logger(),"Object: %i",obj->label);
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud_vox(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> segment_cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> segment_cloud_pcl_neg(new pcl::PointCloud<pcl::PointXYZRGB>);

  std::shared_ptr<sensor_msgs::msg::PointCloud2> segment_cloud_ros(new sensor_msgs::msg::PointCloud2);


  this->letterbox_filter(point_cloud,segment_cloud_pcl,segment_cloud_pcl_neg,obj); // < 20ms

  this->pcl_voxelization(segment_cloud_pcl,segment_cloud_pcl); // < 20ms

  this->statistical_outlier_filter(segment_cloud_pcl,segment_cloud_pcl_neg); // < 800ms

  stop = std::chrono::high_resolution_clock::now();
  RCLCPP_WARN(this->get_logger(),"pcl_process time %ims | %i points removed (%5.2f%%)",
    (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count(),
    outliers-segment_cloud_pcl->size(),
    ((outliers-segment_cloud_pcl->size())*100.0/outliers)
  );


  pcl::toROSMsg(*segment_cloud_pcl_neg,*segment_cloud_ros);
  this->test_pcl_pub_unfilt->publish(*segment_cloud_ros);
  pcl::toROSMsg(*segment_cloud_pcl,*segment_cloud_ros);
  this->test_pcl_pub_filt->publish(*segment_cloud_ros);


}



void object_pose_estimator::detections_callback(const smap_interfaces::msg::SmapDetections::SharedPtr input_msg) {
  RCLCPP_INFO(this->get_logger(),"detections_callback");


  // Convert sensor_msgs::pointloud2 to pcl::PointCloud
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcl_point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(input_msg->pointcloud,*pcl_point_cloud);


  // Thread launch

  smap_interfaces::msg::SmapObject obj;
  sensor_msgs::msg::PointCloud2 segment_cloud;

  static int z = 0;
  BOOST_FOREACH(obj, input_msg->objects){
    if(obj.label != 62) continue;

    // Block until thread pool is available
    while(!this->thread_ctl->available()){
      std::this_thread::sleep_for(10ms);
    }
    // while(this->thread_ctl->size() == this->max_threads){
    //   // Check thread status
    //   for(it = this->thread_ctl->begin(); it != this->thread_ctl->end(); ++it){
    //     status = (std::get<0>(*it))->wait_for(0ms);
    //     if(status == std::future_status::ready || status == std::future_status::timeout){
    //       this->thread_ctl->erase(it);
    //     }
    //   }
    //   std::this_thread::sleep_for(50ms);
    // }

    // this->smap::object_pose_estimator::object_filtering_thread_with_timeout(
    //   std::make_shared<sensor_msgs::msg::PointCloud2>(input_msg->pointcloud),
    //   std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    // );

    // this->object_filtering_thread(
    //   std::make_shared<sensor_msgs::msg::PointCloud2>(input_msg->pointcloud),
    //   std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    // );



    this->object_filtering_thread(
      pcl_point_cloud,
      std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    );

    // this->thread_ctl->push_back(
    //   {
    //     std::make_shared<std::future<void>>(
    //       std::async(
    //         std::launch::async,
    //         &smap::object_pose_estimator::object_filtering_thread,this,
    //         pcl_point_cloud,
    //         std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    //       )
    //     ),
    //     this->__compute_timeout(this->thread_ctl->size())
    //   }
    // );

    // this->thread_ctl->push_back(
    //   {
    //     std::make_shared<std::future<void>>(
    //       std::async(
    //         std::launch::async,
    //         &smap::object_pose_estimator::object_filtering_thread,this,
    //         pcl_point_cloud,
    //         std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    //       )
    //     ),
    //     this->__compute_timeout(this->thread_ctl->size())*1000
    //   }
    // );

    RCLCPP_INFO(this->get_logger(),"Item launched: %i| label: %i | Total: %i",this->thread_ctl->size(),obj.label,++z);

    // std::async(
    //   std::launch::async,
    //   &smap::object_pose_estimator::object_thread,this,
    //   std::make_shared<sensor_msgs::msg::PointCloud2>(input_msg->pointcloud),
    //   std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    // )

    // this->thread_ctl->push_back();

    // create new thread
    // is_free[i] = false;
    // futures[i] = std::async(
    //   std::launch::async,
    //   &smap::object_pose_estimator::object_thread,this,
    //   std::make_shared<sensor_msgs::msg::PointCloud2>(input_msg->pointcloud),
    //   std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    // );
    // std::set<0>
    // std::get

  }
  RCLCPP_INFO(this->get_logger(),"---Callback complete---");
}


void object_pose_estimator::on_process(void) // Pooling
{
  // RCLCPP_DEBUG(this->get_logger(),"Process smap::smap_node");
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<smap::object_pose_estimator> node = std::make_shared<smap::object_pose_estimator>();


  try{
    rclcpp::spin(node);
  }catch (std::exception& e){
    std::cout << "SMAP Exception!" << std::endl;
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}

