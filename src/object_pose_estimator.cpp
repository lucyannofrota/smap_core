#include "../include/smap_core/object_pose_estimator.hpp"

using namespace std::chrono_literals;


// namespace   // anonymous namespace
// {
// inline bool timeout(const std::tuple<std::shared_ptr<std::future<void>>,std::chrono::_V2::system_clock::time_point,float> &element){
//   if(std::get<0>(element)){
//     bool ret = (std::get<0>(element)->wait_for(0ms) == std::future_status::ready) || (std::chrono::duration_cast<std::chrono::milliseconds>(
//             std::chrono::high_resolution_clock::now()-std::get<1>(element)
//     )).count() >= std::get<2>(element);
//     if(ret){
//       // printf("timeout (%f)|%i\n",std::get<2>(element),(std::get<0>(element)->wait_for(0ms) == std::future_status::ready));
//       std::get<0>(element).get(); // Finalize the thread
//     }
//     return ret;
//   }
//   // printf("\tf /timeout\n");
//   return true;
//   // return false;
// }

// class thread_pool : public std::vector<std::tuple<std::shared_ptr<std::future<void>>,std::chrono::_V2::system_clock::time_point,float>>
// {
//   using element_type = std::tuple<std::shared_ptr<std::future<void>>,std::chrono::_V2::system_clock::time_point,float>;
//   private:
//   size_t _max_size;



//   public:
//   thread_pool(size_t max_size) : std::vector<element_type>(){
//     this->_max_size = max_size;
//   }

//   inline bool available(void){ // Return true if the thread pool is not full
//     std::vector<element_type>::erase(
//       std::remove_if(std::vector<element_type>::begin(),std::vector<element_type>::end(),timeout),
//       std::vector<element_type>::end()
//     );
//     // printf("vec size: %i\n",(int)std::vector<element_type>::size());
//     return std::vector<element_type>::size() < this->_max_size;
//   }

//   inline float __compute_timeout(const size_t active_threads) const{
//     const float tau = 2.5;
//     double timeout = (double) this->_max_size-1;
//     if(active_threads > 0) timeout = (double) this->_max_size*(exp(-(active_threads*1.0)/(tau)));
//     return timeout*100;
//   }

//   inline bool push_back(const std::shared_ptr<std::future<void>> &element){

//     // Add a new thread if possible
//     // printf("push\n");
//     if(thread_pool::available()){
//       std::vector<element_type>::push_back(
//         element_type({
//           element,
//           std::chrono::high_resolution_clock::now(),
//           this->__compute_timeout(std::vector<element_type>::size())
//           // std::get<1>(element)
//         })
//       );
//       // printf("/push T\n");
//       return true;
//     }
//     else{
//       // printf("/push F\n");
//       return false;
//     } 
//   }
// };

// }




namespace smap
{



void object_pose_estimator::object_filtering_thread(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud_, const smap_interfaces::msg::SmapObject::SharedPtr obj){
  if(obj->bounding_box_2d.keypoint_1[0] < 896/2) return; // REMOVE
  std::chrono::_V2::system_clock::time_point start, stop;
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // REMOVE
  pcl::io::loadPCDFile("test_pcd.pcd",*point_cloud);
  size_t outliers = point_cloud->size();
  start = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(this->get_logger(),"Object: %i",obj->label);
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud_vox(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> segment_cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> segment_cloud_pcl_neg(new pcl::PointCloud<pcl::PointXYZRGB>);

  std::shared_ptr<sensor_msgs::msg::PointCloud2> segment_cloud_ros(new sensor_msgs::msg::PointCloud2);


  this->box_filter(point_cloud,segment_cloud_pcl,obj);

  this->roi_filter(segment_cloud_pcl);

  this->pcl_voxelization(segment_cloud_pcl);

  this->statistical_outlier_filter(segment_cloud_pcl);

  stop = std::chrono::high_resolution_clock::now();
  RCLCPP_WARN(this->get_logger(),"pcl_process time %ims | %i points removed (%5.2f%%)",
    (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count(),
    outliers-segment_cloud_pcl->size(),
    ((outliers-segment_cloud_pcl->size())*100.0/outliers)
  );


  // pcl::toROSMsg(*segment_cloud_pcl_neg,*segment_cloud_ros);
  // this->test_pcl_pub_unfilt->publish(*segment_cloud_ros);
  pcl::toROSMsg(*segment_cloud_pcl,*segment_cloud_ros);
  segment_cloud_ros->header.frame_id = "map"; // REMOVE
  this->test_pcl_pub_filt->publish(*segment_cloud_ros);

  char c = std::cin.get();
  if(c == 's'){
    pcl::io::savePCDFileBinary("test_pcd.pcd",*point_cloud);
  }


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


    this->object_filtering_thread(
      pcl_point_cloud,
      std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    );

    // this->thread_ctl->push_back(
    //   std::make_shared<std::future<void>>(
    //     std::async(
    //       std::launch::async,
    //       &smap::object_pose_estimator::object_filtering_thread,this,
    //       pcl_point_cloud,
    //       std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    //     )
    //   )
    // );


    RCLCPP_INFO(this->get_logger(),"Item launched: %i| label: %i | Total: %i",this->thread_ctl->size(),obj.label,++z);


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

