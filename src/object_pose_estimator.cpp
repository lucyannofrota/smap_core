#include "../include/smap_core/object_pose_estimator.hpp"

using namespace std::chrono_literals;

#include "../include/smap_core/parameter_tuning.hpp"






namespace smap
{



void object_pose_estimator::object_estimation_thread(const pcl::shared_ptr<cloud_t> point_cloud, const smap_interfaces::msg::SmapObject::SharedPtr obj){

  std::chrono::_V2::system_clock::time_point start, stop;
  // pcl::shared_ptr<cloud_t> point_cloud (new cloud_t); // REMOVE
  // pcl::io::loadPCDFile("test_pcd.pcd",*point_cloud);
  size_t outliers = point_cloud->size();
  start = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(this->get_logger(),"Object: %i",obj->label);
  pcl::shared_ptr<cloud_t> point_cloud_vox(new cloud_t);
  pcl::shared_ptr<cloud_t> segment_cloud_pcl(new cloud_t);
  // pcl::shared_ptr<cloud_t> segment_cloud_pcl_neg(new cloud_t);

  std::shared_ptr<sensor_msgs::msg::PointCloud2> segment_cloud_ros(new sensor_msgs::msg::PointCloud2);


  // Cloud filtering
  this->box_filter(point_cloud,segment_cloud_pcl,obj);

  if(this->roi_filt){
    this->roi_filter(segment_cloud_pcl);
  }

  if(this->voxelization){
    this->pcl_voxelization(segment_cloud_pcl);
  }

  if(this->sof){
    this->statistical_outlier_filter(segment_cloud_pcl);
  }

  // Cloud Segmentation
  // this->min_cut_clustering(segment_cloud_pcl);

  // this->cloud_segmentation(segment_cloud_pcl);

  

  stop = std::chrono::high_resolution_clock::now();
  RCLCPP_WARN(this->get_logger(),"pcl_process time %ims | %i points removed (%5.2f%%)",
    (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count(),
    outliers-segment_cloud_pcl->size(),
    ((outliers-segment_cloud_pcl->size())*100.0/outliers)
  );

  this->total_filter_plot.push_back(
    (std::chrono::duration_cast<std::chrono::milliseconds>(stop-start)).count()
  );




  // pcl::toROSMsg(*segment_cloud_pcl_neg,*segment_cloud_ros);
  // this->test_pcl_pub_unfilt->publish(*segment_cloud_ros);
  pcl::toROSMsg(*segment_cloud_pcl,*segment_cloud_ros);
  segment_cloud_ros->header.frame_id = "map"; // REMOVE
  if(
    (obj->bounding_box_2d.keypoint_1[0] > 240) &&
    (obj->bounding_box_2d.keypoint_1[0] < 260)
  ) this->test_pcl_pub_filt->publish(*segment_cloud_ros);

  // char c = std::cin.get();
  // if(c == 's'){
  //   pcl::io::savePCDFileBinary("test_pcd.pcd",*point_cloud);
  // }


}


void object_pose_estimator::detections_callback(const smap_interfaces::msg::SmapDetections::SharedPtr input_msg) {
  RCLCPP_INFO(this->get_logger(),"detections_callback");

  // Convert sensor_msgs::pointloud2 to pcl::PointCloud
  pcl::shared_ptr<cloud_t> pcl_point_cloud (new cloud_t);
  pcl::fromROSMsg(input_msg->pointcloud,*pcl_point_cloud);
  // printf("Org: %i\n",pcl_point_cloud->isOrganized());


  // Thread launch

  smap_interfaces::msg::SmapObject obj;
  sensor_msgs::msg::PointCloud2 segment_cloud;

  static int z = 0;
  for(auto& obj : input_msg->objects){
  // BOOST_FOREACH(obj, input_msg->objects){
    if(obj.label != 62) continue;

    // Block until thread pool is available
    while(!this->thread_ctl->available()){
      std::this_thread::sleep_for(10ms);
    }

    static pcl::shared_ptr<cloud_t> lock_cloud;
    if(!this->pcl_lock) lock_cloud = pcl_point_cloud;


    this->object_estimation_thread(
      lock_cloud,
      std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    );

    // this->thread_ctl->push_back(
    //   std::make_shared<std::future<void>>(
    //     std::async(
    //       std::launch::async,
    //       &smap::object_pose_estimator::object_estimation_thread,this,
    //       pcl_point_cloud,
    //       std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    //     )
    //   )
    // );


    RCLCPP_INFO(this->get_logger(),"Item launched: %i| label: %i | Total: %i",this->thread_ctl->size(),obj.label,++z);


  }
  RCLCPP_INFO(this->get_logger(),"---Callback complete---");
}

}

// template <typename T>
// inline T RandomRange(T min, T max) {
//     T scale = rand() / (T) RAND_MAX;
//     return min + scale * ( max - min );
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<smap::object_pose_estimator> node = std::make_shared<smap::object_pose_estimator>();

  // auto ui = std::make_shared<std::future<void>>(
  //   std::async(
  //     std::launch::async,
  //     &imgui_thread,
  //   )
  // );
  auto plot_times_lambda = [] (plot_vec &vec)
  { // Plot times
    static char overlay[32];
    float vals[128] = {0};
    vec.get_float_arr_av(vals);
    sprintf(overlay, "avg %4.1fms", vec.get_average());
    ImGui::PlotLines(
      "Execution time",
      vals,
      IM_ARRAYSIZE(vals),
      0,
      overlay, 0.0f, 50.0f, ImVec2(0, 100.0f));
  };

  auto lambda = [&node,&plot_times_lambda] (void){ // ui lambda function
    ImGui::Begin("Object Pose Estimator Parameters");

    if (ImGui::CollapsingHeader("Region Of Interest Filter")){ // roi_filter
      static bool roi_filter = true;
      if(ImGui::Checkbox("ROI Filter", &roi_filter)) node->roi_filt = roi_filter;

      ImGui::Text("pcl_lim");
      ImGui::Text("   Distance limits applyed to the cloud.");

      ImGui::SliderFloat("min", &(node->pcl_lims->first), 0.0f, node->pcl_lims->second);
      ImGui::SliderFloat("max", &(node->pcl_lims->second), node->pcl_lims->first, 10.0f);


      plot_times_lambda(node->box_filter_plot);
    }

    if(ImGui::CollapsingHeader("PCL Voxelization")){ // pcl_voxelization
      static bool voxelization = true;
      if(ImGui::Checkbox("Voxelization", &voxelization)) node->voxelization = voxelization;
      ImGui::Text("LeafSize");
      ImGui::Text("   Greather values increase the size of the voxels (filter more points)");
      ImGui::SliderFloat("LeafSize", &(node->leaf_size), 0.0f, 0.05f);

      plot_times_lambda(node->roi_filter_plot);
    }

    if(ImGui::CollapsingHeader("Statistical Outlier Filter")){ // pcl_voxelization
      static bool sof = true;
      if(ImGui::Checkbox("SOF Filter", &sof)) node->sof = sof;
      ImGui::Text("MeanK");
      ImGui::Text("   Number of neighbours to evaluate");
      ImGui::SliderInt("MeanK", &(node->mean_k), 0, 100);

      ImGui::Text("Mu");
      ImGui::Text("   Local standard deviation");
      ImGui::SliderFloat("Mu", &(node->mu), 0.0f, 2.0f);

      plot_times_lambda(node->sof_filter_plot);
    }

    ImGui::Text("Total execution time");
    plot_times_lambda(node->total_filter_plot);

    static int clicked = 0;
    if (ImGui::Button("Lock PCL"))
        clicked++;
    if (clicked & 1)
    {
        ImGui::SameLine();
        ImGui::Text("PCL locked!");
        node->pcl_lock = true;
    }else node->pcl_lock = false;

    ImGui::End();

  };

  std::thread ui(
    imgui_thread<decltype(lambda)>,
    lambda
  );

  ui.detach();


  try{
    rclcpp::spin(node);
  }catch (std::exception& e){
    std::cout << "SMAP Exception!" << std::endl;
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}

