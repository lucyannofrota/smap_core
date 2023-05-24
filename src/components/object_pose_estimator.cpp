#include "../include/smap_core/components/object_pose_estimator.hpp"

using namespace std::chrono_literals;

#include "../include/smap_core/parameter_tuning.hpp"

namespace smap
{

void object_pose_estimator::box_filter(
  const pcl::shared_ptr<cloud_t>& input_cloud,
  const pcl::shared_ptr<cloud_t>& cloud_segment,
  const smap_interfaces::msg::SmapObject::SharedPtr& obj
) const{
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
  timer.get_time(this->get_logger(), str, box_filter_plot);
}

void object_pose_estimator::roi_filter(
  const pcl::shared_ptr<cloud_t>& point_cloud
) const{
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
  timer.get_time(this->get_logger(), str, roi_filter_plot);
}

void object_pose_estimator::pcl_voxelization(
  const pcl::shared_ptr<cloud_t>& point_cloud
) const{
  count_time timer;
  pcl::VoxelGrid<cloud_point_t> vox_grid;

  vox_grid.setInputCloud(point_cloud);
  // vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
  vox_grid.setLeafSize (this->leaf_size, this->leaf_size, this->leaf_size);
  vox_grid.setDownsampleAllData(true);
  vox_grid.filter(*point_cloud);

  const char str[] = "pcl_vox";
  timer.get_time(this->get_logger(), str, voxelization_plot);
}

void object_pose_estimator::statistical_outlier_filter(
  const pcl::shared_ptr<cloud_t>& cloud_segment
) const{
  count_time timer;
  pcl::StatisticalOutlierRemoval<cloud_point_t> sor;
  sor.setInputCloud(cloud_segment);
  sor.setMeanK(this->mean_k); // Greather values take more time to compute
  sor.setStddevMulThresh(this->mu);

  sor.filter(*cloud_segment);

  const char str[] = "statistical_outlier_filter";
  timer.get_time(this->get_logger(), str, sof_filter_plot);
}

void object_pose_estimator::euclidean_clustering(
  const pcl::shared_ptr<cloud_t>& cloud_segment,
  const pcl::shared_ptr<cloud_t>& object_cloud
) const{
  count_time timer;

  pcl::search::KdTree<cloud_point_t>::Ptr tree (new pcl::search::KdTree<cloud_point_t>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_segment,*cloud_segment,indices);
  tree->setInputCloud (cloud_segment);

  std::vector<pcl::PointIndices> cluster_idx;
  pcl::EuclideanClusterExtraction<cloud_point_t> ece;
  ece.setClusterTolerance(this->ClusterTolerance); // cm
  ece.setMinClusterSize(25);
  ece.setSearchMethod(tree);
  ece.setInputCloud(cloud_segment);
  ece.extract(cluster_idx);

  // Idxs of the biggest cluster
  if(cluster_idx.size() >= 1){
    for (const auto& idx : cluster_idx[0].indices) object_cloud->push_back((*cloud_segment)[idx]);
  }

  const char str[] = "euclidean_clustering";
  timer.get_time(this->get_logger(), str, euclidean_clustering_plot);
}

void object_pose_estimator::estimate_object_3D_AABB(
  const pcl::shared_ptr<cloud_t>& object_cloud,
  const smap_interfaces::msg::SmapObject::SharedPtr& obj
) const{
  count_time timer;

  obj->aabb.min.point.x = object_cloud->points[0].x;
  obj->aabb.min.point.y = object_cloud->points[0].y;
  obj->aabb.min.point.z = object_cloud->points[0].z;

  obj->aabb.max.point.x = object_cloud->points[0].x;
  obj->aabb.max.point.y = object_cloud->points[0].y;
  obj->aabb.max.point.z = object_cloud->points[0].z;

  for(cloud_point_t& point : *object_cloud){
    if(point.x < obj->aabb.min.point.x) obj->aabb.min.point.x = point.x;
    if(point.y < obj->aabb.min.point.y) obj->aabb.min.point.y = point.y;
    if(point.z < obj->aabb.min.point.y) obj->aabb.min.point.z = point.z;

    if(point.x > obj->aabb.max.point.x) obj->aabb.max.point.x = point.x;
    if(point.y > obj->aabb.max.point.y) obj->aabb.max.point.y = point.y;
    if(point.z > obj->aabb.max.point.z) obj->aabb.max.point.z = point.z;
  }

  obj->obj_pose.pose.position.x = (obj->aabb.min.point.x + obj->aabb.max.point.x)/2;
  obj->obj_pose.pose.position.y = (obj->aabb.min.point.y + obj->aabb.max.point.y)/2;
  obj->obj_pose.pose.position.z = (obj->aabb.min.point.z + obj->aabb.max.point.z)/2;

  const char str[] = "3D_AABB";
  timer.get_time(this->get_logger(), str, centroid_plot);
}

void object_pose_estimator::transform_object_param(
  const smap_interfaces::msg::SmapObject::SharedPtr& obj,
  const std::shared_ptr<geometry_msgs::msg::TransformStamped> &transform
) const{
  count_time timer;

  // Centroid
  tf2::doTransform<geometry_msgs::msg::PoseStamped>(obj->obj_pose,obj->obj_pose,*transform);

  // Limits
  tf2::doTransform<geometry_msgs::msg::PointStamped>(obj->aabb.min,obj->aabb.min,*transform);
  tf2::doTransform<geometry_msgs::msg::PointStamped>(obj->aabb.max,obj->aabb.max,*transform);

  // Point cloud
  tf2::doTransform<sensor_msgs::msg::PointCloud2>(obj->obj_pointcloud,obj->obj_pointcloud,*transform);

  const char str[] = "transform";
  timer.get_time(this->get_logger(), str, transform_plot);
}











void object_pose_estimator::object_estimation_thread(
  const pcl::shared_ptr<cloud_t>& point_cloud,
  const std::shared_ptr<geometry_msgs::msg::TransformStamped>& transform,
  const smap_interfaces::msg::SmapObject::SharedPtr& obj
){

  // printf("POS [%i,%i]\n",
  //   obj->bounding_box_2d.keypoint_1[0],
  //   obj->bounding_box_2d.keypoint_1[1]
  // );
  // if( 
  //   ! ((obj->bounding_box_2d.keypoint_1[0] > 180) &&
  //   (obj->bounding_box_2d.keypoint_1[0] < 300))
  // ) return;

  // if(obj->confidence <= 70) return;

  count_time timer;
  RCLCPP_INFO(this->get_logger(),"Object: %i",obj->label);
  pcl::shared_ptr<cloud_t> point_cloud_vox(new cloud_t);
  pcl::shared_ptr<cloud_t> segment_cloud_pcl(new cloud_t);
  pcl::shared_ptr<cloud_t> object_cloud_pcl(new cloud_t);
  // pcl::shared_ptr<cloud_t> segment_cloud_pcl_neg(new cloud_t);

  // std::shared_ptr<sensor_msgs::msg::PointCloud2> segment_cloud_ros(new sensor_msgs::msg::PointCloud2);


  // Cloud filtering
  count_time filter_timer;
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
  if(this->euclidean_clust){
    this->euclidean_clustering(segment_cloud_pcl,object_cloud_pcl);
  }
  const char str_filtering[] = "filtering_time";
  filter_timer.get_time(this->get_logger(), str_filtering, total_filter_plot);
  


  if(this->euclidean_clust) pcl::toROSMsg(*object_cloud_pcl,obj->obj_pointcloud);


  // Parameter Estimation
  count_time estimation_timer;
  if(this->euclidean_clust) this->estimate_object_3D_AABB(object_cloud_pcl,obj);
  else this->estimate_object_3D_AABB(segment_cloud_pcl,obj);

  const char str_estimation_time[] = "estimation_time";
  estimation_timer.get_time(this->get_logger(), str_estimation_time, total_estimation_plot);



  // Transform
  pcl::toROSMsg(*object_cloud_pcl,obj->obj_pointcloud);
  this->transform_object_param(obj,transform);

  






  const char str_process_time[] = "pcl_process";
  timer.get_time(this->get_logger(), str_process_time, total_thread_time);
  // plot_vec centroid_plot, boundaries_plot, total_estimation_plot;

  obj->obj_pointcloud.header.frame_id = "map"; // TODO: Check if can be removed

  const std::lock_guard<std::mutex> object_pub_lock(this->object_pub_mutex);
  this->object_pub->publish(*obj);

  const std::lock_guard<std::mutex> object_bb_pub_lock(this->object_bb_pub_mutex);
  this->puiblish_bb(0,obj);
  
  const std::lock_guard<std::mutex> debug_object_pcl_pub_lock(this->debug_object_pcl_pub_mutex);
  this->debug_object_pcl_pub->publish(obj->obj_pointcloud);

}


void object_pose_estimator::detections_callback(
  const smap_interfaces::msg::SmapDetections::SharedPtr input_msg
){
  RCLCPP_INFO(this->get_logger(),"detections_callback");

  pcl::shared_ptr<cloud_t> pcl_point_cloud (new cloud_t);
  pcl::fromROSMsg(input_msg->pointcloud,*pcl_point_cloud);


  // Thread launch

  smap_interfaces::msg::SmapObject obj;
  sensor_msgs::msg::PointCloud2 segment_cloud;

  static int z = 0;
  for(auto& obj : input_msg->objects){
  // BOOST_FOREACH(obj, input_msg->objects){
    if(obj.confidence < 70) continue; // TODO: Remove DEBUG

    if(obj.label != 62) continue; // TODO: Remove DEBUG

    obj.module_id = input_msg->module_id;

    // Block until thread pool is available
    while(!this->thread_ctl->available()){
      std::this_thread::sleep_for(10ms);
    }

    static pcl::shared_ptr<cloud_t> lock_cloud;
    if(!this->pcl_lock) lock_cloud = pcl_point_cloud;

    this->object_estimation_thread(
      lock_cloud,
      std::make_shared<geometry_msgs::msg::TransformStamped>(input_msg->robot_to_map),
      std::make_shared<smap_interfaces::msg::SmapObject>(obj)
    );

    // TODO: Activate multi threading

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
    sprintf(overlay, "cur: %4.1f ms| avg %4.1f ms", vals[127], vec.get_average());
    ImGui::PlotLines(
      "Execution time",
      vals,
      IM_ARRAYSIZE(vals),
      0,
      overlay, 0.0f, 200.0f, ImVec2(0, 100.0f));
  };

  auto lambda = [&node,&plot_times_lambda] (void){ // ui lambda function
    ImGui::Begin("Object Pose Estimator Parameters");

    if (ImGui::CollapsingHeader("Filtering")){ // Filtering
      ImGui::Indent();

      if (ImGui::CollapsingHeader("Box Filter")){ // box filter
        ImGui::Indent();
        plot_times_lambda(box_filter_plot);
        ImGui::Unindent();
      }


      if (ImGui::CollapsingHeader("Region Of Interest Filter")){ // roi_filter
        ImGui::Indent();
        static bool roi_filter = true;
        if(ImGui::Checkbox("ROI Filter", &roi_filter)) node->roi_filt = roi_filter;

        ImGui::Text("pcl_lim");
        ImGui::Text("   Distance limits applyed to the cloud.");

        ImGui::SliderFloat("min", &(node->pcl_lims->first), 0.0f, node->pcl_lims->second);
        ImGui::SliderFloat("max", &(node->pcl_lims->second), node->pcl_lims->first, 10.0f);


        plot_times_lambda(roi_filter_plot);
        ImGui::Unindent();
      }

      if(ImGui::CollapsingHeader("PCL Voxelization")){ // pcl_voxelization
        ImGui::Indent();
        static bool voxelization = true;
        if(ImGui::Checkbox("Voxelization", &voxelization)) node->voxelization = voxelization;
        ImGui::Text("LeafSize");
        ImGui::Text("   Greather values increase the size of the voxels (filter more points)");
        ImGui::SliderFloat("LeafSize", &(node->leaf_size), 0.0f, 0.05f);

        plot_times_lambda(voxelization_plot);
        ImGui::Unindent();
      }

      if(ImGui::CollapsingHeader("Statistical Outlier Filter")){ // pcl_voxelization
        ImGui::Indent();
        static bool sof = true;
        if(ImGui::Checkbox("SOF Filter", &sof)) node->sof = sof;
        ImGui::Text("MeanK");
        ImGui::Text("   Number of neighbours to evaluate");
        ImGui::SliderInt("MeanK", &(node->mean_k), 0, 100);

        ImGui::Text("Mu");
        ImGui::Text("   Local standard deviation");
        ImGui::SliderFloat("Mu", &(node->mu), 0.0f, 2.0f);

        plot_times_lambda(sof_filter_plot);
        ImGui::Unindent();
      }

      if(ImGui::CollapsingHeader("Euclidean Clustering")){ // pcl_voxelization
        ImGui::Indent();
        static bool euclidean_clustering = true;
        if(ImGui::Checkbox("Clustering", &euclidean_clustering)) node->euclidean_clust = euclidean_clustering;

        ImGui::Text("Cluster Tolerance");
        ImGui::Text("   Max space between points");
        ImGui::SliderFloat("ClusterTolerance", &(node->ClusterTolerance), 0.0f, 1.0f);

        plot_times_lambda(euclidean_clustering_plot);
        ImGui::Unindent();
      }

      plot_times_lambda(total_filter_plot);

      ImGui::Unindent();
    }

    if (ImGui::CollapsingHeader("Parameter Estimation")){ // Parameter estimation
      ImGui::Indent();

      if (ImGui::CollapsingHeader("Centroid and Limits")){ // Parameter estimation
        ImGui::Indent();

        plot_times_lambda(centroid_plot);

        ImGui::Unindent();
      }

      if (ImGui::CollapsingHeader("Transform")){ // Transform coordinates to map reference
        ImGui::Indent();

        plot_times_lambda(transform_plot);

        ImGui::Unindent();
      }

      plot_times_lambda(total_estimation_plot);

      ImGui::Unindent();
    }

    ImGui::Text("Total execution time");
    plot_times_lambda(total_thread_time);

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

