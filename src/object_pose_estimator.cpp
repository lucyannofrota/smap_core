#include "../include/smap_core/object_pose_estimator.hpp"

using namespace std::chrono_literals;

#include "../include/smap_core/parameter_tuning.hpp"

// void imgui_thread(std::shared_ptr<smap::object_pose_estimator> obj_ptr){
//   // GL 3.0 + GLSL 130
//   const char* glsl_version = "#version 130";
//   SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
//   SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
//   SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
//   SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

//   // Create window with graphics context
//   SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
//   SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
//   SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
//   SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
//   SDL_Window* window = SDL_CreateWindow("Parameter tuning", 0, 0, 480, 320, window_flags);
//   SDL_GLContext gl_context = SDL_GL_CreateContext(window);
//   SDL_GL_MakeCurrent(window, gl_context);
//   SDL_GL_SetSwapInterval(1); // Enable vsync

//   // Setup Dear ImGui context
//   IMGUI_CHECKVERSION();
//   ImGui::CreateContext();
//   ImGuiIO& io = ImGui::GetIO(); (void)io;
//   io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
//   io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

//   // Setup Dear ImGui style
//   ImGui::StyleColorsDark();
//   //ImGui::StyleColorsLight();

//   // Setup Platform/Renderer backends
//   ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
//   ImGui_ImplOpenGL3_Init(glsl_version);

//   // Our state
//   ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

//   // Main loop
//   bool done = false;
//   while (!done)
//   {
//       // Poll and handle events (inputs, window resize, etc.)
//       // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
//       // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
//       // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
//       // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
//       SDL_Event event;
//       while (SDL_PollEvent(&event))
//       {
//           ImGui_ImplSDL2_ProcessEvent(&event);
//           if (event.type == SDL_QUIT)
//               done = true;
//           if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
//               done = true;
//       }

//       // Start the Dear ImGui frame
//       ImGui_ImplOpenGL3_NewFrame();
//       ImGui_ImplSDL2_NewFrame();
//       ImGui::NewFrame();

//       // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
//       {
//           // static float f = 0.0f;
//           // static int counter = 0;

//           ImGui::Begin("Object_pose_estimator parameters");


//           if(ImGui::CollapsingHeader("roi_filter")){ // roi_filter

//             ImGui::Checkbox("roi_filter", &(obj_ptr->roi_filt));

//             ImGui::Text("pcl_lim");       
//             ImGui::Text("   Distance limits applyed to the cloud.");       
            

//             ImGui::SliderFloat("min", &(obj_ptr->pcl_lims->first), 0.0f, obj_ptr->pcl_lims->second);
//             ImGui::SliderFloat("max", &(obj_ptr->pcl_lims->second), obj_ptr->pcl_lims->first, 10.0f);
//           }

//           if(ImGui::CollapsingHeader("pcl_voxelization")){ // pcl_voxelization
//             ImGui::Checkbox("pcl_voxelization", &(obj_ptr->voxelization));
//             ImGui::Text("LeafSize");
//             ImGui::Text("   Greather values increase the size of the voxels (filter more points)");
//             ImGui::SliderFloat("LeafSize", &(obj_ptr->leaf_size), 0.0f, 0.05f);
//           }

//           if(ImGui::CollapsingHeader("statistical_outlier_filter")){ // statistical_outlier_filter
//             ImGui::Checkbox("statistical_outlier_filter", &(obj_ptr->sof));
//             ImGui::Text("MeanK");
//             ImGui::Text("   Number of neighbours to evaluate");
//             ImGui::SliderInt("MeanK", &(obj_ptr->mean_k), 0, 100);

//             ImGui::Text("Mu");
//             ImGui::Text("   Local standard deviation");
//             ImGui::SliderFloat("Mu", &(obj_ptr->mu), 0.0f, 2.0f);
//           }

//           // ImGui::SetNextWindowPos()
//           ImGui::End();
//       }


//       // Rendering
//       ImGui::Render();
//       glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
//       glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
//       glClear(GL_COLOR_BUFFER_BIT);
//       ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
//       SDL_GL_SwapWindow(window);
//   }
// #ifdef __EMSCRIPTEN__
//   EMSCRIPTEN_MAINLOOP_END;
// #endif

//   // Cleanup
//   ImGui_ImplOpenGL3_Shutdown();
//   ImGui_ImplSDL2_Shutdown();
//   ImGui::DestroyContext();

//   SDL_GL_DeleteContext(gl_context);
//   SDL_DestroyWindow(window);
//   SDL_Quit();
// }




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
  this->min_cut_clustering(segment_cloud_pcl);

  this->cloud_segmentation(segment_cloud_pcl);

  

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

  bool a = false;

  auto lambda = [&node,&a] (void){ // ui lambda function
    ImGui::Begin("Object_pose_estimator parameters");
    // ImGui::Button

    if (ImGui::CollapsingHeader("roi_filter")){ // roi_filter

      static bool check_roi_filter = true;
      if(ImGui::Checkbox("roi_filter", &check_roi_filter)) node->roi_filt = check_roi_filter;

      ImGui::Text("pcl_lim");
      ImGui::Text("   Distance limits applyed to the cloud.");

      ImGui::SliderFloat("min", &(node->pcl_lims->first), 0.0f, node->pcl_lims->second);
      ImGui::SliderFloat("max", &(node->pcl_lims->second), node->pcl_lims->first, 10.0f);
    }


    if(ImGui::CollapsingHeader("pcl_voxelization")){ // pcl_voxelization
      static bool check_voxelization = true;
      if(ImGui::Checkbox("voxelization", &check_voxelization)) node->voxelization = check_voxelization;
      ImGui::Text("LeafSize");
      ImGui::Text("   Greather values increase the size of the voxels (filter more points)");
      ImGui::SliderFloat("LeafSize", &(node->leaf_size), 0.0f, 0.05f);
    }

    if(ImGui::CollapsingHeader("statistical_outlier_filter")){ // statistical_outlier_filter
      static bool check_sof = true;
      if(ImGui::Checkbox("statistical_outlier_filter", &check_sof)) node->sof = check_sof;
      ImGui::Text("MeanK");
      ImGui::Text("   Number of neighbours to evaluate");
      ImGui::SliderInt("MeanK", &(node->mean_k), 0, 100);

      ImGui::Text("Mu");
      ImGui::Text("   Local standard deviation");
      ImGui::SliderFloat("Mu", &(node->mu), 0.0f, 2.0f);
    }

    // ImGui::Button("Lock pcl",)

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

  // auto ui = std::async(
  //   std::launch::async,
  //   &imgui_thread,
  // );


  try{
    rclcpp::spin(node);
  }catch (std::exception& e){
    std::cout << "SMAP Exception!" << std::endl;
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}

