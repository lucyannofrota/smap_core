#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "../include/smap_core/macros.hpp"

//#include <pcl-1.10/pcl/point_cloud.h>
//#include <pcl-1.10/pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/range_image/range_image.h>


#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <boost/foreach.hpp>

#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// #include "smap_interfaces/msg"
#include "smap_interfaces/msg/smap_detections.hpp"
#include "smap_interfaces/msg/smap_data.hpp"
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/bounding_box2_d.hpp"
// #include "smap_interfaces/msg/smap_data.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <iostream>

using namespace std::chrono_literals;

// TODO : Introduzir callback groups


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}




namespace smap
{

class object_pose_estimator : public rclcpp::Node
{
private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
    "pcl", 10);
    
  // rclcpp::Subscription<smap_interfaces::msg::SmapData>::SharedPtr smap_data_sub = this->create_subscription<smap_interfaces::msg::SmapData>(
  //   "/smap/sampler/data",10,std::bind(
  //     &smap::object_pose_estimator::_callback, this, std::placeholders::_1)
  // );

  rclcpp::Subscription<smap_interfaces::msg::SmapDetections>::SharedPtr smap_detections_sub = this->create_subscription<smap_interfaces::msg::SmapDetections>(
    "/smap_core/perception/modules/predictions",10,std::bind(
      &smap::object_pose_estimator::detections_callback, this, std::placeholders::_1)
  );

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/smap_core/perception/cpp",10);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("pci",10);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pcl_pub1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("pci1",10);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_test = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pci",10,std::bind(
      &smap::object_pose_estimator::pcl_sub, this, std::placeholders::_1)
  );

  rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
    std::chrono::seconds(5), // Change Frequency
    std::bind(
      &smap::object_pose_estimator::pcl_pub,
      this
    )
  );

  // pcl::visualization::PCLVisualizer viewer();

  // pcl::visualization::PCLVisualizer::Ptr viewer;

  // pcl::visualization::RangeImageVisualizer range_image_widget;

public:
  // Constructor/Destructor
  object_pose_estimator()
  : Node("smap_object_pose_estimator")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing smap_object_pose_estimator");
    // this->pcl_test()

    // this->viewer.reset (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // this->viewer->setBackgroundColor (0, 0, 0);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // // pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    // this->viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");

    // this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // this->viewer->addCoordinateSystem (1.0);
    // this->viewer->initCameraParameters ();
    
  }
  ~object_pose_estimator()
  {
  }

  void pcl_pub(void){
    const int width = 5, height = 5;
    printf(".\n");
    sensor_msgs::msg::PointCloud2 msg;
    msg.data.resize(width*height);
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "map";
    // msg.point_step = (sizeof(float) * 4);
    msg.row_step = width*msg.point_step;

    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(2,"xyz","rgb");
    mod.resize(height*width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg,"x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg,"y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg,"z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg,"r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg,"g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg,"b");
    

    // Salvar em formato de image!

    msg.height = width;
    msg.width = height;

    unsigned int x,y;
    printf("(h,w)|(%i,%i)\n",msg.height,msg.width);
    for(y = 1; y <= msg.height; y++){
      for(x = 1; x <= msg.width; x++){
        *iter_x = (float) x+0.1;
        *iter_y = (float) y+0.2;
        *iter_z = (float) 1;
        *iter_r = (uint8_t) 255;
        *iter_g = (uint8_t) 144;
        *iter_b = (uint8_t) 77;

        printf("(x,y,z,r,g,b)->[%5.2f,%5.2f,%5.2f,%3i,%3i,%3i]\n",
          *iter_x,
          *iter_y,
          *iter_z,
          *iter_r,
          *iter_g,
          *iter_b
        );

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
      }
    }

    this->test_pcl_pub->publish(msg);
  }

  void pcl_sub(const sensor_msgs::msg::PointCloud2::SharedPtr input){

    int x_in = 1;
    int x_fin = 3;
    int y_in = 1;
    int y_fin = 3;

    sensor_msgs::msg::PointCloud2 msg;

    const int width = x_fin - x_in + 1, height = y_fin - y_in + 1;

    msg.data.resize(width*height);
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "map";
    msg.point_step = input->point_step;
    msg.row_step = width*msg.point_step;


    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(2,"xyz","rgb");
    mod.resize(height*width);

    msg.height = height;
    msg.width = width;

    sensor_msgs::PointCloud2Iterator<float> iter_x(*input,"x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*input,"y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*input,"z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*input,"r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*input,"g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*input,"b");

    sensor_msgs::PointCloud2Iterator<float> msg_x(msg,"x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg,"y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg,"z");
    sensor_msgs::PointCloud2Iterator<uint8_t> msg_r(msg,"r");
    sensor_msgs::PointCloud2Iterator<uint8_t> msg_g(msg,"g");
    sensor_msgs::PointCloud2Iterator<uint8_t> msg_b(msg,"b");


    int count = 0;

    int offset;

    for(int h = y_in; h <= x_fin; h++){
      for(int w = x_in; w <= y_fin; w++){
        offset=(w)+input->width*(h);

        *msg_x = *(iter_x+offset); ++msg_x;
        *msg_y = *(iter_y+offset); ++msg_y;
        *msg_z = *(iter_z+offset); ++msg_z;
        *msg_r = *(iter_r+offset); ++msg_r;
        *msg_g = *(iter_g+offset); ++msg_g;
        *msg_b = *(iter_b+offset); ++msg_b;

      }
      count++;
    }
    this->test_pcl_pub1->publish(msg);
  }

  //void pcl_sub(const sensor_msgs::msg::PointCloud2::SharedPtr input){
  //   sensor_msgs::msg::PointCloud2 msg;

  //   msg.height = 2;
  //   msg.width = 2;
  //   msg.data.resize(msg.width*msg.height);
  //   msg.header.stamp = rclcpp::Clock().now();
  //   msg.header.frame_id = "map";
  //   msg.point_step = (sizeof(float) * 4);
  //   msg.row_step = msg.width*msg.point_step;


  //   sensor_msgs::PointCloud2Modifier mod(msg);
  //   mod.setPointCloud2FieldsByString(2,"xyz","rgb");
  //   mod.resize(msg.height*msg.width);

  //   sensor_msgs::PointCloud2Iterator<float> iter_x(*input,"x");
  //   sensor_msgs::PointCloud2Iterator<float> iter_y(*input,"y");
  //   sensor_msgs::PointCloud2Iterator<float> iter_z(*input,"z");
  //   sensor_msgs::PointCloud2Iterator<float> iter_rgb(*input,"rgb");

  //   sensor_msgs::PointCloud2Iterator<float> msg_x(msg,"x");
  //   sensor_msgs::PointCloud2Iterator<float> msg_y(msg,"y");
  //   sensor_msgs::PointCloud2Iterator<float> msg_z(msg,"z");
  //   sensor_msgs::PointCloud2Iterator<float> msg_rgb(msg,"rgb");


  //   for(int h = 1; h < 3; h++){
  //     for(int w = 1; w < 3; w++){
  //       printf("(w,h)=(%i,%i)\n",w,h);
  //       uint8_t *array;
  //       array = reinterpret_cast<uint8_t*>(&iter_rgb);
  //       printf("(x,y,z,rgb)=(%f,%f,%f,%f)\n",
  //         *iter_x,
  //         *iter_y,
  //         *iter_z,
  //         *iter_rgb
  //       );

  //       *msg_x=*iter_x;
  //       *msg_y=*iter_y;
  //       *msg_z=2;
  //       *msg_rgb=*iter_rgb;

  //       ++iter_x;
  //       ++iter_y;
  //       ++iter_z;
  //       ++iter_rgb;

  //       ++msg_x;
  //       ++msg_y;
  //       ++msg_z;
  //       ++msg_rgb;
  //     }
  //   }
  //   this->test_pcl_pub1->publish(msg);
  // }


  void detections_callback(const smap_interfaces::msg::SmapDetections::SharedPtr input_msg) {
    // const sensor_msgs::msg::PointCloud2::SharedPtr input) const{
    RCLCPP_INFO(this->get_logger(),"detections_callback");


    // this->object_segmentation(input_msg);
    this->pcl_pub();


    // Convert to pcl
    // static pcl::PointCloud<pcl::PointXYZRGB> cloud;
    // // sensor_msgs::msg::PointCloud2 pc;
    // // pcl::fromROSMsg(pc,cloud);
    // pcl::fromROSMsg(input_msg->pointcloud,cloud);


    // RCLCPP_INFO(this->get_logger(),"Points: %i, %i",cloud.width,cloud.height);

    // if(!this->viewer->wasStopped()){
    //   this->viewer->removePointCloud("sample cloud");
    //   this->viewer->addPointCloud<pcl::PointXYZRGB>(cloud.makeShared(), "sample cloud");
    //   this->viewer->updatePointCloud(cloud.makeShared(),"sample cloud");
    //   this->viewer->spinOnce(100,true);
    // }



    // // BOOST_FOREACH()

    //     // // while (!viewer->wasStopped ())
    // // while (!this->range_image_widget.wasStopped())
    // // {
    // //   // viewer->spinOnce (100);
    // //   // range_image_widget
    // //   // std::this_thread::sleep_for(100ms);
    // //   this->range_image_widget.spinOnce ();
    // //   // viewer->spinOnce(100);
    // //   pcl_sleep (0.01);
    // // }






    // // Convert to sensor_msgs
    // sensor_msgs::msg::PointCloud2 output;
    // pcl::toROSMsg(cloud,output);
    // // pub
    // this->debug_pcl_pub->publish(output);
  }

  void object_segmentation(const smap_interfaces::msg::SmapDetections::SharedPtr input_msg) const{
    RCLCPP_INFO(this->get_logger(),"Seg");  
    static smap_interfaces::msg::SmapObject obj;
    sensor_msgs::PointCloud2Iterator<float> iter_x(input_msg->pointcloud,"x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(input_msg->pointcloud,"y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(input_msg->pointcloud,"z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(input_msg->pointcloud,"r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(input_msg->pointcloud,"g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(input_msg->pointcloud,"b");

    static sensor_msgs::PointCloud2Modifier mod(obj.obj_pointcloud);
    // input_msg->objects
    BOOST_FOREACH(obj, input_msg->objects){
      RCLCPP_INFO(this->get_logger(),"Object: %i",obj.label);
      // obj.bounding_box_2d.keypoint_2

      obj.obj_pointcloud.width = obj.bounding_box_2d.keypoint_2[0] - obj.bounding_box_2d.keypoint_1[0] + 1;
      obj.obj_pointcloud.height = obj.bounding_box_2d.keypoint_2[1] - obj.bounding_box_2d.keypoint_1[1] + 1;

      obj.obj_pointcloud.point_step = input_msg->pointcloud.point_step;
      obj.obj_pointcloud.row_step = obj.obj_pointcloud.width * obj.obj_pointcloud.point_step;

      RCLCPP_INFO(this->get_logger(),"w: %i, h: %i",obj.obj_pointcloud.width,obj.obj_pointcloud.height);
      int count = 0;
      mod.resize(obj.obj_pointcloud.width*obj.obj_pointcloud.height);

      // sensor_msgs::PointCloud2Iterator

      for(int w = obj.bounding_box_2d.keypoint_1[0]; w <= obj.bounding_box_2d.keypoint_2[0]; w++){
        for(int h = obj.bounding_box_2d.keypoint_1[1]; h <= obj.bounding_box_2d.keypoint_2[1]; h++){
          std::cout << (double)*iter_x << std::endl;
          std::cout << (double)iter_x[0] << std::endl;
          count++;
          // RCLCPP_INFO(this->get_logger(),"val: %u",obj.obj_pointcloud.data[0]);
        }
        break;
      }
      RCLCPP_INFO(this->get_logger(),"sup: %i|%i",obj.obj_pointcloud.width*obj.obj_pointcloud.height,count);

      // obj.obj_pointcloud.data

      obj.obj_pointcloud.is_bigendian = false;



      // Copy parameters from original pointcloud
      obj.obj_pointcloud.header = input_msg->pointcloud.header;
      obj.obj_pointcloud.fields = input_msg->pointcloud.fields;
      obj.obj_pointcloud.is_dense = input_msg->pointcloud.is_dense;
    }
  }

  void _callback(const smap_interfaces::msg::SmapData::SharedPtr input_msg) const {
    // const sensor_msgs::msg::PointCloud2::SharedPtr input) const{
    RCLCPP_INFO(this->get_logger(),"_callback");
    // Convert to pcl
    static pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(input_msg->pointcloud,cloud);


    RCLCPP_INFO(this->get_logger(),"Points: %i, %i",cloud.width,cloud.height);

    // if(!this->viewer->wasStopped()){
    //   this->viewer->removePointCloud("sample cloud");
    //   this->viewer->addPointCloud<pcl::PointXYZRGB>(cloud.makeShared(), "sample cloud");
    //   // this->viewer->updatePointCloud(cloud.makeShared(),"sample cloud");
    //   this->viewer->spinOnce(100,true);
    // }

        // // while (!viewer->wasStopped ())
    // while (!this->range_image_widget.wasStopped())
    // {
    //   // viewer->spinOnce (100);
    //   // range_image_widget
    //   // std::this_thread::sleep_for(100ms);
    //   this->range_image_widget.spinOnce ();
    //   // viewer->spinOnce(100);
    //   pcl_sleep (0.01);
    // }






    // Convert to sensor_msgs
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud,output);
    // pub
  }

  void pcl_test(void){
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
  
    // Generate the data
    for (float y=-0.5f; y<=0.5f; y+=0.01f) {
      for (float z=-0.5f; z<=0.5f; z+=0.01f) {
        pcl::PointXYZRGB point;
        point.x = 2.0f - y;
        point.y = y;
        point.z = z;
        point.r = 255; point.g = 255; point.b = 255;
        pointCloud.points.push_back(point);
      }
    }
    pointCloud.width = pointCloud.size();
    pointCloud.height = 1;
    
    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;
    
    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    
    std::cout << rangeImage << "\n";

    sensor_msgs::msg::Image msg;
    //sensor_msgs::msg::PointCloud2 pc;

    //pcl::toROSMsg(rangeImage,msg);
    //pcl::toROSMsg(pointCloud,msg);
    //pcl::toROSMsg(pointCloud,pc);
    //pcl::toROSMsg(pc,msg);

    //pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    //range_image_widget.showRangeImage (rangeImage);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr = pointCloud.makeShared();
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    // this->viewer = simpleVis(basic_cloud_ptr);

    // pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");

    
    // this->range_image_widget.showRangeImage(rangeImage);


    // pcl::toROSMsg(pointCloud,msg);
    // this->img_publisher->publish(msg);

    // //--------------------
    // // -----Main loop-----
    // //--------------------
    // // while (!viewer->wasStopped ())
    // while (!this->range_image_widget.wasStopped())
    // {
    //   // viewer->spinOnce (100);
    //   // range_image_widget
    //   // std::this_thread::sleep_for(100ms);
    //   this->range_image_widget.spinOnce ();
    //   // viewer->spinOnce(100);
    //   pcl_sleep (0.01);
    // }



  }

  void on_process(void) // Pooling
  {
    // RCLCPP_DEBUG(this->get_logger(),"Process smap::smap_node");
  }

private:

};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<smap::object_pose_estimator> node = std::make_shared<smap::object_pose_estimator>();

  try{
    rclcpp::spin(node);
  }catch (std::exception& e){
    std::cout << "Exception!" << std::endl;
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}

