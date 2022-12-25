#include <cstdio>
#include <chrono>
#include <math.h>
#include <fstream>
#include <iostream>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"

// #include <visualization_msgs/msg/marker.hpp>

#include "../include/semantic_mapping/macros.hpp"
#include "../include/semantic_mapping/detector.hpp"
#include "../include/semantic_mapping/conceptual_map.hpp"
#include "../include/semantic_mapping/concept.hpp"

#define FROM_FRAME std::string("map")
#define TO_FRAME std::string("base_link")

#define RAD2DEG(x) 180*x/M_PI

// Node responsible to manage all the services

/* TODO Parameter POS_RATE
        Position acquisition rate
*/

class smap_node : public rclcpp::Node
{
private:
  //** Variables **//
  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

  // semantic_mapping::Conceptual_Map concept_map;


  // Timer
  rclcpp::TimerBase::SharedPtr timer{nullptr};

  // Logger
  rclcpp::Logger logger = this->get_logger();


public:
  // Constructor/Destructor
  smap_node()
  : Node("semantic_mapper")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing semantic_mapper");
    // tf buffer
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Callbacks
    timer = this->create_wall_timer(
      std::chrono::milliseconds(250),
      // std::chrono::seconds(1),
      std::bind(
        &smap_node::timer_callback,
        this
      )
    );

  }
  ~smap_node(){
  }

  void on_process(void){
    // RCLCPP_DEBUG(this->get_logger(),"Process smap_node");
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transform;

    // https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
    try {
      transform = tf_buffer->lookupTransform(
        FROM_FRAME, TO_FRAME,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger, "Could not transform %s to %s: %s",
        TO_FRAME.c_str(), FROM_FRAME.c_str(), ex.what());

      return;
    }

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w
    )).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Point point;
    point.x = transform.transform.translation.x;
    point.y = transform.transform.translation.y;
    point.z = transform.transform.translation.z;
    concept_map->add_vertex(point);
  }
  public:
  // Map
    std::shared_ptr<semantic_mapping::Conceptual_Map> concept_map;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<smap_node> _smap_node = std::make_shared<smap_node>();
  std::shared_ptr<semantic_mapping::Conceptual_Map> _conceptual_map_node = std::make_shared<semantic_mapping::Conceptual_Map>();
  _smap_node->concept_map = _conceptual_map_node;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(_smap_node);
  executor.add_node(_conceptual_map_node);
  while(rclcpp::ok()){
    _smap_node->on_process();
    _conceptual_map_node->on_process();
    executor.spin_once();
  }
  rclcpp::shutdown();


  printf("hello world semantic_mapping package\n");
  return 0;
}
