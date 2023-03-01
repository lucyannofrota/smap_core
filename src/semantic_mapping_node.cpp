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
// #include "../include/semantic_mapping/detector.hpp"
#include "../include/semantic_mapping/topological_map.hpp"
#include "../include/semantic_mapping/thing.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "semantic_mapping/msg/smap_data.hpp"
#include "semantic_mapping/srv/add_three_ints.hpp"
// #include "../srv/AddThreeInts.hpp"

#include <string>

#include "std_msgs/msg/string.hpp"

#define FROM_FRAME std::string("map")
#define TO_FRAME std::string("base_link")

#define RAD2DEG(x) 180 * x / M_PI

// Node responsible to manage all the services

/* TODO Parameter POS_RATE
        Position acquisition rate
*/

// Node

class smap_node : public rclcpp::Node
{
private:
  //** Variables **//
  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

  // semantic_mapping::topological_map thing_map;


  // Publisher
  rclcpp::Publisher<semantic_mapping::msg::SmapData>::SharedPtr SmapData_pub = this->create_publisher<semantic_mapping::msg::SmapData>("/SMap/classifiers/Data", 10);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);

  // Subscriptions
  //rclcpp::Subscriptions<

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
  ~smap_node()
  {
  }

  void on_process(void) // Pooling
  {
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
    tf2::Matrix3x3(
      tf2::Quaternion(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
      )).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::PoseStamped current_pose;

    current_pose.header.frame_id = "/map";
    current_pose.header.stamp = this->get_clock().get()->now();

    //geometry_msgs::msg::Point point;

    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
    current_pose.pose.position.z = transform.transform.translation.z;
    current_pose.pose.orientation = transform.transform.rotation;
    thing_map->add_vertex(current_pose.pose.position);

    //SmapData_pub->publish(current_pose);

    auto message = std_msgs::msg::String();
    message.data = "Hello, world! ";
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);

  }

public:
  // Map
  std::shared_ptr<semantic_mapping::topological_map> thing_map;
};

// Services

void test_serv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
  (void) request;
  request->structure_needs_at_least_one_member;
  response->success = true;
  response->message = "Serv_SMAP";
  RCLCPP_INFO(rclcpp::get_logger("Serv_SMAP"),"Service Serv_SMAP");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<smap_node> _smap_node = std::make_shared<smap_node>();
  std::shared_ptr<semantic_mapping::topological_map> _topological_map_node =
    std::make_shared<semantic_mapping::topological_map>();
  _smap_node->thing_map = _topological_map_node;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(_smap_node);
  executor.add_node(_topological_map_node);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service = _smap_node->create_service<std_srvs::srv::Trigger>("serv_smap", &test_serv);
  while (rclcpp::ok()) {
    try{
      _smap_node->on_process(); // Pooling
      _topological_map_node->on_process(); // Pooling
      executor.spin_once();
    }catch (std::exception& e){
      std::cout << "Exception!" << std::endl;
      std::cout << e.what() << std::endl;
    }
  }
  rclcpp::shutdown();


  printf("hello world semantic_mapping package\n");
  return 0;
}
