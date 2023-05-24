#include <cstdio>
#include <chrono>
#include <math.h>
#include <fstream>
#include <iostream>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "rclcpp/rclcpp.hpp"

// #include <visualization_msgs/msg/marker.hpp>

#include "../include/smap_core/macros.hpp"
// #include "../include/smap_core/detector.hpp"
#include "../include/smap_core/topological_map.hpp"
#include "../include/smap_core/thing.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "smap_interfaces/msg/smap_data.hpp"
// #include "smap_interfaces/srv/add_three_ints.hpp"
// #include "../srv/AddThreeInts.hpp"

#include <string>

#include "std_msgs/msg/string.hpp"

// #include "smap_core/object_pose_estimator.hpp"
#include "../include/smap_core/components/object_pose_estimator.hpp"
#include "../include/smap_core/components/perception_server.hpp"



#define FROM_FRAME std::string("map")
#define TO_FRAME std::string("base_link")


// Node responsible to manage all the services
/* TODO Parameter POS_RATE
        Position acquisition rate
*/

// Node

namespace smap
{

class smap_node : public rclcpp::Node
{
private:

  // Logger
  rclcpp::Logger logger = this->get_logger();

public:
  // Constructor/Destructor
  smap_node()
  : Node("smap_core")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing smap_core");

  }
  ~smap_node()
  {
  }

  void on_process(void) // Pooling
  {
    // RCLCPP_DEBUG(this->get_logger(),"Process smap::smap_node");
  }

private:


public:
  // Map
  std::shared_ptr<smap::topological_map> topo_map;
};


}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<smap::smap_node> _smap_node = std::make_shared<smap::smap_node>();
  std::shared_ptr<smap::topological_map> _topological_map_node = std::make_shared<smap::topological_map>();
  std::shared_ptr<smap::object_pose_estimator> _object_pose_estimator_node = std::make_shared<smap::object_pose_estimator>(options);
  std::shared_ptr<smap::perception_server> _perception_server_node = std::make_shared<smap::perception_server>(options);
  _smap_node->topo_map = _topological_map_node;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(_smap_node);
  executor.add_node(_topological_map_node);
  executor.add_node(_object_pose_estimator_node);
  executor.add_node(_perception_server_node);
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service = _smap_node->create_service<std_srvs::srv::Trigger>("serv_smap", &smap::test_serv);
  while (rclcpp::ok()) {
    try{
      executor.spin_once();
      _smap_node->on_process(); // Pooling
      _topological_map_node->on_process(); // Pooling
    }catch (std::exception& e){
      std::cout << "Exception!" << std::endl;
      std::cout << e.what() << std::endl;
    }
  }
  rclcpp::shutdown();


  //printf("hello world smap package\n");
  return 0;
}

