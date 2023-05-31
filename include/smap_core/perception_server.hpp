#ifndef SMAP_CORE__PERCEPTION_SERVER_HPP_
#define SMAP_CORE__PERCEPTION_SERVER_HPP_

#include "visibility_control.h"


#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

// SMAP
#include "../../include/smap_core/macros.hpp"
#include "smap_interfaces/srv/add_perception_module.hpp"
#include "smap_interfaces/srv/smap_classes.hpp"
#include "smap_interfaces/msg/smap_detections.hpp"

enum detector_type
{
  object,
  place
};

struct detector_t
{
  std::string name = "";
  int id = 0;
  detector_type type = object;
  std::string architecture = "";
  size_t n_classes = 0;
  std::vector<std::string> classes;
};

namespace smap
{

  class perception_server : public rclcpp::Node
  {
  private:
    std::vector<detector_t> detectors;
    std::list<std::pair<int, std::string>> classes;
    int n_classes = 0;

    rclcpp::Service<smap_interfaces::srv::AddPerceptionModule>::SharedPtr add_perception_module_srv = this->create_service<smap_interfaces::srv::AddPerceptionModule>(
        std::string(this->get_namespace())+std::string("/perception_server/add_perception_module"), std::bind(
                                                                  &smap::perception_server::AddPerceptionModule_callback, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::Service<smap_interfaces::srv::SmapClasses>::SharedPtr list_classes_srv = this->create_service<smap_interfaces::srv::SmapClasses>(
        std::string(this->get_namespace())+std::string("/perception_server/list_classes"), std::bind(
                                                                  &smap::perception_server::ListClasses_callback, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::Subscription<smap_interfaces::msg::SmapObject>::SharedPtr objects_sub = this->create_subscription<smap_interfaces::msg::SmapObject>(
      std::string(this->get_namespace())+std::string("/object_estimator/objects"), 10,std::bind(&perception_server::objects_callback, this, std::placeholders::_1)
    );

  public:
    // Constructor/Destructor
    inline perception_server()
        : Node("perception_server")
    {
      RCLCPP_INFO(this->get_logger(), "Initializing perception_server");
    }

    inline perception_server(const rclcpp::NodeOptions &options)
        : Node("perception_server", options)
    {
      RCLCPP_INFO(this->get_logger(), "Initializing perception_server");
    }

    inline ~perception_server() {}

    std::list<std::pair<int, std::string>> add_detector(detector_t &new_detector);

    void AddPerceptionModule_callback(
        const std::shared_ptr<smap_interfaces::srv::AddPerceptionModule::Request> request,
        std::shared_ptr<smap_interfaces::srv::AddPerceptionModule::Response> response);

    void ListClasses_callback(
      const std::shared_ptr<smap_interfaces::srv::SmapClasses::Request> request,
      std::shared_ptr<smap_interfaces::srv::SmapClasses::Response> response
    );

    

    void objects_callback(const smap_interfaces::msg::SmapObject::SharedPtr object) const;

    // print data

    void print_classes(std::string pref, std::list<std::pair<int, std::string>> &classes);

    void print_classes(std::string pref, std::vector<std::string> &classes);

    void print_detector(std::string pref, detector_t &det);

    void print_server_data(void);
  };

} // namespace smap

// RCLCPP_COMPONENTS_REGISTER_NODE(smap::perception_server)

#endif // SMAP_CORE__PERCEPTION_SERVER_HPP_
