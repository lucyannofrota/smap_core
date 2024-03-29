#ifndef SMAP_CORE__PERCEPTION_SERVER_HPP_
#define SMAP_CORE__PERCEPTION_SERVER_HPP_

// STL
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

// ROS
#include "smap_base/visibility_control.h"
#include "std_msgs/msg/empty.hpp"

#include <rclcpp/rclcpp.hpp>

// SMAP
#include "smap_base/detector_descriptor.hpp"
// #include "smap_base/macros.hpp"
#include "smap_interfaces/msg/smap_detections.hpp"
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/smap_observation.hpp"
#include "smap_interfaces/srv/add_perception_module.hpp"
#include "smap_interfaces/srv/smap_classes.hpp"
#include "topo_map/topo_map.hpp"

using namespace std::chrono_literals;

namespace smap
{

class perception_server : public rclcpp::Node
{
  private:

    friend class topo_map;

    int n_classes = 0;

    rclcpp::Service< smap_interfaces::srv::AddPerceptionModule >::SharedPtr add_perception_module_srv =
        this->create_service< smap_interfaces::srv::AddPerceptionModule >(
            std::string( this->get_namespace() ) + std::string( "/perception_server/add_perception_module" ),
            std::bind(
                &smap::perception_server::AddPerceptionModule_callback, this, std::placeholders::_1,
                std::placeholders::_2 ) );

    rclcpp::Service< smap_interfaces::srv::SmapClasses >::SharedPtr list_classes_srv =
        this->create_service< smap_interfaces::srv::SmapClasses >(
            std::string( this->get_namespace() ) + std::string( "/perception_server/list_classes" ),
            std::bind(
                &smap::perception_server::ListClasses_callback, this, std::placeholders::_1, std::placeholders::_2 ) );

    rclcpp::Subscription< smap_interfaces::msg::SmapObservation >::SharedPtr objects_sub =
        this->create_subscription< smap_interfaces::msg::SmapObservation >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/objects" ), 2,
            std::bind( &perception_server::observations_callback, this, std::placeholders::_1 ) );

    rclcpp::Publisher< std_msgs::msg::Empty >::SharedPtr reset_detectors_pub =
        this->create_publisher< std_msgs::msg::Empty >(
            std::string( this->get_namespace() ) + std::string( "/perception/reboot_detectors" ), 10 );

    // rclcpp::TimerBase::SharedPtr timer_t_ = this->create_wall_timer( 2s, [ this ]() {
    //     for( int i = 0; i < 3; i++ )
    //     {
    //         RCLCPP_WARN( this->get_logger(), "Reboot Request" );
    //         this->reset_detectors_pub->publish( std_msgs::msg::Empty() );  // Request the reboot of all detectors
    //     }
    //     this->timer_t_->cancel();
    // } );

  public:

    std::shared_ptr< std::map< std::string, std::pair< int, int > > > classes =
        std::make_shared< std::map< std::string, std::pair< int, int > > >();  // pair[server_id, detector_id]

    std::shared_ptr< std::vector< detector_t > > detectors = std::make_shared< std::vector< detector_t > >();

    // Constructor/Destructor
    perception_server() : Node( "perception_server" )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing perception_server" );
        (void) std::system( ( std::string( "bash -c " )
                              + std::string( "\"source /workspace/install/setup.sh && ros2 topic pub "
                                             "/smap/perception/reboot_detectors -1 std_msgs/msg/Empty {} \"" ) )
                                .c_str() );
        // for( int i = 0; i < 3; i++ )
        //     this->reset_detectors_pub->publish( std_msgs::msg::Empty() );  // Request the reboot of all detectors
    }

    ~perception_server()
    {
        RCLCPP_WARN( this->get_logger(), "perception_server destructor" );
        // for( int i = 0; i < 3; i++ )
        // {
        //     RCLCPP_WARN( this->get_logger(), "Reboot Request" );
        //     this->reset_detectors_pub->publish( std_msgs::msg::Empty() );  // Request the reboot of all detectors
        // }
        RCLCPP_WARN( this->get_logger(), "Reboot Request" );
        // (void) std::system( ( std::string( "bash -c " )
        //                       + std::string( "\"source /workspace/install/setup.sh && ros2 topic pub "
        //                                      "/smap/perception/reboot_detectors -1 std_msgs/msg/Empty {} \"" ) )
        //                         .c_str() );
        (void) std::system( ( std::string( "bash -c " )
                              + std::string( "\"source /workspace/install/setup.sh && ros2 topic pub "
                                             "/smap/perception/reboot_detectors -1 std_msgs/msg/Empty {} &\"" ) )
                                .c_str() );
    }

    std::map< std::string, std::pair< int, int > > add_detector( detector_t& new_detector );

    void AddPerceptionModule_callback(
        const std::shared_ptr< smap_interfaces::srv::AddPerceptionModule::Request > request,
        std::shared_ptr< smap_interfaces::srv::AddPerceptionModule::Response > response );

    void ListClasses_callback(
        const std::shared_ptr< smap_interfaces::srv::SmapClasses::Request > request,
        std::shared_ptr< smap_interfaces::srv::SmapClasses::Response > response ) const;

    void observations_callback( const smap_interfaces::msg::SmapObservation::SharedPtr object ) const;

    // print data

    // void print_classes( std::string pref, std::map< std::string, int >& classes );
    void print_classes( std::string pref, const std::map< int, std::string >& classes ) const;

    void print_classes( std::string pref, const std::map< std::string, std::pair< int, int > >& classes ) const;

    void print_classes(
        std::string pref, const std::shared_ptr< std::map< std::string, std::pair< int, int > > >& classes ) const;

    void print_classes( std::string pref, const std::vector< std::string >& classes ) const;

    void print_detector( std::string pref, const detector_t& det ) const;

    void print_server_data( void ) const;
};

}  // namespace smap

#endif  // SMAP_CORE__PERCEPTION_SERVER_HPP_
