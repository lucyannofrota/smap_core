
// ROS
#include <rclcpp/rclcpp.hpp>

// #include <visualization_msgs/msg/marker.hpp>

#include <std_msgs/msg/string.hpp>

// #include <string>

// #include "smap_object_estimator/object_estimator.hpp"

// SMAP
#include "../include/smap_core/macros.hpp"
#include "object_estimator/object_estimator.hpp"
#include "perception_server/perception_server.hpp"
#include "smap_interfaces/msg/smap_data.hpp"
#include "thing/thing.hpp"
#include "topo_map/topo_map.hpp"

#define FROM_FRAME std::string( "map" )
#define TO_FRAME std::string( "base_link" )

// Node responsible to manage all the services
/* TODO Parameter POS_RATE
        Position acquisition rate
*/

// Node

using namespace std::chrono_literals;

namespace smap
{

class smap_node : public rclcpp::Node
{
  private:

    // Logger
    rclcpp::Logger logger = this->get_logger();

    // Subscriptions
    // rclcpp::Subscription<smap_interfaces::msg::SmapData>::SharedPtr SmapData_sub =
    // this->create_subscription<smap_interfaces::msg::SmapData>(
    //   std::string(this->get_namespace())+std::string("/sampler/data"),10,std::bind(
    //     &smap::smap_node::SmapData_callback, this, std::placeholders::_1)
    // );

  public:

    // Constructor/Destructor
    smap_node() : Node( "smap_core" ) { RCLCPP_INFO( this->get_logger(), "Initializing smap_core" ); }

    ~smap_node() {}

    // void SmapData_callback(const smap_interfaces::msg::SmapData::SharedPtr input_msg){
    //   // printf("SmapData_callback\n");
    //   // this->topological_map->add_vertex(input_msg->stamped_pose.pose.position);
    // }

    void on_process( void )  // Pooling
    {
        // RCLCPP_DEBUG(this->get_logger(),"Process smap::smap_node");
    }

  private:

  public:

    // Map
    // std::shared_ptr<smap::topological_map> topological_map;
};

}  // namespace smap

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );
    rclcpp::NodeOptions options;
    printf( "main\n" );            // TODO: remove
    std::shared_ptr< smap::smap_node > _smap_node = std::make_shared< smap::smap_node >();
    printf( "_smap_node\n" );      // TODO: remove
    std::shared_ptr< smap::topo_map > _topo_map_node = std::make_shared< smap::topo_map >();
    printf( "_topo_map_node\n" );  // TODO: remove
    std::shared_ptr< smap::object_estimator > _object_estimator_node =
        std::make_shared< smap::object_estimator >( options );
    std::shared_ptr< smap::perception_server > _perception_server_node =
        std::make_shared< smap::perception_server >( options );
    printf( "_perception_server_node\n" );  // TODO: remove

    _topo_map_node->define_reg_classes( _perception_server_node->classes );
    printf( "define_reg_classes\n" );       // TODO: remove
    _topo_map_node->define_reg_detectors( _perception_server_node->detectors );
    printf( "define_reg_detectors\n" );     // TODO: remove
    // _smap_node->topological_map = _topological_map_node;
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node( _smap_node );
    executor.add_node( _topo_map_node );
    executor.add_node( _object_estimator_node );
    executor.add_node( _perception_server_node );
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
    // _smap_node->create_service<std_srvs::srv::Trigger>("serv_smap", &smap::test_serv);
    while( rclcpp::ok() )
    {
        try
        {
            executor.spin_once();
            // _smap_node->on_process(); // Pooling
            // _topological_map_node->on_process(); // Pooling
        }
        catch( std::exception& e )
        {
            std::cout << "Exception!" << std::endl;
            std::cout << e.what() << std::endl;
        }
    }
    rclcpp::shutdown();

    // printf("hello world smap package\n");
    return 0;
}
