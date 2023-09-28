
// ROS
#include <rclcpp/rclcpp.hpp>

// #include <visualization_msgs/msg/marker.hpp>

#include <std_msgs/msg/string.hpp>

// #include <string>

// #include "smap_object_estimator/object_estimator.hpp"

// SMAP
#include "map_exporter/map_exporter.hpp"
#include "object_estimator/object_estimator.hpp"
#include "perception_server/perception_server.hpp"
#include "smap_interfaces/msg/smap_data.hpp"
#include "thing/thing.hpp"
#include "topo_map/topo_map.hpp"

// Node responsible to manage all the services
/* TODO Parameter POS_RATE
        Position acquisition rate
*/

// Node

using namespace std::chrono_literals;

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );
    std::shared_ptr< smap::topo_map > _topo_map_node                   = std::make_shared< smap::topo_map >();
    std::shared_ptr< smap::perception_server > _perception_server_node = std::make_shared< smap::perception_server >();
    std::shared_ptr< smap::map_exporter > _map_exporter_node           = std::make_shared< smap::map_exporter >();

    _topo_map_node->define_reg_classes( _perception_server_node->classes );
    _topo_map_node->define_reg_detectors( _perception_server_node->detectors );

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node( _topo_map_node );

    executor.add_node( _perception_server_node );

    executor.add_node( _map_exporter_node );

    while( rclcpp::ok() )
    {
        try
        {
            executor.spin_once();
        }
        catch( std::exception& e )
        {
            std::cout << "Exception!" << std::endl;
            std::cout << e.what() << std::endl;
        }
    }
    rclcpp::shutdown();
    return 0;
}
