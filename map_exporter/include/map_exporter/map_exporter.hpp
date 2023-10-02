#ifndef SMAP_CORE__MAP_EXPORTER_HPP_
#define SMAP_CORE__MAP_EXPORTER_HPP_

// C
#include <cstdio>

// STL
#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <vector>
// BOOST

// ROS
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// SMAP
#include "smap_base/graph.hpp"

namespace smap
{

class map_exporter : public rclcpp::Node

{
  private:

    rclcpp::Subscription< nav_msgs::msg::OccupancyGrid >::SharedPtr og_sub =
        this->create_subscription< nav_msgs::msg::OccupancyGrid >(
            std::string( "/map" ), 5, std::bind( &map_exporter::og_callback, this, std::placeholders::_1 ) );

    rclcpp::Publisher< nav_msgs::msg::OccupancyGrid >::SharedPtr og_pub =
        this->create_publisher< nav_msgs::msg::OccupancyGrid >(
            std::string( this->get_namespace() ) + std::string( "/map_exporter/occupancy_grid" ), 5 );

    rclcpp::Publisher< nav_msgs::msg::OccupancyGrid >::SharedPtr og_center_pub =
        this->create_publisher< nav_msgs::msg::OccupancyGrid >(
            std::string( this->get_namespace() ) + std::string( "/map_exporter/occupancy_grid_center" ), 5 );

    std::shared_ptr< std::map< std::string, std::pair< int, int > > > reg_classes;

    std::vector< signed char > reference_map;
    std::vector< signed char > map;
    nav_msgs::msg::MapMetaData metadata;

    std::mutex m_gmap;

    const std::string path_name = std::string( "/workspace/src/smap/smap_core/maps/g_smap/" );

  public:

    map_exporter( void ) : Node( "map_exporter" )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing map_exporter" );
        //
    }

    ~map_exporter( void )
    {
        // if( this->reg_classes == nullptr ) return;
        RCLCPP_WARN( this->get_logger(), "Saving Maps" );

        // og_sub = this->create_subscription< nav_msgs::msg::OccupancyGrid >(
        //     std::string( "/map" ), 2, std::bind( &map_exporter::export_maps, this, std::placeholders::_1 ) );

        // while( !this->map_exported )
        // {
        //     RCLCPP_WARN( this->get_logger(), "EXPORTING MAP" );
        //     this->ending = true;
        //     std::this_thread::yield();
        // }
        // RCLCPP_WARN( this->get_logger(), "TOPO-" );
        // this->export_graph( "TopoGraph" );

        //
    }

    void export_map( graph_t& graph, double confidence_threshold );

    template< typename Functor >
    void bresenham_low( const Functor out, const int x0, const int y0, const int x1, const int y1 ) const
    {
        int dx = x1 - x0;
        int dy = y1 - y0;
        int yi = 1;
        if( dy < 0 )
        {
            yi = -1;
            dy = -dy;
        }
        int D = ( 2 * dy ) - dx;
        int y = y0;
        for( int x = x0; x <= x1; x++ )
        {
            // printf( "[%i,%i]\n", x, y );
            out( x, y );
            if( D > 0 )
            {
                y = y + yi;
                D = D + ( 2 * ( dy - dx ) );
            }
            else { D = D + 2 * dy; }
        }
    }

    template< typename Functor >
    void bresenham_high( const Functor out, const int x0, const int y0, const int x1, const int y1 ) const
    {
        int dx = x1 - x0;
        int dy = y1 - y0;
        int xi = 1;
        if( dx < 0 )
        {
            xi = -1;
            dx = -dx;
        }
        int D = ( 2 * dx ) - dy;
        int x = x0;
        for( int y = y0; y <= y1; y++ )
        {
            // printf( "[%i,%i]\n", x, y );
            out( x, y );
            if( D > 0 )
            {
                x = x + xi;
                D = D + ( 2 * ( dx - dy ) );
            }
            else { D = D + 2 * dx; }
        }
    }

    template< typename Functor >
    void bresenham( const Functor out, const int x0, const int y0, const int x1, const int y1 ) const
    {
        if( abs( y1 - y0 ) < abs( x1 - x0 ) )
        {
            //
            if( x0 > x1 ) bresenham_low( out, x1, y1, x0, y0 );
            else bresenham_low( out, x0, y0, x1, y1 );
        }
        else
        {
            if( y0 > y1 ) bresenham_high( out, x1, y1, x0, y0 );
            else bresenham_high( out, x0, y0, x1, y1 );
        }
    }

    void draw_rectangle( std::vector< signed char >& map, const std::array< std::array< double, 2 >, 4 >& corners );

    void mark_rectangle( std::vector< signed char >& map, std::array< std::array< double, 2 >, 4 > corners );

    void og_callback( const nav_msgs::msg::OccupancyGrid::SharedPtr og );

    void save_map(
        const std::string& file_name, std::vector< signed char >& map,
        nav_msgs::msg::OccupancyGrid& occupancy_grid_msg );

    void save_map( const std::string& file_name, std::vector< signed char >& map );

    inline void define_reg_classes( std::shared_ptr< std::map< std::string, std::pair< int, int > > > classes )
    {
        RCLCPP_DEBUG( this->get_logger(), "Defining reg_classes" );
        this->reg_classes = classes;
    }
};
}  // namespace smap

#endif  // SMAP_CORE__MAP_EXPORTER_HPP_
