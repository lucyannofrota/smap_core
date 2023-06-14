#ifndef SMAP_CORE__TOPO_MARKER_HPP_
#define SMAP_CORE__TOPO_MARKER_HPP_

// ROS
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// SMAP
#include "../include/smap_core/interface_templates.hpp"
#include "../include/smap_core/macros.hpp"
#include "graph.hpp"

namespace smap
{

// struct marker_object_t
// {
//     geometry_msgs::msg::Point pos;
//     std::string label;
//     std::vector< float > histogram;
//     std_msgs::msg::ColorRGBA color;
// };

// struct marker_data_t

// {
//     size_t id;
//     geometry_msgs::msg::Point pos;
//     std::string label;
//     std::vector< marker_object_t > objects;
// };

struct triangles_t
{
    geometry_msgs::msg::Point point;
    std_msgs::msg::ColorRGBA color;

    triangles_t()
    {
        this->color.r = 0;
        this->color.g = 0;
        this->color.b = 255;
        this->color.a = 1;
    }
};

class topo_marker

{

  private:

    visualization_msgs::msg::Marker vertex, edge, label, histogram, aabb;
    // std::vector< marker_data_t > vertex_data;

    std::mutex mutex;

    rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr pub;
    rclcpp::Clock::SharedPtr clock;

    visualization_msgs::msg::MarkerArray array;

    std::future< void > fut_pub, fut_upd;

    std::vector< triangles_t > triangles_base;

    // inline void _append_vertex( const geometry_msgs::msg::Point& pos )
    // {
    //     static int32_t idx = 0;
    //     this->vertex.id    = idx++;
    //     for( auto it = this->vertex.points.begin(); it != this->vertex.points.end(); it++ )
    //         if( ( it->x == pos.x ) && ( it->y == pos.y ) && ( it->z == pos.z ) ) return;
    //     this->vertex.points.push_back( pos );
    // }

    // inline void _append_vertex_data( const geometry_msgs::msg::Point& pos, const std::string& label, const size_t& id
    // )
    // {
    //     geometry_msgs::msg::Point pos_up;

    // pos_up.x = pos.x;
    // pos_up.y = pos.y;
    // pos_up.z = pos.z + 0.1;

    // // TODO: Remove
    // std_msgs::msg::ColorRGBA c;
    // c.r = 255;
    // c.g = 0;
    // c.b = 0;
    // c.a = 1;
    // marker_object_t obj;
    // obj.color     = c;
    // obj.pos       = pos;
    // obj.label     = label;
    // obj.histogram = std::vector< float >();

    // std::vector< marker_object_t > vobj;
    // vobj.push_back( obj );
    // // TODO: Remove

    // this->vertex_data.push_back( { id, pos_up, label, vobj } );
    // }

    // void _append_histogram( void );

    // void _gen_triangles(
    //     float r, std::vector< geometry_msgs::msg::Point >& points, std::vector< std_msgs::msg::ColorRGBA >& colors );

    std_msgs::msg::ColorRGBA histogram_color_picker( double min, double max, double value );

  public:

    topo_marker( void );

    inline void set_com(
        rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr pub, rclcpp::Clock::SharedPtr clock )
    {
        this->pub   = pub;
        this->clock = clock;
    }

    // inline void append_vertex( const geometry_msgs::msg::Point& pos, const size_t& id, const std::string& label )
    // {
    //     printf( "append_vertex\n" );
    //     const std::lock_guard< std::mutex > lock( this->mutex );
    //     // static size_t marker_idx = 0;
    //     this->_append_vertex( pos );
    //     this->_append_vertex_data( pos, label, id );
    //     // marker_idx++;
    // }

    // inline void append_edge( const geometry_msgs::msg::Point& pos1, const geometry_msgs::msg::Point& pos2 )
    // {
    //     printf( "append_edge\n" );
    //     const std::lock_guard< std::mutex > lock( this->mutex );
    //     static int32_t idx = 0;
    //     // static bool init_flag = true;
    //     this->edge.id = idx++;
    //     this->edge.points.push_back( pos1 );
    //     this->edge.points.push_back( pos2 );
    //     // if( init_flag )
    //     // {
    //     //     this->edge.points.push_back( pos1 );
    //     //     this->edge.points.push_back( pos2 );
    //     //     init_flag = false;
    //     // }
    //     // else { this->edge.points.push_back( pos2 ); }
    // }

    // inline void update_vertex_data( const std::string& label, const size_t& id )
    // {
    //     (void) label;
    //     (void) id;
    //     const std::lock_guard< std::mutex > lock( this->mutex );
    //     // TODO: implement update
    // }

    inline void publish_markers( void )
    {
        const std::lock_guard< std::mutex > lock( this->mutex );
        // this->pub->publish( this->array );
    }

    inline void async_publish_markers( void )
    {
        this->fut_pub = std::async( std::launch::async, &topo_marker::publish_markers, this );
    }

    void update_markers( const graph_t& graph );

    inline void async_update_markers( const graph_t& graph )
    {
        // (void) graph;
        // printf( "async_update_markers\n" );
        this->fut_upd = std::async( std::launch::async, &topo_marker::update_markers, this, std::ref( graph ) );
    }
};
}  // namespace smap

#endif  // SMAP_CORE__TOPO_MARKER_HPP_
