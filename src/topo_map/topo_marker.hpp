#ifndef SMAP_CORE__TOPO_MARKER_HPP_
#define SMAP_CORE__TOPO_MARKER_HPP_

// ROS
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// SMAP
#include "../include/smap_core/aux_functions.hpp"
#include "../include/smap_core/interface_templates.hpp"
#include "../include/smap_core/macros.hpp"
#include "graph.hpp"

// TODO: change the strong_marker policy

namespace smap
{

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

    visualization_msgs::msg::Marker vertex, edge, label, histogram, aabb, aabb_label;
    // std::vector< marker_data_t > vertex_data;

    std::mutex mutex;

    rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr pub;
    rclcpp::Clock::SharedPtr clock;

    visualization_msgs::msg::MarkerArray array;

    std::future< void > fut_pub, fut_upd;

    std::vector< triangles_t > triangles_base;

    std_msgs::msg::ColorRGBA histogram_color_picker( double min, double max, double value );

    bool com_set = false;

  public:

    topo_marker( void );

    inline void set_com(
        rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr pub, rclcpp::Clock::SharedPtr clock )
    {
        this->pub     = pub;
        this->clock   = clock;
        this->com_set = true;
    }

    inline void publish_markers( void )
    {
        if( !this->com_set ) return;
        const std::lock_guard< std::mutex > lock( this->mutex );
        this->pub->publish( this->array );
    }

    inline void async_publish_markers( void )
    {
        this->fut_pub = std::async( std::launch::async, &topo_marker::publish_markers, this );
    }

    void update_markers( const graph_t& graph, std::mutex& map_mutex, const double confidence_threshold );

    inline void async_update_markers( const graph_t& graph, std::mutex& map_mutex, const double confidence_threshold )
    {
        this->fut_upd = std::async(
            std::launch::async, &topo_marker::update_markers, this, std::ref( graph ), std::ref( map_mutex ),
            confidence_threshold );
    }
};
}  // namespace smap

#endif  // SMAP_CORE__TOPO_MARKER_HPP_
