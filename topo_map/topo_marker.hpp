#ifndef SMAP_CORE__TOPO_MARKER_HPP_
#define SMAP_CORE__TOPO_MARKER_HPP_

// // STL
// #include <thread>
// #include <vector>

// ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>

// SMAP
#include "../pch/pch.hpp"

namespace smap
{
class topo_marker
{

  private:

    visualization_msgs::msg::Marker vertex, edge, label, histogram;
    std::vector< std::tuple< size_t, geometry_msgs::msg::Point, std::string > > vertex_label;

    std::mutex mutex;

    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr vertex_pub, edge_pub, label_pub;
    rclcpp::Clock::SharedPtr clock;

    std::future< void > fut;

    inline void _append_vertex( const geometry_msgs::msg::Point& pos )
    {
        static int32_t idx = 0;
        this->vertex.id    = idx++;
        for( auto it = this->vertex.points.begin(); it != this->vertex.points.end(); it++ )
            if( ( it->x == pos.x ) && ( it->y == pos.y ) && ( it->z == pos.z ) ) return;
        this->vertex.points.push_back( pos );
    }

    inline void _append_vertex_label( const geometry_msgs::msg::Point& pos, const std::string& label, const size_t& id )
    {
        geometry_msgs::msg::Point pos_up;

        pos_up.x = pos.x;
        pos_up.y = pos.y;
        pos_up.z = pos.z + 0.1;
        this->vertex_label.push_back(
            std::tuple< size_t, geometry_msgs::msg::Point, std::string >( id, pos_up, label ) );
    }

  public:

    topo_marker( void )
    {
        // Marker msg initialization
        // Vertex
        this->vertex.header.frame_id = "/map";
        this->vertex.ns              = "vertices";
        this->vertex.type            = visualization_msgs::msg::Marker::POINTS;
        this->vertex.action          = visualization_msgs::msg::Marker::ADD;
        this->vertex.scale.x         = 0.075 * 4;
        this->vertex.scale.y         = 0.075 * 4;
        this->vertex.scale.z         = 0.075 * 4;
        this->vertex.color.r         = 102.0 / ( 102.0 + 51.0 );
        this->vertex.color.g         = 51.0 / ( 102.0 + 51.0 );
        this->vertex.color.b         = 0.0;
        this->vertex.color.a         = 1.0;

        // Histogram
        this->histogram.header.frame_id = "/map";
        this->histogram.ns              = "histogram";
        this->histogram.type            = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        this->histogram.action          = visualization_msgs::msg::Marker::ADD;
        // this->vertex.scale.x         = 0.075 * 4;
        // this->vertex.scale.y         = 0.075 * 4;
        // this->vertex.scale.z         = 0.075 * 4;
        // this->vertex.color.r = 102.0 / ( 102.0 + 51.0 );
        // this->vertex.color.g = 51.0 / ( 102.0 + 51.0 );
        // this->vertex.color.b = 0.0;
        // this->vertex.color.a = 1.0;

        // Edge
        this->edge.header.frame_id    = "/map";
        this->edge.ns                 = "edges";
        this->edge.type               = visualization_msgs::msg::Marker::LINE_LIST;
        this->edge.action             = visualization_msgs::msg::Marker::ADD;
        this->edge.scale.x            = 0.007;
        this->edge.scale.y            = 0.007;
        this->edge.scale.z            = 0.007;
        this->edge.color.r            = 1.0;
        this->edge.color.g            = 0.0;
        this->edge.color.b            = 0.0;
        this->edge.pose.orientation.x = 0;
        this->edge.pose.orientation.y = 0;
        this->edge.pose.orientation.z = 0;
        this->edge.pose.orientation.w = 1.0;
        this->edge.color.a            = 1.0;

        // Label
        this->label.header.frame_id    = "/map";
        this->label.ns                 = "labels";
        this->label.type               = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        this->label.action             = visualization_msgs::msg::Marker::ADD;
        this->label.scale.x            = 0.05;
        this->label.scale.y            = 0.05;
        this->label.scale.z            = 0.05;
        this->label.color.r            = 0.0;
        this->label.color.g            = 1.0;
        this->label.color.b            = 0.0;
        this->label.pose.orientation.x = 0;
        this->label.pose.orientation.y = 0;
        this->label.pose.orientation.z = 0;
        this->label.pose.orientation.w = 1.0;
        this->label.color.a            = 1.0;
    }

    inline void set_com(
        rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr vertex_pub,
        rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr edge_pub,
        rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr label_pub, rclcpp::Clock::SharedPtr clock )
    {
        this->vertex_pub = vertex_pub;
        this->edge_pub   = edge_pub;
        this->label_pub  = label_pub;
        this->clock      = clock;
    }

    inline void append_vertex( const geometry_msgs::msg::Point& pos, const size_t& id, const std::string& label )
    {
        printf( "append_vertex\n" );
        const std::lock_guard< std::mutex > lock( this->mutex );
        // static size_t marker_idx = 0;
        this->_append_vertex( pos );
        this->_append_vertex_label( pos, label, id );
        // marker_idx++;
    }

    inline void append_edge( const geometry_msgs::msg::Point& pos1, const geometry_msgs::msg::Point& pos2 )
    {
        printf( "append_edge\n" );
        const std::lock_guard< std::mutex > lock( this->mutex );
        static int32_t idx = 0;
        // static bool init_flag = true;
        this->edge.id = idx++;
        this->edge.points.push_back( pos1 );
        this->edge.points.push_back( pos2 );
        // if( init_flag )
        // {
        //     this->edge.points.push_back( pos1 );
        //     this->edge.points.push_back( pos2 );
        //     init_flag = false;
        // }
        // else { this->edge.points.push_back( pos2 ); }
    }

    inline void update_vertex_label( const std::string& label, const size_t& id )
    {
        (void) label;
        (void) id;
        const std::lock_guard< std::mutex > lock( this->mutex );
        // TODO: implement update
    }

    void publish_markers( void );

    // {
    //     bool init_flag            = true;
    //     this->vertex.header.stamp = clock->now();
    //     vertex_pub->publish( this->vertex );
    //     this->edge.header.stamp = clock->now();
    //     edge_pub->publish( this->edge );
    //     this->label.header.stamp = clock->now();
    //     for( auto e: this->vertex_label )
    //     {
    //         this->label.id            = std::get< 0 >( e );
    //         this->label.pose.position = std::get< 1 >( e );
    //         this->label.text          = std::get< 2 >( e );
    //         label_pub->publish( this->label );
    //     }
    //     if( init_flag )
    //     {
    //         this->vertex.action = visualization_msgs::msg::Marker::MODIFY;
    //         this->edge.action   = visualization_msgs::msg::Marker::MODIFY;
    //         init_flag           = false;
    //     }
    // }

    inline void async_publish_markers( void )
    {
        this->fut = std::async( std::launch::async, &topo_marker::publish_markers, this );
    }
};
}  // namespace smap

#endif  // SMAP_CORE__TOPO_MARKER_HPP_
