#ifndef SMAP_CORE__TOPO_MAP_HPP_
#define SMAP_CORE__TOPO_MAP_HPP_

// STL

// BOOST
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/histogram/axis.hpp>
#include <boost/histogram/make_histogram.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>

// ROS
#include "../include/smap_core/visibility_control.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// SMAP
#include "../include/smap_core/aux_functions.hpp"
#include "../include/smap_core/macros.hpp"
#include "../perception_server/perception_server.hpp"
#include "../thing/thing.hpp"
#include "graph.hpp"
#include "label_writers.hpp"
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/smap_observation.hpp"
#include "topo_marker.hpp"

/* XXX current_vertex and previous_vertex can be a problem in the future!
       Check based on the location of the robot when loading
*/
/* TODO Parameter D_START
        Distance considered to be in the initialization. While in the range D_START from the first position
        acquired, vertex creation will gonna be blocked
*/

namespace smap
{
class topo_map : public rclcpp::Node

{
  private:

    friend class boost::serialization::access;

    // Variables
    graph_t graph;

    size_t previous_idx = -1;
    size_t current_idx  = -1;

    size_t v_index      = 1;

    // Timers
    rclcpp::TimerBase::SharedPtr marker_timer =
        this->create_wall_timer( std::chrono::milliseconds( 500 ), std::bind( &topo_map::timer_callback, this ) );

    rclcpp::TimerBase::SharedPtr monitor_timer = this->create_wall_timer(
        std::chrono::milliseconds( 2000 / 2 ), std::bind( &topo_map::monitor_callback, this ) );

    // Subscriptions
    rclcpp::Subscription< geometry_msgs::msg::PoseStamped >::SharedPtr pose_sub =
        this->create_subscription< geometry_msgs::msg::PoseStamped >(
            std::string( this->get_namespace() ) + std::string( "/sampler/pose" ), 10,
            std::bind( &topo_map::pose_callback, this, std::placeholders::_1 ) );
    rclcpp::Subscription< smap_interfaces::msg::SmapObservation >::SharedPtr object_sub =
        this->create_subscription< smap_interfaces::msg::SmapObservation >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/observations" ), 10,
            std::bind( &topo_map::observation_callback, this, std::placeholders::_1 ) );

    // Publishers
    rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr publisher_marker_vertex =
        this->create_publisher< visualization_msgs::msg::MarkerArray >(
            std::string( this->get_namespace() ) + std::string( "/topo_map/markers" ), 10 );

    // Threads
    // std::thread marker_thread;
    topo_marker markers;

    // topo_marker markers;

    // Internal Functions
    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar& graph;
        ar& v_index;
    }

    inline void timer_callback( void )
    {
        RCLCPP_DEBUG( this->get_logger(), "timer_callback" );
        this->markers.async_publish_markers();
    }

    inline void monitor_callback( void )
    {
        RCLCPP_DEBUG( this->get_logger(), "monitor_callback" );
        this->markers.async_update_markers( this->graph );
    }

    inline void pose_callback( const geometry_msgs::msg::PoseStamped::SharedPtr pose )
    {
        RCLCPP_DEBUG( this->get_logger(), "pose_callback" );
        this->add_vertex( pose->pose.position, true );
        // if( boost::num_vertices( this->graph ) == 1 )
        // {
        //     for( auto e: boost::make_iterator_range( boost::vertices( this->graph ) ) )
        //     {
        //         thing t;
        //         t.pos.x         = 3;
        //         t.pos.y         = 3;
        //         t.pos.z         = 3;
        //         t.aabb.first.x  = 2.5;
        //         t.aabb.first.y  = 2.5;
        //         t.aabb.first.z  = 2.5;
        //         t.aabb.second.x = 3.5;
        //         t.aabb.second.y = 3.5;
        //         t.aabb.second.z = 3.5;
        //         t.type          = semantic_type_t::OBJECT;
        //         this->graph[ e ].related_things.push_back( t );
        //     }
        // }
    }

    void observation_callback( const smap_interfaces::msg::SmapObservation::SharedPtr observation );

    inline void add_vertex( const geometry_msgs::msg::Point& pos, bool strong_vertex )
    {

        this->add_vertex( pos, this->current_idx, this->previous_idx, strong_vertex );
    }

    void add_vertex( const geometry_msgs::msg::Point& pos, size_t& current, size_t& previous, bool strong_vertex );

    void add_object( const smap_interfaces::msg::SmapObservation::SharedPtr observation, detector_t& det );

    inline size_t _add_vertex( size_t v_index, const geometry_msgs::msg::Point& pos, bool strong_vertex )
    {
        vertex_data_t vert { v_index, pos, thing( &( this->reg_classes ) ), std::list< smap::thing >(), strong_vertex };
        size_t ret = boost::add_vertex( vert, this->graph );
        // publish_vertex = true;
        // if( strong_vertex )
        //     this->markers.append_vertex(
        //         pos, v_index, vert.this_thing.get_label() + std::string( "_" ) + std::to_string( v_index ) );

        RCLCPP_INFO( this->get_logger(), "Vertex added (%li) [%4.1f,%4.1f,%4.1f]", this->v_index, pos.x, pos.y, pos.z );
        return ret;
    }

    inline size_t _get_vertex( const size_t& v_index )
    {
        vertex_data_t prop;
        prop.index = v_index;
        if( auto v = this->graph.vertex_by_property( prop ) ) return *v;
        // Case vertex don't exists
        RCLCPP_DEBUG( this->get_logger(), "Vertex not found!" );
        return -1;
    }

    bool add_edge( const size_t& previous, const size_t& current );

    inline void get_vertex( const size_t& v_index, vertex_data_t& vertex )
    {
        vertex_data_t prop;
        prop.index = v_index;
        if( auto v = this->graph.vertex_by_property( prop ) ) vertex = this->graph[ *v ];
        else
        {
            // Case vertex don't exists
            RCLCPP_DEBUG( this->get_logger(), "Vertex not found!" );
            vertex.index = -1;
        }
    }

    size_t get_closest_vertex( const geometry_msgs::msg::Point pos, double& min );

    inline static double _calc_distance( const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2 )
    {
        return sqrt( pow( p1.x - p2.x, 2 ) + pow( p1.y - p2.y, 2 ) + pow( p1.z - p2.z, 2 ) );
    }

  public:

    topo_map( void ) : Node( "topo_map" )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing topo_map" );
        if( NEW_EDGE_FACTOR > 1 )
        {
            RCLCPP_ERROR( this->get_logger(), "NEW_EDGE_FACTOR must be >= 1" );
            rclcpp::exceptions::throw_from_rcl_error(  // TODO error handling
                RCL_RET_INVALID_ARGUMENT, "NEW_EDGE_FACTOR must be <= 1", nullptr, nullptr );
        }

        this->markers.set_com( this->publisher_marker_vertex, this->get_clock() );
    }

    ~topo_map( void ) { this->export_graph( "TopoGraph" ); }

    inline void print_vertex( const std::string& prefix, const size_t& idx )
    {
        vertex_data_t prop;
        prop.index = idx;
        if( auto v = this->graph.vertex_by_property( prop ) )
        {
            RCLCPP_DEBUG(
                this->get_logger(), "%s (%i) [%4.1f,%4.1f,%4.1f]\n", prefix.c_str(), (int) this->graph[ *v ].index,
                this->graph[ *v ].pos.x, this->graph[ *v ].pos.y, this->graph[ *v ].pos.z );
        }
    };

    std::map< std::string, std::pair< int, int > >* reg_classes;

    std::vector< detector_t >* reg_detectors;

    inline void define_reg_classes( std::map< std::string, std::pair< int, int > >& classes )
    {
        RCLCPP_DEBUG( this->get_logger(), "Defining reg_classes" );
        this->reg_classes = &classes;
    }

    inline void define_reg_detectors( std::vector< detector_t >& dets )
    {
        RCLCPP_DEBUG( this->get_logger(), "Defining reg_detectors" );
        this->reg_detectors = &dets;
    }

    inline void print_graph( void )
    {
        boost::print_graph( this->graph, boost::get( &vertex_data_t::index, this->graph ) );
    }

    inline void export_graph( void ) { this->export_graph( "topo_map" ); }

    void export_graph( const std::string& f_name )
    {
        std::ofstream dotfile( OUTPUT_PATH + f_name + ".dot" );

        write_graphviz(
            dotfile, this->graph,
            make_vertex_label_writer(
                boost::get( &vertex_data_t::this_thing, this->graph ), boost::get( &vertex_data_t::pos, this->graph ) ),
            make_cost_label_writer(
                boost::get( &edge_data_t::distance, this->graph ),
                boost::get( &edge_data_t::modifier, this->graph ) ) );

        if( std::system( ( "dot -Tpng " + OUTPUT_PATH + f_name + ".dot > " + OUTPUT_PATH + f_name + ".png" ).c_str() )
            == 0 )
        {
            if( std::system( ( "rm " + OUTPUT_PATH + f_name + ".dot" ).c_str() ) == 0 )
            {
                // https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
                printf(
                    "\033[42m[Export Complete]\033[0m png file successfully "
                    "exported to: %s.png\n",
                    std::string( OUTPUT_PATH + f_name ).c_str() );
            }
        }
    }

    inline double compute_direction( geometry_msgs::msg::Point& p1, geometry_msgs::msg::Pose& p2 )
    {
        // Result defined in [-pi,pi]
        tf2::Quaternion q( p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w );
        tf2::Matrix3x3 m( q );
        double row, pitch, yaw;
        m.getEulerYPR( yaw, pitch, row );
        double ret = atan2( p2.position.y - p1.y, p2.position.x - p1.x ) - yaw;
        return atan2( sin( ret ), cos( ret ) );
    }
};
}  // namespace smap

BOOST_CLASS_VERSION( vertex_data_t, 0 ) BOOST_CLASS_VERSION( edge_data_t, 0 ) BOOST_CLASS_VERSION( smap::topo_map, 0 )

#endif  // SMAP_CORE__TOPO_MAP_HPP_
