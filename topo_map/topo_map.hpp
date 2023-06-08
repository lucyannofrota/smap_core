#ifndef SMAP_CORE__topo_map_HPP_
#define SMAP_CORE__topo_map_HPP_

// STL
#include "stdio.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <math.h>
#include <mutex>
#include <string>
#include <thread>

// ROS
#include "../include/smap_core/macros.hpp"
#include "../include/smap_core/thing.hpp"
#include "../include/smap_core/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>

// SMAP
#include "../perception_server/perception_server.hpp"
#include "label_writers.hpp"
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/smap_observation.hpp"

// BOOST
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>

/* XXX current_vertex and previous_vertex can be a problem in the future!
       Check based on the location of the robot when loading
*/
/* TODO Parameter D_START
        Distance considered to be in the initialization. While in the range D_START from the first position
        acquired, vertex creation will gonna be blocked
*/

struct vertex_data_t

{
    size_t index = (size_t) -1;
    geometry_msgs::msg::Point pos;
    smap::thing this_thing;
    std::list< smap::thing > related_things;

    bool strong_vertex = false;

    friend class boost::serialization::access;

    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar& index;
        ar& pos.x;
        ar& pos.y;
        ar& pos.z;
        ar& this_thing;
        ar& related_things;
    }
};

struct edge_data_t
{
    // The cost of the edge will be distance*modifier

    double distance = 0;
    double modifier = 1;

    double get_cost( void ) { return round( distance * modifier * 100 ) / 100.0; }

    friend class boost::serialization::access;

    template< class Archive >
    inline void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar& distance;
        ar& modifier;
    }
};

typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, vertex_data_t, edge_data_t > graph_t;

// traits
template<>
struct boost::graph::internal_vertex_name< vertex_data_t >
{
    // https://stackoverflow.com/questions/71488845/how-to-configure-boostgraph-to-use-my-own-stable-index-for-vertices
    struct type
    {
        using result_type = size_t;

        const result_type& operator()( const vertex_data_t& bundle ) const { return bundle.index; }
    };
};

template<>
struct boost::graph::internal_vertex_constructor< vertex_data_t >
{
    // https://stackoverflow.com/questions/71488845/how-to-configure-boostgraph-to-use-my-own-stable-index-for-vertices
    struct type
    {
      private:

        using extractor = typename internal_vertex_name< vertex_data_t >::type;
        using name_t    = std::decay_t< typename extractor::result_type >;

      public:

        using argument_type = name_t;
        using result_type   = vertex_data_t;

        result_type operator()( const name_t& index ) const
        {
            result_type ret;
            ret.index = index;
            return ret;
        }
    };
};

namespace smap
{
class topo_marker
{

  private:

    visualization_msgs::msg::Marker vertex, edge, label;
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

  protected:

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

    void set_com(
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
    }

    inline void publish_markers( void )
    {

        // TODO: should run independently
        bool init_flag            = true;
        this->vertex.header.stamp = clock->now();
        vertex_pub->publish( this->vertex );
        this->edge.header.stamp = clock->now();
        edge_pub->publish( this->edge );
        this->label.header.stamp = clock->now();
        for( auto e: this->vertex_label )
        {
            this->label.id            = std::get< 0 >( e );
            this->label.pose.position = std::get< 1 >( e );
            this->label.text          = std::get< 2 >( e );
            label_pub->publish( this->label );
        }
        if( init_flag )
        {
            this->vertex.action = visualization_msgs::msg::Marker::MODIFY;
            this->edge.action   = visualization_msgs::msg::Marker::MODIFY;
            init_flag           = false;
        }
    }

    inline void async_publish_markers( void )
    {
        this->fut = std::async( std::launch::async, &topo_marker::publish_markers, this );
    }
};

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
    rclcpp::TimerBase::SharedPtr maker_timer =
        this->create_wall_timer( std::chrono::milliseconds( 500 ), std::bind( &topo_map::timer_callback, this ) );

    rclcpp::TimerBase::SharedPtr monitor_timer =
        this->create_wall_timer( std::chrono::milliseconds( 2000 ), std::bind( &topo_map::monitor_callback, this ) );

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
    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr publisher_marker_vertex =
        this->create_publisher< visualization_msgs::msg::Marker >(
            std::string( this->get_namespace() ) + std::string( "/topo_map/markers/vertex" ), 10 );

    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr publisher_marker_label =
        this->create_publisher< visualization_msgs::msg::Marker >(
            std::string( this->get_namespace() ) + std::string( "/topo_map/markers/label" ), 10 );

    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr publisher_marker_edge =
        this->create_publisher< visualization_msgs::msg::Marker >(
            std::string( this->get_namespace() ) + std::string( "/topo_map/markers/edge" ), 10 );

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

    inline void timer_callback( void ) { this->markers.async_publish_markers(); }

    inline void monitor_callback( void )
    {
        // TODO: Implement monitor (should update markers attributes based on the strength of the vertex )
    }

    inline void pose_callback( const geometry_msgs::msg::PoseStamped::SharedPtr pose )
    {
        this->add_vertex( pose->pose.position, true );
    }

    void observation_callback( const smap_interfaces::msg::SmapObservation::SharedPtr observation );

    inline void add_vertex( const geometry_msgs::msg::Point& pos, bool strong_vertex )
    {

        this->add_vertex( pos, this->current_idx, this->previous_idx, strong_vertex );
    }

    void add_vertex( const geometry_msgs::msg::Point& pos, size_t& current, size_t& previous, bool strong_vertex );

    // void add_object( const smap_interfaces::msg::SmapObject& object );
    void add_object( const smap_interfaces::msg::SmapObservation::SharedPtr observation );

    // void add_object( const smap_interfaces::msg::SmapObject& object, double& angle ); TODO: Revert

    inline size_t _add_vertex( size_t v_index, const geometry_msgs::msg::Point& pos, bool strong_vertex )
    {
        vertex_data_t vert { v_index, pos, thing( &( this->reg_classes ) ), std::list< smap::thing >(), strong_vertex };
        size_t ret = boost::add_vertex( vert, this->graph );
        // publish_vertex = true;
        if( strong_vertex )
            this->markers.append_vertex(
                pos, v_index, vert.this_thing.get_label() + std::string( "_" ) + std::to_string( v_index ) );

        RCLCPP_INFO( this->get_logger(), "Vertex added (%li) [%4.1f,%4.1f,%4.1f]", this->v_index, pos.x, pos.y, pos.z );
        return ret;
    }

    inline size_t _get_vertex( const size_t& v_index )
    {
        vertex_data_t prop;
        prop.index = v_index;
        if( auto v = this->graph.vertex_by_property( prop ) ) return *v;
        // Case vertex don't exists
        printf( "Vertex not found!\n" );
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
            printf( "Vertex not found!\n" );
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

        this->markers.set_com(
            this->publisher_marker_vertex, this->publisher_marker_edge, this->publisher_marker_label,
            this->get_clock() );
    }

    ~topo_map( void ) { this->export_graph( "TopoGraph" ); }

    inline void print_vertex( const std::string& prefix, const size_t& idx )
    {
        vertex_data_t prop;
        prop.index = idx;
        if( auto v = this->graph.vertex_by_property( prop ) )
        {
            printf(
                "%s (%i) [%4.1f,%4.1f,%4.1f]\n", prefix.c_str(), (int) this->graph[ *v ].index, this->graph[ *v ].pos.x,
                this->graph[ *v ].pos.y, this->graph[ *v ].pos.z );
        }
    };

    std::map< std::string, std::pair< int, int > >* reg_classes;

    inline void define_reg_classes( std::map< std::string, std::pair< int, int > >& classes )
    {
        printf( "Defining reg_classes\n" );
        this->reg_classes = &classes;
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
        // printf( "R: %6.2f, P: %6.2f, Y: %6.2f\n", row, pitch, yaw );
        double ret = atan2( p2.position.y - p1.y, p2.position.x - p1.x ) - yaw;
        return atan2( sin( ret ), cos( ret ) );
    }

    inline static double rad2deg( double rad ) { return rad * ( 180 / M_PI ); }

    inline static double deg2rad( double deg ) { return deg * ( M_PI / 180 ); }
};
}  // namespace smap

BOOST_CLASS_VERSION( vertex_data_t, 0 ) BOOST_CLASS_VERSION( edge_data_t, 0 ) BOOST_CLASS_VERSION( smap::topo_map, 0 )

#endif  // SMAP_CORE__topo_map_HPP_
