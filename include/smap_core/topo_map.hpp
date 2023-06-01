#ifndef SMAP_CORE__topo_map_HPP_
#define SMAP_CORE__topo_map_HPP_

#include "../include/smap_core/label_writers.hpp"
#include "../include/smap_core/macros.hpp"
#include "../include/smap_core/thing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "smap_interfaces/msg/smap_object.hpp"
#include "stdio.h"
#include "visibility_control.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <list>
#include <mutex>
#include <string>
#include <visualization_msgs/msg/marker.hpp>

// #include <boost/graph/properties.hpp>
// #include <boost/graph/named_function_params.hpp>
// #include <boost/property_map/property_map.hpp>

// #include "smap_core/msg/smap_data.hpp"

// #include "../include/smap_core/msg/smap_data.hpp"

/* XXX current_vertex and previous_vertex can be a problem in the future!
       Check based on the location of the robot when loading
*/
/* TODO Parameter D_START
        Distance considered to be in the initialization. While in the range D_START from the first position
        acquired, vertex creation will gonna be blocked
*/

struct point_t
{
    double x;
    double y;
    double z;
};

struct vertex_data_t

{
    size_t index = (size_t) -1;
    geometry_msgs::msg::Point pos;
    smap::thing this_thing;
    std::list< smap::thing > related_things;

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

    visualization_msgs::msg::Marker vertex;
    visualization_msgs::msg::Marker edge;
    visualization_msgs::msg::Marker label;
    std::vector< std::tuple< size_t, geometry_msgs::msg::Point, std::string > > vertex_label;

    std::mutex mutex;

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

    inline void append_vertex( const geometry_msgs::msg::Point& pos, const size_t& id, const std::string& label )
    {
        const std::lock_guard< std::mutex > lock( this->mutex );
        // static size_t marker_idx = 0;
        this->_append_vertex( pos );
        this->_append_vertex_label( pos, label, id );
        // marker_idx++;
    }

    inline void append_edge( const geometry_msgs::msg::Point& pos1, const geometry_msgs::msg::Point& pos2 )
    {
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

    inline void publish_markers(
        rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr& pub_vertex,
        rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr& pub_edge,
        rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr& pub_label, const rclcpp::Time& timestamp )
    {
        bool init_flag            = true;
        this->vertex.header.stamp = timestamp;
        pub_vertex->publish( this->vertex );
        this->edge.header.stamp = timestamp;
        pub_edge->publish( this->edge );
        for( auto e: this->vertex_label )
        {
            this->label.id            = std::get< 0 >( e );
            this->label.pose.position = std::get< 1 >( e );
            this->label.text          = std::get< 2 >( e );
            this->label.header.stamp  = timestamp;
            pub_label->publish( this->label );
        }
        if( init_flag )
        {
            this->vertex.action = visualization_msgs::msg::Marker::MODIFY;
            this->edge.action   = visualization_msgs::msg::Marker::MODIFY;
            init_flag           = false;
        }
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
    rclcpp::TimerBase::SharedPtr timer =
        this->create_wall_timer( std::chrono::milliseconds( 500 ), std::bind( &topo_map::timer_callback, this ) );

    // Subscriptions
    rclcpp::Subscription< geometry_msgs::msg::PoseStamped >::SharedPtr pose_sub =
        this->create_subscription< geometry_msgs::msg::PoseStamped >(
            std::string( this->get_namespace() ) + std::string( "/sampler/pose" ), 10,
            std::bind( &topo_map::pose_callback, this, std::placeholders::_1 ) );
    rclcpp::Subscription< smap_interfaces::msg::SmapObject >::SharedPtr object_sub =
        this->create_subscription< smap_interfaces::msg::SmapObject >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/objects" ), 10,
            std::bind( &topo_map::object_callback, this, std::placeholders::_1 ) );

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

    topo_marker markers;

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
        // TODO: Implement in a separate thread
        this->markers.publish_markers(
            this->publisher_marker_vertex, this->publisher_marker_edge, this->publisher_marker_label,
            this->get_clock()->now() );
    }

    inline void pose_callback( const geometry_msgs::msg::PoseStamped::SharedPtr pose )
    {
        this->add_vertex( pose->pose.position );

        std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
    }

    inline void object_callback( const smap_interfaces::msg::SmapObject::SharedPtr object )
    {
        // (void) object;
        this->add_object( object->obj_pose.pose );
    }

    inline size_t _add_vertex( size_t v_index, const geometry_msgs::msg::Point& pos )
    {
        vertex_data_t vert { v_index, pos, thing(), std::list< smap::thing >() };
        size_t ret = boost::add_vertex( vert, this->graph );
        // publish_vertex = true;
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
        RCLCPP_INFO( this->get_logger(), "Initializing topol_map" );
        if( NEW_EDGE_FACTOR > 1 )
        {
            RCLCPP_ERROR( this->get_logger(), "NEW_EDGE_FACTOR must be >= 1" );
            rclcpp::exceptions::throw_from_rcl_error(  // TODO error handling
                RCL_RET_INVALID_ARGUMENT, "NEW_EDGE_FACTOR must be <= 1", nullptr, nullptr );
        }

        geometry_msgs::msg::Point point;
        // point.x              = 1;
        // point.y              = 2;
        // point.z              = 1;
        auto conv_point = [ & ]( point_t p ) {
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            return point;
        };
        (void) conv_point;
        // this->add_vertex( conv_point( { 0, 1, 0 } ) );
        // this->add_vertex( conv_point( { 1, 0, 0 } ) );
        // this->add_vertex( conv_point( { 0, 0, 0 } ) );
        // this->add_vertex( conv_point( { 0, -1, 0 } ) );
        // this->add_vertex( conv_point( { 1, -1, 0 } ) );
        // this->add_vertex( conv_point( { 1, 0, 0 } ) );
        // this->add_vertex( conv_point( { 2, -1, 0 } ) );
        // this->add_vertex( conv_point( { 1, -1, 0 } ) );
        // this->add_vertex( conv_point( { 0, -1, 0 } ) );
        // this->add_vertex( conv_point( { 0, 0, 0 } ) );
        // this->add_vertex( conv_point( { 0, 1, 0 } ) );
        // this->add_vertex( conv_point( { 0, 2, 0 } ) );

        // this->print_graph();
        // this->export_graph();
        // this->~topo_map();
    }

    ~topo_map( void ) { this->export_graph( "TopoGraph" ); }

    inline void add_vertex( const geometry_msgs::msg::Point& pos )
    {
        this->add_vertex( pos, this->current_idx, this->previous_idx );
    }

    void add_vertex( const geometry_msgs::msg::Point& pos, size_t& current, size_t& previous );

    void add_object( const geometry_msgs::msg::Pose pose );

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
};
}  // namespace smap

BOOST_CLASS_VERSION( vertex_data_t, 0 ) BOOST_CLASS_VERSION( edge_data_t, 0 ) BOOST_CLASS_VERSION( smap::topo_map, 0 )

#endif  // SMAP_CORE__topo_map_HPP_
