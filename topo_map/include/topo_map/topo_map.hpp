#ifndef SMAP_CORE__TOPO_MAP_HPP_
#define SMAP_CORE__TOPO_MAP_HPP_

// STL
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

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
#include "smap_base/visibility_control.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// SMAP
#include "label_writers.hpp"
#include "smap_base/graph.hpp"
// #include "perception_server/perception_server.hpp"
#include "smap_base/aux_functions.hpp"
#include "smap_base/counters.hpp"
// #include "smap_base/macros.hpp"
#include "map_exporter/map_exporter.hpp"
#include "smap_interfaces/msg/depth_map.hpp"
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/smap_observation.hpp"
#include "thing/thing.hpp"
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
    // std::shared_ptr< graph_t > graph_ = std::make_shared< graph_t >;
    graph_t graph;

    size_t previous_idx = -1;
    size_t current_idx  = -1;

    size_t v_index      = 1;

    int thing_id_count  = 0;

    // Callback group
    rclcpp::CallbackGroup::SharedPtr map_cb_group =
        this->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );

    // Timers
    rclcpp::TimerBase::SharedPtr marker_timer;
    // = this->create_wall_timer( std::chrono::milliseconds( 500 ), std::bind( &topo_map::timer_callback, this ) );

    rclcpp::TimerBase::SharedPtr monitor_timer;
    // = this->create_wall_timer(std::chrono::milliseconds( 2000 / 2 ), std::bind( &topo_map::monitor_callback, this )

    rclcpp::TimerBase::SharedPtr map_cleaning_timer;
    // );

    // Subscriptions
    rclcpp::SubscriptionOptions sub_options;

    rclcpp::Subscription< geometry_msgs::msg::PoseStamped >::SharedPtr pose_sub;
    rclcpp::Subscription< smap_interfaces::msg::SmapObservation >::SharedPtr object_sub;
    rclcpp::Subscription< smap_interfaces::msg::DepthMap >::SharedPtr depth_map_sub;
    // rclcpp::Publisher< std_msgs::msg::Float32MultiArray >::SharedPtr depth_map_pub =
    // this->create_publisher< std_msgs::msg::Float32MultiArray >(
    //     std::string( this->get_namespace() ) + std::string( "/object_estimator/depth_map" ), 10 );

    // Publishers
    rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr publisher_marker_vertex =
        this->create_publisher< visualization_msgs::msg::MarkerArray >(
            std::string( this->get_namespace() ) + std::string( "/topo_map/markers" ), 2 );

    rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr selected_panels_pub =
        this->create_publisher< visualization_msgs::msg::MarkerArray >(
            std::string( this->get_namespace() ) + std::string( "/topo_map/selected_panels" ), 2 );

    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr selected_face_pub =
        this->create_publisher< visualization_msgs::msg::Marker >(
            std::string( this->get_namespace() ) + std::string( "/topo_map/selected_face" ), 2 );

    // rclcpp::Subscription< nav_msgs::msg::OccupancyGrid >::SharedPtr og_sub =
    //     this->create_subscription< nav_msgs::msg::OccupancyGrid >(
    //         std::string( "/map" ), 2, std::bind( &topo_map::export_maps, this, std::placeholders::_1 ) );

    // Threads
    // std::thread marker_thread;
    topo_marker markers;

    visualization_msgs::msg::MarkerArray panels_maker_array;

    visualization_msgs::msg::Marker panels_maker, face_marker;

    // Mutex
    std::mutex map_mutex;

    bool map_exported = false, ending = false;

    // topo_marker markers;

    double confidence_threshold;

    count_time tim_observation_callback {
        std::string( "/workspace/src/smap/smap_core/timers/topo_map/tim_observation_callback.txt" ) },
        tim_depth_map_callback {
            std::string( "/workspace/src/smap/smap_core/timers/topo_map/tim_depth_map_callback.txt" ) },
        tim_cleaning_callback {
            std::string( "/workspace/src/smap/smap_core/timers/topo_map/tim_cleaning_callback.txt" ) };

    // Internal Functions
    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar & graph;
        ar & v_index;
    }

    inline void timer_callback( void )
    {
        if( this->reg_classes == nullptr || this->reg_detectors == nullptr ) return;
        if( boost::num_vertices( this->graph ) == 0 ) return;
        RCLCPP_DEBUG( this->get_logger(), "timer_callback" );
        if( this->publisher_marker_vertex->get_subscription_count() > 0 ) this->markers.async_publish_markers();
    }

    inline void monitor_callback( void )
    {
        if( this->reg_classes == nullptr || this->reg_detectors == nullptr ) return;
        if( boost::num_vertices( this->graph ) == 0 ) return;
        RCLCPP_DEBUG( this->get_logger(), "monitor_callback" );
        if( this->publisher_marker_vertex->get_subscription_count() > 0 )
            this->markers.async_update_markers(
                this->graph, this->map_mutex, this->get_parameter( "Confidence_Object_Valid" ).as_double() );
    }

    inline void pose_callback( const geometry_msgs::msg::PoseStamped::SharedPtr pose )
    {
        const std::lock_guard< std::mutex > lock( this->map_mutex );
        if( this->reg_classes == nullptr || this->reg_detectors == nullptr ) return;
        RCLCPP_DEBUG( this->get_logger(), "pose_callback" );
        this->add_vertex( pose->pose.position, true );
    }

    void observation_callback( const smap_interfaces::msg::SmapObservation::SharedPtr observation );

    void depth_map_callback( const smap_interfaces::msg::DepthMap::SharedPtr msg );

    void cleaning_map_callback( void );

    inline void add_vertex( const geometry_msgs::msg::Point& pos, bool strong_vertex )
    {
        this->add_vertex( pos, this->current_idx, this->previous_idx, strong_vertex );
    }

    void add_vertex( const geometry_msgs::msg::Point& pos, size_t& current, size_t& previous, bool strong_vertex );

    thing& add_object(
        const smap_interfaces::msg::SmapObservation::SharedPtr observation, detector_t& det, bool& is_valid );

    inline size_t _add_vertex( size_t v_index, const geometry_msgs::msg::Point& pos, bool strong_vertex )
    {
        vertex_data_t vert {
            v_index, pos, thing( this->reg_classes, ++this->thing_id_count, this->get_logger() ),
            std::list< smap::thing >(), strong_vertex };
        size_t ret = boost::add_vertex( vert, this->graph );

        RCLCPP_DEBUG(
            this->get_logger(), "Vertex added (%li) [%4.1f,%4.1f,%4.1f]", this->v_index, pos.x, pos.y, pos.z );
        return ret;
    }

    inline size_t _get_vertex( const size_t& v_index )
    {
        vertex_data_t prop;
        prop.index = v_index;
        if( const auto& v = this->graph.vertex_by_property( prop ) ) return *v;
        // Case vertex don't exists
        RCLCPP_DEBUG( this->get_logger(), "Vertex not found!" );
        return -1;
    }

    bool add_edge( const size_t& previous, const size_t& current );

    inline void get_vertex( const size_t& v_index, vertex_data_t& vertex )
    {
        vertex_data_t prop;
        prop.index = v_index;
        if( const auto& v = this->graph.vertex_by_property( prop ) ) vertex = this->graph[ *v ];
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

    inline void get_adjacent_vertices(
        std::vector< size_t >& idxs_checked, const geometry_msgs::msg::Point& pos, const size_t n_layers )
    {
        RCLCPP_DEBUG( this->get_logger(), "Get all adjacent vertices %i layers deep", (int) n_layers );
        std::vector< size_t > idxs_checking, idxs_to_check;  // Using iterator graph idxs (not v_indexes)
        double min_distance;
        idxs_checking.push_back( this->_get_vertex( this->get_closest_vertex( pos, min_distance ) ) );
        for( size_t i = 0; i <= n_layers; i++ )
        {
            idxs_to_check.clear();
            for( const auto& checking: idxs_checking )
            {
                idxs_checked.push_back( checking );
                for( const auto to_check: boost::make_iterator_range(
                         boost::adjacent_vertices( boost::vertex( checking, this->graph ), this->graph ) ) )
                {
                    if( std::find( idxs_checked.begin(), idxs_checked.end(), to_check ) != idxs_checked.end() )
                        continue;
                    else idxs_to_check.push_back( to_check );
                }
            }
            idxs_checking = idxs_to_check;
        }
        RCLCPP_DEBUG( this->get_logger(), "1.1 idxs_checked size: %i", (int) idxs_checked.size() );
    }

    // Transform inline
    inline void filter_vertices(
        std::vector< size_t >& idxs_checked, std::vector< std::pair< size_t, std::vector< thing* > > >& candidates,
        const uint8_t& module_id, const uint8_t& label, const geometry_msgs::msg::Point& obj_pos,
        const geometry_msgs::msg::Pose& robot_pose )
    {
        RCLCPP_DEBUG( this->get_logger(), "2. Filter possible vertices" );
        RCLCPP_DEBUG( this->get_logger(), "2.1.1 idxs_checked.size(): %i", (int) idxs_checked.size() );
        std::vector< thing* > local_candidates;
        for( const auto& checking: idxs_checked )
        {
            // Check list of objects inside each vertex
            local_candidates.clear();
            bool has_candidate = false;

            RCLCPP_DEBUG(
                this->get_logger(), "2.1.2 this->graph[ *checking ].related_things.size(): %i",
                (int) this->graph[ checking ].related_things.size() );
            for( auto& obj: this->graph[ checking ].related_things )
            {
                // Check labels
                if( !obj.label_is_equal( module_id, label ) )
                {
                    RCLCPP_DEBUG( this->get_logger(), "2.2.1 Check label: FAIL" );
                    continue;
                }
                else RCLCPP_DEBUG( this->get_logger(), "2.2.1 Check label: PASS" );

                // Check active cone
                if( abs( rad2deg( this->compute_object_direction( obj_pos, robot_pose ) ) )
                    > this->get_parameter( "Active_FOV" ).as_double() )
                {

                    RCLCPP_DEBUG(
                        this->get_logger(), "2.2.2 Check active cone [val: %7.2f|rad2deg: %7.2f|abs: %7.2f]: FAIL",
                        this->compute_object_direction( obj_pos, robot_pose ),
                        rad2deg( this->compute_object_direction( obj_pos, robot_pose ) ),
                        abs( rad2deg( this->compute_object_direction( obj_pos, robot_pose ) ) ) );
                    // RCLCPP_DEBUG(
                    //     this->get_logger(), "2.2 Check active cone [val: %7.2f|rad2deg: %7.2f|abs: %7.2f]: FAIL",
                    //     this->compute_object_direction( obj_pos, robot_pose
                    //     ), rad2deg( this->compute_object_direction(
                    //         obj_pos, robot_pose ) ),
                    //     abs( rad2deg( this->compute_object_direction(
                    //         obj_pos, robot_pose ) ) ) );
                    continue;
                }
                else RCLCPP_DEBUG( this->get_logger(), "2.2.2 Check active cone: PASS" );

                // Check position
                if( !( ( ( obj_pos.x > obj.aabb.first.x
                                           - this->get_parameter( "Object_Error_Distance" ).as_double()
                                                 * this->get_parameter( "Object_Tracking_Factor" ).as_double() )
                         && ( obj_pos.x < obj.aabb.second.x
                                              + this->get_parameter( "Object_Error_Distance" ).as_double()
                                                    * this->get_parameter( "Object_Tracking_Factor" ).as_double() ) )
                       && ( ( obj_pos.y > obj.aabb.first.y
                                              - this->get_parameter( "Object_Error_Distance" ).as_double()
                                                    * this->get_parameter( "Object_Tracking_Factor" ).as_double() )
                            && ( obj_pos.y < obj.aabb.second.y
                                                 + this->get_parameter( "Object_Error_Distance" ).as_double()
                                                       * this->get_parameter( "Object_Tracking_Factor" ).as_double() ) )
                       && ( ( obj_pos.z > obj.aabb.first.z
                                              - this->get_parameter( "Object_Error_Distance" ).as_double()
                                                    * this->get_parameter( "Object_Tracking_Factor" ).as_double() )
                            && ( obj_pos.z
                                 < obj.aabb.second.z
                                       + this->get_parameter( "Object_Error_Distance" ).as_double()
                                             * this->get_parameter( "Object_Tracking_Factor" ).as_double() ) ) )
                    && ( this->_calc_distance( obj_pos, obj.pos )
                         > this->get_parameter( "Object_Error_Distance" ).as_double() ) )
                {
                    RCLCPP_DEBUG( this->get_logger(), "2.3 Check position: FAIL" );
                    continue;
                }
                else RCLCPP_DEBUG( this->get_logger(), "2.3 Check position: PASS" );

                local_candidates.push_back( &obj );
                has_candidate = true;
            }
            // Update candidates
            RCLCPP_DEBUG( this->get_logger(), "2.2.4 Update candidate list" );
            if( has_candidate )
                candidates.push_back( std::pair< size_t, std::vector< thing* > >( checking, local_candidates ) );
        }
        // RCLCPP_DEBUG( this->get_logger(), "2| candidates size: %i", (int) candidates.size() );
        RCLCPP_DEBUG( this->get_logger(), "2.3 candidate list size: %i", (int) candidates.size() );
    }

    inline bool is_detector_valid( const uint8_t& module_id, std::vector< detector_t >::iterator& det ) const
    {
        det            = this->reg_detectors->begin();
        bool det_found = false;
        for( ; det != this->reg_detectors->end(); det++ )
        {
            if( det->id == module_id )
            {
                det_found = true;
                break;
            }
        }
        return det_found;
    }

    inline void vertex_transaction(
        const smap_interfaces::msg::SmapObservation::SharedPtr observation,
        const std::vector< std::pair< size_t, std::vector< thing* > > >& candidates,
        std::vector< detector_t >::iterator& det, std::pair< size_t, thing* >& closest, bool& valid_transaction )
    {
        double min_distance = 0;
        RCLCPP_DEBUG( this->get_logger(), "3. Update vertex" );
        if( candidates.size() == 0 )
        {
            RCLCPP_DEBUG( this->get_logger(), "3.1.1 Object add" );
            closest.second = &( this->add_object( observation, *det, valid_transaction ) );
            if( !valid_transaction ) return;
        }
        else
        {
            // Select the closest object
            RCLCPP_DEBUG( this->get_logger(), "3.1.2.1 Select the closest object" );

            for( const auto& c: candidates )
            {
                for( const auto& lc: c.second )
                {
                    if( closest.second == nullptr )
                    {
                        closest.second = lc;
                        closest.first  = c.first;
                        min_distance   = this->_calc_distance( observation->object.pose.pose.position, lc->pos );
                        continue;
                    }

                    if( this->_calc_distance( observation->object.pose.pose.position, lc->pos ) < min_distance )
                    {
                        closest.second = lc;
                        closest.first  = c.first;
                        min_distance   = this->_calc_distance( observation->object.pose.pose.position, lc->pos );
                    }
                }
            }
            RCLCPP_DEBUG( this->get_logger(), "Object update" );
            closest.second->update(
                observation->object.probability_distribution, observation->object.pose.pose.position,
                std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point >(
                    observation->object.aabb.min.point, observation->object.aabb.max.point ),
                observation->object.aabb.confidence, min_distance, (double) observation->direction, *det );
        }
    }

    inline void occlusion_transaction(
        thing& object, const double& distance_camera_to_object,
        const std::shared_ptr< smap_interfaces::msg::DepthMap >& msg, const std::string& format, double decay )
    {
        RCLCPP_DEBUG( this->get_logger(), format, decay );
        object.observations->register_obs(
            distance_camera_to_object, this->compute_corner_direction( msg->camera_pose, object.pos ), false );
        object.decay_confidence(
            this->get_parameter( "Object_Prob_Decay" ).as_double(), distance_camera_to_object, decay );
    }

    inline void object_combination(
        const std::vector< std::pair< size_t, std::vector< thing* > > >& candidates,
        std::pair< size_t, thing* >& closest, std::pair< size_t, thing* >& closest_valid )
    {
        // TODO: debug pos_confidence
        double min_distance_valid = std::numeric_limits< double >::infinity();
        for( const auto& c: candidates )
        {
            for( const auto& lc: c.second )
            {
                if( ( this->_calc_distance( closest.second->pos, lc->pos ) < min_distance_valid )
                    && ( lc->id != closest.second->id ) )
                {
                    closest_valid.second = lc;
                    closest_valid.first  = c.first;
                    min_distance_valid   = this->_calc_distance( closest.second->pos, lc->pos );
                }
            }
        }
        if( ( min_distance_valid < this->get_parameter( "Object_Error_Distance" ).as_double() * 0.8 )
            && closest_valid.second->id != closest.second->id )
        {
            RCLCPP_DEBUG( this->get_logger(), "Object Combination" );
            int pred_obj_idx =
                ( closest.second->id < closest_valid.second->id ? closest.second->id : closest_valid.second->id );
            if( closest.first == closest_valid.first )
            {
                // Same vertex
                this->graph[ closest_valid.first ].related_things.remove_if(
                    [ &closest_valid ]( thing& th ) { return th.id == closest_valid.second->id; } );
                closest.second->id = pred_obj_idx;
            }
            else
            {
                // Different vertex
                for( auto& th: this->graph[ closest_valid.first ].related_things )
                    if( th.id == closest_valid.second->id )
                    {
                        th    = *closest.second;
                        th.id = pred_obj_idx;
                        this->graph[ closest.first ].related_things.remove_if(
                            [ &closest ]( thing& th ) { return th.id == closest.second->id; } );
                        closest.second = &th;
                        closest.first  = closest_valid.first;
                        break;
                    }
            }
        }
    }

    inline void object_vert_move( std::vector< size_t >& idxs_checked, std::pair< size_t, thing* >& closest )
    {
        std::vector< std::pair< size_t, double > > distances;
        std::pair< size_t, double > min_vertex = std::pair< size_t, double >( 0, std::numeric_limits< double >::max() );
        for( const auto& idx: idxs_checked )
        {
            if( ( this->_calc_distance( closest.second->pos, this->graph[ idx ].pos ) < min_vertex.second )
                && ( idx != closest.first ) )
            {
                min_vertex.first  = idx;
                min_vertex.second = this->_calc_distance( closest.second->pos, this->graph[ idx ].pos );
            }
        }
        if( min_vertex.second < this->_calc_distance( closest.second->pos, this->graph[ closest.first ].pos ) )
        {
            RCLCPP_DEBUG( this->get_logger(), "Object move" );
            thing aux   = *closest.second;
            size_t size = this->graph[ closest.first ].related_things.size();
            this->graph[ closest.first ].related_things.remove_if(
                [ &closest ]( thing& th ) { return th.id == closest.second->id; } );
            if( this->graph[ closest.first ].related_things.size() < size )
                this->graph[ min_vertex.first ].related_things.push_back( aux );
            // closest became invalid at this point!
        }
    }

  public:

    topo_map( void ) : Node( "topo_map" )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing topo_map" );
        // Declare parameters
        auto param_desc        = rcl_interfaces::msg::ParameterDescriptor {};
        param_desc.description = "Default path to save maps";
        this->declare_parameter( "Map_Path", DEFAULT_OUTPUT_PATH, param_desc );
        param_desc.description = "Minimum distance to create a new vertex";
        this->declare_parameter( "Vertex_Distance", DEFAULT_VERTEX_DISTANCE, param_desc );
        param_desc.description = "Distance factor to create a new edge";
        this->declare_parameter( "Edge_Factor", DEFAULT_NEW_EDGE_FACTOR, param_desc );
        param_desc.description = "FOV of the robot";
        this->declare_parameter( "Active_FOV", DEFAULT_ACTIVE_FOV_H, param_desc );
        param_desc.description = "Max positional error allowed to objects to be considered the same instance";
        this->declare_parameter( "Object_Error_Distance", DEFAULT_OBJECT_ERROR_DISTANCE, param_desc );
        param_desc.description = "Distance factor considered in moving objects";
        this->declare_parameter( "Object_Tracking_Factor", DEFAULT_OBJECT_TRACKING_FACTOR, param_desc );
        // param_desc.description = "Tracking tolerance of moving objects";
        // this->declare_parameter( "Object_Tracking_Tolerance", OBJECT_TRACKING_TOLERANCE, param_desc );
        param_desc.description = "Distance factor applied to objects to be considered as an occlusion";
        this->declare_parameter(
            "Occlusion_Object_Distance_Tolerance_Factor", DEFAULT_OCCLUSION_OBJECT_DISTANCE_TOLERANCE_FACTOR,
            param_desc );
        param_desc.description = "Max distance allowed for an observation to be considered as an occlusion";
        this->declare_parameter(
            "Occlusion_Object_Distance_Tolerance_Max", DEFAULT_OCCLUSION_OBJECT_DISTANCE_TOLERANCE_MAX, param_desc );
        param_desc.description =
            "The max percentage of cells to be considerate as an occlusion [Occlusion_Max_Percentage*100 = %]";
        this->declare_parameter( "Occlusion_Max_Percentage", DEFAULT_OCCLUSION_MAX_PERCENTAGE, param_desc );
        param_desc.description = "The probability decay factor for subtractive observations";
        this->declare_parameter( "Object_Prob_Decay", DEFAULT_OBJECT_PROB_DECAY, param_desc );
        param_desc.description = "The minimum value of combined confidence for an object to be considered as valid";
        this->declare_parameter( "Confidence_Object_Valid", DEFAULT_CONFIDENCE_OBJECT_VALID, param_desc );

        // Topics Setup
        this->sub_options.callback_group = this->map_cb_group;
        this->pose_sub                   = this->create_subscription< geometry_msgs::msg::PoseStamped >(
            std::string( this->get_namespace() ) + std::string( "/sampler/pose" ), 2,
            std::bind( &topo_map::pose_callback, this, std::placeholders::_1 ), this->sub_options );
        this->object_sub = this->create_subscription< smap_interfaces::msg::SmapObservation >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/observations" ), 2,
            std::bind( &topo_map::observation_callback, this, std::placeholders::_1 ), this->sub_options );
        this->depth_map_sub = this->create_subscription< smap_interfaces::msg::DepthMap >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/depth_map" ), 2,
            std::bind( &topo_map::depth_map_callback, this, std::placeholders::_1 ), this->sub_options );

        this->marker_timer =
            this->create_wall_timer( std::chrono::milliseconds( 250 ), std::bind( &topo_map::timer_callback, this ) );

        this->monitor_timer = this->create_wall_timer(
            std::chrono::milliseconds( 1000 ), std::bind( &topo_map::monitor_callback, this ) );

        this->map_cleaning_timer = this->create_wall_timer(
            std::chrono::seconds( 15 ), std::bind( &topo_map::cleaning_map_callback, this ), this->map_cb_group );

        if( this->get_parameter( "Edge_Factor" ).as_double() > 1 )
        {
            RCLCPP_ERROR( this->get_logger(), "Edge_Factor must be >= 1" );
            rclcpp::exceptions::throw_from_rcl_error(  // TODO error handling
                RCL_RET_INVALID_ARGUMENT, "Edge_Factor must be <= 1", nullptr, nullptr );
        }

        this->markers.set_com( this->publisher_marker_vertex, this->get_clock() );

        this->panels_maker.header.frame_id    = "map";
        this->panels_maker.header.stamp       = this->get_clock()->now();
        this->panels_maker.type               = visualization_msgs::msg::Marker::CUBE;
        this->panels_maker.action             = visualization_msgs::msg::Marker::ADD;
        this->panels_maker.pose.orientation.x = 0;
        this->panels_maker.pose.orientation.y = 0;
        this->panels_maker.pose.orientation.z = 0;
        this->panels_maker.pose.orientation.w = 1;
        this->panels_maker.color.b            = 1;
        this->panels_maker.color.g            = 0;
        this->panels_maker.color.r            = 0;
        this->panels_maker.color.a            = 1;
        this->panels_maker.ns                 = "selected_panels";
        this->panels_maker.lifetime.sec       = 1;
        this->panels_maker.lifetime.nanosec   = 500 * 1000 * 1000;

        this->face_marker.header.frame_id     = "map";
        this->face_marker.header.stamp        = this->get_clock()->now();
        this->face_marker.type                = visualization_msgs::msg::Marker::POINTS;
        this->face_marker.action              = visualization_msgs::msg::Marker::ADD;
        this->face_marker.pose.orientation.x  = 0;
        this->face_marker.pose.orientation.y  = 0;
        this->face_marker.pose.orientation.z  = 0;
        this->face_marker.pose.orientation.w  = 1;
        this->face_marker.color.b             = 1;
        this->face_marker.color.g             = 1;
        this->face_marker.color.r             = 0;
        this->face_marker.color.a             = 1;
        this->face_marker.ns                  = "selected_face";
        this->face_marker.scale.x             = 0.01;
        this->face_marker.scale.y             = 0.01;
        this->face_marker.scale.z             = 0.01;
        this->face_marker.lifetime.sec        = 1;
        this->face_marker.lifetime.nanosec    = 500 * 1000 * 1000;
    }

    std::shared_ptr< smap::map_exporter > map_exporter = nullptr;

    ~topo_map( void )
    {
        RCLCPP_WARN( this->get_logger(), "topo_map destructor" );
        this->map_exporter->export_map( this->graph, confidence_threshold );

        // og_sub = this->create_subscription< nav_msgs::msg::OccupancyGrid >(
        //     std::string( "/map" ), 2, std::bind( &topo_map::export_maps, this, std::placeholders::_1 ) );

        // while( !this->map_exported )
        // {
        //     RCLCPP_WARN( this->get_logger(), "EXPORTING MAP" );
        //     this->ending = true;
        //     std::this_thread::yield();
        // }
        // RCLCPP_WARN( this->get_logger(), "TOPO-" );
        this->export_graph( "TopoGraph" );

        //
    }

    // inline void export_maps( const nav_msgs::msg::OccupancyGrid::SharedPtr og )
    // {
    //     (void) og;
    //     RCLCPP_WARN( this->get_logger(), "OG CB" );
    //     if( !this->ending ) return;
    //     RCLCPP_WARN( this->get_logger(), "OG CB END" );
    //     this->map_exported = true;
    // }

    inline void print_vertex( const std::string& prefix, const size_t& idx )
    {
        vertex_data_t prop;
        prop.index = idx;
        if( const auto v = this->graph.vertex_by_property( prop ) )
        {
            RCLCPP_DEBUG(
                this->get_logger(), "%s (%i) [%4.1f,%4.1f,%4.1f]\n", prefix.c_str(), (int) this->graph[ *v ].index,
                this->graph[ *v ].pos.x, this->graph[ *v ].pos.y, this->graph[ *v ].pos.z );
        }
    };

    std::shared_ptr< std::map< std::string, std::pair< int, int > > > reg_classes = nullptr;

    std::shared_ptr< std::vector< detector_t > > reg_detectors                    = nullptr;

    inline void define_reg_classes( std::shared_ptr< std::map< std::string, std::pair< int, int > > >& classes )
    {
        RCLCPP_DEBUG( this->get_logger(), "Defining reg_classes" );
        this->reg_classes = classes;
    }

    inline void define_reg_detectors( std::shared_ptr< std::vector< detector_t > >& dets )
    {
        RCLCPP_DEBUG( this->get_logger(), "Defining reg_detectors" );
        this->reg_detectors = dets;
    }

    inline void print_graph( void )
    {
        boost::print_graph( this->graph, boost::get( &vertex_data_t::index, this->graph ) );
    }

    inline void export_graph( void ) { this->export_graph( "topo_map" ); }

    void export_graph( const std::string& f_name )
    {
        std::ofstream dotfile( this->get_parameter( "Map_Path" ).as_string() + f_name + ".dot" );

        write_graphviz(
            dotfile, this->graph,
            make_vertex_label_writer(
                boost::get( &vertex_data_t::this_thing, this->graph ), boost::get( &vertex_data_t::pos, this->graph ) ),
            make_cost_label_writer(
                boost::get( &edge_data_t::distance, this->graph ),
                boost::get( &edge_data_t::modifier, this->graph ) ) );

        if( std::system( ( "dot -Tpng " + this->get_parameter( "Map_Path" ).as_string() + f_name + ".dot > "
                           + this->get_parameter( "Map_Path" ).as_string() + f_name + ".png" )
                             .c_str() )
            == 0 )
        {
            if( std::system( ( "rm " + this->get_parameter( "Map_Path" ).as_string() + f_name + ".dot" ).c_str() )
                == 0 )
            {
                // https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
                printf(
                    "\033[42m[Export Complete]\033[0m png file successfully "
                    "exported to: %s.png\n",
                    std::string( this->get_parameter( "Map_Path" ).as_string() + f_name ).c_str() );
            }
        }
    }

    inline double compute_object_direction( const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Pose& p2 )
    {
        // Result defined in [-pi,pi]
        tf2::Quaternion q( p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w );
        tf2::Matrix3x3 m( q );
        double row, pitch, yaw;
        m.getEulerYPR( yaw, pitch, row );
        double ret = atan2( p1.y - p2.position.y, p1.x - p2.position.x ) - yaw;
        return atan2( sin( ret ), cos( ret ) );
    }

    inline double compute_corner_direction( const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Point& p2 )
    {
        // Result defined in [-pi,pi]
        tf2::Quaternion q( p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w );
        tf2::Matrix3x3 m( q );
        double row, pitch, yaw;
        m.getEulerYPR( yaw, pitch, row );
        double ret = atan2( p1.position.y - p2.y, p1.position.x - p2.x ) - yaw;
        return atan2( sin( ret ), cos( ret ) );
    }

    inline std::array< long, 4 > find_coplanar_cube_point_idxs(
        const std::array< geometry_msgs::msg::Point, 8 >& AABB,
        const std::array< std::pair< long, double >, 8 > idx_points ) const
    {
        // Only works for AABB's
        std::array< long, 4 > ret;
        // P1
        ret[ 0 ] = 0;
        // P2
        ret[ 1 ] = 1;
        // P3
        bool P3   = false;
        uint8_t c = 0;
        if( AABB[ idx_points[ ret[ 0 ] ].first ].x == AABB[ idx_points[ ret[ 1 ] ].first ].x )
        {
            for( long idx = 2; idx < 8; idx++ )
                if( AABB[ idx_points[ idx ].first ].x == AABB[ idx_points[ 0 ].first ].x )
                {
                    P3       = true;
                    c        = 0;
                    ret[ 2 ] = idx;
                    break;
                }
        }
        if( ( AABB[ idx_points[ ret[ 0 ] ].first ].y == AABB[ idx_points[ ret[ 1 ] ].first ].y ) && !P3 )
        {
            for( long idx = 2; idx < 8; idx++ )
                if( AABB[ idx_points[ idx ].first ].y == AABB[ idx_points[ 0 ].first ].y )
                {
                    P3       = true;
                    c        = 1;
                    ret[ 2 ] = idx;
                    break;
                }
        }
        if( ( AABB[ idx_points[ ret[ 0 ] ].first ].z == AABB[ idx_points[ ret[ 1 ] ].first ].z ) && !P3 )
        {
            for( long idx = 2; idx < 8; idx++ )
                if( AABB[ idx_points[ idx ].first ].z == AABB[ idx_points[ 0 ].first ].z )
                {
                    P3       = true;
                    c        = 2;
                    ret[ 2 ] = idx;
                    break;
                }
        }
        // P4
        switch( c )
        {
        case 0:
            for( long idx = 2; idx < 8; idx++ )
            {
                if( idx == ret[ 2 ] ) continue;
                if( AABB[ idx_points[ idx ].first ].x == AABB[ idx_points[ 0 ].first ].x )
                {
                    ret[ 3 ] = idx;
                    break;
                }
            }
            break;
        case 1:
            for( long idx = 2; idx < 8; idx++ )
            {
                if( idx == ret[ 2 ] ) continue;
                if( AABB[ idx_points[ idx ].first ].y == AABB[ idx_points[ 0 ].first ].y )
                {
                    ret[ 3 ] = idx;
                    break;
                }
            }
            break;
        case 2:
            for( long idx = 2; idx < 8; idx++ )
            {
                if( idx == ret[ 2 ] ) continue;
                if( AABB[ idx_points[ idx ].first ].z == AABB[ idx_points[ 0 ].first ].z )
                {
                    ret[ 3 ] = idx;
                    break;
                }
            }
            break;
        }

        return ret;
    }
};
}  // namespace smap

BOOST_CLASS_VERSION( vertex_data_t, 0 ) BOOST_CLASS_VERSION( edge_data_t, 0 ) BOOST_CLASS_VERSION( smap::topo_map, 0 )

#endif  // SMAP_CORE__TOPO_MAP_HPP_
