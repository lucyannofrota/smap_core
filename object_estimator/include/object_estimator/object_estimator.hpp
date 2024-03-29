#ifndef SMAP_CORE__OBJECT_ESTIMATOR_HPP_
#define SMAP_CORE__OBJECT_ESTIMATOR_HPP_

// STL
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdlib.h>
#include <thread>

// ROS
#include "smap_base/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/io/pcd_io.h>    // TODO: Remove
#include <pcl/point_types.h>  // TODO: Remove
#include <pcl_conversions/pcl_conversions.h>

// SMAP
#include "macros.hpp"
#include "pcl_processing/pcl_processing.hpp"
#include "smap_base/counters.hpp"
#include "smap_base/macros.hpp"
#include "smap_interfaces/msg/bounding_box2_d.hpp"
#include "smap_interfaces/msg/depth_map.hpp"
#include "smap_interfaces/msg/smap_detections.hpp"
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/smap_observation.hpp"

// using namespace std::chrono_literals;

// TODO: Reject unregistered requests

inline bool timeout(
    const std::tuple< std::shared_ptr< std::future< void > >, std::chrono::_V2::system_clock::time_point, float >&
        element )
{
    if( std::get< 0 >( element ) )
    {
        bool ret = ( std::get< 0 >( element )->wait_for( std::chrono::milliseconds( 0 ) ) == std::future_status::ready )
                || ( std::chrono::duration_cast< std::chrono::milliseconds >(
                         std::chrono::high_resolution_clock::now() - std::get< 1 >( element ) ) )
                           .count()
                       >= std::get< 2 >( element );
        if( ret ) std::get< 0 >( element ).get();  // Finalize the thread
        return ret;
    }
    return true;
}

class thread_queue :
    public std::vector<
        std::tuple< std::shared_ptr< std::future< void > >, std::chrono::_V2::system_clock::time_point, float > >
{
    using element_type =
        std::tuple< std::shared_ptr< std::future< void > >, std::chrono::_V2::system_clock::time_point, float >;

  private:

    const size_t _max_size;

  public:

    thread_queue( size_t max_size ) : std::vector< element_type >(), _max_size( max_size ) {}

    inline bool available( void )
    {  // Return true if the thread queue is not full
        std::vector< element_type >::erase(
            std::remove_if( std::vector< element_type >::begin(), std::vector< element_type >::end(), timeout ),
            std::vector< element_type >::end() );

        return std::vector< element_type >::size() < this->_max_size;
    }

    inline float __compute_timeout( const size_t active_threads ) const
    {
        const float tau = 1.5205;
        // double timeout  = (double) this->_max_size - 1;
        // if( active_threads > 0 ) timeout = (double) this->_max_size * ( exp( -( active_threads * 1.0 ) / ( tau ) ) );
        const double ms_timeout = (double) 1.0 * ( exp( -( active_threads * 1.0 ) / ( tau ) ) );
        return ms_timeout * 1000;
    }

    inline bool push_back( const std::shared_ptr< std::future< void > >& element )
    {

        // Add a new thread if possible
        if( thread_queue::available() )
        {
            std::vector< element_type >::push_back( element_type(
                { element, std::chrono::high_resolution_clock::now(),
                  this->__compute_timeout( std::vector< element_type >::size() ) } ) );
            return true;
        }
        else { return false; }
    }
};

namespace smap
{

using cloud_point_t = pcl::PointXYZRGB;
using cloud_t       = pcl::PointCloud< cloud_point_t >;

class object_estimator : public rclcpp::Node
{
  private:

    const size_t max_threads = 1;  // Should be grater then 1 because of the function "__compute_timeout"

    rclcpp::Subscription< smap_interfaces::msg::SmapDetections >::SharedPtr smap_detections_sub =
        this->create_subscription< smap_interfaces::msg::SmapDetections >(
            std::string( this->get_namespace() ) + std::string( "/perception/predictions" ), 2,
            std::bind( &smap::object_estimator::detections_callback, this, std::placeholders::_1 ) );

    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr pcl_pub =
        this->create_publisher< sensor_msgs::msg::PointCloud2 >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/pcl" ), 2 );
    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr object_pcl_pub =
        this->create_publisher< sensor_msgs::msg::PointCloud2 >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/object_pcl" ), 2 );
    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr object_bb_pub =
        this->create_publisher< visualization_msgs::msg::Marker >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/object_bb" ), 2 );
    rclcpp::Publisher< smap_interfaces::msg::SmapObservation >::SharedPtr object_pub =
        this->create_publisher< smap_interfaces::msg::SmapObservation >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/observations" ), 2 );

    rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr occlusion_boxes_pub =
        this->create_publisher< visualization_msgs::msg::MarkerArray >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/occlusionBoxes" ), 2 );

    rclcpp::Publisher< smap_interfaces::msg::DepthMap >::SharedPtr depth_map_pub =
        this->create_publisher< smap_interfaces::msg::DepthMap >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/depth_map" ), 2 );

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker box_marker;

    smap_interfaces::msg::DepthMap occ_map;

    std::mutex pcl_pub_mutex, object_bb_pub_mutex, object_pub_mutex, object_pcl_pub_mutex;

    std::shared_ptr< thread_queue > thread_ctl = std::make_shared< thread_queue >( thread_queue( this->max_threads ) );

    count_time tim_object_estimation_thread {
        std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_object_estimation_thread.txt" ) },
        tim_box_filter { std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_box_filter.txt" ) },
        tim_roi_filter { std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_roi_filter.txt" ) },
        tim_voxelization {
            std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_voxelization.txt" ) },
        tim_sof { std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_sof.txt" ) },
        tim_euclidean_clustering {
            std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_euclidean_clustering.txt" ) },
        tim_estimate_confidence {
            std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_estimate_confidence.txt" ) },
        tim_transform { std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_transform.txt" ) },
        tim_3D_AABB { std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_3D_AABB.txt" ) },
        tim_depth_map_thread {
            std::string( "/workspace/src/smap/smap_core/timers/object_estimator/tim_depth_map_thread.txt" ) };

    count_points box_pc { std::string( "/workspace/src/smap/smap_core/vals/object_estimator/box_filter.txt" ) },
        roi_pc { std::string( "/workspace/src/smap/smap_core/vals/object_estimator/roi_filter.txt" ) },
        voxelization_pc { std::string( "/workspace/src/smap/smap_core/vals/object_estimator/voxelization.txt" ) },
        sof_pc { std::string( "/workspace/src/smap/smap_core/vals/object_estimator/sof_filter.txt" ) },
        clustering_pc { std::string( "/workspace/src/smap/smap_core/vals/object_estimator/clustering_filter.txt" ) };
    // TODO: COMPLETE EXPORTS

  public:

    // TODO: move to private
    std::shared_ptr< std::pair< float, float > > pcl_lims = std::make_shared< std::pair< float, float > >(
        DEFAULT_OBJECT_MIN_DIST, DEFAULT_OBJECT_MAX_DIST );  // TODO: Create parameter

    float leaf_size        = 0.03f;                          // 0.00 <= leaf_size <= 0.05 | 0.03

    int mean_k             = 8;
    float mu               = 0.3f;

    float ClusterTolerance = 0.065f;
    int minClusterSize     = 50;
    int maxClusterSize     = 10000;

    bool roi_filt = true, voxelization = true, sof = true, euclidean_clust = true, pcl_lock = false;

    // Constructor/Destructor
    inline object_estimator() : Node( "object_estimator" )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing object_estimator" );
        if( ( pcl_lims->second - pcl_lims->first ) < OBJECT_SIZE_LIM_CONF )
        {
            RCLCPP_ERROR(
                this->get_logger(), "pcl_lims difference [%4.2] cannot be smaller than OBJECT_SIZE_LIM_CONF [%4.2].",
                ( pcl_lims->second - pcl_lims->first ), OBJECT_SIZE_LIM_CONF );
            this->~object_estimator();
        }

        auto param_desc        = rcl_interfaces::msg::ParameterDescriptor {};
        param_desc.description = "Max occlusion cell volume";
        this->declare_parameter( "Max_Occlusion_Cell_Volume", DEFAULT_MAX_OCCLUSION_CELL_VOLUME, param_desc );
        param_desc.description = "Max occlusion cell length in each dimension";
        this->declare_parameter(
            "Max_Occlusion_Cell_Volume_Factor", DEFAULT_MAX_OCCLUSION_CELL_VOLUME_FACTOR, param_desc );
        param_desc.description = "Minimum confidence for an object to be considered";
        this->declare_parameter( "Minimum_Object_Confidence", DEFAULT_CONFIDENCE_OBJECT_VALID, param_desc );

        this->box_marker.header.frame_id    = "map";
        this->box_marker.header.stamp       = this->get_clock()->now();
        this->box_marker.type               = visualization_msgs::msg::Marker::CUBE;
        this->box_marker.action             = visualization_msgs::msg::Marker::ADD;
        this->box_marker.pose.orientation.x = 0;
        this->box_marker.pose.orientation.y = 0;
        this->box_marker.pose.orientation.z = 0;
        this->box_marker.pose.orientation.w = 1;
        this->box_marker.color.b            = 0;
        this->box_marker.color.g            = 1;
        this->box_marker.color.r            = 0;
        this->box_marker.color.a            = 0.2;
        this->box_marker.ns                 = "occlusion box";
        this->box_marker.lifetime.sec       = 2;
        this->box_marker.lifetime.nanosec   = 500 * 1000 * 1000;

        this->occ_map.map.resize( DEPTH_MAP_ROWS * DEPTH_MAP_COLS * DEPTH_MAP_FIELDS );
    }

    inline ~object_estimator() {}

    inline void publish_bb( int32_t id, smap_interfaces::msg::SmapObject& obj )
    {
        visualization_msgs::msg::Marker bbx_marker;
        bbx_marker.id                 = id;
        bbx_marker.header.frame_id    = "map";
        bbx_marker.header.stamp       = this->get_clock()->now();
        bbx_marker.type               = visualization_msgs::msg::Marker::CUBE;
        bbx_marker.action             = visualization_msgs::msg::Marker::ADD;
        bbx_marker.pose.position      = obj.pose.pose.position;
        bbx_marker.pose.orientation.x = 0;
        bbx_marker.pose.orientation.y = 0;
        bbx_marker.pose.orientation.z = 0;
        bbx_marker.pose.orientation.w = 1;
        bbx_marker.scale.x            = abs( obj.aabb.max.point.x - obj.aabb.min.point.x );
        bbx_marker.scale.y            = abs( obj.aabb.max.point.y - obj.aabb.min.point.y );
        bbx_marker.scale.z            = abs( obj.aabb.max.point.z - obj.aabb.min.point.z );
        bbx_marker.color.b            = 0;
        bbx_marker.color.g            = 0;
        bbx_marker.color.r            = 255;
        bbx_marker.color.a            = 0.5;
        bbx_marker.lifetime.sec       = 2;
        bbx_marker.lifetime.nanosec   = 500 * 1000 * 1000;
        this->object_bb_pub->publish( bbx_marker );
    }

    void object_estimation_thread(
        const pcl::shared_ptr< cloud_t > point_cloud,
        const std::shared_ptr< geometry_msgs::msg::TransformStamped > transform,
        const std::shared_ptr< geometry_msgs::msg::PoseStamped > pose,
        const smap_interfaces::msg::SmapObject::SharedPtr obj );

    void detections_callback( const smap_interfaces::msg::SmapDetections::SharedPtr input_msg );

    inline void on_process( void )
    {  // Pooling
    }

    void depth_map_thread(
        const std::shared_ptr< sensor_msgs::msg::PointCloud2 > ros_pcl,
        const std::shared_ptr< geometry_msgs::msg::TransformStamped > transform,
        const std::shared_ptr< geometry_msgs::msg::PoseStamped > robot_pose );

  private:
};

}  // namespace smap

// RCLCPP_COMPONENTS_REGISTER_NODE(smap::object_estimator)

#endif  // SMAP_CORE__OBJECT__ESTIMATOR_HPP_
