#ifndef SMAP_CORE__OBJECT_ESTIMATOR_HPP_
#define SMAP_CORE__OBJECT_ESTIMATOR_HPP_

// STL
#include <chrono>
#include <memory>
#include <mutex>
#include <stdlib.h>
#include <thread>

// ROS
#include "smap_core/visibility_control.h"

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
#include "../../pcl_processing/include/pcl_processing.hpp"
#include "smap_core/count_time.hpp"
#include "smap_core/macros.hpp"
#include "smap_interfaces/msg/bounding_box2_d.hpp"
#include "smap_interfaces/msg/smap_detections.hpp"
#include "smap_interfaces/msg/smap_object.hpp"
#include "smap_interfaces/msg/smap_observation.hpp"

// // SMAP
// #include "pcl_processing/include/pcl_processing.hpp"

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
        if( ret )
        {
            // printf("timeout (%f)|%i\n",std::get<2>(element),(std::get<0>(element)->wait_for(0ms) ==
            // std::future_status::ready));
            std::get< 0 >( element ).get();  // Finalize the thread
        }
        return ret;
    }
    // printf("\tf /timeout\n");
    return true;
    // return false;
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
        const float tau = 2.5;
        double timeout  = (double) this->_max_size - 1;
        if( active_threads > 0 ) timeout = (double) this->_max_size * ( exp( -( active_threads * 1.0 ) / ( tau ) ) );
        return timeout * 100;
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

plot_vec total_thread_time;
plot_vec box_filter_plot, roi_filter_plot, voxelization_plot, sof_filter_plot, euclidean_clustering_plot,
    total_filter_plot;
plot_vec centroid_plot, transform_plot, total_estimation_plot;

namespace smap
{

using cloud_point_t = pcl::PointXYZRGB;
using cloud_t       = pcl::PointCloud< cloud_point_t >;

class object_estimator : public rclcpp::Node
{
  private:

    const size_t max_threads = 8;  // Should be grater then 1 because of the function "__compute_timeout"

    rclcpp::Subscription< smap_interfaces::msg::SmapDetections >::SharedPtr smap_detections_sub =
        this->create_subscription< smap_interfaces::msg::SmapDetections >(
            std::string( this->get_namespace() ) + std::string( "/perception/predictions" ), 10,
            std::bind( &smap::object_estimator::detections_callback, this, std::placeholders::_1 ) );

    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr object_pcl_pub =
        this->create_publisher< sensor_msgs::msg::PointCloud2 >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/object_pcl" ), 10 );
    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr object_bb_pub =
        this->create_publisher< visualization_msgs::msg::Marker >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/object_bb" ), 10 );
    rclcpp::Publisher< smap_interfaces::msg::SmapObservation >::SharedPtr object_pub =
        this->create_publisher< smap_interfaces::msg::SmapObservation >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/observations" ), 10 );

    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr transf_pcl_pub =
        this->create_publisher< sensor_msgs::msg::PointCloud2 >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/transf_pcl" ), 10 );

    rclcpp::Publisher< visualization_msgs::msg::MarkerArray >::SharedPtr occlusion_boxes_pub =
        this->create_publisher< visualization_msgs::msg::MarkerArray >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/occlusion_boxes" ), 10 );
    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr single_occlusion_b_pub =
        this->create_publisher< visualization_msgs::msg::Marker >(
            std::string( this->get_namespace() ) + std::string( "/object_estimator/single_occlusion_b" ), 10 );

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker box_marker;
    visualization_msgs::msg::Marker arrow_marker;
    const std::shared_ptr< occlusion_matrix_t > occlusion_matrix = std::make_shared< occlusion_matrix_t >();
    // rclcpp::Publisher< smap_interfaces::msg::SmapObservation >::SharedPtr object_pub =
    //     this->create_publisher< smap_interfaces::msg::SmapObservation >(
    //         std::string( this->get_namespace() ) + std::string( "/object_estimator/occlusion_map" ), 10 );
    std::mutex object_bb_pub_mutex, object_pub_mutex, object_pcl_pub_mutex;

    std::shared_ptr< thread_queue > thread_ctl = std::make_shared< thread_queue >( thread_queue( this->max_threads ) );

  public:

    // TODO: move to private
    std::shared_ptr< std::pair< float, float > > pcl_lims =
        std::make_shared< std::pair< float, float > >( 0.4, 4 );  // TODO: Create parameter

    float leaf_size        = 0.02f;                               // 0.00 <= leaf_size <= 0.05 | 0.03

    int mean_k             = 10;
    float mu               = 0.3f;

    float ClusterTolerance = 0.021f;
    int minClusterSize     = 100;
    int maxClusterSize     = 25000;

    bool roi_filt = true, voxelization = true, sof = true, euclidean_clust = true, pcl_lock = false;

    // Constructor/Destructor
    inline object_estimator() : Node( "object_estimator" )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing object_estimator" );
        // this->viewer = std::make_shared<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D
        // Viewer"));
        if( ( pcl_lims->second - pcl_lims->first ) < OBJECT_SIZE_LIM_CONF )
        {
            RCLCPP_ERROR(
                this->get_logger(), "pcl_lims difference [%4.2] cannot be smaller than OBJECT_SIZE_LIM_CONF [%4.2].",
                ( pcl_lims->second - pcl_lims->first ), OBJECT_SIZE_LIM_CONF );
            this->~object_estimator();
        }
        this->box_marker.header.frame_id      = "map";
        this->box_marker.header.stamp         = this->get_clock()->now();
        this->box_marker.type                 = visualization_msgs::msg::Marker::CUBE;
        this->box_marker.action               = visualization_msgs::msg::Marker::ADD;
        this->box_marker.pose.orientation.x   = 0;
        this->box_marker.pose.orientation.y   = 0;
        this->box_marker.pose.orientation.z   = 0;
        this->box_marker.pose.orientation.w   = 1;
        this->box_marker.color.b              = 0;
        this->box_marker.color.g              = 1;
        this->box_marker.color.r              = 0;
        this->box_marker.color.a              = 0.75;
        this->box_marker.ns                   = "occlusion box";

        this->arrow_marker.header.frame_id    = "map";
        this->arrow_marker.header.stamp       = this->get_clock()->now();
        this->arrow_marker.type               = visualization_msgs::msg::Marker::ARROW;
        this->arrow_marker.action             = visualization_msgs::msg::Marker::ADD;
        this->arrow_marker.pose.orientation.x = 0;
        this->arrow_marker.pose.orientation.y = 0;
        this->arrow_marker.pose.orientation.z = 0;
        this->arrow_marker.pose.orientation.w = 1;
        this->arrow_marker.color.b            = 0;
        this->arrow_marker.color.g            = 0;
        this->arrow_marker.color.r            = 1;
        this->arrow_marker.color.a            = 0.5;
        this->arrow_marker.ns                 = "transform";
        this->arrow_marker.scale.x            = 1;
        this->arrow_marker.scale.y            = 1;
        this->arrow_marker.scale.z            = 1;
    }

    inline object_estimator( const rclcpp::NodeOptions& options ) : Node( "object_estimator", options )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing object_estimator" );
        // this->viewer = std::make_shared<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D
        // Viewer"));
        this->box_marker.header.frame_id      = "map";
        this->box_marker.header.stamp         = this->get_clock()->now();
        this->box_marker.type                 = visualization_msgs::msg::Marker::CUBE;
        this->box_marker.action               = visualization_msgs::msg::Marker::ADD;
        this->box_marker.pose.orientation.x   = 0;
        this->box_marker.pose.orientation.y   = 0;
        this->box_marker.pose.orientation.z   = 0;
        this->box_marker.pose.orientation.w   = 1;
        this->box_marker.color.b              = 0;
        this->box_marker.color.g              = 1;
        this->box_marker.color.r              = 0;
        this->box_marker.color.a              = 0.5;
        this->box_marker.ns                   = "occlusion box";

        this->arrow_marker.header.frame_id    = "map";
        this->arrow_marker.header.stamp       = this->get_clock()->now();
        this->arrow_marker.type               = visualization_msgs::msg::Marker::ARROW;
        this->arrow_marker.action             = visualization_msgs::msg::Marker::ADD;
        this->arrow_marker.pose.orientation.x = 0;
        this->arrow_marker.pose.orientation.y = 0;
        this->arrow_marker.pose.orientation.z = 0;
        this->arrow_marker.pose.orientation.w = 1;
        this->arrow_marker.color.b            = 0;
        this->arrow_marker.color.g            = 0;
        this->arrow_marker.color.r            = 1;
        this->arrow_marker.color.a            = 0.75;
        this->arrow_marker.ns                 = "transform";
        this->arrow_marker.scale.x            = 1;
        this->arrow_marker.scale.y            = 1;
        this->arrow_marker.scale.z            = 1;
    }

    inline ~object_estimator() {}

    // bool estimate_object_3D_AABB(
    //     const pcl::shared_ptr< cloud_t >& object_cloud, smap_interfaces::msg::SmapObject& obj ) const;

    // bool estimate_confidence( const pcl::shared_ptr< cloud_t >& object_cloud, float& conf ) const;

    // void transform_object_pcl(
    //     smap_interfaces::msg::SmapObject& obj,
    //     const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform ) const;

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
        this->object_bb_pub->publish( bbx_marker );
    }

    /*inline void estimate_object_3D_OBB(const pcl::shared_ptr<cloud_t>& object_cloud,
    smap_interfaces::msg::SmapObject::SharedPtr obj){ count_time timer; pcl::MomentOfInertiaEstimation <cloud_point_t>
    feature_extractor; feature_extractor.setInputCloud (object_cloud); feature_extractor.compute (); cloud_point_t
    min_point_AABB; cloud_point_t max_point_AABB; cloud_point_t min_point_OBB; cloud_point_t max_point_OBB;
      cloud_point_t position_OBB;
      Eigen::Matrix3f rotational_matrix_OBB;
      feature_extractor.getAABB(min_point_AABB, max_point_AABB);
      obj->aabb.min.x = min_point_AABB.x; obj->aabb.min.y = min_point_AABB.y; obj->aabb.min.z = min_point_AABB.z;
      obj->aabb.max.x = max_point_AABB.x; obj->aabb.max.y = max_point_AABB.y; obj->aabb.max.z = max_point_AABB.z;

      feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

      obj->obb.keypoint_1[0] = min_point_OBB.x; obj->obb.keypoint_1[1] = min_point_OBB.y; obj->obb.keypoint_1[2] =
    min_point_OBB.z; obj->obb.keypoint_2[0] = max_point_OBB.x; obj->obb.keypoint_2[1] = max_point_OBB.y;
    obj->obb.keypoint_2[2] = max_point_OBB.z;


      Eigen::Quaternionf quat (rotational_matrix_OBB);
      cloud_point_t pos, pos0;


      pos.x = position_OBB.x;
      pos.y = position_OBB.y;
      pos.z = position_OBB.z;

      this->publish_bb(
        0, pos, min_point_OBB, max_point_OBB, quat
      );

      pos0.x = 0;
      pos0.y = 0;
      pos0.z = 0;

      this->publish_bb(
        1, pos0, min_point_OBB, max_point_OBB, quat
      );

      this->publish_bb(
        2, pos, min_point_OBB, max_point_OBB, Eigen::Quaternionf::Identity()
      );

      const char str[] = "object_parameters_estimation";
      timer.get_time(this->get_logger(), str, this->centroid_plot);
    }*/

    void object_estimation_thread(
        const pcl::shared_ptr< cloud_t >& point_cloud,
        const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform,
        const std::shared_ptr< geometry_msgs::msg::PoseStamped >& pose,
        const smap_interfaces::msg::SmapObject::SharedPtr& obj );

    void detections_callback( const smap_interfaces::msg::SmapDetections::SharedPtr input_msg );

    inline void on_process( void )
    {  // Pooling
    }

    inline void occlusion_matrix_thread(
        const std::shared_ptr< sensor_msgs::msg::PointCloud2 >& ros_pcl,
        const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform )
    {
        count_time timer;

        // TODO: make mutually exclusive
        printf( "occlusion_matrix_thread\n" );
        if( ros_pcl->data.empty() ) return;
        // std::unique_ptr< sensor_msgs::msg::PointCloud2 > transformed_ros_pcl =
        //     std::make_unique< sensor_msgs::msg::PointCloud2 >();
        // pcl::shared_ptr< cloud_t > transformed_pcl( new cloud_t );

        // Voxelization
        // pcl::fromROSMsg( *ros_pcl, *transformed_pcl );

        // pcl_voxelization( transformed_pcl, this->leaf_size );

        // pcl::toROSMsg( *transformed_pcl, *transformed_ros_pcl );

        // tf2::doTransform< sensor_msgs::msg::PointCloud2 >( *transformed_ros_pcl, *transformed_ros_pcl, *transform );
        // printf( "transform\n" );
        // this->transf_pcl_pub->publish( *transformed_ros_pcl );
        // printf( "transform pub\n" );
        // pcl::fromROSMsg( *transformed_ros_pcl, *transformed_pcl );
        // printf( "pcl-fill\n" );

        // printf( "\tisOrganized: %i\n", transformed_pcl->isOrganized() );
        // printf( "\tis_dense: %i\n", transformed_pcl->is_dense );

        for( auto& v: *( this->occlusion_matrix ) )
            for( auto& e: v )
            {
                e.first.x  = DBL_MAX;
                e.first.y  = DBL_MAX;
                e.first.z  = DBL_MAX;

                e.second.x = DBL_MIN;
                e.second.y = DBL_MIN;
                e.second.z = DBL_MIN;
            }

        compute_occlusion_matrix( occlusion_matrix, ros_pcl );
        // printf( "compute_occlusion_matrix\n" );

        // TODO: Transform only the centroid point of the cube

        // printf( "beg occlusion_boxes_pub\n" );
        marker_array.markers.clear();
        this->box_marker.id = 0;
        for( auto& row_array: *occlusion_matrix )
        {
            for( auto& col_array: row_array )
            {
                if( !is_valid( col_array.first ) || !is_valid( col_array.second ) ) continue;
                if( col_array.first == col_array.second ) continue;
                this->box_marker.header.stamp = this->get_clock()->now();
                this->box_marker.id++;
                this->box_marker.scale.x         = abs( col_array.second.x - col_array.first.x );
                this->box_marker.scale.y         = abs( col_array.second.y - col_array.first.y );
                this->box_marker.scale.z         = abs( col_array.second.z - col_array.first.z );
                this->box_marker.pose.position.x = col_array.first.x + this->box_marker.scale.x / 2;
                this->box_marker.pose.position.y = col_array.first.y + this->box_marker.scale.y / 2;
                this->box_marker.pose.position.z = col_array.first.z + this->box_marker.scale.z / 2;
                marker_array.markers.push_back( this->box_marker );
            }
        }
        this->occlusion_boxes_pub->publish( marker_array );

        timer.print_time( "occlusion_matrix_callback" );
    }

  private:
};

}  // namespace smap

// RCLCPP_COMPONENTS_REGISTER_NODE(smap::object_estimator)

#endif  // SMAP_CORE__OBJECT__ESTIMATOR_HPP_
