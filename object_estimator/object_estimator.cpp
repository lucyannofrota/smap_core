#include "include/object_estimator/object_estimator.hpp"

#include "include/object_estimator/imgui_vars.hpp"

#include <math.h>

// SMAP
#include "include/object_estimator/macros.hpp"
#include "pcl_processing/depth_map.hpp"
#include "smap_base/aux_functions.hpp"

plot_vec total_thread_time;
plot_vec box_filter_plot, roi_filter_plot, voxelization_plot, sof_filter_plot, euclidean_clustering_plot,
    total_filter_plot;
plot_vec centroid_plot, transform_plot, total_estimation_plot;

using namespace std::chrono_literals;

// TODO: Check the slider parameters to avoid seg fault

namespace smap
{

void object_estimator::object_estimation_thread(
    const pcl::shared_ptr< cloud_t >& point_cloud,
    const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform,
    const std::shared_ptr< geometry_msgs::msg::PoseStamped >& pose,
    const smap_interfaces::msg::SmapObject::SharedPtr& obj )
{
    if( !point_cloud || !transform || !pose || !obj ) return;

    this->tim_object_estimation_thread.start();

    try
    {
        RCLCPP_DEBUG_EXPRESSION( this->get_logger(), DEBUG_MODE, "object_estimation_thread" );
        count_time timer;
        smap_interfaces::msg::SmapObservation obs;
        obs.robot_pose = *pose;
        obs.object     = *obj;
        obs.direction  = 0;

        // RCLCPP_INFO(this->get_logger(),"Object: %i",obj->label);
        // pcl::shared_ptr< cloud_t > point_cloud( new cloud_t );
        pcl::shared_ptr< cloud_t > segment_cloud_pcl( new cloud_t );
        pcl::shared_ptr< cloud_t > object_cloud_pcl( new cloud_t );
        // pcl::shared_ptr<cloud_t> segment_cloud_pcl_neg(new cloud_t);

        // std::shared_ptr<sensor_msgs::msg::PointCloud2> segment_cloud_ros(new sensor_msgs::msg::PointCloud2);

        // Cloud filtering
        count_time filter_timer;
        // this->box_filter( point_cloud, segment_cloud_pcl, obs.object );
        count_time box_filter_timer;
        this->tim_box_filter.start();
        this->box_pc.start( point_cloud->width * point_cloud->height );
        box_filter( point_cloud, segment_cloud_pcl, obs.object.bb_2d.keypoint_1, obs.object.bb_2d.keypoint_2 );
        this->box_pc.stop( segment_cloud_pcl->width * segment_cloud_pcl->height );
        this->tim_box_filter.stop();
        const char box_filt_str[] = "box_filter";
        box_filter_timer.get_time( this->get_logger(), box_filt_str, box_filter_plot );

        if( this->roi_filt )
        {
            // this->roi_filter( segment_cloud_pcl );
            count_time roi_filt_timer;
            this->tim_roi_filter.start();
            this->roi_pc.start( segment_cloud_pcl->width * segment_cloud_pcl->height );
            roi_filter( segment_cloud_pcl, this->pcl_lims );
            this->roi_pc.stop( segment_cloud_pcl->width * segment_cloud_pcl->height );
            this->tim_roi_filter.stop();
            const char roi_filt_str[] = "roi_filter";
            roi_filt_timer.get_time( this->get_logger(), roi_filt_str, roi_filter_plot );
        }
        if( this->voxelization )
        {

            // this->pcl_voxelization( segment_cloud_pcl );
            count_time pcl_vox_timer;
            this->tim_voxelization.start();
            this->voxelization_pc.start( segment_cloud_pcl->width * segment_cloud_pcl->height );
            pcl_voxelization( segment_cloud_pcl, this->leaf_size );
            this->voxelization_pc.stop( segment_cloud_pcl->width * segment_cloud_pcl->height );
            this->tim_voxelization.stop();
            const char pcl_vox_str[] = "pcl_vox";
            pcl_vox_timer.get_time( this->get_logger(), pcl_vox_str, voxelization_plot );
        }

        if( this->sof )
        {
            // this->statistical_outlier_filter( segment_cloud_pcl );
            count_time statistical_outlier_filter_timer;
            this->tim_sof.start();
            this->sof_pc.start( segment_cloud_pcl->width * segment_cloud_pcl->height );
            statistical_outlier_filter( segment_cloud_pcl, this->mean_k, this->mu );
            this->sof_pc.stop( segment_cloud_pcl->width * segment_cloud_pcl->height );
            this->tim_sof.stop();
            const char sof_filt_str[] = "statistical_outlier_filter";
            statistical_outlier_filter_timer.get_time( this->get_logger(), sof_filt_str, sof_filter_plot );
        }

        // Cloud Segmentation
        if( this->euclidean_clust )
        {
            // this->euclidean_clustering( segment_cloud_pcl, segment_cloud_pcl );
            count_time euclidean_clustering_timer;
            this->tim_euclidean_clustering.start();
            this->clustering_pc.start( segment_cloud_pcl->width * segment_cloud_pcl->height );
            euclidean_clustering( segment_cloud_pcl, object_cloud_pcl, this->ClusterTolerance );
            this->clustering_pc.stop( segment_cloud_pcl->width * segment_cloud_pcl->height );
            this->tim_euclidean_clustering.stop();
            const char euc_clust_str[] = "euclidean_clustering";
            euclidean_clustering_timer.get_time( this->get_logger(), euc_clust_str, euclidean_clustering_plot );
        }

        const char str_filtering[] = "filtering_time";
        filter_timer.get_time( this->get_logger(), str_filtering, total_filter_plot );

        // Object cloud confidence
        if( this->euclidean_clust )
        {
            // this->estimate_confidence( segment_cloud_pcl, obs.object.aabb.confidence );
            this->tim_estimate_confidence.start();
            estimate_confidence( object_cloud_pcl, obs.object.aabb.confidence, this->pcl_lims, OBJECT_SIZE_LIM_CONF );
            this->tim_estimate_confidence.stop();
        }
        else
        {
            // this->estimate_confidence( segment_cloud_pcl, obs.object.aabb.confidence );
            this->tim_estimate_confidence.start();
            estimate_confidence( segment_cloud_pcl, obs.object.aabb.confidence, this->pcl_lims, OBJECT_SIZE_LIM_CONF );
            this->tim_estimate_confidence.stop();
        }

        // Transform
        // TODO: check the possibility to apply the transform only to the centroids
        this->tim_transform.start();
        if( this->euclidean_clust ) pcl::toROSMsg( *object_cloud_pcl, obs.object.pointcloud );
        else pcl::toROSMsg( *segment_cloud_pcl, obs.object.pointcloud );
        // this->transform_object_pcl( obs.object, transform );
        count_time transform_timer;
        tf2::doTransform< sensor_msgs::msg::PointCloud2 >( obs.object.pointcloud, obs.object.pointcloud, *transform );
        // transform_object_pcl( obs.object, transform );
        const char transform_str[] = "transform";
        transform_timer.get_time( this->get_logger(), transform_str, transform_plot );

        // Parameter Estimation
        count_time estimation_timer;
        pcl::fromROSMsg( obs.object.pointcloud, *segment_cloud_pcl );
        this->tim_transform.stop();
        // if( !this->estimate_object_3D_AABB( segment_cloud_pcl, obs.object ) ) return;
        count_time timer_3D_AABB;
        this->tim_3D_AAB.start();
        if( !estimate_object_3D_AABB(
                segment_cloud_pcl, obs.object.pose.pose.position, obs.object.aabb.min.point,
                obs.object.aabb.max.point ) )
        {
            this->tim_3D_AAB.stop();
            return;
        }
        this->tim_3D_AAB.stop();
        const char AABB_str[] = "3D_AABB";
        timer_3D_AABB.get_time( this->get_logger(), AABB_str, centroid_plot );

        const char str_estimation_time[] = "estimation_time";
        estimation_timer.get_time( this->get_logger(), str_estimation_time, total_estimation_plot );

        const char str_process_time[] = "pcl_process";
        timer.get_time( this->get_logger(), str_process_time, total_thread_time );
        // plot_vec centroid_plot, boundaries_plot, total_estimation_plot;

        obs.object.pointcloud.header.frame_id = "map";  // TODO: Check if can be removed

        // Direction of observation
        obs.direction = atan2(
            obs.robot_pose.pose.position.y - obs.object.pose.pose.position.y,
            obs.robot_pose.pose.position.x - obs.object.pose.pose.position.x );

        const std::lock_guard< std::mutex > pcl_pub_lock( this->pcl_pub_mutex );
        if( this->pcl_pub->get_subscription_count() > 0 )
        {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg( *point_cloud, msg );
            tf2::doTransform< sensor_msgs::msg::PointCloud2 >( msg, msg, *transform );
            this->pcl_pub->publish( msg );
        }

        const std::lock_guard< std::mutex > object_pub_lock( this->object_pub_mutex );
        if( this->object_pub->get_subscription_count() > 0 ) this->object_pub->publish( obs );

        const std::lock_guard< std::mutex > object_bb_pub_lock( this->object_bb_pub_mutex );
        if( this->object_bb_pub->get_subscription_count() > 0 ) this->publish_bb( 0, obs.object );

        const std::lock_guard< std::mutex > object_pcl_pub_lock( this->object_pcl_pub_mutex );
        if( this->object_pcl_pub->get_subscription_count() > 0 ) this->object_pcl_pub->publish( obs.object.pointcloud );
        this->tim_object_estimation_thread.stop();
        RCLCPP_INFO( this->get_logger(), "Object Detected" );
    }
    catch( std::exception& e )
    {
        std::cout << "object_estimation_thread error!" << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void object_estimator::detections_callback( const smap_interfaces::msg::SmapDetections::SharedPtr input_msg )
{
    RCLCPP_DEBUG( this->get_logger(), "detections_callback" );

    pcl::shared_ptr< cloud_t > pcl_point_cloud( new cloud_t );
    std::shared_ptr< geometry_msgs::msg::TransformStamped > transform =
        std::make_shared< geometry_msgs::msg::TransformStamped >( input_msg->camera_to_map );
    std::shared_ptr< geometry_msgs::msg::PoseStamped > pose =
        std::make_shared< geometry_msgs::msg::PoseStamped >( input_msg->stamped_pose );

    pcl::fromROSMsg( input_msg->pointcloud, *pcl_point_cloud );

    // Thread launch

    smap_interfaces::msg::SmapObject obj;

    static pcl::shared_ptr< cloud_t > lock_cloud;
    if( !this->pcl_lock )
    {
        lock_cloud = pcl_point_cloud;
        // pcl::io::savePCDFileASCII( "src/smap/smap_core/samples/example_pcd.pcd", *lock_cloud );
    }

    // launch depth_map_thread
    // input_msg->

    // std::async(
    //     std::launch::async, &object_estimator::validation_thread, this,
    //     std::make_shared< sensor_msgs::msg::PointCloud2 >( input_msg->pointcloud ), transform );

    if( this->pcl_lock )
    {
        sensor_msgs::msg::PointCloud2 segment_cloud;
        pcl::toROSMsg( *lock_cloud, segment_cloud );
        std::async(
            std::launch::async, &object_estimator::depth_map_thread, this,
            std::make_shared< sensor_msgs::msg::PointCloud2 >( segment_cloud ), transform, input_msg->stamped_pose );
    }
    else
    {
        std::async(
            std::launch::async, &object_estimator::depth_map_thread, this,
            std::make_shared< sensor_msgs::msg::PointCloud2 >( input_msg->pointcloud ), transform,
            input_msg->stamped_pose );
    }

    for( auto& obj: input_msg->objects )
    {
        if( obj.confidence < this->get_parameter( "Minimum_Object_Confidence" ).as_double() )
            continue;  // TODO: Remove DEBUG

        // if( obj.label != 62 ) continue;  // TODO: Remove DEBUG

        // if( ( obj.bb_2d.keypoint_1[ 0 ] < ( ( 672 / 2 ) - ( 672 / 6 ) ) )
        //     || ( obj.bb_2d.keypoint_1[ 0 ] > ( ( 672 / 2 ) + ( 672 / 6 ) ) ) )
        //     continue;  // TODO: Remove DEBUG

        obj.module_id = input_msg->module_id;

        // Block until thread pool is available
        while( !this->thread_ctl->available() ) std::this_thread::sleep_for( 1ms );

        // this->object_estimation_thread(
        //     lock_cloud, transform, pose, std::make_shared< smap_interfaces::msg::SmapObject >( obj ) );

        // TODO: Activate multi threading

        this->thread_ctl->push_back( std::make_shared< std::future< void > >( std::async(
            std::launch::async, &smap::object_estimator::object_estimation_thread, this, lock_cloud, transform, pose,
            std::make_shared< smap_interfaces::msg::SmapObject >( obj ) ) ) );

        RCLCPP_DEBUG(
            this->get_logger(), "Object detection thread launched! | %i threads running | label: %i",
            this->thread_ctl->size(), obj.label );
    }
    // input_msg->
    RCLCPP_DEBUG( this->get_logger(), "---Callback complete---" );
}

void object_estimator::depth_map_thread(
    const std::shared_ptr< sensor_msgs::msg::PointCloud2 >& ros_pcl,
    const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform,
    const geometry_msgs::msg::PoseStamped& robot_pose )
{

    // TODO: make mutually exclusive
    if( ros_pcl->data.empty() ) return;
    if( !is_valid(
            transform->transform.translation.x, transform->transform.translation.y, transform->transform.translation.z )
        || !is_valid(
            transform->transform.rotation.x, transform->transform.rotation.y, transform->transform.rotation.z )
        || std::isnan( transform->transform.rotation.w ) || std::isinf( transform->transform.rotation.w ) )
        printf( "\t\tInvalid transform\n" );

    // Occlusion Map Initialization
    depth_map_t depth_map;
    for( auto& v: depth_map )
    {
        for( auto& e: v )
        {
            e[ 0 ].x = std::numeric_limits< double >::infinity();
            e[ 0 ].y = std::numeric_limits< double >::infinity();
            e[ 0 ].z = std::numeric_limits< double >::infinity();

            e[ 1 ].x = -std::numeric_limits< double >::infinity();
            e[ 1 ].y = -std::numeric_limits< double >::infinity();
            e[ 1 ].z = -std::numeric_limits< double >::infinity();

            e[ 2 ].x = std::numeric_limits< double >::quiet_NaN();
            e[ 2 ].y = std::numeric_limits< double >::quiet_NaN();
            e[ 2 ].z = std::numeric_limits< double >::quiet_NaN();
        }
    }

    // Compute Occlusion Map
    auto cell_dims = compute_depth_map(
        depth_map, ros_pcl, transform, this->pcl_lims, this->get_parameter( "Max_Occlusion_Cell_Volume" ).as_double(),
        this->get_parameter( "Max_Occlusion_Cell_Volume_Factor" ).as_double() );

    // Publish marker
    if( this->occlusion_boxes_pub->get_subscription_count() > 0 )
    {
        marker_array.markers.clear();
        this->box_marker.id = 0;
        for( const auto& row_array: depth_map )
        {
            for( const auto& element: row_array )
            {
                if( ( !is_valid( element[ 0 ] ) ) || ( !is_valid( element[ 1 ] ) ) || ( !is_valid( element[ 2 ] ) ) )
                    continue;
                this->box_marker.header.stamp = this->get_clock()->now();
                this->box_marker.id++;
                this->box_marker.scale.x       = abs( element[ 1 ].x - element[ 0 ].x );
                this->box_marker.scale.y       = abs( element[ 1 ].y - element[ 0 ].y );
                this->box_marker.scale.z       = abs( element[ 1 ].z - element[ 0 ].z );
                this->box_marker.pose.position = element[ 2 ];
                this->marker_array.markers.push_back( this->box_marker );
            }
        }
        this->occlusion_boxes_pub->publish( marker_array );
    }

    // Publish occlusion map
    if( this->depth_map_pub->get_subscription_count() > 0 )
    {
        to_msg( depth_map, this->occ_map, cell_dims );  // REVERT
        this->occ_map.camera_pose = robot_pose.pose;    // REVERT

        this->depth_map_pub->publish( this->occ_map );  // REVERT
    }
}

}  // namespace smap
