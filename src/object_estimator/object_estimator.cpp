#include "object_estimator.hpp"

#include "parameter_tuning.hpp"

#include <math.h>

using namespace std::chrono_literals;

// TODO: Check the slider parameters to avoid seg fault

namespace smap
{

void object_estimator::box_filter(
    const pcl::shared_ptr< cloud_t >& input_cloud, const pcl::shared_ptr< cloud_t >& cloud_segment,
    smap_interfaces::msg::SmapObject& obj ) const
{
    count_time timer;
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices() );

    for( size_t h = obj.bb_2d.keypoint_1[ 1 ]; h <= obj.bb_2d.keypoint_2[ 1 ]; h++ )
        for( size_t w = obj.bb_2d.keypoint_1[ 0 ]; w <= obj.bb_2d.keypoint_2[ 0 ]; w++ )
            inliers->indices.push_back( ( w ) + input_cloud->width * (h) -1 );

    pcl::ExtractIndices< cloud_point_t > extract;
    extract.setInputCloud( input_cloud );
    extract.setIndices( inliers );

    extract.filter( *cloud_segment );

    const char str[] = "box_filter";
    timer.get_time( this->get_logger(), str, box_filter_plot );
}

void object_estimator::roi_filter( const pcl::shared_ptr< cloud_t >& point_cloud ) const
{
    count_time timer;

    point_cloud->erase(
        std::remove_if(
            point_cloud->begin(), point_cloud->end(),
            [ this ]( const cloud_point_t& point ) {
                return (
                    ( ( point.getVector3fMap() ).norm() < this->pcl_lims->first )
                    || ( ( point.getVector3fMap() ).norm() > this->pcl_lims->second ) );
            } ),
        point_cloud->end() );

    const char str[] = "roi_filter";
    timer.get_time( this->get_logger(), str, roi_filter_plot );
}

void object_estimator::pcl_voxelization( const pcl::shared_ptr< cloud_t >& point_cloud ) const
{
    count_time timer;
    pcl::VoxelGrid< cloud_point_t > vox_grid;

    vox_grid.setInputCloud( point_cloud );
    // vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
    vox_grid.setLeafSize( this->leaf_size, this->leaf_size, this->leaf_size );
    vox_grid.setDownsampleAllData( true );
    vox_grid.filter( *point_cloud );

    const char str[] = "pcl_vox";
    timer.get_time( this->get_logger(), str, voxelization_plot );
}

void object_estimator::statistical_outlier_filter( const pcl::shared_ptr< cloud_t >& cloud_segment ) const
{
    count_time timer;
    pcl::StatisticalOutlierRemoval< cloud_point_t > sor;
    sor.setInputCloud( cloud_segment );
    sor.setMeanK( this->mean_k );  // Greater ther values take more time to compute
    sor.setStddevMulThresh( this->mu );

    sor.filter( *cloud_segment );

    const char str[] = "statistical_outlier_filter";
    timer.get_time( this->get_logger(), str, sof_filter_plot );
}

void object_estimator::euclidean_clustering(
    const pcl::shared_ptr< cloud_t >& cloud_segment, const pcl::shared_ptr< cloud_t >& object_cloud ) const
{
    count_time timer;

    pcl::search::KdTree< cloud_point_t >::Ptr tree( new pcl::search::KdTree< cloud_point_t > );
    std::vector< int > indices;
    pcl::removeNaNFromPointCloud( *cloud_segment, *cloud_segment, indices );
    tree->setInputCloud( cloud_segment );

    std::vector< pcl::PointIndices > cluster_idx;
    pcl::EuclideanClusterExtraction< cloud_point_t > ece;
    ece.setClusterTolerance( this->ClusterTolerance );  // cm
    ece.setMinClusterSize( 25 );
    ece.setSearchMethod( tree );
    ece.setInputCloud( cloud_segment );
    ece.extract( cluster_idx );

    // Idxs of the biggest cluster
    if( cluster_idx.size() >= 1 )
        for( const auto& idx: cluster_idx[ 0 ].indices ) object_cloud->push_back( ( *cloud_segment )[ idx ] );

    const char str[] = "euclidean_clustering";
    timer.get_time( this->get_logger(), str, euclidean_clustering_plot );
}

void object_estimator::estimate_object_3D_AABB(
    const pcl::shared_ptr< cloud_t >& object_cloud, smap_interfaces::msg::SmapObject& obj ) const
{
    // TODO: Try to use medians
    count_time timer;

    obj.AABB.min.point.x = object_cloud->points[ 0 ].x;
    obj.AABB.min.point.y = object_cloud->points[ 0 ].y;
    obj.AABB.min.point.z = object_cloud->points[ 0 ].z;

    obj.AABB.max.point.x = object_cloud->points[ 0 ].x;
    obj.AABB.max.point.y = object_cloud->points[ 0 ].y;
    obj.AABB.max.point.z = object_cloud->points[ 0 ].z;

    for( cloud_point_t& point: *object_cloud )
    {
        if( point.x < obj.AABB.min.point.x ) obj.AABB.min.point.x = point.x;
        if( point.y < obj.AABB.min.point.y ) obj.AABB.min.point.y = point.y;
        if( point.z < obj.AABB.min.point.z ) obj.AABB.min.point.z = point.z;

        if( point.x > obj.AABB.max.point.x ) obj.AABB.max.point.x = point.x;
        if( point.y > obj.AABB.max.point.y ) obj.AABB.max.point.y = point.y;
        if( point.z > obj.AABB.max.point.z ) obj.AABB.max.point.z = point.z;
    }

    obj.pose.pose.position.x = ( obj.AABB.min.point.x + obj.AABB.max.point.x ) / 2;
    obj.pose.pose.position.y = ( obj.AABB.min.point.y + obj.AABB.max.point.y ) / 2;
    obj.pose.pose.position.z = ( obj.AABB.min.point.z + obj.AABB.max.point.z ) / 2;

    const char str[]         = "3D_AABB";
    timer.get_time( this->get_logger(), str, centroid_plot );
}

void object_estimator::transform_object_pcl(
    smap_interfaces::msg::SmapObject& obj,
    const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform ) const
{
    count_time timer;

    // // Centroid
    // tf2::doTransform< geometry_msgs::msg::PoseStamped >( obj.pose, obj.pose, *transform );

    // // Limits
    // tf2::doTransform< geometry_msgs::msg::PointStamped >( obj.AABB.min, obj.AABB.min, *transform );
    // tf2::doTransform< geometry_msgs::msg::PointStamped >( obj.AABB.max, obj.AABB.max, *transform );

    // Point cloud
    tf2::doTransform< sensor_msgs::msg::PointCloud2 >( obj.pointcloud, obj.pointcloud, *transform );

    const char str[] = "transform";
    timer.get_time( this->get_logger(), str, transform_plot );
}

void object_estimator::object_estimation_thread(
    const pcl::shared_ptr< cloud_t >& point_cloud,
    const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform,
    const std::shared_ptr< geometry_msgs::msg::PoseStamped >& pose,
    const smap_interfaces::msg::SmapObject::SharedPtr& obj )
{

    // printf("POS [%i,%i]\n",
    //   obj->bb_2d.keypoint_1[0],
    //   obj->bb_2d.keypoint_1[1]
    // );
    // if(
    //   ! ((obj->bb_2d.keypoint_1[0] > 180) &&
    //   (obj->bb_2d.keypoint_1[0] < 300))
    // ) return;

    // if(obj->confidence <= 70) return;

    try
    {
        printf( "object_estimation_thread\n" );
        count_time timer;
        smap_interfaces::msg::SmapObservation obs;
        obs.robot_pose = *pose;
        obs.object     = *obj;
        obs.direction  = 0;

        // RCLCPP_INFO(this->get_logger(),"Object: %i",obj->label);
        pcl::shared_ptr< cloud_t > point_cloud_vox( new cloud_t );
        pcl::shared_ptr< cloud_t > segment_cloud_pcl( new cloud_t );
        pcl::shared_ptr< cloud_t > object_cloud_pcl( new cloud_t );
        // pcl::shared_ptr<cloud_t> segment_cloud_pcl_neg(new cloud_t);

        // std::shared_ptr<sensor_msgs::msg::PointCloud2> segment_cloud_ros(new sensor_msgs::msg::PointCloud2);

        // Cloud filtering
        count_time filter_timer;
        this->box_filter( point_cloud, segment_cloud_pcl, obs.object );

        if( this->roi_filt ) this->roi_filter( segment_cloud_pcl );

        if( this->voxelization ) this->pcl_voxelization( segment_cloud_pcl );

        if( this->sof ) this->statistical_outlier_filter( segment_cloud_pcl );

        // Cloud Segmentation
        if( this->euclidean_clust ) this->euclidean_clustering( segment_cloud_pcl, object_cloud_pcl );
        const char str_filtering[] = "filtering_time";
        filter_timer.get_time( this->get_logger(), str_filtering, total_filter_plot );

        // if( this->euclidean_clust ) pcl::toROSMsg( *object_cloud_pcl, obs.object.pointcloud );

        // Transform
        if( this->euclidean_clust ) pcl::toROSMsg( *object_cloud_pcl, obs.object.pointcloud );
        else pcl::toROSMsg( *segment_cloud_pcl, obs.object.pointcloud );
        this->transform_object_pcl( obs.object, transform );

        // Parameter Estimation
        count_time estimation_timer;
        pcl::fromROSMsg( obs.object.pointcloud, *object_cloud_pcl );
        this->estimate_object_3D_AABB( object_cloud_pcl, obs.object );

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

        const std::lock_guard< std::mutex > object_pub_lock( this->object_pub_mutex );
        this->object_pub->publish( obs );

        const std::lock_guard< std::mutex > object_bb_pub_lock( this->object_bb_pub_mutex );
        this->publish_bb( 0, obs.object );

        const std::lock_guard< std::mutex > debug_object_pcl_pub_lock( this->debug_object_pcl_pub_mutex );
        this->debug_object_pcl_pub->publish( obs.object.pointcloud );
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
    pcl::fromROSMsg( input_msg->pointcloud, *pcl_point_cloud );

    // Thread launch

    smap_interfaces::msg::SmapObject obj;
    sensor_msgs::msg::PointCloud2 segment_cloud;

    for( auto& obj: input_msg->objects )
    {
        // BOOST_FOREACH(obj, input_msg->objects){
        if( obj.confidence < 0.70 ) continue;  // TODO: Remove DEBUG

        if( obj.label != 62 ) continue;        // TODO: Remove DEBUG

        obj.module_id = input_msg->module_id;

        // Block until thread pool is available
        while( !this->thread_ctl->available() ) std::this_thread::sleep_for( 10ms );

        static pcl::shared_ptr< cloud_t > lock_cloud;
        if( !this->pcl_lock ) lock_cloud = pcl_point_cloud;

        this->object_estimation_thread(
            lock_cloud, std::make_shared< geometry_msgs::msg::TransformStamped >( input_msg->robot_to_map ),
            std::make_shared< geometry_msgs::msg::PoseStamped >( input_msg->stamped_pose ),
            std::make_shared< smap_interfaces::msg::SmapObject >( obj ) );

        // TODO: Activate multi threading

        // this->thread_ctl->push_back(
        //   std::make_shared<std::future<void>>(
        //     std::async(
        //       std::launch::async,
        //       &smap::object_estimator::object_estimation_thread,this,
        //       pcl_point_cloud,
        //       std::make_shared<smap_interfaces::msg::SmapObject>(obj)
        //     )
        //   )
        // );

        RCLCPP_DEBUG(
            this->get_logger(), "Object detection thread launched! | %i threads running | label: %i",
            this->thread_ctl->size(), obj.label );
    }
    RCLCPP_DEBUG( this->get_logger(), "---Callback complete---" );
}

}  // namespace smap

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );

    std::shared_ptr< smap::object_estimator > node = std::make_shared< smap::object_estimator >();

    // auto ui = std::make_shared<std::future<void>>(
    //   std::async(
    //     std::launch::async,
    //     &imgui_thread,
    //   )
    // );
    auto plot_times_lambda = []( plot_vec& vec ) {  // Plot times
        static char overlay[ 32 ];
        float vals[ 128 ] = { 0 };
        vec.get_float_arr_av( vals );
        sprintf( overlay, "cur: %4.1f ms| avg %4.1f ms", vals[ 127 ], vec.get_average() );
        ImGui::PlotLines( "Execution time", vals, IM_ARRAYSIZE( vals ), 0, overlay, 0.0f, 200.0f, ImVec2( 0, 100.0f ) );
    };

    auto lambda = [ &node, &plot_times_lambda ]( void ) {  // ui lambda function
        ImGui::Begin( "Object Pose Estimator Parameters" );

        if( ImGui::CollapsingHeader( "Filtering" ) )
        {  // Filtering
            ImGui::Indent();

            if( ImGui::CollapsingHeader( "Box Filter" ) )
            {  // box filter
                ImGui::Indent();
                plot_times_lambda( box_filter_plot );
                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "Region Of Interest Filter" ) )
            {  // roi_filter
                ImGui::Indent();
                static bool roi_filter = true;
                if( ImGui::Checkbox( "ROI Filter", &roi_filter ) ) node->roi_filt = roi_filter;

                ImGui::Text( "pcl_lim" );
                ImGui::Text( "   Distance limits applied to the cloud." );

                ImGui::SliderFloat( "min", &( node->pcl_lims->first ), 0.0f, node->pcl_lims->second );
                ImGui::SliderFloat( "max", &( node->pcl_lims->second ), node->pcl_lims->first, 10.0f );

                plot_times_lambda( roi_filter_plot );
                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "PCL Voxelization" ) )
            {  // pcl_voxelization
                ImGui::Indent();
                static bool voxelization = true;
                if( ImGui::Checkbox( "Voxelization", &voxelization ) ) node->voxelization = voxelization;
                ImGui::Text( "LeafSize" );
                ImGui::Text( "   Greater values increase the size of the voxels (filter more points)" );
                ImGui::SliderFloat( "LeafSize", &( node->leaf_size ), 0.0f, 0.05f );

                plot_times_lambda( voxelization_plot );
                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "Statistical Outlier Filter" ) )
            {  // pcl_voxelization
                ImGui::Indent();
                static bool sof = true;
                if( ImGui::Checkbox( "SOF Filter", &sof ) ) node->sof = sof;
                ImGui::Text( "MeanK" );
                ImGui::Text( "   Number of neighbors to evaluate" );
                ImGui::SliderInt( "MeanK", &( node->mean_k ), 0, 100 );

                ImGui::Text( "Mu" );
                ImGui::Text( "   Local standard deviation" );
                ImGui::SliderFloat( "Mu", &( node->mu ), 0.0f, 2.0f );

                plot_times_lambda( sof_filter_plot );
                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "Euclidean Clustering" ) )
            {  // pcl_voxelization
                ImGui::Indent();
                static bool euclidean_clustering = true;
                if( ImGui::Checkbox( "Clustering", &euclidean_clustering ) )
                    node->euclidean_clust = euclidean_clustering;

                ImGui::Text( "Cluster Tolerance" );
                ImGui::Text( "   Max space between points" );
                ImGui::SliderFloat( "ClusterTolerance", &( node->ClusterTolerance ), 0.0f, 1.0f );

                plot_times_lambda( euclidean_clustering_plot );
                ImGui::Unindent();
            }

            plot_times_lambda( total_filter_plot );

            ImGui::Unindent();
        }

        if( ImGui::CollapsingHeader( "Parameter Estimation" ) )
        {  // Parameter estimation
            ImGui::Indent();

            if( ImGui::CollapsingHeader( "Centroid and Limits" ) )
            {  // Parameter estimation
                ImGui::Indent();

                plot_times_lambda( centroid_plot );

                ImGui::Unindent();
            }

            if( ImGui::CollapsingHeader( "Transform" ) )
            {  // Transform coordinates to map reference
                ImGui::Indent();

                plot_times_lambda( transform_plot );

                ImGui::Unindent();
            }

            plot_times_lambda( total_estimation_plot );

            ImGui::Unindent();
        }

        ImGui::Text( "Total execution time" );
        plot_times_lambda( total_thread_time );

        static int clicked = 0;
        if( ImGui::Button( "Lock PCL" ) ) clicked++;
        if( clicked & 1 )
        {
            ImGui::SameLine();
            ImGui::Text( "PCL locked!" );
            node->pcl_lock = true;
        }
        else node->pcl_lock = false;

        ImGui::End();

    };

    std::thread ui( imgui_thread< decltype( lambda ) >, lambda );

    ui.detach();

    try
    {
        rclcpp::spin( node );
    }
    catch( std::exception& e )
    {
        std::cout << "SMAP Exception!" << std::endl;
        std::cout << e.what() << std::endl;
    }
    rclcpp::shutdown();

    return 0;
}
