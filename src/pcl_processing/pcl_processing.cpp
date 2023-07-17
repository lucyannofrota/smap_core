#include "include/pcl_processing.hpp"

// STL
#include <cmath>

// PCL
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

// ROS
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// SMAP
#include "../../../include/smap_core/interface_templates.hpp"
#include "smap_core/macros.hpp"

namespace smap
{

// void roi_filter(
//     const pcl::shared_ptr< cloud_t >& point_cloud, const std::shared_ptr< std::pair< float, float > >& pcl_lims );

void box_filter(
    const pcl::shared_ptr< cloud_t >& input_cloud, const pcl::shared_ptr< cloud_t >& cloud_segment,
    const std::array< uint16_t, 2UL >& keypoint_1, const std::array< uint16_t, 2UL >& keypoint_2 )
{
    if( !input_cloud ) return;
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices() );

    for( size_t h = keypoint_1[ 1 ]; h <= keypoint_2[ 1 ]; h++ )
        for( size_t w = keypoint_1[ 0 ]; w <= keypoint_2[ 0 ]; w++ )
            inliers->indices.push_back( ( w ) + input_cloud->width * (h) -1 );

    pcl::ExtractIndices< cloud_point_t > extract;
    extract.setInputCloud( input_cloud );
    extract.setIndices( inliers );

    extract.filter( *cloud_segment );
}

void roi_filter(
    const pcl::shared_ptr< cloud_t >& point_cloud, const std::shared_ptr< std::pair< float, float > >& pcl_lims )
{
    if( !point_cloud ) return;
#if ROI_FILTER_METHOD == 0
    point_cloud->erase(
        std::remove_if(
            point_cloud->begin(), point_cloud->end(),
            [ &pcl_lims ]( const cloud_point_t& point ) {
                return (
                    ( ( point.getVector3fMap() ).norm() < pcl_lims->first )
                    || ( ( point.getVector3fMap() ).norm() > pcl_lims->second ) );
            } ),
        point_cloud->end() );
#else
    point_cloud->erase(
        std::remove_if(
            point_cloud->begin(), point_cloud->end(),
            [ &pcl_lims ]( const cloud_point_t& point ) {
                return ( ( point.x < pcl_lims->first ) || ( point.x > pcl_lims->second ) );
            } ),
        point_cloud->end() );
#endif
}

void pcl_voxelization( const pcl::shared_ptr< cloud_t >& point_cloud, const float& leaf_size )
{
    if( !point_cloud ) return;
    pcl::VoxelGrid< cloud_point_t > vox_grid;

    vox_grid.setInputCloud( point_cloud );
    vox_grid.setLeafSize( leaf_size, leaf_size, leaf_size );
    vox_grid.setDownsampleAllData( true );
    vox_grid.filter( *point_cloud );
}

void statistical_outlier_filter( const pcl::shared_ptr< cloud_t >& cloud_segment, const int& mean_k, const float& mu )
{
    if( !cloud_segment ) return;
    pcl::StatisticalOutlierRemoval< cloud_point_t > sor;
    sor.setInputCloud( cloud_segment );
    sor.setMeanK( mean_k );  // Greater ther values take more time to compute
    sor.setStddevMulThresh( mu );

    sor.filter( *cloud_segment );
}

void euclidean_clustering(
    const pcl::shared_ptr< cloud_t >& cloud_segment, const pcl::shared_ptr< cloud_t >& object_cloud,
    const float& ClusterTolerance )
{
    if( !cloud_segment ) return;

    pcl::search::KdTree< cloud_point_t >::Ptr tree( new pcl::search::KdTree< cloud_point_t > );
    std::vector< int > indices;
    pcl::removeNaNFromPointCloud( *cloud_segment, *cloud_segment, indices );
    tree->setInputCloud( cloud_segment );

    std::vector< pcl::PointIndices > cluster_idx;
    pcl::EuclideanClusterExtraction< cloud_point_t > ece;
    ece.setClusterTolerance( ClusterTolerance );  // cm
    ece.setMinClusterSize( 25 );
    ece.setSearchMethod( tree );
    ece.setInputCloud( cloud_segment );
    ece.extract( cluster_idx );

    // Idxs of the biggest cluster
    if( cluster_idx.size() >= 1 )
        for( const auto& idx: cluster_idx[ 0 ].indices ) object_cloud->push_back( ( *cloud_segment )[ idx ] );
}

bool estimate_object_3D_AABB(
    const pcl::shared_ptr< cloud_t >& object_cloud, geometry_msgs::msg::Point& pos, geometry_msgs::msg::Point& min,
    geometry_msgs::msg::Point& max )
{
    // TODO: Try to use medians
    if( !object_cloud || object_cloud->points.empty() ) return false;

    min.x = object_cloud->points[ 0 ].x;
    min.y = object_cloud->points[ 0 ].y;
    min.z = object_cloud->points[ 0 ].z;

    max.x = object_cloud->points[ 0 ].x;
    max.y = object_cloud->points[ 0 ].y;
    max.z = object_cloud->points[ 0 ].z;

    for( cloud_point_t& point: *object_cloud )
    {
        if( point.x < min.x ) min.x = point.x;
        if( point.y < min.y ) min.y = point.y;
        if( point.z < min.z ) min.z = point.z;

        if( point.x > max.x ) max.x = point.x;
        if( point.y > max.y ) max.y = point.y;
        if( point.z > max.z ) max.z = point.z;
    }

    pos.x = ( min.x + max.x ) / 2;
    pos.y = ( min.y + max.y ) / 2;
    pos.z = ( min.z + max.z ) / 2;

    return true;
}

bool estimate_confidence(
    const pcl::shared_ptr< cloud_t >& object_cloud, float& conf,
    const std::shared_ptr< std::pair< float, float > >& pcl_lims, const float& object_size_lim_conf )
{
    if( !object_cloud ) return false;
    float lims[ 2 ] = { std::numeric_limits< float >::infinity(), -std::numeric_limits< float >::infinity() };
    for( cloud_point_t& point: *object_cloud )
    {
        if( norm( point.x, point.y, point.z ) < lims[ 0 ] ) lims[ 0 ] = norm( point.x, point.y, point.z );
        if( norm( point.x, point.y, point.z ) > lims[ 1 ] ) lims[ 1 ] = norm( point.x, point.y, point.z );
    }

    if( std::isinf( lims[ 0 ] ) || std::isinf( lims[ 1 ] ) ) return false;

    double den = pcl_lims->second - pcl_lims->first, num = ( lims[ 1 ] - lims[ 0 ] );
    if( num <= object_size_lim_conf ) conf = 1;
    else conf = ( den - ( num - object_size_lim_conf ) ) / den;

    return true;
}

std::pair< int, int > compute_depth_map(
    depth_map_t& depth_map, const std::shared_ptr< sensor_msgs::msg::PointCloud2 >& pcl_ros,
    const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform,
    const std::shared_ptr< std::pair< float, float > >& pcl_lims )
{

    std::pair< int, int > ret(
        ceil( pcl_ros->height / ( (double) DEPTH_MAP_ROWS ) ), ceil( pcl_ros->width / ( (double) DEPTH_MAP_COLS ) ) );

    geometry_msgs::msg::Point pt;
    uint16_t occlusion_r, occlusion_c;

    sensor_msgs::PointCloud2Iterator< float > iter_x( *pcl_ros, "x" );
    sensor_msgs::PointCloud2Iterator< float > iter_y( *pcl_ros, "y" );
    sensor_msgs::PointCloud2Iterator< float > iter_z( *pcl_ros, "z" );

    // Fill the map on the point cloud frame
    for( uint32_t pcl_r = 0; pcl_r < pcl_ros->height; pcl_r++ )
    {
        occlusion_r           = floor( pcl_r / ret.first );
        auto& occlusion_array = depth_map[ occlusion_r ];
        for( uint32_t pcl_c = 0; pcl_c < pcl_ros->width; pcl_c++, ++iter_x, ++iter_y, ++iter_z )
        {
            occlusion_c = floor( pcl_c / ret.second );

            if( ( !is_valid( *iter_x, *iter_y, *iter_z ) )
                || ( ( norm( *iter_x, *iter_y, *iter_z ) < pcl_lims->first )
                     || ( norm( *iter_x, *iter_y, *iter_z ) > pcl_lims->second ) ) )
                continue;

            // Min
            if( *iter_x < occlusion_array[ occlusion_c ][ 0 ].x ) occlusion_array[ occlusion_c ][ 0 ].x = *iter_x;
            if( *iter_y < occlusion_array[ occlusion_c ][ 0 ].y ) occlusion_array[ occlusion_c ][ 0 ].y = *iter_y;
            if( *iter_z < occlusion_array[ occlusion_c ][ 0 ].z ) occlusion_array[ occlusion_c ][ 0 ].z = *iter_z;

            // Max
            if( *iter_x > occlusion_array[ occlusion_c ][ 1 ].x ) occlusion_array[ occlusion_c ][ 1 ].x = *iter_x;
            if( *iter_y > occlusion_array[ occlusion_c ][ 1 ].y ) occlusion_array[ occlusion_c ][ 1 ].y = *iter_y;
            if( *iter_z > occlusion_array[ occlusion_c ][ 1 ].z ) occlusion_array[ occlusion_c ][ 1 ].z = *iter_z;
        }
    }

    // Transpose map to the world frame
    std::array< geometry_msgs::msg::PointStamped, 8 > AABB;
    geometry_msgs::msg::Point min, max;
    for( auto& row_array: depth_map )
    {
        for( auto& element: row_array )
        {
            if( ( element[ 0 ] == element[ 1 ] ) || ( !is_valid( element[ 0 ] ) ) || ( !is_valid( element[ 1 ] ) ) )
                continue;

            set_AABB( AABB, element[ 0 ], element[ 1 ] );

            min = AABB[ 0 ].point;
            max = AABB[ 0 ].point;
            for( size_t idx = 0; idx < 8; idx++ )
            {
                tf2::doTransform< geometry_msgs::msg::PointStamped >( AABB[ idx ], AABB[ idx ], *transform );
                if( idx == 0 )
                {
                    min = AABB[ idx ].point;
                    max = AABB[ idx ].point;
                    continue;
                }
                else
                {
                    // Min
                    if( AABB[ idx ].point.x < min.x ) min.x = AABB[ idx ].point.x;
                    if( AABB[ idx ].point.y < min.y ) min.y = AABB[ idx ].point.y;
                    if( AABB[ idx ].point.z < min.z ) min.z = AABB[ idx ].point.z;
                    // Max
                    if( AABB[ idx ].point.x > max.x ) max.x = AABB[ idx ].point.x;
                    if( AABB[ idx ].point.y > max.y ) max.y = AABB[ idx ].point.y;
                    if( AABB[ idx ].point.z > max.z ) max.z = AABB[ idx ].point.z;
                }
            }
            double sx = abs( max.x - min.x );
            double sy = abs( max.y - min.y );
            double sz = abs( max.z - min.z );
            if( ( ( sx == 0 ) || ( sy == 0 ) || ( sz == 0 ) ) || ( sx * sy * sz > MAX_OCCLUSION_CELL_VOLUME )
                || ( sx > MAX_OCCLUSION_CELL_VOLUME_FACTOR ) || ( sy > MAX_OCCLUSION_CELL_VOLUME_FACTOR )
                || ( sz > MAX_OCCLUSION_CELL_VOLUME_FACTOR ) )
            {
                element[ 0 ].x = std::numeric_limits< double >::infinity();
                element[ 0 ].y = std::numeric_limits< double >::infinity();
                element[ 0 ].z = std::numeric_limits< double >::infinity();
                element[ 1 ].x = -std::numeric_limits< double >::infinity();
                element[ 1 ].y = -std::numeric_limits< double >::infinity();
                element[ 1 ].z = -std::numeric_limits< double >::infinity();
                element[ 2 ].x = std::numeric_limits< double >::quiet_NaN();
                element[ 2 ].y = std::numeric_limits< double >::quiet_NaN();
                element[ 2 ].z = std::numeric_limits< double >::quiet_NaN();
                continue;
            }

            element[ 0 ] = min;
            element[ 1 ] = max;
            element[ 2 ] = ( min + max ) / 2;
        }
    }
    return ret;
}

bool check_occlusions( void )
{
    //
    return false;
}

}  // namespace smap
