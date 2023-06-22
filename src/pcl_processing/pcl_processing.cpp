#include "include/pcl_processing.hpp"

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

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
    point_cloud->erase(
        std::remove_if(
            point_cloud->begin(), point_cloud->end(),
            [ &pcl_lims ]( const cloud_point_t& point ) {
                return (
                    ( ( point.getVector3fMap() ).norm() < pcl_lims->first )
                    || ( ( point.getVector3fMap() ).norm() > pcl_lims->second ) );
            } ),
        point_cloud->end() );
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
    float lims[ 2 ] = { FLT_MAX, FLT_MIN };
    for( cloud_point_t& point: *object_cloud )
    {
        if( point.z < lims[ 0 ] ) lims[ 0 ] = point.z;
        if( point.z > lims[ 1 ] ) lims[ 1 ] = point.z;
    }

    conf = ( ( lims[ 1 ] - lims[ 0 ] ) - object_size_lim_conf )
         / ( ( pcl_lims->second - pcl_lims->first ) - object_size_lim_conf );
    return true;
}

bool check_occlusions( void )
{
    //
    return false;
}

}  // namespace smap
