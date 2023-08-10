#ifndef SMAP_CORE__PCL_PROCESSING_HPP_
#define SMAP_CORE__PCL_PROCESSING_HPP_

// STL
#include <algorithm>
#include <array>
#include <memory>
#include <tuple>
#include <utility>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <visualization_msgs/msg/marker.hpp>

// SMAP
#include "../../../include/smap_core/macros.hpp"
#include "../../object_estimator/include/depth_map.hpp"

// TODO: Transpose all PCL related function to this file
// TODO: Solve compile error

namespace smap
{
using cloud_point_t = pcl::PointXYZRGB;
using cloud_t       = pcl::PointCloud< cloud_point_t >;

template< typename IteratorT, typename Functor_comp, typename Functor_value >
std::array< double, 2 > variable_percentiles(
    IteratorT begin, IteratorT end, double percentile, Functor_comp f_comp, Functor_value f_val )
{

    const size_t size             = std::distance( begin, end );
    const size_t lower_percentile = int( ceil( size * percentile ) ),
                 upper_percentile = int( floor( size * ( 1.0 - percentile ) ) );
    std::array< double, 2 > ret {
        -std::numeric_limits< double >::infinity(), std::numeric_limits< double >::infinity() };
    if( lower_percentile < size && upper_percentile > 0 )
    {
        std::nth_element( begin, begin + lower_percentile, end, f_comp );
        ret[ 0 ] = f_val( begin[ lower_percentile ] );
        std::nth_element( begin, begin + upper_percentile, end, f_comp );
        ret[ 1 ] = f_val( begin[ upper_percentile ] );
    }
    return ret;
}

template< typename IteratorT >
std::array< std::array< double, 2 >, 3 > struct_variable_percentiles(
    IteratorT begin, IteratorT end, double percentile )
{
    std::array< std::array< double, 2 >, 3 > ret;
    using Point = typename std::iterator_traits< IteratorT >::value_type;
    ret[ 0 ]    = variable_percentiles(
        begin, end, percentile, []( const Point& t1, const Point& t2 ) { return t1.x < t2.x; },
        []( const Point& p ) { return p.x; } );
    ret[ 1 ] = variable_percentiles(
        begin, end, percentile, []( const Point& t1, const Point& t2 ) { return t1.y < t2.y; },
        []( const Point& p ) { return p.y; } );
    ret[ 2 ] = variable_percentiles(
        begin, end, percentile, []( const Point& t1, const Point& t2 ) { return t1.z < t2.z; },
        []( const Point& p ) { return p.z; } );
    return ret;
}

void box_filter(
    const pcl::shared_ptr< cloud_t >& input_cloud, const pcl::shared_ptr< cloud_t >& cloud_segment,
    const std::array< uint16_t, 2UL >& keypoint_1, const std::array< uint16_t, 2UL >& keypoint_2 );

void roi_filter(
    const pcl::shared_ptr< cloud_t >& point_cloud, const std::shared_ptr< std::pair< float, float > >& pcl_lims );

void pcl_voxelization( const pcl::shared_ptr< cloud_t >& point_cloud, const float& leaf_size );

void statistical_outlier_filter( const pcl::shared_ptr< cloud_t >& cloud_segment, const int& mean_k, const float& mu );

void euclidean_clustering(
    const pcl::shared_ptr< cloud_t >& cloud_segment, const pcl::shared_ptr< cloud_t >& object_cloud,
    const float& ClusterTolerance );

bool estimate_object_3D_AABB(
    const pcl::shared_ptr< cloud_t >& object_cloud, geometry_msgs::msg::Point& pos, geometry_msgs::msg::Point& min,
    geometry_msgs::msg::Point& max );

bool estimate_confidence(
    const pcl::shared_ptr< cloud_t >& object_cloud, float& conf,
    const std::shared_ptr< std::pair< float, float > >& pcl_lims, const float& object_size_lim_conf );

inline void set_marker(
    int32_t i, depth_cell_t cell, rclcpp::Clock::SharedPtr clock, visualization_msgs::msg::Marker& marker )
{
    marker.header.stamp    = clock->now();
    marker.id              = i;
    marker.scale.x         = abs( std::get< 1 >( cell ).x - std::get< 0 >( cell ).x );
    marker.scale.y         = abs( std::get< 1 >( cell ).y - std::get< 0 >( cell ).y );
    marker.scale.z         = abs( std::get< 1 >( cell ).z - std::get< 0 >( cell ).z );
    marker.pose.position.x = ( std::get< 1 >( cell ).x + std::get< 0 >( cell ).x ) / 2;
    marker.pose.position.y = ( std::get< 1 >( cell ).y + std::get< 0 >( cell ).y ) / 2;
    marker.pose.position.z = ( std::get< 1 >( cell ).z + std::get< 0 >( cell ).z ) / 2;
}

std::pair< int, int > compute_depth_map(
    depth_map_t& depth_map, const std::shared_ptr< sensor_msgs::msg::PointCloud2 >& pcl_ros,
    const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform,
    const std::shared_ptr< std::pair< float, float > >& pcl_lims, const double& Max_Occlusion_Cell_Volume, const double& Max_Occlusion_Cell_Volume_Factor );

// inline void set_AABB(
//     std::array< geometry_msgs::msg::PointStamped, 8 >& AABB, const geometry_msgs::msg::Point& min,
//     const geometry_msgs::msg::Point& max )
// {
//     // [0]
//     AABB[ 0 ].point = min;

// // [1]
// AABB[ 1 ].point.x = min.x;
// AABB[ 1 ].point.y = min.y;
// AABB[ 1 ].point.z = max.z;

// // [2]
// AABB[ 2 ].point.x = min.x;
// AABB[ 2 ].point.y = max.y;
// AABB[ 2 ].point.z = min.z;

// // [3]
// AABB[ 3 ].point.x = max.x;
// AABB[ 3 ].point.y = min.y;
// AABB[ 3 ].point.z = min.z;

// // [4]
// AABB[ 4 ].point.x = min.x;
// AABB[ 4 ].point.y = max.y;
// AABB[ 4 ].point.z = max.z;

// // [5]
// AABB[ 5 ].point.x = max.x;
// AABB[ 5 ].point.y = min.y;
// AABB[ 5 ].point.z = max.z;

// // [6]
// AABB[ 6 ].point.x = max.x;
// AABB[ 6 ].point.y = max.y;
// AABB[ 6 ].point.z = min.z;

// // [7]
// AABB[ 7 ].point = max;
// }

bool check_occlusions( void );

inline double& depth_map_indexer(
    std_msgs::msg::Float64MultiArray& occ_mat, const size_t& r, const size_t& c, const size_t& lims,
    const size_t& comp )
{
    return occ_mat.data
        [ occ_mat.layout.dim[ 0 ].stride * r + occ_mat.layout.dim[ 1 ].stride * c
          + occ_mat.layout.dim[ 2 ].stride * lims + comp ];
}

}  // namespace smap

#endif  // SMAP_CORE__PCL_PROCESSING_HPP_
