#ifndef SMAP_CORE__PCL_PROCESSING_HPP_
#define SMAP_CORE__PCL_PROCESSING_HPP_

// STL
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
#include "../../object_estimator/include/occlusion_map.hpp"

// TODO: Transpose all PCL related function to this file
// TODO: Solve compile error

namespace smap
{
using cloud_point_t = pcl::PointXYZRGB;
using cloud_t       = pcl::PointCloud< cloud_point_t >;

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
    int32_t i, occlusion_cell_t cell, rclcpp::Clock::SharedPtr clock, visualization_msgs::msg::Marker& marker )
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

std::pair< int, int > compute_occlusion_map(
    occlusion_map_t& occlusion_map, const std::shared_ptr< sensor_msgs::msg::PointCloud2 >& pcl_ros,
    const std::shared_ptr< geometry_msgs::msg::TransformStamped >& transform,
    const std::shared_ptr< std::pair< float, float > >& pcl_lims );

inline void set_AABB(
    std::array< geometry_msgs::msg::PointStamped, 8 >& AABB, const geometry_msgs::msg::Point& min,
    const geometry_msgs::msg::Point& max )
{
    // [0]
    AABB[ 0 ].point = min;

    // [1]
    AABB[ 1 ].point.x = min.x;
    AABB[ 1 ].point.y = min.y;
    AABB[ 1 ].point.z = max.z;

    // [2]
    AABB[ 2 ].point.x = min.x;
    AABB[ 2 ].point.y = max.y;
    AABB[ 2 ].point.z = min.z;

    // [3]
    AABB[ 3 ].point.x = max.x;
    AABB[ 3 ].point.y = min.y;
    AABB[ 3 ].point.z = min.z;

    // [4]
    AABB[ 4 ].point.x = min.x;
    AABB[ 4 ].point.y = max.y;
    AABB[ 4 ].point.z = max.z;

    // [5]
    AABB[ 5 ].point.x = max.x;
    AABB[ 5 ].point.y = min.y;
    AABB[ 5 ].point.z = max.z;

    // [6]
    AABB[ 6 ].point.x = max.x;
    AABB[ 6 ].point.y = max.y;
    AABB[ 6 ].point.z = min.z;

    // [7]
    AABB[ 7 ].point = max;
}

inline bool is_valid( const geometry_msgs::msg::Point& p )
{
    return !(
        ( std::isnan( p.x ) || std::isnan( p.y ) || std::isnan( p.z ) )
        || ( ( std::isinf( p.x ) || std::isinf( p.y ) || std::isinf( p.z ) ) ) );
}

inline bool is_valid( const double& x, const double& y, const double& z )
{
    return !(
        ( std::isnan( x ) || std::isnan( y ) || std::isnan( z ) )
        || ( ( std::isinf( x ) || std::isinf( y ) || std::isinf( z ) ) ) );
}

bool check_occlusions( void );

inline double& occlusion_map_indexer(
    std_msgs::msg::Float64MultiArray& occ_mat, const size_t& r, const size_t& c, const size_t& lims,
    const size_t& comp )
{
    return occ_mat.data
        [ occ_mat.layout.dim[ 0 ].stride * r + occ_mat.layout.dim[ 1 ].stride * c
          + occ_mat.layout.dim[ 2 ].stride * lims + comp ];
}

}  // namespace smap

#endif  // SMAP_CORE__PCL_PROCESSING_HPP_
