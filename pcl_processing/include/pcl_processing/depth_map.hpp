#ifndef SMAP_CORE__OCCLUSION_MAP_HPP_
#define SMAP_CORE__OCCLUSION_MAP_HPP_

// STL
#include <array>

// ROS
#include <geometry_msgs/msg/point.hpp>

// SMAP
#include "smap_interfaces/msg/depth_map.hpp"

#define DEPTH_MAP_FIELDS 3
#define DEPTH_MAP_ROWS 16  // 32
#define DEPTH_MAP_COLS 24  // 64

namespace smap
{

using depth_cell_t  = std::array< geometry_msgs::msg::Point, 3 >;
using depth_array_t = std::array< depth_cell_t, DEPTH_MAP_COLS >;
using depth_map_t   = std::array< depth_array_t, DEPTH_MAP_ROWS >;

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

inline void from_msg( const smap_interfaces::msg::DepthMap& msg, depth_map_t& map )
{
    for( int row = 0; row < msg.height; row++ )
    {
        for( int col = 0; col < msg.width; col++ )
        {
            for( int lims = 0; lims < DEPTH_MAP_FIELDS; lims++ )
            {
                map[ row ][ col ][ lims ] =
                    msg.map[ row * msg.width * DEPTH_MAP_FIELDS + col * DEPTH_MAP_FIELDS + lims ];
                //
            }
        }
    }
}

inline void to_msg( const depth_map_t& map, smap_interfaces::msg::DepthMap& msg, const std::pair< int, int >& dims )
{
    //
    //
    msg.cell_height = dims.first;
    msg.cell_width  = dims.second;
    msg.height      = DEPTH_MAP_ROWS;
    msg.width       = DEPTH_MAP_COLS;
    for( int row = 0; row < DEPTH_MAP_ROWS; row++ )
    {
        for( int col = 0; col < DEPTH_MAP_COLS; col++ )
        {
            for( int lims = 0; lims < DEPTH_MAP_FIELDS; lims++ )
            {
                msg.map[ row * DEPTH_MAP_COLS * DEPTH_MAP_FIELDS + col * DEPTH_MAP_FIELDS + lims ] =
                    map[ row ][ col ][ lims ];
                //
            }
        }
    }
}

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

inline void set_AABB(
    std::array< geometry_msgs::msg::Point, 8 >& AABB, const geometry_msgs::msg::Point& min,
    const geometry_msgs::msg::Point& max )
{
    // [0]
    AABB[ 0 ] = min;

    // [1]
    AABB[ 1 ].x = min.x;
    AABB[ 1 ].y = min.y;
    AABB[ 1 ].z = max.z;

    // [2]
    AABB[ 2 ].x = min.x;
    AABB[ 2 ].y = max.y;
    AABB[ 2 ].z = min.z;

    // [3]
    AABB[ 3 ].x = max.x;
    AABB[ 3 ].y = min.y;
    AABB[ 3 ].z = min.z;

    // [4]
    AABB[ 4 ].x = min.x;
    AABB[ 4 ].y = max.y;
    AABB[ 4 ].z = max.z;

    // [5]
    AABB[ 5 ].x = max.x;
    AABB[ 5 ].y = min.y;
    AABB[ 5 ].z = max.z;

    // [6]
    AABB[ 6 ].x = max.x;
    AABB[ 6 ].y = max.y;
    AABB[ 6 ].z = min.z;

    // [7]
    AABB[ 7 ] = max;
}

}  // namespace smap

#endif  // SMAP_CORE__OCCLUSION_MAP_HPP_
