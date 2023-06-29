#ifndef SMAP_CORE__OCCLUSION_MAP_HPP_
#define SMAP_CORE__OCCLUSION_MAP_HPP_

// STL
#include <array>

// ROS
#include <geometry_msgs/msg/point.hpp>

// SMAP
#include "../../../include/smap_core/macros.hpp"
#include "smap_interfaces/msg/occlusion_map.hpp"

#define OCCLUSION_MAP_FIELDS 3

namespace smap
{

using occlusion_cell_t  = std::array< geometry_msgs::msg::Point, 3 >;
using occlusion_array_t = std::array< occlusion_cell_t, OCCLUSION_MAP_COLS >;
using occlusion_map_t   = std::array< occlusion_array_t, OCCLUSION_MAP_ROWS >;

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

inline void from_msg( const smap_interfaces::msg::OcclusionMap& msg, occlusion_map_t& map )
{
    for( int row = 0; row < msg.height; row++ )
    {
        for( int col = 0; col < msg.width; col++ )
        {
            for( int lims = 0; lims < OCCLUSION_MAP_FIELDS; lims++ )
            {
                map[ row ][ col ][ lims ] =
                    msg.map[ row * msg.width * OCCLUSION_MAP_FIELDS + col * OCCLUSION_MAP_FIELDS + lims ];
                //
            }
        }
    }
}

inline void to_msg(
    const occlusion_map_t& map, smap_interfaces::msg::OcclusionMap& msg, const std::pair< int, int >& dims )
{
    //
    //
    msg.cell_height = dims.first;
    msg.cell_width  = dims.second;
    msg.height      = OCCLUSION_MAP_ROWS;
    msg.width       = OCCLUSION_MAP_COLS;
    for( int row = 0; row < OCCLUSION_MAP_ROWS; row++ )
    {
        for( int col = 0; col < OCCLUSION_MAP_COLS; col++ )
        {
            for( int lims = 0; lims < OCCLUSION_MAP_FIELDS; lims++ )
            {
                msg.map[ row * OCCLUSION_MAP_COLS * OCCLUSION_MAP_FIELDS + col * OCCLUSION_MAP_FIELDS + lims ] =
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
