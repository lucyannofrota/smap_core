#ifndef SMAP_CORE__OCCLUSION_MAP_HPP_
#define SMAP_CORE__OCCLUSION_MAP_HPP_

// STL
#include <array>

// ROS
#include <geometry_msgs/msg/point.hpp>

// SMAP
#include "../../../include/smap_core/macros.hpp"
#include "smap_interfaces/msg/occlusion_map.hpp"

namespace smap
{

using occlusion_cell_t  = std::array< geometry_msgs::msg::Point, 2 >;
using occlusion_array_t = std::array< occlusion_cell_t, OCCLUSION_MAP_COLS >;
using occlusion_map_t   = std::array< occlusion_array_t, OCCLUSION_MAP_ROWS >;

inline void from_msg( const smap_interfaces::msg::OcclusionMap& msg, occlusion_map_t& map )
{
    //
    //
    // msg.cell_height = dims.first;
    // msg.cell_width  = dims.second;
    // msg.height      = OCCLUSION_MAP_ROWS;
    // msg.height      = OCCLUSION_MAP_COLS;
    printf( "[w,h] [%i,%i]\n", msg.width, msg.height );
    for( int row = 0; row < msg.height; row++ )
    {
        for( int col = 0; col < msg.width; col++ )
        {
            for( int lims = 0; lims < 2; lims++ )
            {
                map[ row ][ col ][ lims ] = msg.map[ row * msg.width * 2 + col * 2 + lims ];
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
            for( int lims = 0; lims < 2; lims++ )
            {
                msg.map[ row * OCCLUSION_MAP_COLS * 2 + col * 2 + lims ] = map[ row ][ col ][ lims ];
                //
            }
        }
    }
    // for()
    // std::copy( &map[ 0 ], &map[ OCCLUSION_MAP_ROWS ], std::back_inserter( msg.map ) );
}

}  // namespace smap

#endif  // SMAP_CORE__OCCLUSION_MAP_HPP_
