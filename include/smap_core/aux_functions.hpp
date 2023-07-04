#ifndef SMAP_CORE__AUX_FUNCTIONS_HPP_
#define SMAP_CORE__AUX_FUNCTIONS_HPP_

// STL
#include <math.h>
#include <vector>

// ROS
#include "visibility_control.h"

// #include <std_msgs/msg/float32_multi_array.hpp>
// #include <std_msgs/msg/multi_array_dimension.hpp>
// SMAP

#include "macros.hpp"

namespace smap
{
// TODO: convert all to inline
double rad2deg( double rad ) { return rad * ( 180 / M_PI ); }

double deg2rad( double deg ) { return deg * ( M_PI / 180 ); }

double log_odds( double prob )
{
    assert( ( prob < 1 ) && ( prob > 0 ) );
    return log( prob / ( 1 - prob ) );
}

double log_odds_inv( double lodds )
{
    assert( ( lodds < LOG_ODDS_CLAMPING ) && ( lodds > -LOG_ODDS_CLAMPING ) );
    return ( 1 - 1 / ( 1 + exp( lodds ) ) );
}

// inline float& occlusion_matrix_indexer(
//     std_msgs::msg::Float32MultiArray& occ_mat, const size_t& r, const size_t& c, const size_t& lims,
//     const size_t& comp )
// {
//     return occ_mat.data
//         [ occ_mat.layout.dim[ 0 ].stride * r + occ_mat.layout.dim[ 1 ].stride * c
//           + occ_mat.layout.dim[ 2 ].stride * lims + comp ];
// }
}  // namespace smap

#endif  // SMAP_CORE__AUX_FUNCTIONS_HPP_
