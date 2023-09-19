#ifndef SMAP_CORE__AUX_FUNCTIONS_HPP_
#define SMAP_CORE__AUX_FUNCTIONS_HPP_

// STL
#include <math.h>
#include <string>
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
    // assert( ( prob <= 1 ) && ( prob >= 0 ) );
    double ret = log( prob / ( 1 - prob ) );
    if( ret < -LOG_ODDS_CLAMPING ) ret = -LOG_ODDS_CLAMPING;
    if( ret > LOG_ODDS_CLAMPING ) ret = LOG_ODDS_CLAMPING;
    return ret;
}

double log_odds_inv( double lodds )
{
    double ret = ( 1 - 1 / ( 1 + exp( lodds ) ) );
    if( ret < 0 ) ret = 0;
    if( ret > 1 ) ret = 1;
    return ret;
}

bool compare_str( const std::string str1, const std::string str2 ) { return str1 == str2; }

template< typename T >
T clamping_log_odds_sum( const T likelihood, const T& p_value )
{
    T new_likelihood = likelihood;
    if( ( p_value <= 0 ) || ( p_value <= log_odds_inv( -LOG_ODDS_CLAMPING ) )
        || ( ( new_likelihood + log_odds( p_value ) ) < -LOG_ODDS_CLAMPING ) )
        new_likelihood = -LOG_ODDS_CLAMPING;
    else
    {
        //
        if( p_value >= 1 || p_value >= log_odds_inv( +LOG_ODDS_CLAMPING )
            || ( ( new_likelihood + log_odds( p_value ) ) > +LOG_ODDS_CLAMPING ) )
            new_likelihood = +LOG_ODDS_CLAMPING;

        else new_likelihood += log_odds( p_value );
    }
    assert( new_likelihood >= -LOG_ODDS_CLAMPING && new_likelihood <= LOG_ODDS_CLAMPING );
    assert( ( !std::isnan( new_likelihood ) ) && !( std::isinf( new_likelihood ) ) );
    return new_likelihood;
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
