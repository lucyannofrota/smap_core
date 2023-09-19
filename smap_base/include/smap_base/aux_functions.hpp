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
double rad2deg( double rad );

double deg2rad( double deg );

double log_odds( double prob );

double log_odds_inv( double lodds );

bool compare_str( const std::string str1, const std::string str2 );

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

}  // namespace smap

#endif  // SMAP_CORE__AUX_FUNCTIONS_HPP_
