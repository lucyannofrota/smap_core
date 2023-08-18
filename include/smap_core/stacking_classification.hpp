#ifndef SMAP_CORE__STACKING_CLASSIFICATION_HPP_
#define SMAP_CORE__STACKING_CLASSIFICATION_HPP_

// STL
#include <map>
#include <string>
#include <vector>

// SMAP
#include "../../src/perception_server/detector_descriptor.hpp"
#include "aux_functions.hpp"
#include "visibility_control.h"

namespace smap
{

inline void stack_normalization( std::map< std::string, float >& prob_map )
{
    // XXX: If another type of normalization is needed. Changes to initialization and updating of vectors will be
    // required

    // Min-max feature scaling:
    //														X' = (X - X_min)/(X_max - X_min)
    float sum = 0, aux = 0 /*, bef*/;
    // printf( "Normalization: \n" );
    for( const auto& x: prob_map ) sum += log_odds_inv( x.second );
    for( auto& x: prob_map )
    {
        if( log_odds_inv( x.second ) > 0.02 ) x.second = log_odds( log_odds_inv( x.second ) / sum );
        else x.second = -LOG_ODDS_CLAMPING;
        aux += x.second;
        // x.second =
        //     log_odds( ( ( log_odds_inv( x.second ) > 0.02 ) ? ( log_odds_inv( x.second ) > 0.02 ) : 0.5 ) / sum );
    }
    assert( log_odds_inv( aux ) <= 1.1 );
}

inline void stack_vectors(
    std::map< std::string, float >& current_likelihood, const std::vector< float >& new_vector, const detector_t& det )
{
    // current_likelihood - is a map containing a vector of probabilities that represents the probability of beeing each
    // 											class given the current observation
    // 1. Probability combination
    int i           = 0;
    auto it         = new_vector.begin();
    float max       = 0;
    float new_value = 0;
    // int idx_max = 0;
    for( i = 0; it != new_vector.end(); ++it, i++ )
    {
        if( *it > max )
        {
            // idx_max = i;
            max = *it;
        }

        new_value = current_likelihood[ det.classes.at( i ) ] + log_odds( *it );

        // Clamping
        assert( !( std::isnan( new_value ) || std::isinf( new_value ) ) );
        // if(std::isnan(new_value) || std::isinf(new_value))
        if( new_value > LOG_ODDS_CLAMPING ) new_value = LOG_ODDS_CLAMPING;
        if( new_value < -LOG_ODDS_CLAMPING ) new_value = -LOG_ODDS_CLAMPING;

        current_likelihood[ det.classes.at( i ) ] = new_value;
    }
    stack_normalization( current_likelihood );
}
}  // namespace smap

#endif  // SMAP_CORE__STACKING_CLASSIFICATION_HPP_
