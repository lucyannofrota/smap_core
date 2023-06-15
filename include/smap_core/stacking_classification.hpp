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
    double sum = 0, bef;
    int i      = 0;
    printf( "Normalization: \n" );
    for( auto x: prob_map ) sum += log_odds_inv( x.second );
    for( auto x: prob_map )
    {
        bef      = x.second;
        x.second = log_odds( x.second / sum );
        if( x.second > 0.001 ) printf( "[%i]: %f|%f\n", i, bef, x.second );
        i++;
    }
}

inline void stack_vectors(
    std::map< std::string, float >& current_likelihood, const std::vector< float >& new_vector, const detector_t& det )
{
    // current_likelihood - is a map containing a vector of probabilities that represents the probability of beeing each
    // 											class given the current observation
    // 1. Probability combination
    int i   = 0;
    auto it = new_vector.begin();
    for( i = 0; it != new_vector.end(); ++it, i++ ) current_likelihood[ det.classes.at( i ) ] += log_odds( *it );
    // stack_normalization( current_likelihood );  // TODO: Test the influence of the normalization
}
}  // namespace smap

#endif  // SMAP_CORE__STACKING_CLASSIFICATION_HPP_
