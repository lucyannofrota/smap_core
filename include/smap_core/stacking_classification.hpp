#ifndef SMAP_CORE__STACKING_CLASSIFICATION_HPP_
#define SMAP_CORE__STACKING_CLASSIFICATION_HPP_

// STL
#include <map>
#include <vector>

// SMAP
#include "../../perception_server/detector_descriptor.hpp"
#include "visibility_control.h"

namespace smap
{

void stack_normalization( void ) {}

inline void stack_vectors(
    std::map< std::string, float >& current_likelihood, const std::vector< float >& new_vector, const detector_t& det )
{
    // current_likelihood - is a map containing a vector of probabilities that represents the probability of beeing each
    // 											class given the current observation
    (void) current_likelihood;
    (void) new_vector;
    (void) det;
    bool initializing = current_likelihood.size() == 0;
    if( current_likelihood.size() == 0 )
    {  // Initialization
        printf( "current_likelihood.size() == 0\n" );
        // for(auto c : )
    }

    for( auto c: det.classes )
        if( initializing ) current_likelihood[ c.second ] = new_vector[ c.first ];
        else
        {
            // TODO: Probability combination
            (void) current_likelihood;
            (void) new_vector;
            (void) det;
        }
    // for( auto c: det.classes )
    //     // (*this->reg_classes)[ c.second ]
    //     current printf(
    //         "\t[%2i] (%s) | [%2i,%2i]\n", c.first, c.second.c_str(), ( *this->reg_classes )[ c.second ].first,
    //         ( *this->reg_classes )[ c.second ].second );
}
}  // namespace smap

#endif  // SMAP_CORE__STACKING_CLASSIFICATION_HPP_
