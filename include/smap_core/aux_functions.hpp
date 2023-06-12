#ifndef SMAP_CORE__AUX_FUNCTIONS_HPP_
#define SMAP_CORE__AUX_FUNCTIONS_HPP_

#include "../../pch/pch.hpp"
#include "visibility_control.h"

namespace smap
{
inline double rad2deg( double rad ) { return rad * ( 180 / M_PI ); }

inline double deg2rad( double deg ) { return deg * ( M_PI / 180 ); }

inline double log_odds( double prob ) { return log( prob / ( 1 - prob ) ); }

inline double log_odds_inv( double lodds ) { return 1 - 1 / ( 1 + exp( lodds ) ); }
}  // namespace smap

#endif  // SMAP_CORE__AUX_FUNCTIONS_HPP_
