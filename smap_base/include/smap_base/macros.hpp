#ifndef SMAP_BASE__MACROS_HPP_
#define SMAP_BASE__MACROS_HPP_

// STL
// #include "visibility_control.h"

#include <cmath>
#include <math.h>
#include <string>

namespace smap
{

#define UNDEFINED_LABEL std::string( "unknown" )

#define LOG_ODDS_CLAMPING 10  // log_odds(0.9999) == -log_odds(0.0001) == 9.2102

#define DEFAULT_CONFIDENCE_OBJECT_VALID 0.5

#define DEFAULT_OBJECT_MIN_DIST 0.2

#define DEFAULT_OBJECT_MAX_DIST 1.7

}  // namespace smap

#endif  // SMAP_BASE__MACROS_HPP_
