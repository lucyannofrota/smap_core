#ifndef OBJECT_ESTIMATOR__MACROS_HPP_
#define OBJECT_ESTIMATOR__MACROS_HPP_

// STL

#include <cmath>
#include <math.h>
#include <string>

namespace smap
{

#define OBJECT_SIZE_LIM_CONF 0.2  // Objects with size under this value will suffer no confidence penalty

#define DEFAULT_MAX_OCCLUSION_CELL_VOLUME 0.25

#define DEFAULT_MAX_OCCLUSION_CELL_VOLUME_FACTOR std::cbrt( DEFAULT_MAX_OCCLUSION_CELL_VOLUME )

#define DEBUG_MODE 1

}  // namespace smap

#endif  // OBJECT_ESTIMATOR__MACROS_HPP_
