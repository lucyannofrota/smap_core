#ifndef TOPO_MAP__MACROS_HPP_
#define TOPO_MAP__MACROS_HPP_

// STL

#include <cmath>
#include <math.h>
#include <string>

namespace smap
{

#define R_TRIANGLES 0.025               // Size of the histogram triangles

#define HISTOGRAM_MARKER_ALPHA_LIM 0.6  // Alpha values under this value will be consider 0

#define DEFAULT_OUTPUT_PATH std::string( "src/smap/smap_core/maps/" )  // Output path of files

#define DEFAULT_VERTEX_DISTANCE 1.0                                    // Minimum distance to create a new vertex

#define DEFAULT_NEW_EDGE_FACTOR 0.95                                   // Must be < 1

#define DEFAULT_ACTIVE_FOV_H 60.0                                      // Horizontal active FOV

#define DEFAULT_OBJECT_ERROR_DISTANCE 0.2  // Max position error of an object and max distance between objects

#define DEFAULT_OBJECT_TRACKING_FACTOR ( 1.0 / 2.0 )

#define DEFAULT_OCCLUSION_OBJECT_DISTANCE_TOLERANCE_FACTOR 0.4

#define DEFAULT_OCCLUSION_OBJECT_DISTANCE_TOLERANCE_MAX 0.25

#define DEFAULT_OCCLUSION_MAX_PERCENTAGE                                                                               \
    0.4  // The max percentage of cells to be considerate as an occlusion [OCCLUSION_MAX_PERCENTAGE*100 = %]

#define DEFAULT_OBJECT_PROB_DECAY                                                                                      \
    0.5  // It should be 0 <= value < 1. The decay will vary between OBJECT_PROB_DECAY and 2xOBJECT_PROB_DECAY

#define DEFAULT_OCCLUSION_DECAY_PENALTY 1.0 / 24.0

}  // namespace smap

#endif  // TOPO_MAP__MACROS_HPP_
