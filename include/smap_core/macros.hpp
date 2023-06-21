#ifndef SMAP_CORE__MACROS_HPP_
#define SMAP_CORE__MACROS_HPP_

// STL
#include "visibility_control.h"

#include <string>

namespace smap
{

#define DEBUG_MODE 1

#define OUTPUT_PATH std::string( "src/smap/smap_core/maps/" )  // Output path of files

#define DEFAULT_FILE_NAME "Sem_Map.smp"

#define SAVE_LOAD_PATH std::string( "src/smap/smap_core/maps/" )  // Save/Load path

#define VERTEX_DISTANCE 1                                         // Minimum distance to create a new vertex

#define NEW_EDGE_FACTOR 0.95                                      // Must be < 1

#define ACTIVE_FOV_H 100                                          // Horizontal active FOV

#define OBJECT_ERROR_DISTANCE 0.2  // Max position error of an object and max distance between objects

#define OBJECT_TRACKING_FACTOR 1 / 2

#define OBJECT_TRACKING_TOLERANCE OBJECT_ERROR_DISTANCE*OBJECT_TRACKING_FACTOR

#define R_TRIANGLES 0.025               // Size of the histogram triangles

#define HISTOGRAM_BINS 36               // Number of bins of the polar histogram

#define HISTOGRAM_MARKER_ALPHA_LIM 0.6  // Alpha values under this value will be consider 0

#define UNDEFINED_LABEL std::string( "unknown" )

#define MAX_POS_PROB 0.85

#define OBJECT_SIZE_LIM_CONF 0.8  // Objects with size under this value will suffer no confidence penalty

}  // namespace smap

#endif  // SMAP_CORE__MACROS_HPP_
