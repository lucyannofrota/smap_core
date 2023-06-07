#ifndef SMAP_CORE__MACROS_HPP_
#define SMAP_CORE__MACROS_HPP_

#include "visibility_control.h"

#include <string.h>

namespace smap
{

#define OUTPUT_PATH std::string( "src/smap/smap_core/maps/" )  // Output path of files

#define DEFAULT_FILE_NAME "Sem_Map.smp"

#define SAVE_LOAD_PATH std::string( "src/smap/smap_core/maps/" )  // Save/Load path

#define VERTEX_DISTANCE 1                                         // Minimum distance to create a new vertex

#define NEW_EDGE_FACTOR 0.95                                      // Must be < 1

#define ACTIVE_FOV_H 100                                          // Horizontal active FOV

#define OBJECT_ERROR_DISTANCE 0.1  // Max position error of an object and max distance between objects

#define UNDEFINED_LABEL std::string( "unknown" )

}  // namespace smap

#endif  // SMAP_CORE__MACROS_HPP_
