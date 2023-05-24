#ifndef SMAP_CORE__MACROS_HPP_
#define SMAP_CORE__MACROS_HPP_

#include "visibility_control.h"

#include <string.h>

namespace smap
{


  #define OUTPUT_PATH std::string("src/smap/smap_core/maps/") // Output path of files

  #define DEFAULT_FILE_NAME "Sem_Map.smp"

  #define SAVE_LOAD_PATH std::string("src/smap/smap_core/maps/") // Save/Load path

  #define VERTEX_DISTANCE 0.7 // Minimun distance to create a new vertece

  #define NEW_EDGE_FACTOR 1.15 // Must be < 1


}  // namespace smap

#endif  // SMAP_CORE__MACROS_HPP_
