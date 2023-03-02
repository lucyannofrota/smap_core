#ifndef SMAP__MACROS_HPP_
#define SMAP__MACROS_HPP_

#include "visibility_control.h"

#include <string.h>

namespace smap
{


  #define OUTPUT_PATH std::string("src/smap/Outputs/") // Output path of files

  #define DEFAULT_FILE_NAME "Sem_Map.smp"

  #define SAVE_LOAD_PATH std::string("src/smap/SemMap/") // Save/Load path

  #define VERTEX_DISTANCE 0.5 // Minimun distance to create a new vertece

  #define NEW_EDGE_FACTOR 1.0 / 2.0


}  // namespace smap

#endif  // SMAP__MACROS_HPP_
