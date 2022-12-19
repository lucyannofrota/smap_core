#ifndef SEMANTIC_MAPPING__MACROS_HPP_
#define SEMANTIC_MAPPING__MACROS_HPP_

#include "visibility_control.h"

#include <string.h>

namespace semantic_mapping
{


  #define OUTPUT_PATH std::string("src/semantic_mapping/Outputs/") // Output path of files

  #define DEFAULT_FILE_NAME "Sem_Map.smp"

  #define SAVE_LOAD_PATH std::string("src/semantic_mapping/SemMap/") // Save/Load path

  #define VERTEX_DISTANCE 2.5 // Minimun distance to create a new vertece



}  // namespace semantic_mapping

#endif  // SEMANTIC_MAPPING__MACROS_HPP_
