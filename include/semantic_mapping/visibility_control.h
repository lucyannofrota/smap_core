#ifndef SEMANTIC_MAPPING__VISIBILITY_CONTROL_H_
#define SEMANTIC_MAPPING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SEMANTIC_MAPPING_EXPORT __attribute__ ((dllexport))
    #define SEMANTIC_MAPPING_IMPORT __attribute__ ((dllimport))
  #else
    #define SEMANTIC_MAPPING_EXPORT __declspec(dllexport)
    #define SEMANTIC_MAPPING_IMPORT __declspec(dllimport)
  #endif
  #ifdef SEMANTIC_MAPPING_BUILDING_LIBRARY
    #define SEMANTIC_MAPPING_PUBLIC SEMANTIC_MAPPING_EXPORT
  #else
    #define SEMANTIC_MAPPING_PUBLIC SEMANTIC_MAPPING_IMPORT
  #endif
  #define SEMANTIC_MAPPING_PUBLIC_TYPE SEMANTIC_MAPPING_PUBLIC
  #define SEMANTIC_MAPPING_LOCAL
#else
  #define SEMANTIC_MAPPING_EXPORT __attribute__ ((visibility("default")))
  #define SEMANTIC_MAPPING_IMPORT
  #if __GNUC__ >= 4
    #define SEMANTIC_MAPPING_PUBLIC __attribute__ ((visibility("default")))
    #define SEMANTIC_MAPPING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SEMANTIC_MAPPING_PUBLIC
    #define SEMANTIC_MAPPING_LOCAL
  #endif
  #define SEMANTIC_MAPPING_PUBLIC_TYPE
#endif

#endif  // SEMANTIC_MAPPING__VISIBILITY_CONTROL_H_
