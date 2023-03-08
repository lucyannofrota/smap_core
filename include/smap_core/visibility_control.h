#ifndef SMAP_CORE__VISIBILITY_CONTROL_H_
#define SMAP_CORE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SMAP_EXPORT __attribute__ ((dllexport))
    #define SMAP_IMPORT __attribute__ ((dllimport))
  #else
    #define SMAP_EXPORT __declspec(dllexport)
    #define SMAP_IMPORT __declspec(dllimport)
  #endif
  #ifdef SMAP_BUILDING_LIBRARY
    #define SMAP_PUBLIC SMAP_EXPORT
  #else
    #define SMAP_PUBLIC SMAP_IMPORT
  #endif
  #define SMAP_PUBLIC_TYPE SMAP_PUBLIC
  #define SMAP_LOCAL
#else
  #define SMAP_EXPORT __attribute__ ((visibility("default")))
  #define SMAP_IMPORT
  #if __GNUC__ >= 4
    #define SMAP_PUBLIC __attribute__ ((visibility("default")))
    #define SMAP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SMAP_PUBLIC
    #define SMAP_LOCAL
  #endif
  #define SMAP_PUBLIC_TYPE
#endif

#endif  // SMAP_CORE__VISIBILITY_CONTROL_H_
