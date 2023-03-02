#ifndef SMAP__VISIBILITY_CONTROL_H_
#define SMAP__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define smap_EXPORT __attribute__ ((dllexport))
    #define smap_IMPORT __attribute__ ((dllimport))
  #else
    #define smap_EXPORT __declspec(dllexport)
    #define smap_IMPORT __declspec(dllimport)
  #endif
  #ifdef smap_BUILDING_LIBRARY
    #define smap_PUBLIC smap_EXPORT
  #else
    #define smap_PUBLIC smap_IMPORT
  #endif
  #define smap_PUBLIC_TYPE smap_PUBLIC
  #define smap_LOCAL
#else
  #define smap_EXPORT __attribute__ ((visibility("default")))
  #define smap_IMPORT
  #if __GNUC__ >= 4
    #define smap_PUBLIC __attribute__ ((visibility("default")))
    #define smap_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define smap_PUBLIC
    #define smap_LOCAL
  #endif
  #define smap_PUBLIC_TYPE
#endif

#endif  // SMAP__VISIBILITY_CONTROL_H_
