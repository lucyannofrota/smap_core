cmake_minimum_required(VERSION 3.5)
project(smap_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(PCL 1.1 REQUIRED)

add_library(${PROJECT_NAME} 
    include/smap_base/aux_functions.hpp
    include/smap_base/count_time.hpp
    include/smap_base/detector_descriptor.hpp
    include/smap_base/interface_templates.hpp
    include/smap_base/macros.hpp
    include/smap_base/stacking_classification.hpp
    include/smap_base/visibility_control.h
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

include_directories(
  include/smap_base
  ${colcon_INCLUDE_DIRS}
)


install(
  DIRECTORY include/
  DESTINATION include
)

file(MAKE_DIRECTORY 
    ../SemMap 
    ../maps 
    ../timers 
    ../timers/object_estimator
    ../timers/topo_map
)

install(DIRECTORY
  ../launch
  ../scripts
  ../config
  ../SemMap
  ../maps
  ../timers
  ../timers/object_estimator
  ../timers/topo_map
  DESTINATION share/${PROJECT_NAME}/
)

# install(
#   DIRECTORY include/smap_base
#   DESTINATION include
# )

target_include_directories(smap_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexp<depend>smap_interfaces</depend>ort rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(smap_base PRIVATE "SMAP_BUILDING_LIBRARY")

include_directories(include/smap_base)

install(
  TARGETS smap_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/smap_base
  DESTINATION include
)

install(
  TARGETS smap_base
  EXPORT smap_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
