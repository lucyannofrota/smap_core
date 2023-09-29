cmake_minimum_required(VERSION 3.5)
project(map_exporter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
# endif()

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
    # ament
        find_package(ament_cmake REQUIRED)

    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        # find_package(tf2 REQUIRED)
        # find_package(tf2_geometry_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(nav_msgs REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)
        find_package(object_estimator REQUIRED)
        # find_package(pcl_processing REQUIRED)
        find_package(smap_base REQUIRED)
        find_package(perception_server REQUIRED)
        # find_package(topo_map REQUIRED)
        find_package(thing REQUIRED)


###################
## ADD BOOST LIB ##
###################

find_package(Boost REQUIRED system serialization) 
set(Boost_USE_MULTITHREADED ON)  

####################
## /ADD BOOST LIB ##
####################

##################
## map_exporter ##
##################

include_directories(include/map_exporter)

  ## Lib ##
  add_library(map_exporter
    include/map_exporter/map_exporter.hpp
    map_exporter.cpp
  )

  target_include_directories(map_exporter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
      ${Boost_INCLUDE_DIRS}
  )
#   pcl_processing
#   target_link_libraries(map_exporter
#     # thing::thing
#     # smap_base::smap_base
#     # pcl_processing::pcl_processing
#     # topo_marker
#     # ${Boost_LIBRARIES}
# )

target_link_libraries(map_exporter 
#   thing::thing
  smap_base::smap_base
    # topo_map::topo_map
)

ament_target_dependencies(
  map_exporter
#   geometry_msgs
#   smap_interfaces
#   visualization_msgs
  nav_msgs
#   std_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(map_exporter PRIVATE "SMAP_BUILDING_LIBRARY")

install(
    TARGETS map_exporter
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
    
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/map_exporter
  DESTINATION include
)

install(
  TARGETS map_exporter
  EXPORT map_exporter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

## /Lib ##

###############
## /map_exporter ##
###############

ament_package()