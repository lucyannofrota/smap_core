cmake_minimum_required(VERSION 3.5)
project(topo_map)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
    # ament
        find_package(ament_cmake REQUIRED)
        find_package(ament_cmake_ros REQUIRED)
    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(visualization_msgs REQUIRED)
        find_package(tf2 REQUIRED)
        find_package(tf2_geometry_msgs REQUIRED)
        find_package(nav_msgs REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)
        find_package(smap_base REQUIRED)
        find_package(thing REQUIRED)
        find_package(map_exporter REQUIRED)

###################
## ADD BOOST LIB ##
###################

find_package(Boost REQUIRED system serialization) 
set(Boost_USE_MULTITHREADED ON)  



  # if(Boost_FOUND)
  #   include_directories(${Boost_INCLUDE_DIRS}) 
	# endif()

####################
## /ADD BOOST LIB ##
####################

set(DEPENDENCIES
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    smap_interfaces
    smap_base
    thing
    map_exporter
    Boost
)

#################
## topo_marker ##
#################

  ## Lib ##
  add_library(topo_marker
    include/topo_map/topo_marker.hpp
    topo_marker.cpp
  )

  target_link_libraries(topo_marker 
    thing::thing
    map_exporter::map_exporter
  )

target_include_directories(topo_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
    #   ${Boost_INCLUDE_DIRS}
  )

ament_target_dependencies(
    topo_marker
    visualization_msgs
    # geometry_msgs
    # smap_interfaces
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(topo_marker PRIVATE "SMAP_BUILDING_LIBRARY")

install(
  TARGETS topo_marker
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## /Lib ##

##################
## /topo_marker ##
##################



##############
## topo_map ##
##############

include_directories(include/topo_map)

  ## Lib ##
  add_library(topo_map
    include/topo_map/topo_map.hpp
    topo_map.cpp
    include/topo_map/label_writers.hpp
  )

  target_include_directories(topo_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
      ${Boost_INCLUDE_DIRS}
  )
#   pcl_processing
  target_link_libraries(topo_map
    thing::thing
    smap_base::smap_base
    map_exporter::map_exporter
    topo_marker
    ${Boost_LIBRARIES}
)

ament_target_dependencies(
  topo_map
  ${DEPENDENCIES}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(topo_map PRIVATE "SMAP_BUILDING_LIBRARY")

install(
    TARGETS topo_map
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
    
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

install(
  DIRECTORY include/topo_map
  DESTINATION include
)

install(
  TARGETS topo_map
  EXPORT topo_map
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

## /Lib ##

###############
## /topo_map ##
###############

ament_package()