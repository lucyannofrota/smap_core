cmake_minimum_required(VERSION 3.5)
project(thing)


# find dependencies
    # ament
        find_package(ament_cmake REQUIRED)
    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(tf2 REQUIRED)
        find_package(tf2_geometry_msgs REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)


###################
## ADD BOOST LIB ##
###################

find_package(Boost REQUIRED system serialization) 
set(Boost_USE_MULTITHREADED ON)  


####################
## /ADD BOOST LIB ##
####################

# ###########
# ## thing ##
# ###########

# add_library(thing thing.cpp)
# target_include_directories(thing PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(
#   thing
#   rclcpp
#   geometry_msgs
#   tf2
#   tf2_geometry_msgs
#   sensor_msgs
#   smap_interfaces
# )

# # Causes the visibility macros to use dllexp<depend>smap_interfaces</depend>ort rather than dllimport,
# # which is appropriate when building the dll but not consuming it.
# target_compile_definitions(thing PRIVATE "SMAP_BUILDING_LIBRARY")

# install(
#   TARGETS thing
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ############
# ## /thing ##
# ############

###########################
## observation_histogram ##
###########################

add_library(observation_histogram observation_histogram.cpp)
target_include_directories(observation_histogram PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(
#   observation_histogram 
# )

# Causes the visibility macros to use dllexp<depend>smap_interfaces</depend>ort rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(observation_histogram PRIVATE "SMAP_BUILDING_LIBRARY")

install(
  TARGETS observation_histogram
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

############################
## /observation_histogram ##
############################

###########
## thing ##
###########

add_library(thing include/thing/thing.hpp thing.cpp)
target_include_directories(thing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(
    thing
    # topo_map_node 
    # topo_marker
    observation_histogram
    # ${Boost_LIBRARIES}
)

ament_target_dependencies(
  thing
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  smap_interfaces
)

# Causes the visibility macros to use dllexp<depend>smap_interfaces</depend>ort rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(thing PRIVATE "SMAP_BUILDING_LIBRARY")

include_directories(include/thing)

install(
  TARGETS thing
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/thing
  DESTINATION include
)

install(
  TARGETS thing
  EXPORT thing
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

############
## /thing ##
############

##############
## topo_map ##
##############

#   ## Lib ##
#   add_library(topo_map topo_map.cpp )
#   target_link_libraries(topo_map thing topo_marker ${Boost_LIBRARIES})
# target_include_directories(topo_map PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#       ${Boost_INCLUDE_DIRS}
#   )

# ament_target_dependencies(
#   topo_map
#   geometry_msgs
#   smap_interfaces
#   visualization_msgs
# )

# # Causes the visibility macros to use dllexport rather than dllimport,
# # which is appropriate when building the dll but not consuming it.
# target_compile_definitions(topo_map PRIVATE "SMAP_BUILDING_LIBRARY")

# install(
#   TARGETS topo_map
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ## /Lib ##

# ## Node ##

#   add_executable(
#       topo_map_node
#       topo_map.cpp
#   )

#   target_include_directories(topo_map_node PUBLIC
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<INSTALL_INTERFACE:include>
#         ${Boost_INCLUDE_DIRS}   
#   )

#   ament_target_dependencies(
#       topo_map_node
#       rclcpp
#       geometry_msgs
#       smap_interfaces
#       visualization_msgs
#   )

#   install(TARGETS topo_map_node
#       DESTINATION lib/${PROJECT_NAME}
#   )

## Node ##

###############
## /topo_map ##
###############

ament_package()