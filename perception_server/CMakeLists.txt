cmake_minimum_required(VERSION 3.5)
project(perception_server)

# find dependencies
    # ament
        find_package(ament_cmake REQUIRED)
        # find_package(ament_cmake_ros REQUIRED)
    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        # find_package(std_msgs REQUIRED)
        # find_package(sensor_msgs REQUIRED)
        # find_package(geometry_msgs REQUIRED)
        # find_package(std_srvs REQUIRED)
        find_package(visualization_msgs REQUIRED)
        find_package(tf2 REQUIRED)
        find_package(tf2_geometry_msgs REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)
        find_package(smap_base REQUIRED)
        find_package(topo_map REQUIRED)
        find_package(thing REQUIRED)

        find_package(Boost REQUIRED system serialization) 


#######################
## perception_server ##
#######################

## Lib ##

  add_library(
    perception_server
    perception_server.cpp
  )

  target_link_libraries(perception_server
    smap_base::smap_base
    topo_map::topo_map
    thing::thing
  )

  target_include_directories(perception_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(
    perception_server
    rclcpp
    smap_interfaces
    # visualization_msgs
  )

  install(TARGETS perception_server
    DESTINATION lib
  )

## /Lib ##

## Node ##

  add_executable(
    perception_server_node
    perception_server.cpp
  )

  target_link_libraries(perception_server_node smap_base::smap_base topo_map::topo_map)

  target_include_directories(perception_server_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(
    perception_server_node
    rclcpp
    smap_interfaces
    visualization_msgs
  )

  install(TARGETS perception_server_node
    DESTINATION lib
  )

## /Node ##

########################
## /perception_server ##
########################

ament_package()
