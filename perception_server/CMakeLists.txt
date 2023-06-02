cmake_minimum_required(VERSION 3.5)
project(smap_core)

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
    # ament
        find_package(ament_cmake REQUIRED)
        find_package(ament_cmake_ros REQUIRED)
    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        # find_package(std_msgs REQUIRED)
        # find_package(sensor_msgs REQUIRED)
        # find_package(geometry_msgs REQUIRED)
        find_package(visualization_msgs REQUIRED)
        # find_package(std_srvs REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)


#######################
## perception_server ##
#######################

## Lib ##

  add_library(
    perception_server
    perception_server.cpp
  )

  target_include_directories(perception_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(
    perception_server
    rclcpp
    smap_interfaces
    visualization_msgs
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
