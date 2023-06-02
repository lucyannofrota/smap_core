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


if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
    # PCL
        find_package(PCL 1.1 REQUIRED)
        find_package(pcl_conversions REQUIRED)
    # ament
        find_package(ament_cmake REQUIRED)
        find_package(ament_cmake_ros REQUIRED)
    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(tf2 REQUIRED)
        find_package(tf2_ros REQUIRED)
        find_package(visualization_msgs REQUIRED)
        # find_package(std_srvs REQUIRED)
        # find_package(rmw_implementation_cmake REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)


######################
## PCL DEPENDENCIES ##
######################

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

#######################
## /PCL DEPENDENCIES ##
#######################

######################
## object_estimator ##
######################

  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(tf2_sensor_msgs REQUIRED)

  # Dear ImGui
  set(IMGUI_DIR imgui)
  find_package(SDL2 REQUIRED) # imgui

  file(GLOB sources *.cpp)
  include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends ..)

  set(IMGUI_SRC
    ${IMGUI_DIR}/imgui.cpp #imgui
    ${IMGUI_DIR}/imgui_demo.cpp #imgui
    ${IMGUI_DIR}/imgui_draw.cpp #imgui
    ${IMGUI_DIR}/imgui_tables.cpp #imgui
    ${IMGUI_DIR}/imgui_widgets.cpp #imgui
    ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp #imgui
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp #imgui
  )

  ## Lib ##

    add_library(
      object_estimator
      object_estimator.cpp
      ${IMGUI_SRC}
    )

    target_include_directories(object_estimator PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${SDL2_INCLUDE_DIRS} # imgui
    )

    target_link_libraries(object_estimator 
      ${PCL_LIBRARIES} 
      GL # imgui
      ${SDL2_LIBRARIES} # imgui
    )

    ament_target_dependencies(
      object_estimator
      rclcpp
      PCL
      pcl_conversions
      smap_interfaces
      visualization_msgs
      tf2
      tf2_geometry_msgs
      tf2_sensor_msgs
    )

    install(TARGETS object_estimator
      DESTINATION lib
    )

  ## Lib ##

  ## Node ##

    add_executable(
      object_estimator_node
      object_estimator.cpp
      ${IMGUI_SRC}
    )

    target_include_directories(object_estimator_node PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${SDL2_INCLUDE_DIRS} # imgui
    )

    target_link_libraries(object_estimator_node 
      ${PCL_LIBRARIES} 
      GL # imgui
      ${SDL2_LIBRARIES} # imgui
    )

    ament_target_dependencies(
      object_estimator_node
      rclcpp
      PCL
      pcl_conversions
      smap_interfaces
      visualization_msgs
      tf2
      tf2_geometry_msgs
      tf2_sensor_msgs
    )

    install(TARGETS object_estimator_node
      DESTINATION lib/${PROJECT_NAME}
    )

  ## Node ##

#######################
## /object_estimator ##
#######################
