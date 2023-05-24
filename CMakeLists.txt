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

# TODO: Refactor cmake_list

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(smap_interfaces REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)


#################
## ADD PCL LIB ##
#################

  cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

  find_package(PCL 1.1 REQUIRED)
  find_package(pcl_conversions REQUIRED)


  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

##################
## /ADD PCL LIB ##
##################

###################
## ADD BOOST LIB ##
###################

  set(Boost_USE_MULTITHREADED ON)  
  find_package(Boost REQUIRED system serialization) 

####################
## /ADD BOOST LIB ##
####################

###########
## thing ##
###########

  add_library(thing src/thing.cpp)
  target_include_directories(thing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(
    thing
    rclcpp
  )

  # Causes the visibility macros to use dllexp<depend>smap_interfaces</depend>ort rather than dllimport,
  # which is appropriate when building the dll but not consuming it.
  target_compile_definitions(thing PRIVATE "SMAP_BUILDING_LIBRARY")

  install(
    TARGETS thing
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

############
## /thing ##
############

#####################
## Topological_Map ##
#####################

  add_library(topological_map src/topological_map.cpp src/thing.cpp)
  target_include_directories(topological_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(
    topological_map
    geometry_msgs
    smap_interfaces
    visualization_msgs
  )

  # Causes the visibility macros to use dllexport rather than dllimport,
  # which is appropriate when building the dll but not consuming it.
  target_compile_definitions(topological_map PRIVATE "SMAP_BUILDING_LIBRARY")

  install(
    TARGETS topological_map
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

######################
## /Topological_Map ##
######################

###########################
## object_pose_estimator ##
###########################

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
      object_pose_estimator_lib
      src/object_pose_estimator.cpp
      ${IMGUI_SRC}
    )

    target_include_directories(object_pose_estimator_lib PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${SDL2_INCLUDE_DIRS} # imgui
    )

    target_link_libraries(object_pose_estimator_lib 
      ${PCL_LIBRARIES} 
      GL # imgui
      ${SDL2_LIBRARIES} # imgui
    )

    ament_target_dependencies(
      object_pose_estimator_lib
      rclcpp
      PCL
      pcl_conversions
      smap_interfaces
      visualization_msgs
      tf2
      tf2_geometry_msgs
      tf2_sensor_msgs
      rclcpp_components
    )

    install(TARGETS object_pose_estimator_lib
      DESTINATION lib
    )

    # rclcpp_components_register_node(
    #   object_pose_estimator_lib
    #   PLUGIN "smap::object_pose_estimator"
    #   EXECUTABLE object_pose_estimator_libonent
    # )

  ## Lib ##

  ## Node ##

    add_executable(
      object_pose_estimator_node
      src/object_pose_estimator.cpp
      ${IMGUI_SRC}
    )

    target_include_directories(object_pose_estimator_node PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${SDL2_INCLUDE_DIRS} # imgui
    )

    target_link_libraries(object_pose_estimator_node 
      ${PCL_LIBRARIES} 
      GL # imgui
      ${SDL2_LIBRARIES} # imgui
    )

    ament_target_dependencies(
      object_pose_estimator_node
      rclcpp
      PCL
      pcl_conversions
      smap_interfaces
      visualization_msgs
      tf2
      tf2_geometry_msgs
      tf2_sensor_msgs
      rclcpp_components
    )

    install(TARGETS object_pose_estimator_node
      DESTINATION lib/${PROJECT_NAME}
    )

  ## Node ##

############################
## /object_pose_estimator ##
############################

#######################
## perception_server ##
#######################

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(smap_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

## Lib ##

  add_library(
    perception_server_lib
    src/perception_server.cpp
  )

  target_include_directories(perception_server_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # target_link_libraries(perception_server_lib 
  #   ${PCL_LIBRARIES} 
  # )

  ament_target_dependencies(
    perception_server_lib
    rclcpp
    rclcpp_components
    smap_interfaces
    visualization_msgs
  )

  install(TARGETS perception_server_lib
    DESTINATION lib
  )

  # rclcpp_components_register_node(
  #   perception_server_lib
  #   PLUGIN "smap::perception_server"
  #   EXECUTABLE perception_server_libonent
  # )

## Lib ##

########################
## /perception_server ##
########################

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  scripts
  config
  outputs
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )


add_executable(smap_node src/smap_node.cpp)



target_include_directories(smap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIR}
)



ament_target_dependencies(
  smap_node
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
  smap_interfaces
  std_srvs
)


target_link_libraries(
  smap_node
  thing
  topological_map
  ${Boost_LIBRARIES}
  object_pose_estimator_lib
  perception_server_lib
)

install(TARGETS smap_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  thing
  topological_map
)


##################
## python nodes ##
##################

  install(PROGRAMS
    src/perception_server_node.py
    DESTINATION lib/${PROJECT_NAME}
  )

###################
## /python nodes ##
###################





############
## Launch ##
############

#install(PROGRAMS
#  launch/py_node_launch.py
#  DESTINATION lib/${PROJECT_NAME}
#)

############
## Launch ##
############



ament_package()
