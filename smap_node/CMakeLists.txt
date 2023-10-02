cmake_minimum_required(VERSION 3.5)
project(smap_node)

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
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smap_interfaces REQUIRED)
find_package(object_estimator REQUIRED)
find_package(smap_base REQUIRED)
# find_package(pcl_processing REQUIRED)
find_package(perception_server REQUIRED)
find_package(topo_map REQUIRED)
find_package(thing REQUIRED)
find_package(map_exporter REQUIRED)

find_package(Boost REQUIRED system serialization) 

# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(PCL 1.1 REQUIRED)

include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)




# add_subdirectory(src/pcl_processing)
# add_subdirectory(src/object_estimator)
# add_subdirectory(src/thing)
# add_subdirectory(src/perception_server)
# add_subdirectory(src/topo_map)


# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# file(MAKE_DIRECTORY 
#     ../maps 
#     ../timers 
#     ../timers/object_estimator
#     ../timers/topo_map
# )

# install(DIRECTORY
#   ../launch
#   ../scripts
#   ../config
#   ../maps
#   ../timers
#   ../timers/object_estimator
#   ../timers/topo_map
#   DESTINATION share/${PROJECT_NAME}/
# )


add_executable(smap_node smap_node.cpp)
# add_executable(smap_node_headless smap_node_headless.cpp)




target_include_directories(smap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



ament_target_dependencies(
  smap_node
  rclcpp
  smap_interfaces
)

# ament_target_dependencies(
#   smap_node_headless
#   rclcpp
#   smap_interfaces
# )


target_link_libraries(
  smap_node
  smap_base::smap_base
  object_estimator::object_estimator
  pcl_processing::pcl_processing
  perception_server::perception_server
  topo_map::topo_map
  thing::thing
  map_exporter::map_exporter
)

# target_link_libraries(
#     smap_node_headless
#   smap_base::smap_base
#   object_estimator::object_estimator
#   pcl_processing::pcl_processing
#   perception_server::perception_server
#   topo_map::topo_map
#   thing::thing
# )

install(TARGETS smap_node
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS smap_node_headless
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

install(DIRECTORY
  ../launch
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
