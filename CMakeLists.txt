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
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smap_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL 1.1 REQUIRED)

###################
## ADD BOOST LIB ##
###################

#   set(Boost_USE_MULTITHREADED ON)  
#   find_package(Boost REQUIRED system serialization) 



  # if(Boost_FOUND)
  #   include_directories(${Boost_INCLUDE_DIRS}) 
	# endif()

####################
## /ADD BOOST LIB ##
####################

include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)




add_subdirectory(src/pcl_processing)
add_subdirectory(src/object_estimator)
add_subdirectory(src/thing)
add_subdirectory(src/perception_server)
add_subdirectory(src/topo_map)


install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  scripts
  config
  # outputs
  SemMap
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
)



ament_target_dependencies(
  smap_node
  rclcpp
  smap_interfaces
)


target_link_libraries(
  smap_node
  thing
  topo_map
  object_estimator
  perception_server
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
  topo_map
#   object_estimator
  perception_server
)


##################
## python nodes ##
##################

  # install(PROGRAMS
  #   src/perception_server_node.py
  #   DESTINATION lib/${PROJECT_NAME}
  # )

###################
## /python nodes ##
###################



ament_package()
