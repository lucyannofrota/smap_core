cmake_minimum_required(VERSION 3.5)
project(semantic_mapping)

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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system serialization)
find_package(geometry_msgs REQUIRED)

##############
## Detector ##
##############

add_library(detector src/detector.cpp)
target_include_directories(detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  detector
  # "rclcpp"
  # "rclpy"
  # "std_msgs"
  # "sensor_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(detector PRIVATE "SEMANTIC_MAPPING_BUILDING_LIBRARY")

install(
  TARGETS detector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

##############
## Detector ##
##############

#############
## Concept ##
#############

add_library(concept src/concept.cpp)
target_include_directories(concept PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  concept
  # "rclcpp"
  # "rclpy"
  # "std_msgs"
  # "sensor_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(concept PRIVATE "SEMANTIC_MAPPING_BUILDING_LIBRARY")

install(
  TARGETS concept
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#############
## Concept ##
#############

####################
## Conceptual_Map ##
####################

add_library(conceptual_map src/conceptual_map.cpp src/concept.cpp)
target_include_directories(conceptual_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  conceptual_map
  geometry_msgs
  # "rclcpp"
  # "rclpy"
  # "std_msgs"
  # "sensor_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(conceptual_map PRIVATE "SEMANTIC_MAPPING_BUILDING_LIBRARY")

install(
  TARGETS conceptual_map
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

####################
## Conceptual_Map ##
####################

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(semantic_mapping_node src/semantic_mapping_node.cpp)
# include_directories(${Boost_INCLUDE_DIR})
target_include_directories(semantic_mapping_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIR})
target_link_libraries(semantic_mapping_node detector concept conceptual_map ${Boost_LIBRARIES})

install(TARGETS semantic_mapping_node
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
  detector
  concept
  conceptual_map
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
