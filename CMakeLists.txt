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
# find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system serialization)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

##############
## Detector ##
##############

add_library(detector src/detector.cpp)
target_include_directories(detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  detector
  rclcpp
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
  rclcpp
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
  visualization_msgs
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


###############
## Msgs/Srvs ##
###############

include_directories(include)
# set(message_files
#   "msg/FieldsWithSameType.msg"
#   "msg/UInt32.msg"
# )
set(service_files
  "srv/AddThreeInts.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ${message_files}
  ${service_files}
)

ament_export_dependencies(rosidl_default_runtime)

###############
## Msgs/Srvs ##
###############


install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/
)



add_executable(semantic_mapping_node src/semantic_mapping_node.cpp)

#######################
## Msgs/Srvs inlcude ##
#######################

get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(semantic_mapping_node
      ${PROJECT_NAME} ${typesupport_impl}
    )
  endforeach()
endforeach()

#######################
## Msgs/Srvs inlcude ##
#######################

target_include_directories(semantic_mapping_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIR})

ament_target_dependencies(
  semantic_mapping_node
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
  std_srvs
)
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
