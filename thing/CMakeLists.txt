cmake_minimum_required(VERSION 3.5)
project(thing)

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
    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        find_package(geometry_msgs REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)
        find_package(smap_base REQUIRED)


###################
## ADD BOOST LIB ##
###################

find_package(Boost REQUIRED system serialization) 
set(Boost_USE_MULTITHREADED ON)  


####################
## /ADD BOOST LIB ##
####################

set(DEPENDENCIES
    rclcpp
    geometry_msgs
    sensor_msgs
    smap_interfaces
)


include_directories(include/thing)

###########################
## observation_histogram ##
###########################

add_library(observation_histogram observation_histogram.cpp)
target_include_directories(observation_histogram PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(
    observation_histogram
    smap_base::smap_base
)

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
    observation_histogram
    smap_base::smap_base
)

ament_target_dependencies(
  thing
  ${DEPENDENCIES}
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

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


ament_package()