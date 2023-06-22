cmake_minimum_required(VERSION 3.5)
project(pcl_processing)

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# DEPENDENCIES

set(DEPENDENCIES
    PCL
    sensor_msgs
    geometry_msgs
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

    # find dependencies
        # PCL
            find_package(PCL 1.1 REQUIRED)
            # find_package(pcl_conversions REQUIRED)
        # ament
            find_package(ament_cmake REQUIRED)
            # find_package(ament_cmake_ros REQUIRED)
        # standard ros pkgs
            find_package(std_msgs REQUIRED)
            find_package(sensor_msgs REQUIRED)
            find_package(geometry_msgs REQUIRED)
        # smap
            # find_package(smap_interfaces REQUIRED)


# include_directories(
#     include
# )
include_directories(include ${PCL_INCLUDE_DIRS})

set(LIB_INC
    include/pcl_processing.hpp
)

set(LIB_SRC
    pcl_processing.cpp
)


set(DEPENDENCIES
    PCL
    pcl_conversions
    smap_interfaces
)

# include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

    # find dependencies
        # PCL
            find_package(PCL 1.1 REQUIRED)
            find_package(pcl_conversions REQUIRED)
        # ament
            find_package(ament_cmake REQUIRED)
            find_package(ament_cmake_ros REQUIRED)
        # standard ros pkgs
            find_package(std_msgs REQUIRED)
            find_package(sensor_msgs REQUIRED)
            find_package(geometry_msgs REQUIRED)
        # smap
            find_package(smap_interfaces REQUIRED)


add_library(
    pcl_processing
    ${LIB_INC}
    ${LIB_SRC}
    # pcl_processing.cpp
)

target_include_directories(pcl_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(
    pcl_processing
    ${PCL_LIBRARIES}
)

ament_target_dependencies(
    pcl_processing
    ${DEPENDENCIES}
)

# Causes the visibility macros to use dllexp<depend>smap_interfaces</depend>ort rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pcl_processing PRIVATE "SMAP_BUILDING_LIBRARY")

install(
  TARGETS pcl_processing
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(include)
ament_export_libraries(pcl_processing)
ament_export_dependencies(${DEPENDENCIES})



# install(
#   TARGETS pcl_processing
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(
  DIRECTORY include/
  DESTINATION include/smap_core/pcl_processing
)

ament_package()
