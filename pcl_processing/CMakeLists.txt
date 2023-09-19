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
            find_package(tf2 REQUIRED)
            find_package(tf2_geometry_msgs REQUIRED)
        # smap
            find_package(smap_base REQUIRED)


# include_directories(
#     include
# )
include_directories(include ${PCL_INCLUDE_DIRS})

set(LIB_INC
    include/pcl_processing/pcl_processing.hpp
    include/pcl_processing/depth_map.hpp
)

set(LIB_SRC
    pcl_processing.cpp
)


set(DEPENDENCIES
    PCL
    pcl_conversions
    smap_interfaces
    tf2
    tf2_geometry_msgs
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

)

target_include_directories(pcl_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(
    pcl_processing
    ${PCL_LIBRARIES}
    smap_base::smap_base
)

ament_target_dependencies(
    pcl_processing
    ${DEPENDENCIES}
)

# Causes the visibility macros to use dllexp<depend>smap_interfaces</depend>ort rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pcl_processing PRIVATE "SMAP_BUILDING_LIBRARY")


include_directories(include/pcl_processing)


install(
    TARGETS pcl_processing
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
    
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(pcl_processing)
ament_export_dependencies(${DEPENDENCIES})


install(
  DIRECTORY include/pcl_processing
  DESTINATION include
)

install(
  TARGETS pcl_processing
  EXPORT pcl_processing
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
