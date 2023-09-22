cmake_minimum_required(VERSION 3.5)
project(smap_base)

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()


    # find dependencies
        # ament
            find_package(ament_cmake REQUIRED)
            # find_package(ament_cmake_ros REQUIRED)
        # standard ros pkgs
            find_package(rclcpp REQUIRED)


set(LIB_INC
    include/smap_base/aux_functions.hpp
    src/aux_functions.cpp
    include/smap_base/count_time.hpp
    include/smap_base/detector_descriptor.hpp
    # include/smap_base/interface_templates.hpp
    include/smap_base/macros.hpp
    include/smap_base/stacking_classification.hpp
    include/smap_base/visibility_control.h
)

add_library(
    smap_base
    ${LIB_INC}
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(smap_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexp<depend>smap_interfaces</depend>ort rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(smap_base PRIVATE "SMAP_BUILDING_LIBRARY")


include_directories(include/smap_base)


install(
    TARGETS smap_base
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
    
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(smap_base)


install(
  DIRECTORY include/smap_base
  DESTINATION include
)

file(MAKE_DIRECTORY 
    ../SemMap 
    ../maps 
    ../timers 
    ../timers/object_estimator
    ../timers/topo_map
)

install(DIRECTORY
  # ../launch
  ../scripts
  ../config
  ../SemMap
  ../maps
  ../timers
  ../timers/object_estimator
  ../timers/topo_map
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS smap_base
  EXPORT smap_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
