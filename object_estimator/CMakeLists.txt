cmake_minimum_required(VERSION 3.5)
project(object_estimator)

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
    # PCL
        find_package(PCL 1.1 REQUIRED)
        find_package(pcl_conversions REQUIRED)
    # ament
        find_package(ament_cmake REQUIRED)
    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(tf2 REQUIRED)
        find_package(tf2_ros REQUIRED)
        find_package(visualization_msgs REQUIRED)
        find_package(std_msgs REQUIRED)
        # find_package(std_srvs REQUIRED)
        # find_package(rmw_implementation_cmake REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)
        find_package(pcl_processing REQUIRED)
        find_package(smap_base REQUIRED)


set(INC
    include/object_estimator/object_estimator.hpp
)
    
set(SRC
    object_estimator.cpp
)
    
    
set(DEPENDENCIES
    rclcpp
    pcl_conversions
    smap_interfaces
    visualization_msgs
    tf2
    tf2_geometry_msgs
    tf2_sensor_msgs
    std_msgs
)

# ######################
# ## PCL DEPENDENCIES ##
# ######################

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# #######################
# ## /PCL DEPENDENCIES ##
# #######################

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

  include_directories(include/object_estimator)

  ## Lib ##

    add_library(
      object_estimator
      ${INC}
      ${SRC}
    #   ${IMGUI_SRC}
    )

    target_include_directories(object_estimator PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    #   ${SDL2_INCLUDE_DIRS} # imgui
    )

    target_link_libraries(object_estimator 
    #   ${PCL_LIBRARIES} 
    #   GL # imgui
    #   ${SDL2_LIBRARIES} # imgui
      smap_base::smap_base
      pcl_processing::pcl_processing
    )

    ament_target_dependencies(
      object_estimator
      ${DEPENDENCIES}
    )

    ament_export_include_directories(include)
    ament_export_libraries(object_estimator)
    ament_export_dependencies(${DEPENDENCIES})

    install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

    install(
        DIRECTORY include/object_estimator
        DESTINATION include
    )

    # install(TARGETS object_estimator
    #   DESTINATION lib/
    # )

    # install(
    #     TARGETS object_estimator
    #     EXPORT object_estimator
    #     LIBRARY DESTINATION lib
    #     ARCHIVE DESTINATION lib
    #     RUNTIME DESTINATION bin
    #     INCLUDES DESTINATION include
    # )


  ## Lib ##

#   find_package(object_estimator REQUIRED)
  ## Node ##

    add_executable(
        object_estimator_node
        ${INC}
        # ${IMGUI_SRC}
        object_estimator.cpp
        object_estimator_node.cpp
    )


    add_executable(
        object_estimator_tuning_node
        ${INC}
        ${IMGUI_SRC}
        object_estimator.cpp
        object_estimator_tuning_node.cpp
    )


    target_include_directories(object_estimator_node PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${SDL2_INCLUDE_DIRS} # imgui
    )

    target_include_directories(object_estimator_tuning_node PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${SDL2_INCLUDE_DIRS} # imgui
    )

    target_link_libraries(object_estimator_node 
      smap_base::smap_base
      pcl_processing::pcl_processing
    )

    target_link_libraries(object_estimator_tuning_node 
      GL # imgui
      ${SDL2_LIBRARIES} # imgui
      smap_base::smap_base
      pcl_processing::pcl_processing
    )
    
    ament_target_dependencies(
      object_estimator_node
      ${DEPENDENCIES}
    )

    ament_target_dependencies(
      object_estimator_tuning_node
      ${DEPENDENCIES}
    )

    install(TARGETS object_estimator_node
        DESTINATION lib/${PROJECT_NAME}/)
    
    install(TARGETS object_estimator_tuning_node
        DESTINATION lib/${PROJECT_NAME}/)

    # install(TARGETS object_estimator_node
    #   DESTINATION lib/
    # )

  ## Node ##

#######################
## /object_estimator ##
#######################

ament_package()
