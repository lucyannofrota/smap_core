cmake_minimum_required(VERSION 3.5)
project(topo_map)


# find dependencies
    # ament
        find_package(ament_cmake REQUIRED)
        find_package(ament_cmake_ros REQUIRED)
    # standard ros pkgs
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(visualization_msgs REQUIRED)
        find_package(std_srvs REQUIRED)
    # smap
        find_package(smap_interfaces REQUIRED)


###################
## ADD BOOST LIB ##
###################

find_package(Boost REQUIRED system serialization) 
set(Boost_USE_MULTITHREADED ON)  



  # if(Boost_FOUND)
  #   include_directories(${Boost_INCLUDE_DIRS}) 
	# endif()

####################
## /ADD BOOST LIB ##
####################

#################
## topo_marker ##
#################

  ## Lib ##
  add_library(topo_marker
    topo_marker.cpp
  )

target_include_directories(topo_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
    #   ${Boost_INCLUDE_DIRS}
  )

ament_target_dependencies(
    topo_marker
    visualization_msgs
    # geometry_msgs
    # smap_interfaces
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(topo_marker PRIVATE "SMAP_BUILDING_LIBRARY")

install(
  TARGETS topo_marker
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## /Lib ##

##################
## /topo_marker ##
##################


##############
## topo_map ##
##############

  ## Lib ##
  add_library(topo_map
    topo_map.cpp
  )
  target_link_libraries(topo_map thing topo_marker pcl_processing ${Boost_LIBRARIES})
target_include_directories(topo_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
      ${Boost_INCLUDE_DIRS}
  )

ament_target_dependencies(
  topo_map
  geometry_msgs
  smap_interfaces
  visualization_msgs
  std_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(topo_map PRIVATE "SMAP_BUILDING_LIBRARY")

install(
  TARGETS topo_map
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## /Lib ##

## Node ##

  add_executable(topo_map_node
      topo_map.cpp
  )

  target_include_directories(topo_map_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Boost_INCLUDE_DIRS}   
  )

  target_link_libraries(
        topo_map_node 
        thing
        topo_marker
        pcl_processing
        ${Boost_LIBRARIES}
  )

  ament_target_dependencies(
      topo_map_node
      rclcpp
      geometry_msgs
      smap_interfaces
      visualization_msgs
      std_msgs
  )

  install(TARGETS topo_map_node
      DESTINATION lib/${PROJECT_NAME}
  )

## Node ##

###############
## /topo_map ##
###############
