cmake_minimum_required(VERSION 3.5)
project(smap_core)

# find dependencies
    # PCL
    # find_package(PCL 1.1 REQUIRED)
    # find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smap_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(smap_interfaces REQUIRED)

######################
## PCL DEPENDENCIES ##
######################

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

#######################
## /PCL DEPENDENCIES ##
#######################

#########
## PCH ##
#########

add_library(pch pch.cpp)
target_precompile_headers(pch PUBLIC pch.hpp)



install(TARGETS pch
  DESTINATION lib/${PROJECT_NAME})

##########
## /PCH ##
##########