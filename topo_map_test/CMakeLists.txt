cmake_minimum_required(VERSION 3.5)
project(topo_map)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 COMPONENTS system serialization) 

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS}) 
  add_executable(topo_map topo_map.cpp) 
  target_link_libraries(topo_map ${Boost_LIBRARIES})
endif()