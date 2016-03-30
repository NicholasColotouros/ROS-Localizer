cmake_minimum_required(VERSION 2.8.3)
project(aqua_utils)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall -O2 -Wno-deprecated")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    dynamic_reconfigure
    aquacore
    sensor_msgs
)

SET ( Boost_USE_MULTITHREADED ON )
find_package(Boost REQUIRED COMPONENTS system thread)


#######################################
## Declare ROS messages and services ##
#######################################

generate_dynamic_reconfigure_options(
    cfg/depth_emulator.cfg
    cfg/depth_filter.cfg
    cfg/gps_emulator.cfg
    cfg/gps_filter.cfg
  )
  
  
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS
    roscpp
    dynamic_reconfigure
    aquacore
    sensor_msgs
  #  DEPENDS system_lib
)
  

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  cfg/cpp
)

add_executable(depth_filter src/DepthFilterNode.cpp)
target_link_libraries(depth_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(depth_filter ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp aquacore_generate_messages_cpp)

