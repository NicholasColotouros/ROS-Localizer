cmake_minimum_required(VERSION 2.8.3)
project(aquajoy)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall -O2 -Wno-deprecated")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    dynamic_reconfigure
    aquacore
    sensor_msgs
    geometry_msgs
    angles
    tf
    message_generation
)

SET ( Boost_USE_MULTITHREADED ON )
find_package(Boost REQUIRED COMPONENTS thread)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  DIRECTORY msg
  FILES
    AquaTeleopJoyParams.msg
    AquaTeleopJoyState.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    UpdateAquaTeleopJoyParams.srv
)

generate_messages(
  DEPENDENCIES
    aquacore
    sensor_msgs
    geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/AquaTeleopJoy.cfg
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
    geometry_msgs
    angles
    tf
    message_runtime
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

add_executable(aqua_key3D src/aqua_key3D.cpp)
target_link_libraries(aqua_key3D  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(aqua_key3D ${PROJECT_NAME}_generate_messages_cpp)


