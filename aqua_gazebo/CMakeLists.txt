cmake_minimum_required(VERSION 2.8.3)
project(aqua_gazebo)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aquacore
  aqua_description
  aqua_gait
  roscpp
  gazebo_ros
  sensor_msgs
  std_msgs
  genmsg 
  message_generation
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES aqua_gazebo
  CATKIN_DEPENDS 
  aquacore
  aqua_gait
  aqua_description 
  roscpp
  sensor_msgs
  std_msgs
  DEPENDS
  gazebo_ros
  message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${aqua_gait_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(aqua_flippers_plugin src/aqua_flippers_plugin.cpp)
target_link_libraries(aqua_flippers_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${aqua_gait_LIBRARIES} ${Boost_LIBRARIES})

add_library(aqua_hardware_emulator src/aqua_hardware_emulator.cpp)
target_link_libraries(aqua_hardware_emulator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(aqua_hydrodynamics_plugin src/aqua_hydrodynamics_plugin.cpp)
target_link_libraries(aqua_hydrodynamics_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
