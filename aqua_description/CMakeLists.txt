cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(aqua_description)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  xacro
  urdf
  angles
  aquacore
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    xacro
    urdf
    angles
    aquacore
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(leg_state_publisher src/leg_state_publisher.cpp) 
target_link_libraries(leg_state_publisher ${catkin_LIBRARIES})
add_dependencies(leg_state_publisher aquacore_generate_messages_cpp)

