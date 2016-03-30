cmake_minimum_required(VERSION 2.8.3)
project(aquaautopilot)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure 
             message_generation geometry_msgs roscpp std_msgs tf)

add_message_files(FILES UberpilotStatus.msg)
generate_messages(DEPENDENCIES geometry_msgs)
generate_dynamic_reconfigure_options(cfg/Autopilot.cfg)

catkin_package( CATKIN_DEPENDS aquacore roscpp message_runtime geometry_msgs tf)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(local_autopilot_node src/local_autopilot_node.cpp)
add_dependencies(local_autopilot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                                      ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(local_autopilot_node ${catkin_LIBRARIES})

