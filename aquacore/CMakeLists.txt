cmake_minimum_required(VERSION 2.8.3)
project(aquacore)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  message_generation
  actionlib_msgs
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    AutopilotModes.msg
    Command.msg
    cuROSTityScore.msg
    GPSFixList.msg
    Health.msg
    KeepAlive.msg
    StateMsg.msg
    SurfaceTaskTypes.msg
    Velocity.msg
    PeriodicLegCommand.msg
)

add_service_files(
  FILES
    DumpAllVars.srv
    EmptyBool.srv
    EnableAutopilot.srv
    EnableDepthControl.srv
    GetAutopilotParams.srv
    GetAutopilotState.srv
    GetLegParams.srv
    GetNamedBool.srv
    GetNamedFloat.srv
    GetNamedInt.srv
    GetState.srv
    IsCalibrated.srv
    Menu.srv
    SetAutopilotMode.srv
    SetAutopilotParams.srv
    SetBool.srv
    SetDirection.srv
    SetGait.srv
    SetLegParams.srv
    SetNamedFloat.srv
    SetPauseMode.srv
    SetTargetAngles.srv
    SetTargetDepth.srv
    SetUpRegulatedSwimMode.srv
    GetPeriodicLegCommand.srv
    SetPeriodicLegCommand.srv
    SetTargetLegAngles.srv
    GetTargetLegAngles.srv
    StepSimulation.srv
    RunSimulationUntilTime.srv
)

add_action_files(
  FILES
    ChangeDepth.action
    DoSurfaceTask.action
    Goto.action
    SwimForward.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    rospy
    actionlib
    message_runtime
    actionlib_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
