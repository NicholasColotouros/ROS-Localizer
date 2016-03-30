#include "aqua_gait/Gaits.hpp"
#include <ros/ros.h>
#include <chrono>


using namespace std;


int main(int argc, char** argv) {
  float speed_cmd = 1.0;
  float heave_cmd = 0.0;
  float roll_cmd = 0.0;
  float pitch_cmd = 1.0;
  float yaw_cmd = 0.0;

#define SHOW_MOTOR_TARGETS \
  ROS_INFO("- target poses:\n  0: %.4f\n  1: %.4f\n  2: %.4f\n  3: %.4f\n  4: %.4f\n  5: %.4f\n", \
      motorTargets[0].pos, motorTargets[1].pos, motorTargets[2].pos, motorTargets[3].pos, motorTargets[4].pos, motorTargets[5].pos)

  ros::init(argc, argv, "test_gaits");
  ros::NodeHandle nh;
  MotorTarget_t motorTargets[6];
  ros::Rate hz(1000);

  ROS_INFO_STREAM("Creating gait object");
  UnderwaterSwimmerGait gait;
  gait.activate();

  ROS_INFO_STREAM("Issuing gait command");
  gait.setSpeedCmd(speed_cmd);
  gait.heaveControl(heave_cmd);
  gait.rollControl(roll_cmd);
  gait.pitchControl(pitch_cmd);
  gait.yawControl(yaw_cmd);

  ROS_INFO_STREAM("Calling update for 3 seconds");
  hz.reset();
  ros::Time startT = ros::Time::now();
  PeriodicLegState_t legsCmd;
  while (true) {
    gait.updateSineCmd(legsCmd);
    gait.setPeriodicLegCmd(legsCmd);
    gait.updateMotorTarget(motorTargets);
    //gait.update(motorTargets);
    SHOW_MOTOR_TARGETS;
    hz.sleep();
    if ((ros::Time::now() - startT).toSec() > 3.0) break;
  }

  ROS_INFO_STREAM("All done!");

  return 0;
};
