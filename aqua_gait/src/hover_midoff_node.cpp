#include "aqua_gait/Gaits.hpp"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <aquacore/Command.h>
#include <aquacore/KeepAlive.h>
#include <aquacore/PeriodicLegCommand.h>
#include <mutex>


using namespace std;


class GaitWrapper {
public:
  GaitWrapper() : nh(), local_nh("~"), spin_rate(50), timeout_secs(3.0), publishing_cmds(false) {
    local_nh.param<double>("spin_rate", spin_rate, spin_rate);
    local_nh.param<double>("timeout_secs", timeout_secs, timeout_secs);
    
    reset_srv = local_nh.advertiseService("reset", &GaitWrapper::handleReset, this);
    plc_pub = nh.advertise<aquacore::PeriodicLegCommand>("/aqua/periodic_leg_command", 100);
    swim_backwards_sub = nh.subscribe("/aqua/swim_backwards", 1, &GaitWrapper::handleSwimBackwards, this);
    body_cmd_sub = nh.subscribe("/aqua/command", 10, &GaitWrapper::handleBodyCmd, this);
    keep_alive_sub = nh.subscribe("/aqua/keepalive", 10, &GaitWrapper::handleKeepAlive, this);
    
    reset();
    
    ROS_INFO_STREAM("Hover midoff node started");
  };
  
  
  ~GaitWrapper() {
  };
  
  
  void spin() {
    PeriodicLegState_t plc;
    ros::Rate hz(spin_rate);
    
    while (ros::ok()) {
      if (publishing_cmds) {
        // Check for timeout
        ros::Time now = ros::Time::now();
        if ((now - latest_body_cmd_time).toSec() > timeout_secs) {
          gait_mutex.lock();
          gait.setBodyCmd(0, 0, 0, 0, 0);
          gait_mutex.unlock();
          latest_body_cmd_time = now;
          publishing_cmds = false;
        }
        
        // Obtain new PLC command
        gait_mutex.lock();
        gait.updateSineCmd(plc);
        gait_mutex.unlock();
        
        // Publish PLC command
        aquacore::PeriodicLegCommand msg;
        for (int i = 0; i < 6; i++) {
          msg.amplitudes[i] = plc.amplitudes[i];
          msg.frequencies[i] = plc.frequencies[i];
          msg.phase_offsets[i] = plc.phase_offsets[i];
          msg.leg_offsets[i] = plc.leg_offsets[i];
          msg.leg_velocities[i] = plc.leg_velocities[i];
        }
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "aqua";
        plc_pub.publish(msg);
      }
      
      // Spin once and wait a bit
      ros::spinOnce();
      hz.sleep();
    }
  };


protected:
  void handleSwimBackwards(const std_msgs::Bool::ConstPtr& msg) {
    gait_mutex.lock();
    gait.foreaftControl(msg->data ? -1.0 : 1.0);
    gait_mutex.unlock();
  };


  void handleBodyCmd(const aquacore::Command::ConstPtr& msg) {
    gait_mutex.lock();
    gait.setBodyCmd(msg->speed, msg->heave, msg->roll, msg->pitch, msg->yaw);
    gait_mutex.unlock();
    latest_body_cmd_time = ros::Time::now();
    publishing_cmds = true;
  };
  
  
  void handleKeepAlive(const aquacore::KeepAlive::ConstPtr& msg) {
    if (msg->keepalive) {
      latest_body_cmd_time = ros::Time::now();
    } else {
      latest_body_cmd_time = ros::Time::now() - ros::Duration(2*timeout_secs);
    }
  };


  bool handleReset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    reset();
    return true;
  };
  

  void reset() {
    gait_mutex.lock();
    gait.activate();
    gait_mutex.unlock();
    publishing_cmds = false;
  };
  

  ros::NodeHandle nh, local_nh;
  ros::Subscriber swim_backwards_sub;
  ros::Subscriber body_cmd_sub;
  ros::Subscriber keep_alive_sub;
  ros::Publisher plc_pub;
  ros::ServiceServer reset_srv;
  
  UnderwaterSwimmerGait gait;
  std::mutex gait_mutex;
  
  double spin_rate;
  double timeout_secs;
  ros::Time latest_body_cmd_time;
  
  bool publishing_cmds;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "hover_midoff_node");
  GaitWrapper midoff;
  midoff.spin();
  return EXIT_SUCCESS;
};
