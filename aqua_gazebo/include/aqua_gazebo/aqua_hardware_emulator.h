#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <random>

// ROS messages
#include "sensor_msgs/Imu.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Time.h"
//messages
#include "aquacore/Velocity.h"
#include "aquacore/Command.h"
#include "aquacore/KeepAlive.h"
#include "aquacore/StateMsg.h"
#include "aquacore/Health.h"
//services
#include "aquacore/SetTargetDepth.h"
#include "aquacore/SetTargetAngles.h"
#include "aquacore/SetGait.h"
#include "aquacore/DumpAllVars.h"
#include "aquacore/SetPauseMode.h"
#include "aquacore/GetAutopilotState.h"
#include "aquacore/GetAutopilotParams.h"
#include "aquacore/SetAutopilotParams.h"
#include "aquacore/SetAutopilotMode.h"
#include "aquacore/AutopilotModes.h"
#include "aquacore/SetNamedFloat.h"
#include "aquacore/GetNamedFloat.h"
#include "aquacore/SetLegParams.h"
#include "aquacore/GetLegParams.h"
#include "aquacore/GetState.h"
#include "aquacore/SetDirection.h"
#include "aquacore/IsCalibrated.h"
#include "aquacore/PeriodicLegCommand.h"
#include <aquacore/SetPeriodicLegCommand.h>
#include <aquacore/GetPeriodicLegCommand.h>
#include <aquacore/SetTargetLegAngles.h>
#include <aquacore/GetTargetLegAngles.h>
#include <aquacore/StepSimulation.h>
#include <aquacore/RunSimulationUntilTime.h>

#include <aqua_gait/Gaits.hpp>

#define NUM_LEGS 6
const char* JOINT_NAMES[NUM_LEGS] =
       { "left_front_leg_joint",
	 "left_mid_leg_joint",
	 "left_rear_leg_joint",
	 "right_front_leg_joint",
	 "right_mid_leg_joint",
	 "right_rear_leg_joint"};

class AquaHWPlugin: public gazebo::ModelPlugin
{
  public:
    AquaHWPlugin();
    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate(const gazebo::common::UpdateInfo & info);
    void ImuCallback(const sensor_msgs::ImuConstPtr &msg);
    bool set_direction(aquacore::SetDirection::Request  &req, aquacore::SetDirection::Response &res);
    bool get_leg_params(aquacore::GetLegParams::Request  &req, aquacore::GetLegParams::Response &res);
    bool set_leg_params(aquacore::SetLegParams::Request  &req, aquacore::SetLegParams::Response &res);
    bool set_autopilot_mode(aquacore::SetAutopilotMode::Request  &req, aquacore::SetAutopilotMode::Response &res);
    bool get_autopilot_params(aquacore::GetAutopilotParams::Request  &req, aquacore::GetAutopilotParams::Response &res);
    bool get_autopilot_state(aquacore::GetAutopilotState::Request  &req, aquacore::GetAutopilotState::Response &res);
    bool set_autopilot_params(aquacore::SetAutopilotParams::Request  &req, aquacore::SetAutopilotParams::Response &res);
    bool set_targetdepth(aquacore::SetTargetDepth::Request  &req, aquacore::SetTargetDepth::Response &res);
    bool set_targetangles(aquacore::SetTargetAngles::Request  &req, aquacore::SetTargetAngles::Response &res);
    bool set_gait(aquacore::SetGait::Request  &req, aquacore::SetGait::Response &res);
    bool pause(aquacore::SetPauseMode::Request  &req, aquacore::SetPauseMode::Response &res);
    bool dump_all_vars(aquacore::DumpAllVars::Request  &req, aquacore::DumpAllVars::Response &res);
    bool zerodepth(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    bool is_calibrated(aquacore::IsCalibrated::Request  &req, aquacore::IsCalibrated::Response &res);
    bool do_calibrate(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    void publish_health(const ros::TimerEvent& e);
    aquacore::StateMsg current_state();
    bool get_state(aquacore::GetState::Request  &req, aquacore::GetState::Response &res);
    void publish_state(const ros::TimerEvent& e);
    void keepalive(const aquacore::KeepAlive::ConstPtr& msg);
    void keepalive(bool k=true);
    void process_command(const aquacore::Command::ConstPtr& msg);
    void process_periodic_leg_command(const aquacore::PeriodicLegCommand::ConstPtr& msg);
    void reset_world(const std_msgs::Time::ConstPtr& msg);
    void zerospeed();
    void failsafe(const ros::TimerEvent& e);
    void uwsg_controller_update(const ros::TimerEvent& e);
    bool step_simulation(aquacore::StepSimulation::Request  &req, aquacore::StepSimulation::Response &res);
    bool run_simulation_until_time(aquacore::RunSimulationUntilTime::Request  &req, aquacore::RunSimulationUntilTime::Response &res);

  private:
    boost::posix_time::ptime m_last_command_time;
    boost::posix_time::ptime m_start_time;
    int state;
    enum {STOP,SWIM}; 

    gazebo::common::Time current_time;
    gazebo::physics::ModelPtr model;
    gazebo::physics::WorldPtr world;
    gazebo::event::ConnectionPtr updateConnection;

    ros::NodeHandle* nh;
    UnderwaterSwimmerGait uwsg_controller;
    
    tf::TransformBroadcaster tf_br;
    ros::Publisher rate_pub;
    
    ros::Publisher state_pub, health_pub;
    ros::Subscriber cmd_sub, keepalive_sub, imu_sub, periodic_leg_command_sub, new_episode_sub;
    ros::Timer failsafe_timer, state_broadcast_timer, health_broadcast_timer, uwsg_controller_timer;
    ros::ServiceServer pause_service, zerodepth_service, calibrate_service, 
                       is_calibrated_service, set_targetdepth_service, set_targetangles_service,
                       set_gait_service, dump_all_vars_service, get_autopilot_state_service,
                       get_autopilot_params_service,set_autopilot_param_service,set_autopilot_param_by_name_service,
                       get_autopilot_param_by_name_service, set_autopilot_mode_service, set_leg_params_service,
                       get_leg_params_service, set_direction_service, get_state_service,
                       step_simulation_service, run_simulation_until_time_service;
    ros::ServiceClient get_leg_command_client, set_leg_command_client;
    ros::ServiceClient get_target_angles_client, set_target_angles_client;
     
    std::string robot_namespace, imu_topic;
    double surface_level, depth_sensor_noise, aqua_shell_volume, fluid_density;
    std::normal_distribution<> gaussian_noise;

    aquacore::Health health_msg;
    aquacore::StateMsg state_msg;

    bool periodic_leg_command_active;
    bool _debug_print;
    PeriodicLegState_t latest_periodic_leg_command;
    MotorTarget_t _motor_targets[6];
    aquacore::SetPeriodicLegCommand plc_srv;
    aquacore::SetTargetLegAngles ta_srv;
    sensor_msgs::Imu latest_imu_msg;
    boost::array<double,NUM_LEGS> integrated_velocity;
};

