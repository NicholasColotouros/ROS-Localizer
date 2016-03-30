#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <aquacore/GetPeriodicLegCommand.h>
#include <aquacore/SetPeriodicLegCommand.h>
#include <aquacore/GetTargetLegAngles.h>
#include <aquacore/SetTargetLegAngles.h>
#include <stdio.h>

#include <ros/ros.h>

#define TWO_M_PI 2*M_PI
#define NUM_LEGS 6
const char* JOINT_NAMES[NUM_LEGS] =
       { "left_front_leg_joint",
	 "left_mid_leg_joint",
	 "left_rear_leg_joint",
	 "right_front_leg_joint",
	 "right_mid_leg_joint",
	 "right_rear_leg_joint"};

const char* LEG_NAMES[NUM_LEGS] =
       { "left_front_leg",
	 "left_mid_leg",
	 "left_rear_leg",
	 "right_front_leg",
	 "right_mid_leg",
	 "right_rear_leg"};

class AquaFlippersPlugin: public gazebo::ModelPlugin
{
  public:
    AquaFlippersPlugin();
    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate(const gazebo::common::UpdateInfo & info);
    bool SetPeriodicLegCommand_cb(aquacore::SetPeriodicLegCommand::Request  &req, aquacore::SetPeriodicLegCommand::Response &res);
    bool GetPeriodicLegCommand_cb(aquacore::GetPeriodicLegCommand::Request  &req, aquacore::GetPeriodicLegCommand::Response &res);
    bool SetTargetLegAngles_cb(aquacore::SetTargetLegAngles::Request  &req, aquacore::SetTargetLegAngles::Response &res);
    bool GetTargetLegAngles_cb(aquacore::GetTargetLegAngles::Request  &req, aquacore::GetTargetLegAngles::Response &res);

  private:
    gazebo::common::Time last_update_time;
    gazebo::physics::ModelPtr model;
    gazebo::physics::LinkPtr base_link;
    std::vector<gazebo::physics::LinkPtr> leg_links;
    std::vector<gazebo::physics::JointPtr> motor_joints;

    std::vector<gazebo::common::PID> pid;
    gazebo::event::ConnectionPtr updateConnection;

    ros::ServiceServer get_leg_command_service, set_leg_command_service;
    ros::ServiceServer get_target_angles_service, set_target_angles_service;

    ros::NodeHandle* nh;

    std::string robot_namespace;

    boost::array<double,NUM_LEGS> target_angles;
    sdf::Vector3 pid_gains;
    
    boost::array<double,NUM_LEGS> frequency_cmd;
    boost::array<double,NUM_LEGS> phase_offsets_cmd;
    boost::array<double,NUM_LEGS> leg_offsets_cmd;
    boost::array<double,NUM_LEGS> amplitude_cmd;
    boost::array<double,NUM_LEGS> thrust;

    double fluid_density,surface_level;
    sdf::Vector3 K1, K2;
    gazebo::math::Vector3 leg_dims;
    int thrust_model;
    int angle_direction;

    
};

