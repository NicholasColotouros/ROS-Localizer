#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <stdio.h>
#include <chrono>

class AquaHydrodynamicsPlugin: public gazebo::ModelPlugin
{
  public:
    AquaHydrodynamicsPlugin();
    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate(const gazebo::common::UpdateInfo & info);
    bool DisableDegreesOfFreedom(gazebo::math::Vector3 &aqua_lin_vel,
                                 gazebo::math::Vector3 &aqua_ang_vel,
                                 gazebo::math::Vector3 &aqua_lin_accel,
                                 gazebo::math::Vector3 &aqua_ang_accel);

  private:
    gazebo::common::Time current_time;
    gazebo::physics::ModelPtr model;
    gazebo::physics::WorldPtr world;
    gazebo::physics::LinkPtr base_link;
    std::vector<gazebo::physics::LinkPtr> leg_links;
    std::vector<gazebo::physics::LinkPtr> shoulder_links;
    gazebo::event::ConnectionPtr updateConnection;
    gazebo::math::Pose initialPose;

    std::string robot_namespace;
    double surface_level, fluid_density, aqua_volume;
    sdf::Vector3 drag_coeffs, leg_drag_coeffs;
    gazebo::math::Vector3 leg_dims, aqua_dims;
    bool disable_roll_simulation,disable_pitch_simulation,
         disable_yaw_simulation,disable_heave_simulation,
         disable_speed_simulation;
};
