#include <ros/ros.h>
#include <aquacore/Command.h>
#include <aquacore/SetGait.h>
#include <aquacore/SetAutopilotMode.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <aquacore/AutopilotModes.h>
#include <aquaautopilot/AutopilotConfig.h>
#include <aquaautopilot/UberpilotStatus.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <dynamic_reconfigure/server.h>

typedef dynamic_reconfigure::Server<aquaautopilot::AutopilotConfig> ReconfigureServer;
using namespace aquaautopilot;

#define PI 3.14159265359

class LocalAutopilot
{

private:

  // ROS variables
  ros::NodeHandle n_;
  ros::Timer keepalive_timer;
  ros::Publisher cmd_pub_;
  ros::Publisher ap_status_pub_;
  ros::Subscriber target_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber vel_sub_;
  ros::ServiceServer mode_service_;
  ros::ServiceServer reset_state_service_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster br_;
  tf::Transform transform_;
  ros::Publisher vis_pub1_;
  ros::Publisher vis_pub2_;
  ros::Publisher vis_pub3_;
  ros::Publisher vis_pub4_;
  ros::Publisher vis_pub5_;

  // Variables related to the robot's current depth from sensors
  double  current_depth;
  ros::Time last_depth_reading_time_;
  bool have_depth_;

  // Variables related to the user's requested input
  geometry_msgs::PoseStamped current_target_;
  bool have_target_;
  int curr_auto_mode_;

  // Variables related to reading the robot's velocity through the IMU
  geometry_msgs::Twist current_velocity_;
  bool have_velocity_;

  // Gains and parameters for the angle and speed PID controllers
  AutopilotConfig params_;
  ReconfigureServer* dyncfg_server_;
  boost::recursive_mutex params_mutex_;
  bool dyncfg_sync_request_;
  bool use_slerp;
  bool use_robot_frame_depth;

  // Watchdog timer parameters
  ros::Duration depth_lifetime_;
  ros::Duration target_lifetime_;

  // Members used in control computations
  double depth_derivative_;
  double last_depth_;
  double filtered_depth_derivative_;
  bool have_filtered_depth_derivative_;

  geometry_msgs::Twist filtered_velocity_;
  ros::Time last_velocity_reading_time_;
  bool have_roll_filtered_deriv_;
  bool have_pitch_filtered_deriv_;
  bool have_yaw_filtered_deriv_;

  double update_period_;
  aquacore::Command latest_cmd_;
  aquacore::Command integral_errors_;
  tf::Quaternion rotation_from_imu_to_global_, rotation_from_global_to_imu_, rotation_from_target_to_global_, rotation_command_in_robot_frame_;

  bool display_output_;

  // The logging variable
  aquaautopilot::UberpilotStatus stat;

  // This was used in an attempt to differentiate angles. Unused currently
  //tf::Quaternion rotation_from_previous_imu_to_global_;

public:

LocalAutopilot() :
  n_("~"),
  dyncfg_sync_request_(false)
{
  double lifetime_param;

  initialize_local_variables();

  cmd_pub_ = n_.advertise<aquacore::Command>("/aqua/command", 1);
  ap_status_pub_ = n_.advertise<aquaautopilot::UberpilotStatus>("/aqua/autopilot/status", 1);

  // Setup dynamic reconfigure server
  // NOTE: this should be set up prior to loading (static) ROS params, so that launch file param values gets prioritized over older dyncfg values
  dyncfg_server_ = new ReconfigureServer(params_mutex_, n_);
  dyncfg_server_->setCallback(bind(&LocalAutopilot::configCallback, this, _1, _2));

  params_mutex_.lock();
  n_.param<double>("lifetime",lifetime_param, 3.0);

  n_.param<double>("ROLL_P_GAIN", params_.ROLL_P_GAIN, 1.0 );
  n_.param<double>("PITCH_P_GAIN", params_.PITCH_P_GAIN, 2.0 );
  n_.param<double>("YAW_P_GAIN", params_.YAW_P_GAIN, -3.5 );

  n_.param<double>("ROLL_I_GAIN", params_.ROLL_I_GAIN, 0.0 );
  n_.param<double>("PITCH_I_GAIN", params_.PITCH_I_GAIN, 0.0 );
  n_.param<double>("YAW_I_GAIN", params_.YAW_I_GAIN, 0.0 );

  n_.param<double>("ROLL_D_GAIN", params_.ROLL_D_GAIN, 0.0 );
  n_.param<double>("PITCH_D_GAIN", params_.PITCH_D_GAIN, 0.0 );
  n_.param<double>("YAW_D_GAIN", params_.YAW_D_GAIN, 0.0 );

  n_.param<double>("ROLL_CONST_GAIN", params_.ROLL_CONST_GAIN, 0.0 );

  n_.param<double>("ROLL_D_FILTER_PERIOD", params_.ROLL_D_FILTER_PERIOD, 0.0);
  n_.param<double>("PITCH_D_FILTER_PERIOD", params_.PITCH_D_FILTER_PERIOD, 0.0);
  n_.param<double>("YAW_D_FILTER_PERIOD", params_.YAW_D_FILTER_PERIOD, 0.0);

  n_.param<double>("MAX_INTEGRAL_ANGLE_ERROR", params_.MAX_INTEGRAL_ANGLE_ERROR, 10.0 );

  n_.param<double>("KSPEED", params_.KSPEED, 1.0);
  n_.param<double>("KHEAVE",params_.KHEAVE,1.0);
  n_.param<double>("MAX_ROLL", params_.MAX_ROLL,1.0);
  n_.param<double>("MAX_PITCH", params_.MAX_PITCH,1.0);
  n_.param<double>("MAX_YAW", params_.MAX_YAW,1.0);
  n_.param<double>("MAX_SPEED", params_.MAX_SPEED,1.0);
  n_.param<double>("MAX_HEAVE", params_.MAX_HEAVE,1.0);

  n_.param<double>("KDEPTH", params_.KDEPTH, 0.3);
  n_.param<double>("DEPTH_D_GAIN", params_.DEPTH_D_GAIN, 0.0);
  n_.param<double>("DEPTH_D_FILTER_PERIOD", params_.DEPTH_D_FILTER_PERIOD, 0.6);

  n_.param<bool>("use_slerp", use_slerp, true);
  n_.param<bool>("use_robot_frame_depth", use_robot_frame_depth, true);

  n_.param<bool>("display_output", display_output_,false);

  if( display_output_)
  {
    ROS_INFO( "3D Autopilot will print output because display_output parameter was true.");
  }
  else
  {
    ROS_INFO( "3D Autopilot running silently because display_output parameter was false.");
  }


  dyncfg_sync_request_ = true;

  ROS_INFO_COND( display_output_, "Local AP starting with:\nROLL_P_GAIN: %f,\nPITCH_P_GAIN: %f,\nYAW_P_GAIN: %f,\nKSPEED: %f,\nKHEAVE: %f,\nMAX_ROLL: %f,\nMAX_PITCH: %f,\nMAX_YAW: %f.\n",
      params_.ROLL_P_GAIN, params_.PITCH_P_GAIN, params_.YAW_P_GAIN, params_.KSPEED, params_.KHEAVE, params_.MAX_ROLL, params_.MAX_PITCH, params_.MAX_YAW);

  if( use_slerp )
  {
    ROS_INFO_COND( display_output_, "I am using SLERP to interpolate angles.");
  }
  else
  {
    ROS_INFO_COND( display_output_, "Not using SLERP interpolation.");
  }

  params_mutex_.unlock();

  target_lifetime_ = ros::Duration(lifetime_param);
  depth_lifetime_ = ros::Duration(lifetime_param);

  // Set the gait of the robot to hover-midoff
  aquacore::SetGait typSetGait;
  ros::ServiceClient clnSetGait;
  ROS_INFO_STREAM("Waiting for service /aqua/set_gait...");
  ros::service::waitForService("/aqua/set_gait");
  ROS_INFO_STREAM("... found!");
  clnSetGait = n_.serviceClient<aquacore::SetGait>("/aqua/set_gait");
  typSetGait.request.gait = "flexible-sine"; // TODO: have some way to toggle back to hover-midoff, if we ever want to throw away N months of gait learning code
  if (!clnSetGait.call(typSetGait))
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": failed to set gait to " << typSetGait.request.gait );
    //ros::shutdown();
  }
  else
  {
    ROS_INFO_COND( display_output_, "Gait successfully set to %s.\n", typSetGait.request.gait.c_str());
  }

  // Disable the Robo-devel autopilot
  aquacore::SetAutopilotMode typSetAPMode;
  ros::ServiceClient clnSetAPMode;  
  ROS_INFO_STREAM("Waiting for service /aqua/set_autopilot_mode...");
  ros::service::waitForService("/aqua/set_autopilot_mode");
  ROS_INFO_STREAM("... found!");
  clnSetAPMode = n_.serviceClient<aquacore::SetAutopilotMode>("/aqua/set_autopilot_mode");
  typSetAPMode.request.mode = aquacore::AutopilotModes::OFF;
  if (!clnSetAPMode.call(typSetAPMode) || !typSetAPMode.response.response )
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": failed to set Robodevel AP mode to " << 
        typSetAPMode.request.mode << ". Response was: " << typSetAPMode.response.response );
    //ros::shutdown();
  }
  else
  {
    ROS_INFO_COND( display_output_, "Successfully disabled the Robodevel autopilot.\n");
  }

  vis_pub1_ = n_.advertise<geometry_msgs::PoseStamped>("/aqua/traj3Dvis1", 1);
  vis_pub2_ = n_.advertise<geometry_msgs::PoseStamped>("/aqua/traj3Dvis2", 1);
  vis_pub3_ = n_.advertise<geometry_msgs::PoseStamped>("/aqua/traj3Dvis3", 1);
  vis_pub4_ = n_.advertise<geometry_msgs::PoseStamped>("/aqua/traj3Dvis4", 1);
  vis_pub5_ = n_.advertise<geometry_msgs::PoseStamped>("/aqua/traj3Dvis5", 1);

  mode_service_ = n_.advertiseService("/aqua/set_3Dauto_mode", &LocalAutopilot::set_autopilot_mode,this);
  reset_state_service_ = n_.advertiseService("/aqua/reset_3D_autopilot_state", &LocalAutopilot::reset_state,this);

  depth_sub_ = n_.subscribe<std_msgs::Float32>("/aqua/filtered_depth", 1, &LocalAutopilot::depthCallback, this);
  target_sub_ = n_.subscribe<geometry_msgs::PoseStamped>("/aqua/target_pose", 1, &LocalAutopilot::targetCallback, this);
  vel_sub_ = n_.subscribe<geometry_msgs::Twist>("/aqua/positioning/angular_velocity", 1, &LocalAutopilot::velocityCallback, this);
  keepalive_timer = n_.createTimer(ros::Duration(update_period_), &LocalAutopilot::keepalive, this);
}

void initialize_local_variables()
{
  have_depth_ = false;
  have_target_ = false;
  have_velocity_ = false;
  curr_auto_mode_ = aquacore::AutopilotModes::AP_OFF;

  update_period_  = 0.02;

  have_filtered_depth_derivative_ = false;
  filtered_depth_derivative_ = 0.0;
  depth_derivative_ = 0.0;
  last_depth_ = 0.0;

  integral_errors_.roll = 0.0;
  integral_errors_.pitch = 0.0;
  integral_errors_.yaw = 0.0;
  integral_errors_.speed = 0.0;
  integral_errors_.heave = 0.0;

  filtered_velocity_.angular.x = 0.0;
  filtered_velocity_.angular.y = 0.0;
  filtered_velocity_.angular.z = 0.0;
}

void spin() {
  ros::Rate hz(30);
  while (ros::ok())
  {
    ros::spinOnce();
      
    // Update params back to dyncfg server
    // Make sure that dynamic reconfigure server or config callback is not active
    if (dyncfg_sync_request_ && params_mutex_.try_lock())
    {
      params_mutex_.unlock();
      dyncfg_server_->updateConfig(params_);
      dyncfg_sync_request_ = false;
    }
    
    hz.sleep();
  }
}

protected:

void configCallback(aquaautopilot::AutopilotConfig& config, uint32_t level) {
  params_mutex_.lock();
  params_ = config;
  params_mutex_.unlock();
};

void keepalive(const ros::TimerEvent& e){
  ROS_INFO_COND( display_output_, "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nAquaAutopilot update started.");
  doAutopilotUpdate();
}

bool reset_state( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp )
{
  initialize_local_variables();

  return true;
}

bool set_autopilot_mode(aquacore::SetAutopilotMode::Request  &req, aquacore::SetAutopilotMode::Response &res){

  if( req.mode < 0 || req.mode >= aquacore::AutopilotModes::AP_FIRST_INVALID_AP_MODE )
  {
    res.response = false;
  }
  else
  {
    curr_auto_mode_ = req.mode;
    res.response = true;
  }

  return true;
}

/** Function: applyExpFilter
 *
 * \param filter_time_constant: (in)     user configurable smoothing parameter (time units)
 * \param sample_period       : (in)     time since the last call to this function (same time units)
 * \param current_data        : (in)     the un-filtered target signal that we want to smooth
 * \param previous_valid      : (in/out) indicator for first execution of this function. Set to false initially.
 * \param filter_value        : (out)    smoothed version of current_data
 */
void applyExpFilter( double filter_time_constant, double sample_period, double current_data , bool &previous_valid, double &filter_value )
{
  if( fabs(filter_time_constant) < 0.0000001 )
  {
    filter_value = current_data;
  }

  if( !previous_valid )
  {
    filter_value = current_data;
    previous_valid = true;
  }
  else
  {
    double filter_gain = exp( -sample_period / filter_time_constant );
    filter_value = filter_gain * filter_value + (1.0-filter_gain) * current_data;
  }
}

void depthCallback(const std_msgs::Float32::ConstPtr& filtered_depth )
{
  current_depth = filtered_depth->data;
  
  if (use_robot_frame_depth) {
    tf::StampedTransform T_from_global_to_imu;
    double curr_r_in_global, curr_p_in_global, curr_y_in_global;
    listener_.lookupTransform("/aqua_base", "/latest_fix", ros::Time(0), T_from_global_to_imu);
    tf::Quaternion Q_from_imu_to_global = T_from_global_to_imu.inverse().getRotation();
    getRPY(Q_from_imu_to_global, curr_r_in_global, curr_p_in_global, curr_y_in_global);

    double length_of_robot = 0.6; // in meters
    current_depth = current_depth + sin(curr_p_in_global)*length_of_robot/2.0;
  }
  
  if( !have_depth_ )
  {
    depth_derivative_ = 0.0;
    filtered_depth_derivative_ = 0.0;
  }
  else
  {
    ros::Duration depth_time_difference = ros::Time::now() - last_depth_reading_time_;
    if( depth_time_difference.toSec() < fabs(1e-9))
    {
      ROS_WARN( "Skipping depth derivative update because of near-zero timestep.");
      return;
    }

    depth_derivative_ = ( current_depth - last_depth_ ) / depth_time_difference.toSec();


    applyExpFilter( params_.DEPTH_D_FILTER_PERIOD, depth_time_difference.toSec(), depth_derivative_, have_filtered_depth_derivative_, filtered_depth_derivative_ );

    //ROS_INFO_COND( display_output_, "Depth time difference: %f (s). Depth_derivative_: %f, filtered depth: %f.", depth_time_difference.toSec(), depth_derivative_, filtered_depth_derivative_);
    //if( filtered_depth_derivative_ != filtered_depth_derivative_ )
    //{
    //  ROS_INFO_COND( display_output_, "NaN filtered derivative detected.");
    //  ros::shutdown();
    //}
  }

  have_depth_ = true;
  last_depth_reading_time_ = ros::Time::now();
  last_depth_ = current_depth;

}

void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& targetPose )
{
  current_target_ = *targetPose;
  have_target_ = true;
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel_msg )
{
  current_velocity_ = *vel_msg;

  if( !have_velocity_ )
  {
    filtered_velocity_ = current_velocity_;
  }
  else
  {
    ros::Duration velocity_time_difference = ros::Time::now() -  last_velocity_reading_time_; 
    applyExpFilter( params_.ROLL_D_FILTER_PERIOD, velocity_time_difference.toSec(), current_velocity_.angular.x , have_roll_filtered_deriv_, filtered_velocity_.angular.x );
    applyExpFilter( params_.PITCH_D_FILTER_PERIOD, velocity_time_difference.toSec(), current_velocity_.angular.y , have_pitch_filtered_deriv_, filtered_velocity_.angular.y );
    applyExpFilter( params_.YAW_D_FILTER_PERIOD, velocity_time_difference.toSec(), current_velocity_.angular.z , have_yaw_filtered_deriv_, filtered_velocity_.angular.z );
    //filtered_velocity = 
  }

  last_velocity_reading_time_ = ros::Time::now();
  have_velocity_ = true;
}

double limit_val( double val, double lower, double upper )
{
  if( std::isnan(val) || val != val)
  {
    ROS_ERROR( "NaN value received in limit_val. Autopilot most likely will not work after this!");
    return 0.0;
  }

  if( val > upper )
  {
    return upper;
  }

  if( val < lower )
  {
    return lower;
  }

  return val;
}

double clampPi( double angle )
{
  while( angle > PI )
  {
    angle -= PI * 2.0;
  }

  while( angle < -PI )
  {
    angle += PI * 2.0;
  }

  return angle;
}

void getRPY(const tf::Quaternion& tf_q, double &r, double &p, double &y )
{
  tf::Matrix3x3(tf_q).getEulerYPR( y,p,r );
}

void doDepthCorrection(tf::Vector3 &v_in_global_frame, double &depth_correction_angle ) {

  tf::Vector3 v_cross_z( v_in_global_frame.getY(), -v_in_global_frame.getX(), 0.0 );
  ROS_INFO_COND( display_output_, "v_cross_z vector is: %f, %f, %f", v_cross_z.getX(), v_cross_z.getY(), v_cross_z.getZ() );
  
  tf::Quaternion global_correction;
  
  ROS_INFO_COND( display_output_, "length of v_cross_z is %f", v_cross_z.length2() );

  if( v_cross_z.length2() > 1e-6 )
  {
    double corr_r, corr_p, corr_y;
    global_correction.setRotation( v_cross_z, depth_correction_angle  );
    getRPY(global_correction, corr_r, corr_p, corr_y);
    ROS_INFO_COND( display_output_, "The global correction for depth will be:\n   roll: %f, pitch: %f, yaw: %f", corr_r, corr_p, corr_y );

    rotation_from_target_to_global_ = global_correction * rotation_from_target_to_global_;
  }
  else
  {
    ROS_INFO_COND( display_output_, "Not attempting to adjust global rotation because speed command too small.");
  }

  double target_r_in_global, target_p_in_global, target_y_in_global;
  getRPY(rotation_from_target_to_global_, target_r_in_global, target_p_in_global, target_y_in_global);
  ROS_INFO_COND( display_output_, "The depth-updated target angles in global coordinates will be:\n   roll: %f, pitch: %f, yaw: %f",
            target_r_in_global, target_p_in_global, target_y_in_global);

  stat.resultant_roll = target_r_in_global * 180.0 / PI;
  stat.resultant_pitch = target_p_in_global * 180.0 / PI;
  stat.resultant_yaw = target_y_in_global * 180.0 / PI;

  return;
}

void applyGains( aquacore::Command &errors, aquacore::Command &commands )
{
  double value;

  stat.roll_error = errors.roll;
  stat.pitch_error = errors.pitch;
  stat.yaw_error = errors.yaw;

  stat.roll_p_gain  = params_.ROLL_P_GAIN; 
  stat.pitch_p_gain = params_.PITCH_P_GAIN;
  stat.yaw_p_gain   = params_.YAW_P_GAIN;
  stat.roll_i_gain  = params_.ROLL_I_GAIN;
  stat.pitch_i_gain = params_.PITCH_I_GAIN;
  stat.yaw_i_gain   = params_.YAW_I_GAIN;
  stat.roll_d_gain  = params_.ROLL_D_GAIN;
  stat.pitch_d_gain = params_.PITCH_D_GAIN;
  stat.yaw_d_gain   = params_.YAW_D_GAIN;
  stat.roll_const_gain = params_.ROLL_CONST_GAIN;

  // Compute Integral Errors
  integral_errors_.roll =  limit_val( integral_errors_.roll + errors.roll * update_period_, 
              -params_.MAX_INTEGRAL_ANGLE_ERROR, params_.MAX_INTEGRAL_ANGLE_ERROR );

  integral_errors_.pitch = limit_val( integral_errors_.pitch + errors.pitch * update_period_, 
              -params_.MAX_INTEGRAL_ANGLE_ERROR, params_.MAX_INTEGRAL_ANGLE_ERROR );

  integral_errors_.yaw =   limit_val( integral_errors_.yaw + errors.yaw * update_period_, 
              -params_.MAX_INTEGRAL_ANGLE_ERROR, params_.MAX_INTEGRAL_ANGLE_ERROR );

  stat.roll_error_integral = integral_errors_.roll;
  stat.pitch_error_integral = integral_errors_.pitch;
  stat.yaw_error_integral = integral_errors_.yaw;

  stat.roll_p_contrib = params_.ROLL_P_GAIN * errors.roll; 
  stat.pitch_p_contrib = params_.PITCH_P_GAIN * errors.pitch;  
  stat.yaw_p_contrib = params_.YAW_P_GAIN * errors.yaw;

  stat.roll_i_contrib = params_.ROLL_I_GAIN * integral_errors_.roll; 
  stat.pitch_i_contrib = params_.PITCH_I_GAIN * integral_errors_.pitch;
  stat.yaw_i_contrib = params_.YAW_I_GAIN * integral_errors_.yaw;

  stat.roll_d_contrib = -params_.ROLL_D_GAIN * filtered_velocity_.angular.x;
  stat.pitch_d_contrib = -params_.PITCH_D_GAIN * filtered_velocity_.angular.y;
  stat.yaw_d_contrib = -params_.YAW_D_GAIN * filtered_velocity_.angular.z;
  
  stat.roll_const_contrib = params_.ROLL_CONST_GAIN * sin(PI/180.0*stat.current_roll);

  stat.filtered_roll_deriv = filtered_velocity_.angular.x;
  stat.filtered_pitch_deriv = filtered_velocity_.angular.y;
  stat.filtered_yaw_deriv = filtered_velocity_.angular.z;

  stat.roll_d_filter_period = params_.ROLL_D_FILTER_PERIOD;
  stat.pitch_d_filter_period = params_.PITCH_D_FILTER_PERIOD;
  stat.yaw_d_filter_period = params_.YAW_D_FILTER_PERIOD;

  params_mutex_.lock();

  value = stat.roll_p_contrib + stat.roll_i_contrib + stat.roll_d_contrib + stat.roll_const_contrib;
  commands.roll  = limit_val( value, -params_.MAX_ROLL, params_.MAX_ROLL );

  value = stat.pitch_p_contrib + stat.pitch_i_contrib + stat.pitch_d_contrib;
  commands.pitch = limit_val( value, -params_.MAX_PITCH, params_.MAX_PITCH );

  value = stat.yaw_p_contrib + stat.yaw_i_contrib  + stat.yaw_d_contrib;
  commands.yaw   = limit_val( value, -params_.MAX_YAW, params_.MAX_YAW );

  // TODO: Should we also apply PID on these values?
  commands.speed = limit_val( errors.speed, -params_.MAX_SPEED, params_.MAX_SPEED );
  commands.heave = limit_val( errors.heave, -params_.MAX_HEAVE, params_.MAX_HEAVE );

  params_mutex_.unlock();

  ap_status_pub_.publish(stat);
}

void computeFinalCommands( aquacore::Command &raw_command, aquacore::Command &updated_command ) {
  double target_r_in_global, target_p_in_global, target_y_in_global;
  getRPY(rotation_from_target_to_global_, target_r_in_global, target_p_in_global, target_y_in_global);
  ROS_INFO_COND( display_output_, "Resultant target angles in global frame:\n   roll: %f   pitch: %f   yaw: %f",
     180.0/PI*target_r_in_global, 180.0/PI*target_p_in_global, 180.0/PI*target_y_in_global);

  tf::Quaternion rotation_from_target_to_imu_ = rotation_from_global_to_imu_ * rotation_from_target_to_global_;
  // R_T^I = R_G^I * R_T^G  (makes sense)

  // Now: I LIKE TO SLERP IT SLERP IT!: http://www.youtube.com/watch?v=Dyx4v1QFzhQ
  std::vector<geometry_msgs::PoseStamped> poses;
  for( double interp = 0.1; interp <= 0.9; interp += 0.2 )
  {

    geometry_msgs::PoseStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "/latest_fix";

    cmd.pose.position.x = 0; //curr_speed_;
    cmd.pose.position.y = 0; //curr_heave_;
    cmd.pose.position.z = 0; //curr_depth_;

    tf::Quaternion slerp_rotation_from_target_to_global = rotation_from_imu_to_global_.slerp(rotation_from_target_to_global_, interp );
    tf::quaternionTFToMsg(slerp_rotation_from_target_to_global, cmd.pose.orientation);

    poses.push_back( cmd );
  }

  vis_pub1_.publish(poses[0]);
  vis_pub2_.publish(poses[1]);
  vis_pub3_.publish(poses[2]);
  vis_pub4_.publish(poses[3]);
  vis_pub5_.publish(poses[4]);

  tf::Quaternion identity(0, 0, 0, 1);
  tf::Quaternion rotation_from_slerp_target_to_imu = identity.slerp(rotation_from_target_to_imu_, 0.3); // TODO: Like this to be a param
  tf::Quaternion commanded_rotation_in_imu;

  if (use_slerp) {
    commanded_rotation_in_imu = rotation_from_slerp_target_to_imu;
  } else {
    commanded_rotation_in_imu = rotation_from_target_to_imu_;
  }
 
  double target_r_in_imu, target_p_in_imu, target_y_in_imu;
  getRPY(commanded_rotation_in_imu, target_r_in_imu, target_p_in_imu, target_y_in_imu);
  
  raw_command.roll  = clampPi( target_r_in_imu );
  raw_command.pitch = clampPi( target_p_in_imu );
  raw_command.yaw   = clampPi( target_y_in_imu );
  ROS_INFO_COND( display_output_, "The desired angle changes in IMU frame are:\n   roll: %f   pitch: %f   yaw: %f",
     180.0/PI*raw_command.roll, 180.0/PI*raw_command.pitch, 180.0/PI*raw_command.yaw);

  //transform_.setOrigin( tf::Vector3( 0,0,0 ));
  //transform_.setRotation(  rotation_from_target_to_imu_ );
  //br_.sendTransform( tf::StampedTransform( transform_, ros::Time::now(), "/aqua_base", "/auto_pilot_target_in_imu"));

  //transform_.setRotation(  commanded_rotation_in_imu );
  //br_.sendTransform( tf::StampedTransform( transform_, ros::Time::now(), "/aqua_base", "/auto_pilot_command_in_imu"));

  //transform_.setRotation(rotation_from_target_to_global_);
  //br_.sendTransform( tf::StampedTransform( transform_, ros::Time::now(), "/latest_fix", "/auto_pilot_target_in_global_frame"));

  ROS_INFO_COND( display_output_, "Using gains of:\n   KDEPTH=%f   KSPEED=%f   KHEAVE=%f",
            params_.KDEPTH, params_.KSPEED, params_.KHEAVE);

  ROS_INFO_COND( display_output_, "   ROLL_P=%f   PITCH_P=%f   YAW_P=%f ",
            params_.ROLL_P_GAIN, params_.PITCH_P_GAIN, params_.YAW_P_GAIN );

  ROS_INFO_COND( display_output_, "   ROLL_I=%f   PITCH_I=%f   YAW_I=%f",
            params_.ROLL_I_GAIN, params_.PITCH_I_GAIN, params_.YAW_I_GAIN );

  ROS_INFO_COND( display_output_, "   ROLL_D=%f   PITCH_D=%f   YAW_D=%f",
            params_.ROLL_D_GAIN, params_.PITCH_D_GAIN, params_.YAW_D_GAIN );

  //ROS_INFO_COND( display_output_, "Using max's of:\n   MAX_SPEED=%f  MAX_HEAVE=%f   MAX_ROLL=%f  MAX_PITCH=%f  MAX_YAW=%f",
  //          params_.MAX_SPEED, params_.MAX_HEAVE, params_.MAX_ROLL, params_.MAX_PITCH, params_.MAX_YAW);

  applyGains( raw_command, updated_command );

  return;
}

void doAutopilotUpdate()
{
  if (curr_auto_mode_ == aquacore::AutopilotModes::AP_OFF) {
    ROS_INFO_COND( display_output_, "MODE 0 - Autopilot is disabled, not publishing cmd");
    return;
  }
  
  // For safety default command is zero unless code explicitly changes it:
  aquacore::Command updated_command;
  double depth_error, depth_correction_angle;

  ros::Duration target_age = ( ros::Time::now() - current_target_.header.stamp );
  ros::Duration depth_age = (ros::Time::now() - last_depth_reading_time_);

  updated_command.roll = 0.0;
  updated_command.pitch = 0.0;
  updated_command.yaw = 0.0;
  updated_command.speed = 0.0;
  updated_command.heave = 0.0;

  if( !have_velocity_ )
  {
    ROS_INFO_STREAM_COND( display_output_, "Sending zero command because angular velocity not received yet. have_velocity_:"
                      << have_velocity_  );
    // Note, commands still zero here. Zero will be intentionally published
  }
  else if( !have_target_ || target_age > target_lifetime_ ) {
    ROS_INFO_STREAM_COND( display_output_, "Sending zero command because no fresh target available. have_target_: "
                      << have_target_ << " and target_age: " << target_age );

    // Note, commands still zero here. Zero will be intentionally published
  }
  else if ( !have_depth_ || depth_age > depth_lifetime_ )
  {
    ROS_INFO_STREAM( "Sending zero command because no fresh depth reading. have_depth_: "
                      << have_depth_ << " and depth_age: " << depth_age );
    // Note, commands still zero here. Zero will be intentionally published
  }
  else
  {

    // This block is where all of the actual control commands are calculated. Our inputs are good, git going.
    aquacore::Command raw_command;
    raw_command.roll=0.0;
    raw_command.pitch=0.0;
    raw_command.yaw=0.0;
    raw_command.speed=0.0;
    raw_command.heave=0.0;

    double target_r_in_global, target_p_in_global, target_y_in_global;
    double curr_r_in_global, curr_p_in_global, curr_y_in_global;
    // First step, basic parsing of the target and retrieve the IMU data
    tf::quaternionMsgToTF(current_target_.pose.orientation, rotation_from_target_to_global_);
    getRPY(rotation_from_target_to_global_, target_r_in_global, target_p_in_global, target_y_in_global);
    ROS_INFO_COND( display_output_, "Fresh user target is:\n   speed:%f\n   heave:%f   depth:%f\n   roll:%f   pitch:%f   yaw:%f\n   age: %f.",
              current_target_.pose.position.x, current_target_.pose.position.y, current_target_.pose.position.z,
              180.0/PI*target_r_in_global, 180.0/PI*target_p_in_global, 180.0/PI*target_y_in_global,
              target_age.toSec() );

    stat.roll_target = 180.0/PI*target_r_in_global;
    stat.pitch_target = 180.0/PI*target_p_in_global;
    stat.yaw_target = 180.0/PI*target_y_in_global;

    tf::StampedTransform transform_from_global_to_imu;
    tf::Vector3 v_in_imu_frame(0,0,0);
    tf::Vector3 v_in_global_frame(0,0,0);

    try
    {

      // Can block until the transform is available if want one for a specific time.
      // This code was related to taking differences between angles at different times.
      // Commented because we currently just get the latest, no matter what its time
      // ros::Time now = ros::Time::now();
      // ros::Duration one_tenth(0.1);
      //listener_.waitForTransform("/aqua_base", "/latest_fix", now, one_tenth);
      //listener_.lookupTransform("/aqua_base", "/latest_fix", ros::Time::now(), transform_from_global_to_imu);

      listener_.lookupTransform("/aqua_base", "/latest_fix", ros::Time(0), transform_from_global_to_imu);
      rotation_from_global_to_imu_ = transform_from_global_to_imu.getRotation();
      rotation_from_imu_to_global_ = transform_from_global_to_imu.inverse().getRotation();
      
      getRPY(rotation_from_imu_to_global_, curr_r_in_global, curr_p_in_global, curr_y_in_global);
      ROS_INFO_COND( display_output_, "Robot rotation in global frame:\n   roll: %f   pitch: %f   yaw: %f.",
                180.0/PI*curr_r_in_global, 180.0/PI*curr_p_in_global, 180.0/PI*curr_y_in_global);

      stat.current_roll = 180.0/PI*curr_r_in_global;
      stat.current_pitch = 180.0/PI*curr_p_in_global;
      stat.current_yaw = 180.0/PI*curr_y_in_global;

      // Dave note: I had implemented this method to estimate the robot's velocities by taking
      //            difference of angles over history. However, the IMU reads angular vel directly
      //            so chose to use that at the moment, see velocityCallback etc
//      // TODO: Verify with Phil and on the robot that these computed rates are somewhat similar to those
//      //       input to the AP update function in RD
//      tf::StampedTransform previous_transform;
//      listener_.lookupTransform("/aqua_base", "/latest_fix", now - one_tenth, previous_transform);
//      rotation_from_previous_imu_to_global_ = previous_transform.inverse().getRotation();
//      tf::Quaternion rotation_from_previous_imu_to_current_imu_ = rotation_from_global_to_imu_ * rotation_from_previous_imu_to_global_;
//      // TODO: R_P^C = R_G^C * R_P^G  (Florian verify?)
//
//      getRPY(rotation_from_previous_imu_to_current_imu_, r_rate_in_local_, p_rate_in_local_, y_rate_in_local_);
//      ROS_INFO_COND( display_output_, "The robot is rotating at a rate of:\n   roll: %f   pitch: %f   yaw: %f.",
//               180.0/PI*r_rate_in_local_, 180.0/PI*p_rate_in_local_, 180.0/PI*y_rate_in_local_);

      // Now, process the command request based on the current target and mode from the user:

      double depth_d_contrib;
      switch(curr_auto_mode_)
      {
      case aquacore::AutopilotModes::AP_OFF:

        // MODE 0: Off
        ROS_INFO_COND( display_output_, "MODE 0 - Autopilot is off, sending 0 commands.");
        break;

      case aquacore::AutopilotModes::AP_GLOBAL_ANGLES_LOCAL_THRUST:

        // MODE 1: User commands all 3 angles in global frame (pass-through request to target). Speed and heave are in local frame (also pass-through).
        ROS_INFO_COND( display_output_, "MODE %d - Global Angle Commands with Local Thrusts", curr_auto_mode_);
        raw_command.speed = current_target_.pose.position.x;
        raw_command.heave =  current_target_.pose.position.y;

        computeFinalCommands( raw_command, updated_command);
        break;

      case aquacore::AutopilotModes::AP_GLOBAL_ANGLES_FIXED_DEPTH:
        // MODE 4: The robot tries to maintain a fixed depth by regulation with the depth sensor.
        //         Target angle is interpreted in global frame.
        //         Thrust command is local and must be planar so that it does not conflict with constant depth (TODO: how is this checked/reported?)
        //         Depth regulation takes priority over target angles, so angles modified to keep depth steady.

        ROS_INFO_COND( display_output_, "MODE %d - Depth Regulated Swimming at Global Target Angle.",curr_auto_mode_);
        raw_command.speed = current_target_.pose.position.x;
        raw_command.heave =  current_target_.pose.position.y;

        params_mutex_.lock();
        depth_d_contrib = -params_.DEPTH_D_GAIN * filtered_depth_derivative_;
        depth_error =  current_target_.pose.position.z - current_depth;
        depth_correction_angle = limit_val( params_.KDEPTH * depth_error + depth_d_contrib, -PI/4.0, PI/4.0 );

        stat.depth_p_gain = params_.KDEPTH;
        stat.depth_p_contrib = params_.KDEPTH * depth_error;

        stat.depth_d_gain = params_.DEPTH_D_GAIN;
        stat.depth_d_contrib = depth_d_contrib;
        stat.depth_derivative = depth_derivative_;
        stat.filtered_depth_derivative = filtered_depth_derivative_;
        stat.depth_d_filter_period = params_.DEPTH_D_FILTER_PERIOD;
        stat.depth_error = depth_error;

        params_mutex_.unlock();
        ROS_INFO_COND( display_output_, "Correcting for a depth error of: %f with angle of %f.", depth_error, 180.0/PI*depth_correction_angle);

        v_in_imu_frame.setX(raw_command.speed);
        v_in_imu_frame.setZ(raw_command.heave);
        //v_in_global_frame = quatRotate( rotation_from_imu_to_global_, v_in_imu_frame );
        v_in_global_frame = quatRotate( rotation_from_target_to_global_, v_in_imu_frame );

        doDepthCorrection(v_in_global_frame, depth_correction_angle);
        computeFinalCommands(raw_command, updated_command);
        break;

      default:
        ROS_ERROR("WARNING: Autopilot mode %d is not yet implemented. Sending 0 commands.", curr_auto_mode_);
        break;
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("WARNING: TF exception: %s. Using old transformation information!!!",ex.what());
    }
  }

  latest_cmd_ = updated_command;
  ROS_INFO_COND( display_output_, "Publishing command: r: %f, p: %f, y: %f, speed: %f, heave: %f",
  latest_cmd_.roll, latest_cmd_.pitch, latest_cmd_.yaw,
  latest_cmd_.speed, latest_cmd_.heave );

  cmd_pub_.publish(latest_cmd_);
}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aqua_local_autopilot");
    LocalAutopilot la;
    la.spin();
}
