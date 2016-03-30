#include <ros/ros.h>
#include <string>

#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <aquacore/Command.h>
#include <aquacore/SetGait.h>
#include <aquacore/GetState.h>
#include <aquacore/AutopilotModes.h>
#include <aquacore/SetAutopilotMode.h>
#include <aquacore/IsCalibrated.h>
#include <aquajoy/AquaTeleopJoyState.h>
#include <aquajoy/AquaTeleopJoyParams.h>
#include <aquajoy/UpdateAquaTeleopJoyParams.h>
#include <aquajoy/AquaTeleopJoyConfig.h>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btQuaternion.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>


#define DEGREE (3.1415926535897932384626433832795/180.0)


using namespace aquajoy;
typedef dynamic_reconfigure::Server<AquaTeleopJoyConfig> ReconfigureServer;


struct JoyState {
  // Axes
  double LX; // right-positive
  double LY; // up-positive
  double RX; // right-positive
  double RY; // up-positive
  
  // Buttons
  bool A;
  bool B;
  bool X;
  bool Y;
  bool L1;
  bool L2;
  bool L3; // click into left analog stick
  bool R1;
  bool R2;
  bool R3; // click into right analog stick
  bool Start;
  bool Select;
  
  // D-pad
  bool DL;
  bool DR;
  bool DU;
  bool DD;
  
  bool fromLogitechJoyMsg(const sensor_msgs::Joy::ConstPtr& joy, double deadzone) {
    if (joy->axes.size() == 6 && joy->buttons.size() == 12) {
      LX = -joy->axes[0];
      LY = joy->axes[1];
      RX = -joy->axes[2];
      RY = joy->axes[3];
      if (LX >= -deadzone && LX <= deadzone) LX = 0;
      if (LY >= -deadzone && LY <= deadzone) LY = 0;
      if (RX >= -deadzone && RX <= deadzone) RX = 0;
      if (RY >= -deadzone && RY <= deadzone) RY = 0;
      
      A = joy->buttons[1];
      B = joy->buttons[2];
      X = joy->buttons[0];
      Y = joy->buttons[3];
      L1 = joy->buttons[4];
      L2 = joy->buttons[6];
      L3 = joy->buttons[10];
      R1 = joy->buttons[5];
      R2 = joy->buttons[7];
      R3 = joy->buttons[11];
      Start = joy->buttons[9];
      Select = joy->buttons[8];
      
      DL = (joy->axes[4] == 1);
      DR = (joy->axes[4] == -1);
      DU = (joy->axes[5] == 1);
      DD = (joy->axes[5] == -1);
      return true;
    }
    return false;
  };
};


double wrapAngleDeg(double angleDeg) {
  return (angleDeg - floor(angleDeg/360.0)*360.0);
};


std::string toStr(char ctrl_mode) {
  std::string str = "UNKNOWN";
  #define TO_STR(v) if (ctrl_mode == AquaTeleopJoyState::v) { str = #v; }
  TO_STR(CTRL_MODE_RAW);
  TO_STR(CTRL_MODE_GLOBAL_RP_POS);
  TO_STR(CTRL_MODE_GLOBAL_RP_POS_D);
  TO_STR(CTRL_MODE_GLOBAL_RP_VEL_D);
  TO_STR(CTRL_MODE_FLATSWIM_D);
  #undef TO_STR
  return str;
};


AquaTeleopJoyParams toMsg(const AquaTeleopJoyConfig& cfg) {
  AquaTeleopJoyParams msg;
  #define COPY(v) msg.v = cfg.v
  COPY(joy_axis_deadzone);
  COPY(max_speed_cmd);
  COPY(max_heave_cmd);
  COPY(max_roll_cmd);
  COPY(max_pitch_cmd);
  COPY(max_yaw_cmd);
  COPY(max_roll_pos);
  COPY(max_pitch_pos);
  COPY(max_yaw_pos);
  COPY(min_depth);
  COPY(max_depth);
  COPY(max_roll_vel);
  COPY(max_pitch_vel);
  COPY(max_yaw_vel);
  COPY(max_depth_vel);
  #undef COPY
  return msg;
};


class AquaTeleopJoy {
public:
  AquaTeleopJoy(): nh("~"), lastStateUpdateT(ros::Time()), eStopOn(false), dyncfgSyncRequest(false) {
    // Resolve remapped names or use defaults
    std::string joyTopic = nh.resolveName("joy");
    std::string cmdTopic = nh.resolveName("command");
    std::string targetPoseTopic = nh.resolveName("target_pose");
    std::string getStateService = nh.resolveName("get_state");
    std::string setAPModeService = nh.resolveName("set_autopilot_mode");
    std::string joyStateTopic = nh.resolveName("joy_state");
    std::string joyParamsTopic = nh.resolveName("joy_params");
    std::string updateJoyParamsService = nh.resolveName("update_joy_params");
    std::string calibrateTopic = nh.resolveName("calibrate");
    std::string isCalibratedService = nh.resolveName("is_calibrated");
    std::string setGaitService = nh.resolveName("set_gait");
    std::string setRoboDevelAPModeService = nh.resolveName("set_robodevel_autopilot_mode");
    if (joyTopic == nh.getNamespace() + "/joy") joyTopic = "/joy";
    if (cmdTopic == nh.getNamespace() + "/command") cmdTopic = "/aqua/command";
    if (targetPoseTopic == nh.getNamespace() + "/target_pose") targetPoseTopic = "/aqua/target_pose";
    if (getStateService == nh.getNamespace() + "/get_state") getStateService = "/aqua/get_state";
    if (setAPModeService == nh.getNamespace() + "/set_autopilot_mode") setAPModeService = "/aqua/set_3Dauto_mode";
    if (calibrateTopic == nh.getNamespace() + "/calibrate") calibrateTopic = "/aqua/calibrate";
    if (isCalibratedService == nh.getNamespace() + "/is_calibrated") isCalibratedService = "/aqua/is_calibrated";
    if (setGaitService == nh.getNamespace() + "/set_gait") setGaitService = "/aqua/set_gait";
    if (setRoboDevelAPModeService == nh.getNamespace() + "/set_robodevel_autopilot_mode") setRoboDevelAPModeService = "/aqua/set_autopilot_mode";
    
    // Initialize ROS hooks
    joySub = nh.subscribe<sensor_msgs::Joy>(joyTopic, 1, &AquaTeleopJoy::joyCallback, this);
    cmdPub = nh.advertise<aquacore::Command>(cmdTopic, 1);
    apTargetPub = nh.advertise<geometry_msgs::PoseStamped>(targetPoseTopic, 1);
    getStateCln = nh.serviceClient<aquacore::GetState>(getStateService);
    apModeCln = nh.serviceClient<aquacore::SetAutopilotMode>(setAPModeService);
    statePub = nh.advertise<aquajoy::AquaTeleopJoyState>(joyStateTopic, 1);
    paramsPub = nh.advertise<aquajoy::AquaTeleopJoyParams>(joyParamsTopic, 1);
    aquaCalibrateCln = nh.serviceClient<std_srvs::Empty>(calibrateTopic);
    aquaIsCalibratedCln = nh.serviceClient<aquacore::IsCalibrated>(isCalibratedService);
    aquaSetGaitCln = nh.serviceClient<aquacore::SetGait>(setGaitService);
    aquaSetAutopilotModeCln = nh.serviceClient<aquacore::SetAutopilotMode>(setRoboDevelAPModeService);
    updateParamsSrv = nh.advertiseService(updateJoyParamsService, &AquaTeleopJoy::updateParams, this);
    
    runAquaStartupSequence();
    
    // Setup dynamic reconfigure server
    // NOTE: this should be set up prior to loading (static) ROS params, so that launch file param values gets prioritized over older dyncfg values
    dyncfgServer = new ReconfigureServer(paramsMutex, nh);
    dyncfgServer->setCallback(bind(&AquaTeleopJoy::configCallback, this, _1, _2));
    
    // Query ROS parameters
    double ctrlPubRate = 10.0;
    double paramsPubRate = 1/5.0;
    paramsMutex.lock();
    nh.param<double>("ctrl_pub_rate", ctrlPubRate, ctrlPubRate);
    nh.param<double>("params_pub_rate", paramsPubRate, paramsPubRate);
    nh.param<std::string>("rpy_order", rpyOrder, "rpy");
    #define PARAM(v, d) nh.param<double>(#v, params.v, d)
    PARAM(joy_axis_deadzone, 0.25);
    PARAM(max_speed_cmd, 1.0);
    PARAM(max_heave_cmd, 1.0);
    PARAM(max_roll_cmd, 1.0);
    PARAM(max_pitch_cmd, 1.0);
    PARAM(max_yaw_cmd, 1.0);
    PARAM(max_roll_pos, 90.0);
    PARAM(max_pitch_pos, 90.0);
    PARAM(max_yaw_pos, 180.0);
    PARAM(min_depth, 0.0);
    PARAM(max_depth, 100.0);
    PARAM(max_roll_vel, 90.0);
    PARAM(max_pitch_vel, 90.0);
    PARAM(max_yaw_vel, 180.0);
    PARAM(max_depth_vel, 0.5);
    #undef PARAM
    paramsMutex.unlock();
    dyncfgSyncRequest = true;
    
    // Set default state
    stateMutex.lock();
    state.ctrl_mode = AquaTeleopJoyState::CTRL_MODE_RAW;
    state.modifier = false;
    state.raw_cmd.speed = 0;
    state.raw_cmd.heave = 0;
    state.raw_cmd.roll = 0;
    state.raw_cmd.pitch = 0;
    state.raw_cmd.yaw = 0;
    state.roll_pos = 0;
    state.pitch_pos = 0;
    state.yaw_pos = 0;
    state.roll_vel = 0;
    state.pitch_vel = 0;
    state.yaw_vel = 0;
    state.depth_pos = 0;
    state.depth_vel = 0;
    stateMutex.unlock();
    if (getAquaCurrentState()) {
      stateMutex.lock();
      state.yaw_pos = getStateTyp.response.state.YawAngle;
      state.depth_pos = getStateTyp.response.state.Depth;
      stateMutex.unlock();
    }
    updateAPMode(aquacore::AutopilotModes::AP_OFF);
    
    // Setup repeat timers
    ctrlPubTimer = nh.createTimer(ros::Duration(1.0/ctrlPubRate), &AquaTeleopJoy::publishCtrl, this);
    paramsPubTimer = nh.createTimer(ros::Duration(1.0/paramsPubRate), &AquaTeleopJoy::publishParams, this);
    
    // Notify user
    ROS_INFO_STREAM(ros::this_node::getName() << " initialized");
  };
  

protected:
  void runAquaStartupSequence() {
    std_srvs::Empty emptySrv;

    // Check for calibration
    ROS_INFO_STREAM("Waiting for " << aquaIsCalibratedCln.getService() << "...");
    if (!aquaIsCalibratedCln.waitForExistence()) { return; }
    aquacore::IsCalibrated isCalibratedSrv;
    if (aquaIsCalibratedCln.call(isCalibratedSrv)) {
      if (!isCalibratedSrv.response.value) {
        ROS_INFO_STREAM("Waiting for " << aquaCalibrateCln.getService() << "...");
        if (!aquaCalibrateCln.waitForExistence()) { return; }
        if (aquaCalibrateCln.call(emptySrv)) {
          ROS_INFO_STREAM("Calibrated Aqua");
        } else {
          ROS_INFO_STREAM("Failed to calibrate Aqua");
        }
      } else {
        ROS_INFO_STREAM("Confirmed that Aqua is already calibrated");
      }
    } else {
      ROS_INFO_STREAM("Failed to query " << aquaIsCalibratedCln.getService());
      return;
    }

    // Update to swimming gait
    ROS_INFO_STREAM("Waiting for " << aquaSetGaitCln.getService() << "...");
    if (!aquaSetGaitCln.waitForExistence()) { return; }
    aquacore::SetGait setGaitSrv;
    setGaitSrv.request.gait = "hover-midoff";
    if (aquaSetGaitCln.call(setGaitSrv)) {
      ROS_INFO_STREAM("Set Aqua's gait to " << setGaitSrv.request.gait);
    } else {
      ROS_ERROR_STREAM("Failed to set Aqua's gait to " << setGaitSrv.request.gait);
      return;
    }

    // Set autopilot mode
    ROS_INFO_STREAM("Waiting for " << aquaSetAutopilotModeCln.getService() << "...");
    if (!aquaSetAutopilotModeCln.waitForExistence()) { return; }
    aquacore::SetAutopilotMode setAutopilotModeSrv;
    setAutopilotModeSrv.request.mode = aquacore::AutopilotModes::OFF;
    if (aquaSetAutopilotModeCln.call(setAutopilotModeSrv) && setAutopilotModeSrv.response.response) {
      ROS_INFO_STREAM("Set Aqua's autopilot to " << setAutopilotModeSrv.request.mode);
    } else {
      ROS_ERROR_STREAM("Failed to set Aqua's autopilot to " << setAutopilotModeSrv.request.mode);
      return;
    }

    // Send zero speed command
    state.raw_cmd.speed = 0;
    state.raw_cmd.yaw = 0;
    state.raw_cmd.pitch = 0;
    state.raw_cmd.roll = 0;
    state.raw_cmd.heave = 0;
    cmdPub.publish(state.raw_cmd);
    ROS_INFO_STREAM("Sent zero speed command");
  };


  bool getAquaCurrentState() {
    if (getStateCln.call(getStateTyp)) {
      return true;
    } else {
      ROS_ERROR_STREAM("Failed to query Aqua's current state");
    }
    return false;
  };
  
  
  bool updateAPMode(int mode) {
    apModeTyp.request.mode = mode;
    if (apModeCln.call(apModeTyp)) {
      ROS_DEBUG_STREAM("Set AP mode to " << apModeTyp.request.mode << ", received " << (int) apModeTyp.response.response);
      return true;
    } else {
      ROS_ERROR_STREAM("Failed to set AP mode");
    }
    return false;
  };
  

  void configCallback(AquaTeleopJoyConfig& config, uint32_t level) {
    paramsMutex.lock();
    params = config;
    paramsMutex.unlock();
  };
   

  bool updateParams(UpdateAquaTeleopJoyParams::Request& req, UpdateAquaTeleopJoyParams::Response& res) {
    bool changed = false;
    
    paramsMutex.lock();
    #define UPDATE_PARAM(v) if (req.new_params.v != params.v) { changed = true; params.v = req.new_params.v; }
    UPDATE_PARAM(joy_axis_deadzone);
    UPDATE_PARAM(max_speed_cmd);
    UPDATE_PARAM(max_heave_cmd);
    UPDATE_PARAM(max_roll_cmd);
    UPDATE_PARAM(max_pitch_cmd);
    UPDATE_PARAM(max_yaw_cmd);
    UPDATE_PARAM(max_roll_pos);
    UPDATE_PARAM(max_pitch_pos);
    UPDATE_PARAM(max_yaw_pos);
    UPDATE_PARAM(min_depth);
    UPDATE_PARAM(max_depth);
    UPDATE_PARAM(max_roll_vel);
    UPDATE_PARAM(max_pitch_vel);
    UPDATE_PARAM(max_yaw_vel);
    UPDATE_PARAM(max_depth_vel);
    #undef UPDATE_PARAM
    paramsMutex.unlock();
    
    if (changed) {
      dyncfgSyncRequest = true;
      paramsPub.publish(toMsg(params));
    }
    
    return true;
  };
  

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    // Parse joypad message
    if (!joypad.fromLogitechJoyMsg(joy, params.joy_axis_deadzone)) {
      ROS_WARN_STREAM(ros::this_node::getName() << "does not support current joystick (" << joy->axes.size() << " axes, " << joy->buttons.size() << " buttons)");
      return;
    }
    
    // Process buttons
    stateMutex.lock();
    //bool modifierLock = (!state.modifier && joypad.R2);
    //bool modifierUnlock = (state.modifier && !joypad.R2);
    state.modifier = joypad.R2;
    stateMutex.unlock();
    if (joypad.B) { // Implement E-Stop
      if (!eStopOn) {
        eStopOn = true;
        stateMutex.lock();
        state.ctrl_mode = AquaTeleopJoyState::CTRL_MODE_RAW;
        state.raw_cmd.speed = 0;
        state.raw_cmd.heave = 0;
        state.raw_cmd.roll = 0;
        state.raw_cmd.pitch = 0;
        state.raw_cmd.yaw = 0;
        state.roll_pos = 0;
        state.pitch_pos = 0;
        state.yaw_pos = 0;
        state.roll_vel = 0;
        state.pitch_vel = 0;
        state.yaw_vel = 0;
        state.depth_pos = 0;
        state.depth_vel = 0;
        lastStateUpdateT = ros::Time::now();
        stateMutex.unlock();
        updateAPMode(aquacore::AutopilotModes::AP_OFF);
        cmdPub.publish(state.raw_cmd);
        if (getAquaCurrentState()) {
          stateMutex.lock();
          state.yaw_pos = getStateTyp.response.state.YawAngle;
          state.depth_pos = getStateTyp.response.state.Depth;
          stateMutex.unlock();
        }
      }
      return;
    } else { // Reset E-Stop when B button released
      eStopOn = false;
    }
    if (joypad.X) { // Implement Flat-Pos (no gating needed)
      stateMutex.lock();
      state.roll_pos = 0;
      state.pitch_pos = 0;
      lastStateUpdateT = ros::Time::now();
      stateMutex.unlock();
    } else if (joypad.Y) { // Implement Zero-Depth (no gating needed)
      stateMutex.lock();
      state.depth_pos = 0;
      lastStateUpdateT = ros::Time::now();
      stateMutex.unlock();
    }
    
    // Update control mode
    stateMutex.lock();
    if ((state.ctrl_mode != AquaTeleopJoyState::CTRL_MODE_RAW) &&
        (!joypad.L1 && !joypad.L2 && !joypad.R1)) {
      state.ctrl_mode = AquaTeleopJoyState::CTRL_MODE_RAW;
      updateAPMode(aquacore::AutopilotModes::AP_OFF);
      ROS_DEBUG_STREAM("Switched to " << toStr(state.ctrl_mode));
      
    } else if ((state.ctrl_mode != AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_POS) &&
        (joypad.L1 && !joypad.L2 && !joypad.R1)) {
      state.ctrl_mode = AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_POS;
      updateAPMode(aquacore::AutopilotModes::AP_GLOBAL_ANGLES_LOCAL_THRUST);
      if (getAquaCurrentState()) {
        state.yaw_pos = getStateTyp.response.state.YawAngle;
      }
      ROS_DEBUG_STREAM("Switched to " << toStr(state.ctrl_mode));
      
    } else if ((state.ctrl_mode != AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_POS_D) &&
        (!joypad.L1 && joypad.L2 && !joypad.R1)) {
      state.ctrl_mode = AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_POS_D;
      updateAPMode(aquacore::AutopilotModes::AP_GLOBAL_ANGLES_FIXED_DEPTH);
      if (getAquaCurrentState()) {
        state.yaw_pos = getStateTyp.response.state.YawAngle;
        state.depth_pos = getStateTyp.response.state.Depth;
      }
      ROS_DEBUG_STREAM("Switched to " << toStr(state.ctrl_mode));
      
    } else if ((state.ctrl_mode != AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_VEL_D) &&
        (joypad.L1 && joypad.L2 && !joypad.R1)) {
      state.ctrl_mode = AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_VEL_D;
      updateAPMode(aquacore::AutopilotModes::AP_GLOBAL_ANGLES_FIXED_DEPTH);
      if (getAquaCurrentState()) {
        state.yaw_pos = getStateTyp.response.state.YawAngle;
        state.depth_pos = getStateTyp.response.state.Depth;
      }
      ROS_DEBUG_STREAM("Switched to " << toStr(state.ctrl_mode));
      
    } else if ((state.ctrl_mode != AquaTeleopJoyState::CTRL_MODE_FLATSWIM_D) &&
        (joypad.R1)) {
      state.ctrl_mode = AquaTeleopJoyState::CTRL_MODE_FLATSWIM_D;
      updateAPMode(aquacore::AutopilotModes::AP_GLOBAL_ANGLES_FIXED_DEPTH);
      if (getAquaCurrentState()) {
        state.yaw_pos = getStateTyp.response.state.YawAngle;
        state.depth_pos = getStateTyp.response.state.Depth;
      }
      ROS_DEBUG_STREAM("Switched to " << toStr(state.ctrl_mode));
      
    }
    
    // Process axes, and handle modifier-lock/unlock
    paramsMutex.lock();
    switch (state.ctrl_mode) {
      case AquaTeleopJoyState::CTRL_MODE_RAW:
      default:
        state.raw_cmd.speed = joypad.LY*params.max_speed_cmd;
        state.raw_cmd.roll = joypad.LX*params.max_roll_cmd;
        state.raw_cmd.yaw = joypad.RX*params.max_yaw_cmd;
        if (state.modifier) {
          state.raw_cmd.heave = joypad.RY*params.max_heave_cmd;
          state.raw_cmd.pitch = 0;
        } else {
          state.raw_cmd.heave = 0;
          state.raw_cmd.pitch = joypad.RY*params.max_pitch_cmd;
        }
        break;
      case AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_POS:
        state.raw_cmd.speed = joypad.LY*params.max_speed_cmd;
        state.yaw_vel = -joypad.RX*params.max_yaw_vel;
        if (state.modifier) {
          // Lock roll_pos and pitch_pos
          state.roll_vel = 0;
          state.pitch_vel = 0;
          state.raw_cmd.heave = joypad.RY*params.max_heave_cmd;
        } else {
          state.roll_pos = joypad.LX*params.max_roll_pos;
          state.pitch_pos = joypad.RY*params.max_pitch_pos;
          state.roll_vel = 0;
          state.pitch_vel = 0;
          state.raw_cmd.heave = 0;
        }
        break;
      case AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_POS_D:
        state.raw_cmd.speed = joypad.LY*params.max_speed_cmd;
        state.yaw_vel = -joypad.RX*params.max_yaw_vel;
        if (state.modifier) {
          // Lock roll_pos and pitch_pos
          state.roll_vel = 0;
          state.pitch_vel = 0;
          state.raw_cmd.heave = 0;
          state.depth_vel = -joypad.RY*params.max_depth_vel;
        } else {
          state.roll_pos = joypad.LX*params.max_roll_pos;
          state.pitch_pos = joypad.RY*params.max_pitch_pos;
          state.roll_vel = 0;
          state.pitch_vel = 0;
          state.raw_cmd.heave = 0;
          state.depth_vel = 0;
        }
        break;
      case AquaTeleopJoyState::CTRL_MODE_GLOBAL_RP_VEL_D:
        state.raw_cmd.speed = joypad.LY*params.max_speed_cmd;
        state.roll_vel = joypad.LX*params.max_roll_vel;
        state.yaw_vel = -joypad.RX*params.max_yaw_vel;
        if (state.modifier) {
          state.pitch_vel = 0;
          state.raw_cmd.heave = 0;
          state.depth_vel = -joypad.RY*params.max_depth_vel;
        } else {
          state.pitch_vel = joypad.RY*params.max_pitch_vel;
          state.raw_cmd.heave = 0;
          state.depth_vel = 0;
        }
        break;
      case AquaTeleopJoyState::CTRL_MODE_FLATSWIM_D:
        state.raw_cmd.speed = joypad.LY*params.max_speed_cmd;
        state.raw_cmd.heave = 0;
        state.yaw_vel = joypad.RX*params.max_yaw_vel;
        state.roll_pos = 0;
        state.pitch_pos = 0;
        state.roll_vel = 0;
        state.pitch_vel = 0;
        state.depth_vel = -joypad.RY*params.max_depth_vel;
        break;
    }
    paramsMutex.unlock();
    stateMutex.unlock();
  };
  

  void publishCtrl(const ros::TimerEvent& e) {
    if (eStopOn || !ros::ok()) {
      return;
    }
    
    // Advance AP pos based on time
    stateMutex.lock();
    paramsMutex.lock();
    ros::Time now = ros::Time::now();
    double dt = (now - lastStateUpdateT).toSec();
    state.roll_pos = wrapAngleDeg(state.roll_pos + state.roll_vel * dt);
    state.pitch_pos = wrapAngleDeg(state.pitch_pos + state.pitch_vel * dt);
    state.yaw_pos = wrapAngleDeg(state.yaw_pos + state.yaw_vel * dt);
    state.depth_pos = std::min(std::max(state.depth_pos + state.depth_vel * dt, params.min_depth), params.max_depth);
    lastStateUpdateT = now;
    paramsMutex.unlock();
    AquaTeleopJoyState pubState = state;
    stateMutex.unlock();
    
    // Publish command or pose
    if (pubState.ctrl_mode == AquaTeleopJoyState::CTRL_MODE_RAW) {
      cmdPub.publish(pubState.raw_cmd);
    } else {
      geometry_msgs::PoseStamped cmd;
      cmd.header.stamp = ros::Time::now();
      cmd.pose.position.x = pubState.raw_cmd.speed;
      cmd.pose.position.y = pubState.raw_cmd.heave;
      cmd.pose.position.z = pubState.depth_pos;

      btMatrix3x3 rotT;
      btQuaternion quat;
      if (rpyOrder == "ryp") {
        rotT.setEulerYPR(pubState.pitch_pos*DEGREE, pubState.yaw_pos*DEGREE, pubState.roll_pos*DEGREE);
	  } else if (rpyOrder == "pry") {
        rotT.setEulerYPR(pubState.yaw_pos*DEGREE, pubState.roll_pos*DEGREE, pubState.pitch_pos*DEGREE);
	  } else if (rpyOrder == "pyr") {
        rotT.setEulerYPR(pubState.roll_pos*DEGREE, pubState.yaw_pos*DEGREE, pubState.pitch_pos*DEGREE);
	  } else if (rpyOrder == "yrp") {
        rotT.setEulerYPR(pubState.pitch_pos*DEGREE, pubState.roll_pos*DEGREE, pubState.yaw_pos*DEGREE);
	  } else if (rpyOrder == "ypr") {
        rotT.setEulerYPR(pubState.roll_pos*DEGREE, pubState.pitch_pos*DEGREE, pubState.yaw_pos*DEGREE);
	  } else {
        rpyOrder = "rpy";
        rotT.setEulerYPR(pubState.yaw_pos*DEGREE, pubState.pitch_pos*DEGREE, pubState.roll_pos*DEGREE);
	  }
      rotT.getRotation(quat);
      tf::quaternionTFToMsg(quat, cmd.pose.orientation);
      //cmd.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pubState.roll_pos*DEGREE, pubState.pitch_pos*DEGREE, pubState.yaw_pos*DEGREE);

      apTargetPub.publish(cmd);
    }

    // Publish state
    statePub.publish(state);
    
    // Publish dynamic reconfigure parameters to server
    if (dyncfgSyncRequest && paramsMutex.try_lock()) { // Make sure that dynamic reconfigure server or config callback is not active
      paramsMutex.unlock();
      dyncfgServer->updateConfig(params);
      dyncfgSyncRequest = false;
    }
  };

  
  void publishParams(const ros::TimerEvent& e) {
    paramsPub.publish(toMsg(params));
  };
  
  
  ros::NodeHandle nh;
  
  ros::Subscriber joySub;
  ros::Publisher cmdPub;
  ros::Publisher apTargetPub;
  ros::ServiceClient getStateCln;
  aquacore::GetState getStateTyp;
  ros::ServiceClient apModeCln;
  aquacore::SetAutopilotMode apModeTyp;
  ros::ServiceClient aquaCalibrateCln;
  ros::ServiceClient aquaIsCalibratedCln;
  ros::ServiceClient aquaSetGaitCln;
  ros::ServiceClient aquaSetAutopilotModeCln;
  
  ros::Publisher statePub;
  ros::Publisher paramsPub;
  ros::ServiceServer updateParamsSrv;

  ros::Time lastStateUpdateT;
  AquaTeleopJoyState state;
  AquaTeleopJoyConfig params;
  boost::recursive_mutex stateMutex; // WARNING: always prioritize stateMutex over paramsMutex, to prevent deadlock
  
  JoyState joypad;
  bool eStopOn;
  
  ReconfigureServer* dyncfgServer;
  boost::recursive_mutex paramsMutex;
  bool dyncfgSyncRequest;
  
  ros::Timer ctrlPubTimer;
  ros::Timer paramsPubTimer;
  
  std::string rpyOrder;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "aqua_joy");
  AquaTeleopJoy joy;
  ros::spin();
  return EXIT_SUCCESS;
};
