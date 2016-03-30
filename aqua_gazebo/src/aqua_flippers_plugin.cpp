#include <aqua_gazebo/aqua_flippers_plugin.h>

AquaFlippersPlugin::AquaFlippersPlugin(){

}

void AquaFlippersPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Make sure gzserver is running, or load the Gazebo system plugin 'libgazebo_ros_api_plugin.so')");
        return;
    }

    model = _parent;
    angle_direction = -1;
    target_angles.fill(0.0);
    frequency_cmd.fill(0.0);
    amplitude_cmd.fill(0.0);
    leg_offsets_cmd.fill(0.0);
    phase_offsets_cmd.fill(0.0);
    thrust.fill(0.0);

    //phase_offsets_cmd[1] = 3.14159;
    //phase_offsets_cmd[4] = 3.14159;

    if (_sdf->HasElement("robotNamespace"))
        robot_namespace = _sdf->Get<std::string>("robotNamespace") + "/";
    else{
        ROS_INFO("aqua flippers plugin missing <robotNameSpace>, defaults to /aqua");
        robot_namespace = "aqua";
    }

    if (_sdf->HasElement("motorPidGains")){
        pid_gains = _sdf->Get<sdf::Vector3>("motorPidGains");
    } else {
        pid_gains = sdf::Vector3(0.0,0.0,0.0);
        ROS_INFO_STREAM("aqua flippers plugin missing <motorPidGains>, defaults to "<<pid_gains);
    }

    if (_sdf->HasElement("surfaceLevel")){
        surface_level = _sdf->Get<double>("surfaceLevel");
    } else {
        surface_level = 0.0;
        ROS_INFO("aqua hydrpdynamics plugin missing <surfaceLevel>, defaults to %f", surface_level);
    }
    if (_sdf->HasElement("fluidDensity")){
        fluid_density = _sdf->Get<double>("fluidDensity");
    } else {
        fluid_density = 999.97;
        ROS_INFO("aqua flippers plugin missing <fluidDensity>, defaults to %f", fluid_density);
    }

    if (_sdf->HasElement("thrustModel")){
        thrust_model = _sdf->Get<int>("thrustModel");
    } else {
        thrust_model = 0;
        ROS_INFO("aqua flippers plugin missing <thrustModel>, defaults to %d", thrust_model);
    }
    /*
    if (_sdf->HasElement("momentK1")){
        K1 = _sdf->Get<sdf::Vector3>("momentK1");
    } else {
        K1 = sdf::Vector3(0.0,0.0,0.0);
        ROS_INFO_STREAM("aqua flippers plugin missing <momentK1>, defaults to "<<K1);
    }
    if (_sdf->HasElement("momentK2")){
        K2 = _sdf->Get<sdf::Vector3>("momentK2");
    } else {
        K2 = sdf::Vector3(0.0,0.0,0.0);
        ROS_INFO_STREAM("aqua flippers plugin missing <momentK2>, defaults to "<<K2);
    }*/

    if (_sdf->HasElement("flipperFrequency")){
        double freq = _sdf->Get<double>("flipperFrequency");
        frequency_cmd.fill(freq);
    } else {
        ROS_INFO("aqua flippers plugin missing <flipperFrequency>, defaults to 0");
    }
    if (_sdf->HasElement("flipperAmplitude")){
        double amp = _sdf->Get<double>("flipperAmplitude");
        amplitude_cmd.fill(amp);
    } else {
        ROS_INFO("aqua flippers plugin missing <flipperAmplitude>, defaults to 0");
    }
    if (_sdf->HasElement("flipperAngleOffset")){
        double ang = _sdf->Get<double>("flipperAngleOffset");
        leg_offsets_cmd.fill(ang);
    } else {
        ROS_INFO("aqua flippers plugin missing <flipperAngleOffset>, defaults to 0");
    }
    pid.resize(NUM_LEGS);
    for (size_t i=0;i<NUM_LEGS;++i) {
        pid[i].Init(pid_gains.x, pid_gains.y, pid_gains.z, 20.0, -20.0, 20.0, -20.0);
    }

    // Initialize the ROS node handle
    nh = new ros::NodeHandle(robot_namespace);
    get_leg_command_service = nh->advertiseService("/aqua_gazebo/set_leg_command", &AquaFlippersPlugin::SetPeriodicLegCommand_cb,this);
    set_leg_command_service = nh->advertiseService("/aqua_gazebo/get_leg_command", &AquaFlippersPlugin::GetPeriodicLegCommand_cb,this);
    get_target_angles_service = nh->advertiseService("/aqua_gazebo/set_target_angles", &AquaFlippersPlugin::SetTargetLegAngles_cb,this);
    set_target_angles_service = nh->advertiseService("/aqua_gazebo/get_target_angles", &AquaFlippersPlugin::GetTargetLegAngles_cb,this);

    // this event is triggered called on every simulation iteration
    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&AquaFlippersPlugin::OnUpdate, this, _1));

    last_update_time =  model->GetWorld()->GetSimTime();

    base_link = model->GetLink("aqua_base");
    motor_joints.resize(NUM_LEGS);
    leg_links.resize(NUM_LEGS);
    for (size_t i=0;i<NUM_LEGS;++i) {
        motor_joints[i] = model->GetJoint(JOINT_NAMES[i]);
        leg_links[i] = model->GetLink(LEG_NAMES[i]);
    }

    // get leg dimensions
    leg_dims = leg_links[0]->GetBoundingBox().GetSize();

}

void AquaFlippersPlugin::OnUpdate(const gazebo::common::UpdateInfo & info){
    auto current_time = model->GetWorld()->GetSimTime();
    double dt = current_time.Double() - last_update_time.Double();
    last_update_time = current_time;
    
    auto aqua_lin_vel = base_link->GetRelativeLinearVel();

   // TODO get the motor commands here!
   for (size_t i=0;i<NUM_LEGS;++i) {
        double joint_angle = std::fmod(motor_joints[i]->GetAngle(1).Radian(),TWO_M_PI);
        target_angles[i] = std::fmod(target_angles[i],TWO_M_PI);

        //---------------------------------- THRUST MODEL --------------------------------------//
        // here we have the thrust model of [Giguère et al., 2006], [Plamondon & Nahon, 2009], or [Georgiades, 2005]
        auto leg_velocity = leg_links[i]->GetRelativeLinearVel();

        // just for fun, if the leg is above the surface of water (probably negligible at the given air density)
        double rho = 1.225;
        if(leg_links[i]->GetWorldCoGPose().pos.z <= surface_level){ 
           // below surface of water
            rho = fluid_density;
        } 
        
        if ( thrust_model == 1){
            // only apply thrust if it is going to be greater than zero
            if (amplitude_cmd[i]*frequency_cmd[i]>0.0){
                double thrust = 5.0*(0.1963*((leg_dims.y + 2*1.05*leg_dims.y)*leg_dims.x*leg_dims.x/3)*rho*amplitude_cmd[i]*frequency_cmd[i]);
                gazebo::math::Vector3 leg_thrust(thrust,0,0);
                leg_links[i]->AddRelativeForce(leg_thrust);
            } else {
                thrust[i] = 0.0;
            }
        } 
        
        // apply torque moment to the body frame according to philippe's model
        //double M = K1*joint_angle + K2*leg_links[i]->GetRelative
        //auto leg_ang_vel = leg_links[i]->GetRelativeAngularVel();
        // TODO implement this


        //---------------------------------- MOTOR CONTROLLER --------------------------------------//
        //target_angles[i] = amplitude_cmd[i]*std::sin(TWO_M_PI*frequency_cmd[i]*ros::Time::now().toSec() + phase_offsets_cmd[i]) + leg_offsets_cmd[i];
        double error = 0;
        
        pid[i].SetCmd(target_angles[i]);
        error = joint_angle - target_angles[i];

        // limit the error to be between -180 and 180
        error = std::fmod((error + M_PI),TWO_M_PI) - M_PI;
        if (error < -M_PI)
            error += TWO_M_PI;

        pid[i].Update(error,dt);

        motor_joints[i]->SetForce(0,pid[i].GetCmd());
    }
}

bool AquaFlippersPlugin::SetPeriodicLegCommand_cb(aquacore::SetPeriodicLegCommand::Request  &req, aquacore::SetPeriodicLegCommand::Response &res){
  std::copy(req.frequencies.begin(), req.frequencies.end(), frequency_cmd.begin());
  std::copy(req.amplitudes.begin(), req.amplitudes.end(), amplitude_cmd.begin());
  std::copy(req.leg_offsets.begin(), req.leg_offsets.end(), leg_offsets_cmd.begin());
  std::copy(req.phase_offsets.begin(), req.phase_offsets.end(), phase_offsets_cmd.begin());
  return true;
}

bool AquaFlippersPlugin::GetPeriodicLegCommand_cb(aquacore::GetPeriodicLegCommand::Request  &req, aquacore::GetPeriodicLegCommand::Response &res){
  std::copy(frequency_cmd.begin(), frequency_cmd.end(), res.frequencies.begin());
  std::copy(amplitude_cmd.begin(), amplitude_cmd.end(), res.amplitudes.begin());
  std::copy(leg_offsets_cmd.begin(), leg_offsets_cmd.end(), res.leg_offsets.begin());
  std::copy(phase_offsets_cmd.begin(), phase_offsets_cmd.end(), res.phase_offsets.begin());
  return true;
}

bool AquaFlippersPlugin::SetTargetLegAngles_cb(aquacore::SetTargetLegAngles::Request  &req, aquacore::SetTargetLegAngles::Response &res){
  std::copy(req.target_angles.begin(), req.target_angles.end(), target_angles.begin());
  return true;
}

bool AquaFlippersPlugin::GetTargetLegAngles_cb(aquacore::GetTargetLegAngles::Request  &req, aquacore::GetTargetLegAngles::Response &res){
  //
  std::copy(target_angles.begin(), target_angles.end(), res.target_angles.begin());
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(AquaFlippersPlugin);
