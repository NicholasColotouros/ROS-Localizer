#include <aqua_gazebo/aqua_hydrodynamics_plugin.h>
#include "tf/transform_datatypes.h"

#define NUM_LEGS 6
const char* LEG_NAMES[NUM_LEGS] =
{ "left_front_leg",
    "left_mid_leg",
    "left_rear_leg",
    "right_front_leg",
    "right_mid_leg",
    "right_rear_leg"};
const char* SHOULDER_NAMES[NUM_LEGS] =
{ "left_front_shoulder",
    "left_mid_shoulder",
    "left_rear_shoulder",
    "right_front_shoulder",
    "right_mid_shoulder",
    "right_rear_shoulder"};

AquaHydrodynamicsPlugin::AquaHydrodynamicsPlugin(){

}

void AquaHydrodynamicsPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    model = _parent;
    robot_namespace.clear();

    // get parameters from SDF file (gazebo tags in the URDF file)
    if (_sdf->HasElement("robotNamespace"))
        robot_namespace = _sdf->Get<std::string>("robotNamespace") + "/";
    else{
        ROS_INFO("aqua hydrodynamics plugin missing <robotNameSpace>, defaults to /aqua");
        robot_namespace = "aqua";
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
        ROS_INFO("aqua hydrodynamics plugin missing <fluidDensity>, defaults to %f", fluid_density);
    }
    if (_sdf->HasElement("dragCoeffs")){
        drag_coeffs = _sdf->Get<sdf::Vector3>("dragCoeffs");
    } else {
        drag_coeffs = sdf::Vector3(0.9,1.08,1.28);
        ROS_INFO_STREAM("aqua hydrodynamics plugin missing <dragCoeffs>, defaults to "<<drag_coeffs);
    }
    if (_sdf->HasElement("legDragCoeffs")){
        leg_drag_coeffs = _sdf->Get<sdf::Vector3>("legDragCoeffs");
    } else {
        leg_drag_coeffs = sdf::Vector3(0.0,0.0,1.12);
        ROS_INFO_STREAM("aqua hydrodynamics plugin missing <legDragCoeffs>, defaults to "<<leg_drag_coeffs);
    }

    disable_roll_simulation=false;
    if (_sdf->HasElement("disableRollSimulation")){
        disable_roll_simulation = _sdf->Get<bool>("disableRollSimulation");
        if (disable_roll_simulation){
            ROS_INFO_STREAM("Not including roll in the simulation");
        }
    } 
    disable_pitch_simulation=false;
    if (_sdf->HasElement("disablePitchSimulation")){
        disable_pitch_simulation = _sdf->Get<bool>("disablePitchSimulation");
        if (disable_pitch_simulation){
            ROS_INFO_STREAM("Not including pitch in the simulation");
        }
    } 
    disable_yaw_simulation=false;
    if (_sdf->HasElement("disableYawSimulation")){
        disable_yaw_simulation = _sdf->Get<bool>("disableYawSimulation");
        if (disable_yaw_simulation){
            ROS_INFO_STREAM("Not including yaw in the simulation");
        }
    } 
    disable_heave_simulation=false;
    if (_sdf->HasElement("disableHeaveSimulation")){
        disable_heave_simulation = _sdf->Get<bool>("disableHeaveSimulation");
        if (disable_heave_simulation){
            ROS_INFO_STREAM("Not including heave in the simulation");
        }
    } 
    disable_speed_simulation=false;
    if (_sdf->HasElement("disableSpeedSimulation")){
        disable_speed_simulation = _sdf->Get<bool>("disableSpeedSimulation");
        if (disable_speed_simulation){
            ROS_INFO_STREAM("Not including speed in the simulation");
        }
    } 

    // this event is triggered called on every simulation iteration
    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&AquaHydrodynamicsPlugin::OnUpdate, this, _1));

    base_link = model->GetLink("aqua_base");
    leg_links.resize(NUM_LEGS);
    shoulder_links.resize(NUM_LEGS);
    for (size_t i=0;i<NUM_LEGS;++i) {
        leg_links[i] = model->GetLink(LEG_NAMES[i]);
        shoulder_links[i] = model->GetLink(SHOULDER_NAMES[i]);
    }
    // get leg dimensions
    leg_dims = leg_links[0]->GetBoundingBox().GetSize();
    aqua_volume = 0;
    for(auto &col : base_link->GetCollisions()){
        if(col->GetName().find("aqua_shell") != std::string::npos){
            //get the robot dimensions
            aqua_dims = col->GetBoundingBox().GetSize();
            // add the sheel volume
            aqua_volume += aqua_dims.x*aqua_dims.y*aqua_dims.z;
        } else  if(col->GetName().find("hip") != std::string::npos){
            // add the hip volume
            auto hip_shape = static_cast<gazebo::physics::CylinderShape*>(col->GetShape().get());  
            double r = hip_shape->GetRadius();
            double h = hip_shape->GetLength();
            aqua_volume += M_PI*r*r*h;
        }

    }
    std::cout<<"leg dimensions: "<<leg_dims.x<<", "<<leg_dims.y<<", "<<leg_dims.z<<std::endl;
    std::cout<<"aqua dimensions: "<<aqua_dims.x<<", "<<aqua_dims.y<<", "<<aqua_dims.z<<std::endl;
    std::cout<<"aqua volume: "<<aqua_volume<<" m^3 "<<std::endl;
    std::cout<<"aqua mass: "<<base_link->GetInertial()->GetMass()<<" kg "<<std::endl;
    std::cout<<"combined leg volume: "<<6*leg_dims.x*leg_dims.y*leg_dims.z<<" m^3 "<<std::endl;
    std::cout<<"combined leg mass: "<<6*leg_links[0]->GetInertial()->GetMass()<<" kg "<<std::endl;

    world = model->GetWorld();
    current_time = world->GetSimTime();

    initialPose = base_link->GetWorldPose();
}


void AquaHydrodynamicsPlugin::OnUpdate(const gazebo::common::UpdateInfo & info){
    // get the current value of gravity
    double gravity = world->GetPhysicsEngine()->GetGravity().GetLength();
    // get current simulation time
    auto previous_time = current_time;
    current_time = world->GetSimTime();
    // get aqua's depth
    double current_depth = base_link->GetWorldCoGPose().pos.z;
    // get aqua's velocities and accelerations in body frame
    auto aqua_lin_vel = base_link->GetRelativeLinearVel();
    auto aqua_ang_vel = base_link->GetRelativeAngularVel();
    auto aqua_lin_accel = base_link->GetRelativeLinearAccel();
    auto aqua_ang_accel = base_link->GetRelativeAngularAccel();
    // disable some degrees or freedom if requested.
    /*
       if (DisableDegreesOfFreedom(aqua_lin_vel,aqua_ang_vel,aqua_lin_accel,aqua_ang_accel)){
       base_link->SetAngularAccel(aqua_ang_accel);
       base_link->SetAngularVel(aqua_ang_vel);
       base_link->SetLinearAccel(aqua_lin_accel);
       base_link->SetLinearVel(aqua_lin_vel);
       }
       */


    //if the robot is above the surface of water set the fluid density to that of air
    double rho = 1.225;
    if(current_depth <= surface_level){ 
        // below surface of water
        rho = fluid_density;
        // the parameters for the added mass effects come from the M.Eng thesis by Christina Georgiades
        // added mass effects
        static const float AM[] = {6.98,14.5,32.41,0.4,1.19,0.55};
        gazebo::math::Vector3 added_mass_linear(AM[0]*aqua_lin_accel.x,AM[1]*aqua_lin_accel.y,AM[2]*aqua_lin_accel.z);
        gazebo::math::Vector3 added_mass_angular(AM[3]*aqua_ang_accel.x,AM[4]*aqua_ang_accel.y,AM[5]*aqua_ang_accel.z);
        base_link->AddRelativeForce(added_mass_linear);
        base_link->AddRelativeTorque(added_mass_angular);
        // added mass on coriolis matrix
        gazebo::math::Vector3 added_mass_coriolis_linear(AM[2]*aqua_lin_vel.z*aqua_ang_vel.y - AM[1]*aqua_lin_vel.y*aqua_ang_vel.z,
                -AM[2]*aqua_lin_vel.z*aqua_ang_vel.x + AM[0]*aqua_lin_vel.x*aqua_ang_vel.z,
                AM[1]*aqua_lin_vel.y*aqua_ang_vel.x - AM[0]*aqua_lin_vel.x*aqua_ang_vel.y);

        gazebo::math::Vector3 added_mass_coriolis_angular(AM[2]*aqua_lin_vel.z*aqua_lin_vel.y - AM[1]*aqua_lin_vel.y*aqua_lin_vel.z
                +AM[5]*aqua_ang_vel.z*aqua_ang_vel.y - AM[4]*aqua_ang_vel.y*aqua_ang_vel.z,
                -AM[2]*aqua_lin_vel.z*aqua_lin_vel.x + AM[0]*aqua_lin_vel.x*aqua_lin_vel.z
                -AM[5]*aqua_ang_vel.z*aqua_ang_vel.x + AM[3]*aqua_ang_vel.x*aqua_ang_vel.z,
                AM[1]*aqua_lin_vel.y*aqua_lin_vel.x - AM[0]*aqua_lin_vel.x*aqua_lin_vel.y
                +AM[4]*aqua_ang_vel.y*aqua_ang_vel.x - AM[3]*aqua_ang_vel.x*aqua_ang_vel.y);

        // Dave disabled this force for in-plane restricted simulation
        if( disable_heave_simulation ){
            base_link->AddRelativeForce(added_mass_coriolis_linear);
            base_link->AddRelativeTorque(added_mass_coriolis_angular);
        }
    } 

    // compute drag (for a rectangular prism in body coordinates, drag coefficients from M.Eng thesis by Georgiades (2005))
    gazebo::math::Vector3 drag(-0.5*rho*drag_coeffs.x*(aqua_dims.y*aqua_dims.z)*std::abs(aqua_lin_vel.x)*aqua_lin_vel.x,
            -0.5*rho*drag_coeffs.y*(aqua_dims.x*aqua_dims.z)*std::abs(aqua_lin_vel.y)*aqua_lin_vel.y,
            -0.5*rho*drag_coeffs.z*(aqua_dims.x*aqua_dims.y)*std::abs(aqua_lin_vel.z)*aqua_lin_vel.z);
    base_link->AddRelativeForce(drag);

    // The robot's angular velocities will contribute to drag. Here we assume this contribution is independent of the linear drag (this is a rough approximation)
    gazebo::math::Vector3 drag_moment(
            -rho*std::abs(aqua_ang_vel.x)*aqua_ang_vel.x*aqua_dims.x*(drag_coeffs.z*std::pow(aqua_dims.y,4) + drag_coeffs.y*std::pow(aqua_dims.z,4))*0.015625,
            -rho*std::abs(aqua_ang_vel.y)*aqua_ang_vel.y*aqua_dims.y*(drag_coeffs.x*std::pow(aqua_dims.z,4) + drag_coeffs.z*std::pow(aqua_dims.x,4))*0.015625,
            -rho*std::abs(aqua_ang_vel.z)*aqua_ang_vel.z*aqua_dims.z*(drag_coeffs.x*std::pow(aqua_dims.y,4) + drag_coeffs.y*std::pow(aqua_dims.x,4))*0.015625);

    //std::cout<<"Body drag moment: "<<drag_moment<<std::endl;
    base_link->AddRelativeTorque(drag_moment);

    // apply buoyancy to robot
    gazebo::math::Vector3 buoyancy(0.0, 0.0, rho*aqua_volume*gravity);
    base_link->AddForce(buoyancy);

    // something similar can be done for each flipper
    gazebo::math::Vector3 leg_buoyancy(0.0, 0.0, rho*leg_dims.x*leg_dims.y*leg_dims.z*gravity);
    for (int i =0; i<NUM_LEGS; i++){
        auto leg_lin_vel = leg_links[i]->GetRelativeLinearVel();
        // if the leg is above the water
        rho = 1.225;
        if(leg_links[i]->GetWorldCoGPose().pos.z <= surface_level){ 
            // below surface of water
            rho = fluid_density;
        } 

        gazebo::math::Vector3 leg_drag(-0.5*rho*leg_drag_coeffs.x*(leg_dims.y*leg_dims.z)*std::abs(leg_lin_vel.x)*leg_lin_vel.x,
                -0.5*rho*leg_drag_coeffs.y*(leg_dims.x*leg_dims.z)*std::abs(leg_lin_vel.y)*leg_lin_vel.y,
                -0.5*rho*leg_drag_coeffs.z*(leg_dims.x*leg_dims.y)*std::abs(leg_lin_vel.z)*leg_lin_vel.z);

        // the force is applied to the center of mass, but in body cooridnates
        // for our current setup, the body reference frame is at the hip. the center of mass
        // should be somewhere in the middle of the leg
        leg_links[i]->AddRelativeForce(leg_drag);

        //  Removed this because it causes numerical instability. I suspect this happens when applying a torque to a joint with a pid controller
           auto leg_ang_vel = leg_links[i]->GetRelativeAngularVel();
        // limit the maximum drag moment by limiting the angular velocity with which it is computed

        gazebo::math::Vector3 leg_drag_moment(    
        -0.1*rho*std::abs(leg_ang_vel.x)*leg_ang_vel.x*leg_dims.x*(leg_drag_coeffs.z*std::pow(leg_dims.y,4) + leg_drag_coeffs.y*std::pow(leg_dims.z,4))*0.015625,
        -0.1*rho*std::abs(leg_ang_vel.y)*leg_ang_vel.y*leg_dims.y*(leg_drag_coeffs.x*std::pow(leg_dims.z,4) + leg_drag_coeffs.z*std::pow(leg_dims.x,4))*0.015625,
        -0.1*rho*std::abs(leg_ang_vel.z)*leg_ang_vel.z*leg_dims.z*(leg_drag_coeffs.x*std::pow(leg_dims.y,4) + leg_drag_coeffs.y*std::pow(leg_dims.x,4))*0.015625);

        // I'm not entirely sure if applying the torque to the leg link will be the same as applying the torque to the shoulder.
        // As I am not sure, I'm applying this to the shoulder ( which is centered at the rotation axis)
        leg_links[i]->AddRelativeTorque(drag_moment);
        //shoulder_links[i]->AddRelativeTorque(drag_moment);

        // the buoyant force is in world coordinate (always opposite to gravity)
        // here we add it at the center of mass of the leg, in world coordiantes
        leg_links[i]->AddForce(leg_buoyancy);
    }

    //check if simulation is become unstable (TODO: instead of doing this, we should be doing implicit integration, which is stable for large steps)
    if ( current_depth <= surface_level){ 
        if( aqua_lin_vel.GetLength() > 50){
            ROS_WARN_STREAM("Robot linear velocity "<<aqua_lin_vel);
            ROS_WARN_STREAM("Force applied to the robot "<<base_link->GetRelativeForce());
            ROS_WARN_STREAM("Simulation is unstable, setting body force to 0");
            base_link->SetForce( gazebo::math::Vector3(0,0,0) );
            base_link->SetLinearAccel(gazebo::math::Vector3(0,0,0)) ;
        }
        if( aqua_ang_vel.GetLength() > 50){
            ROS_WARN_STREAM("Robot angular velocity "<<aqua_ang_vel);
            ROS_WARN_STREAM("Torque applied to the robot "<<base_link->GetRelativeTorque());
            ROS_WARN_STREAM("Simulation is unstable, setting body torque to 0");
            base_link->SetTorque( gazebo::math::Vector3(0,0,0) );
            base_link->SetAngularAccel(gazebo::math::Vector3(0,0,0) );
        }
        for (int i =0; i<NUM_LEGS; i++){
            auto leg_lin_vel = leg_links[i]->GetRelativeLinearVel();
            auto leg_ang_vel = leg_links[i]->GetRelativeAngularVel();
            if( leg_lin_vel.GetLength() > 50){
                ROS_WARN_STREAM("Leg "<<i<<" relative linear velocity "<<leg_lin_vel);
                ROS_WARN_STREAM("Force applied to leg "<<i<<" "<<leg_links[i]->GetRelativeForce());
                ROS_WARN_STREAM("Simulation is unstable, setting leg "<<i<<" force to 0");
                leg_links[i]->SetForce ( gazebo::math::Vector3(0,0,0) );
                leg_links[i]->SetLinearAccel(gazebo::math::Vector3(0,0,0) );
            }
            if( leg_ang_vel.GetLength() > 50){
                ROS_WARN_STREAM("Leg "<<i<<" relative angular velocity "<<leg_ang_vel);
                ROS_WARN_STREAM("Torque applied to leg "<<i<<" "<<leg_links[i]->GetRelativeTorque());
                ROS_WARN_STREAM("Simulation is unstable, setting leg "<<i<<" torque to 0");
                leg_links[i]->SetTorque( gazebo::math::Vector3(0,0,0) );
                leg_links[i]->SetAngularAccel(gazebo::math::Vector3(0,0,0) );
            }
        }
    }

    if( disable_heave_simulation )
    {
        gazebo::math::Pose p = base_link->GetWorldPose();

        gazebo::math::Vector3 v(p.rot.GetAsEuler());
        v.x = 0.0;
        v.y = 0.0;
        p.rot.SetFromEuler(v);

        p.pos.Set( p.pos.x, p.pos.y, initialPose.pos.z );

        base_link->SetWorldPose(p,true,true);
    }
}

bool AquaHydrodynamicsPlugin::DisableDegreesOfFreedom(gazebo::math::Vector3 &aqua_lin_vel,
        gazebo::math::Vector3 &aqua_ang_vel,
        gazebo::math::Vector3 &aqua_lin_accel,
        gazebo::math::Vector3 &aqua_ang_accel){
    bool something_disabled = false;
    // DISABLE SIMULATION FOR THE REQUESTED DEGREES OF FREEDOM
    if(disable_roll_simulation){
        aqua_ang_vel.x=0;
        aqua_ang_accel.x=0;
        something_disabled = true;
    }
    if(disable_pitch_simulation){
        aqua_ang_vel.y=0;
        aqua_ang_accel.y=0;
        something_disabled = true;
    }
    if(disable_yaw_simulation){
        aqua_ang_vel.z=0;
        aqua_ang_accel.z=0;
        something_disabled = true;
    }
    if(disable_heave_simulation){
        aqua_lin_vel.z=0;
        aqua_lin_accel.z=0;
        something_disabled = true;
    }
    if(disable_speed_simulation){
        aqua_lin_vel.x=0;
        aqua_lin_accel.x=0;
        something_disabled = true;
    }
    return something_disabled;
}

GZ_REGISTER_MODEL_PLUGIN(AquaHydrodynamicsPlugin);

