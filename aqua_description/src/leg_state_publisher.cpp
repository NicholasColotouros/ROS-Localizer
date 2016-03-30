/*
 * leg_state_publisher.cpp
 *
 *  @brief: Needs to publish joint angles that are synchronized with the aqua URDF model
 *
 *  Created on: 2012-11-22
 *      Author: dpmeger
 *
 *  Revision history:
 *
 *  Nov 22, 2012: A simple node that publishes fake sinusoidal signals for the 6 legs
 */

#include "ros/ros.h"
#include <string>
#include "angles/angles.h"
#include "sensor_msgs/JointState.h"
#include <boost/algorithm/string.hpp>
#include <vector>

#include "aquacore/Health.h"

#define NUM_LEGS 6
const char* LEG_NAMES[NUM_LEGS] =
   { "left_front_leg_joint",
	 "left_mid_leg_joint",
	 "left_rear_leg_joint",
	 "right_front_leg_joint",
	 "right_mid_leg_joint",
	 "right_rear_leg_joint" };

const char* SHOULDER_NAMES[NUM_LEGS] =
   { "left_front_shoulder_joint",
	 "left_mid_shoulder_joint",
	 "left_rear_shoulder_joint",
	 "right_front_shoulder_joint",
	 "right_mid_shoulder_joint",
	 "right_rear_shoulder_joint" };

const char* HIP_NAMES[NUM_LEGS] =
   { "left_front_hip_joint",
	 "left_mid_hip_joint",
	 "left_rear_hip_joint",
	 "right_front_hip_joint",
	 "right_mid_hip_joint",
	 "right_rear_hip_joint" };

using namespace std;

class LegStatePublisher
{
	ros::NodeHandle nh_;
	ros::Publisher js_pub_;
	sensor_msgs::JointState joint_state_;

	ros::Subscriber aqua_health_sub_;

public:

	LegStatePublisher()
	{
		joint_state_.name.resize(3*NUM_LEGS+1);
		joint_state_.position.resize(3*NUM_LEGS+1);

		for (size_t i=0;i<NUM_LEGS;++i) {
			joint_state_.name[i] = LEG_NAMES[i];
			joint_state_.position[i] = 0.0;
		}
        // this is a hack so that rviz does not complain about the depth sensor, hip and leg adapter joints
		for (size_t i=0;i<NUM_LEGS;++i) {
			joint_state_.name[i+6] = SHOULDER_NAMES[i];
			joint_state_.position[i+6] = 0.0;
			joint_state_.name[i+12] = HIP_NAMES[i];
			joint_state_.position[i+12] = 0.0;
        }
        joint_state_.name[3*NUM_LEGS] = "depth_sensor_joint";
		joint_state_.position[3*NUM_LEGS] = 0.0;

		js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
		aqua_health_sub_ = nh_.subscribe("aqua/health", 10, &LegStatePublisher::aquaHealthCallback, this);
	}

	~LegStatePublisher()
	{
	}

	// The main work is done here. Update the joint angles to match those published by the aquahw node.
	void aquaHealthCallback(const aquacore::HealthConstPtr& health)
	{
		for (size_t i=0;i<NUM_LEGS;++i) {
			joint_state_.position[i] = health->positions[i];
		}

		joint_state_.header.stamp = ros::Time::now();
		js_pub_.publish(joint_state_);
	}

	// In order to keep the legs updated in the visual representation, re-publish the
	// latest values at a faster rate, spinOnce frequently to make sure we get published values.
	void spin()
	{
		ros::Rate r(100);

		while (nh_.ok()) {
			ros::spinOnce();
//			for (size_t i=0;i<NUM_LEGS;++i) {
//				if( i==0 || i==2 || i==3 || i==5 )
//				{
//					joint_state_.position[i] = 0.75*sin(10.0*ros::Time::now().toSec());
//				}
//				else{
//					joint_state_.position[i] = 0.75*sin(10.0*ros::Time::now().toSec()+3.1415);
//				}
//
//			}
			joint_state_.header.stamp = ros::Time::now();
			js_pub_.publish(joint_state_);
			r.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_state_publisher");

	LegStatePublisher lsp;
	lsp.spin();

	return 0;
}


