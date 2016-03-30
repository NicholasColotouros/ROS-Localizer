/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <aquacore/Command.h>
#include <aquacore/GetState.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "angles/angles.h"
#include <tf/transform_datatypes.h>


#define PI 3.14159265359

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42

#define KEYCODE0 0x30
#define KEYCODE1 0x31
#define KEYCODE2 0x32
#define KEYCODE3 0x33
#define KEYCODE4 0x34

#define KEYCODE_ESC 0x1B

#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63

#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_O 0x6F
#define KEYCODE_J 0x6A
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C
#define KEYCODE_M 0x6D
#define KEYCODE_RAB 0x2C
#define KEYCODE_LAB 0x2D

class TurtlebotTeleop
{
public:
  TurtlebotTeleop();
  void keyLoop();
  void watchdog();
  void zeroCommands();

private:

  
  ros::NodeHandle nh_,ph_;

  double linear_step_, angular_step_;
  double curr_speed_;
  double curr_yaw_;

  ros::Time first_publish_;
  ros::Time last_publish_;
  double l_scale_, a_scale_;
  ros::Publisher cmd_pub_;
  void publish();
  boost::mutex publish_mutex_;

  ros::ServiceClient aquaCalibrateCln;
  ros::ServiceClient aquaStandCln;
  ros::ServiceClient aquaWalkCln;

};

TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_step_(0.05),
  angular_step_(0.1),
  curr_speed_(0.0),
  curr_yaw_(0.0)
{
  nh_.param<double>("linear_step",linear_step_, 0.05);
  nh_.param<double>("angular_step",angular_step_, 0.1);
  
  cmd_pub_ = nh_.advertise<aquacore::Command>("/aqua/command", 1);
  aquaCalibrateCln = nh_.serviceClient<std_srvs::Empty>("/aqua/calibrate");
  aquaStandCln = nh_.serviceClient<std_srvs::Empty>("/aqua/stand");
  aquaWalkCln = nh_.serviceClient<std_srvs::Empty>("/aqua/walk");

  zeroCommands();
}

void TurtlebotTeleop::zeroCommands()
{
  curr_yaw_ = 0.0;
  curr_speed_ = 0.0;  
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "aqua_keyboard_teleop_walk");
  TurtlebotTeleop turtlebot_teleop;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&TurtlebotTeleop::keyLoop, &turtlebot_teleop));
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::watchdog, &turtlebot_teleop));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}


void TurtlebotTeleop::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) &&
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish();
}

void TurtlebotTeleop::keyLoop()
{
  char c;

  std_srvs::Empty emptySrv;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtlebot.");


  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c)
    {

      case KEYCODE_Q:
        ROS_DEBUG("Q");
        curr_speed_ = 0.0;
        curr_yaw_ = 0.0;
        zeroCommands();
        break;
      case KEYCODE_C:
        ROS_DEBUG("C");
	if(aquaCalibrateCln.call(emptySrv))
	{
		ROS_INFO_STREAM("Calibrated Aqua");
	}
	else
	{
		ROS_INFO_STREAM("Failed to calibrate Aqua");
	}
        break;
      case KEYCODE_S:
        ROS_DEBUG("S");
	if(aquaStandCln.call(emptySrv))
        {
                ROS_INFO_STREAM("Going to Stand Mode");
        }
        else
        {
                ROS_INFO_STREAM("Stand mode switch failed");
        }
        break;
      case KEYCODE_W:
        ROS_DEBUG("W");
	if(aquaWalkCln.call(emptySrv))
        {
                ROS_INFO_STREAM("Switching to Walk mode");
        }
        else
        {
                ROS_INFO_STREAM("Switch to walk mode failed");
        }
        break;
      case KEYCODE_I:
        ROS_DEBUG("I");
        curr_speed_ += linear_step_;
        break;
      case KEYCODE_M:
        ROS_DEBUG("M");
        curr_speed_ -= linear_step_;
        break;
      case KEYCODE_J:
        ROS_DEBUG("J");
	curr_yaw_ -= angular_step_;
        break;
      case KEYCODE_K:
        ROS_DEBUG("K");
	curr_yaw_ += angular_step_;
        break;
      default:
        printf( "key pressed: 0x%02X\n", c );
        continue;
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
    {
      first_publish_ = ros::Time::now();
    }

    printf( "speed: %f\n", curr_speed_);
    printf( "yaw: %f\n", curr_yaw_);
    printf( "----------------\n");

    last_publish_ = ros::Time::now();
    publish();
  }

  return;
}

void TurtlebotTeleop::publish()
{
  aquacore::Command cmd;
  cmd.speed = curr_speed_;
  cmd.yaw = curr_yaw_;
  cmd.pitch = 0.0;
  cmd.roll = 0.0;
  cmd.heave = 0.0;

  cmd_pub_.publish(cmd);

  return;
}

