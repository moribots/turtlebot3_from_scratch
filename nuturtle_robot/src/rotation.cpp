/// \file
/// \brief
///
/// PARAMETERS:
///
/// PUBLISHES:
/// SUBSCRIBES:
///
/// FUNCTIONS:

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/JointState.h>

#include <string>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"

#include "nuturtle_robot/Rotation.h"
#include "rigid2d/SetPose.h"

#include <functional>  // To use std::bind

// GLOBAL VARS
nuturtle_robot::Rotation::Request request;
bool start_called = false;

int timer_count = 0;
bool cycle_trigger = false;
int rotation_count = 0;
int translation_count = 0;
int pause_count = 0;
int full_rots = 0;
int full_trans = 0;

float rot = 0;
float trans = 0;

bool startCallback(nuturtle_robot::Rotation::Request& req, nuturtle_robot::Rotation::Response& res)
{
  request = req;

  start_called = true;

  res.result = true;

  ROS_INFO("SERVICE CALLED");

  return res.result;
}

void timerCallback(const ros::TimerEvent&, const ros::Publisher& vel_pub)
{

  if (full_rots != 20 && full_trans != 10)
  {
    geometry_msgs::Twist tw;

    if (!cycle_trigger)
    {
      // PUBLISH TWIST
      tw.linear.x = trans;
      tw.angular.z = rot;

      vel_pub.publish(tw);

    } else {
      // DO NOTHING
      tw.linear.x = 0;
      tw.angular.z = 0;

      vel_pub.publish(tw);

      // CONDITION TO RESUME MOVEMENT
      if (timer_count == pause_count)
      {
        cycle_trigger = false;
        timer_count = 0;

        if (request.rot_trans == 0)
        {
          // Increment num of full rotations
          full_rots++;
        } else {
          // Increment num of full translations
          full_trans++;
        }
      }
    }

    timer_count++;
  }
}

int main(int argc, char** argv)
/// The Main Function ///
{
  // Vars
  float frac_vel = 1;
  float frequency = 110.;
  float max_lin_vel_;
  float max_ang_vel_;

  ros::init(argc, argv, "rotation"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  // Parameters
  nh.getParam("/tran_vel_max", max_lin_vel_);
  nh.getParam("/rot_vel_max", max_ang_vel_);

  // Determine how many timer counts per revolution (rot) or sequence (trans)
  float abs_rot = max_ang_vel_ * frac_vel;
  rotation_count = std::round((2. * rigid2d::PI) / (abs_rot / frequency));

  float abs_trans = max_lin_vel_ * frac_vel;
  translation_count = std::round( 2. / ((abs_trans/ frequency) * 10.));

  // Init Service Client
  // SetPose Client
  ros::ServiceClient set_pose_client = nh.serviceClient<rigid2d::SetPose>("/set_pose");
  ros::service::waitForService("/set_pose", -1);
  // setup service parameters
  rigid2d::SetPose set_pose;
  set_pose.request.x = 0;
  set_pose.request.y = 0;
  set_pose.request.theta = 0;
  // reset pose to zero
  set_pose_client.call(set_pose);

  // Init Publisher
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // Init ROSCPP Timer and pass vel_pub to it with std::bind and _1 placeholder for passing timer event when ready
  ros::Timer timer = nh.createTimer(ros::Duration(1. / frequency), std::bind(&timerCallback, std::placeholders::_1, vel_pub));

  // Init Service Server
  ros::ServiceServer start_server = nh.advertiseService("/start", startCallback);

  // Main While
  while (ros::ok())
  {
    ros::spinOnce();

    if (start_called)
    {
      // Call setpose service
      set_pose_client.call(set_pose);
      start_called = false;
      // Reset Timer
      timer_count = 0;
      // Publish 0 Twist
      trans = 0;
      rot = 0;
      // Reset full cycle counters
      full_rots = 0;
      full_trans = 0;
    }

    if (request.rot_trans == 0)
    {
      // Rotate

      if (timer_count == rotation_count)
    {
      // Pause for some time since cycle is done
      timer_count = 0;
      cycle_trigger = true;
      pause_count = std::round(rotation_count / 20.);
    }

      if (request.rot_direction == 0)
      {
        // Rotate Clockwise
        rot = abs_rot;
        trans = 0;
      } else {
        // Rotate Counter-Clockwise
        rot = -abs_rot;
        trans = 0;
      }
    } else {
      // Translate

      if (timer_count == translation_count)
      {
        // Pause for some time since cycle is done
        timer_count = 0;
        cycle_trigger = true;
        pause_count = std::round(translation_count / 2.);
      }

      if (request.trans_direction == 0)
      {
        // Translate Forward
        trans = abs_trans;
        rot = 0;
      } else {
        // Translate Backwards
        trans = -abs_trans;
        rot = 0;
      } 
    }
  }

  return 0;
}