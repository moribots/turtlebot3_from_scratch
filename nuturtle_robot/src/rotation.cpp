/// \file
/// \brief Provides a /start service which makes the turtlebot3 perform either 20 rotation (CW/CCW) or a 2-meter traversal (FWD/BWD)
///
/// PARAMETERS:
/// request (nuturtle_robot::Rotation::Request): service request for /start service, which contains four booleans: rot_trans, rot_direction, trans_direction
/// start_called (bool): flag to indicate that service has been called, and rotation/translation sequence can commence.
/// timer_count (int): keeps track of iterations required to perform one rotation or translation segment.
/// cycle_trigger (bool): flag used to alternate between pause and actuation of turtlebot3
/// rotation_count (int): records the number of rotations completed since the start of the service call
/// translation_count (int): records the number of translation completed since the start of the service call
/// pause_count (bool): keeps track of iterations required to pause for the correct amount of time based on loop rate
/// full_rots (int): value greater than maximum number of rotations, used for loop logic
/// full_trans (int): value greater than maximum number of translations, used for loop logic
/// 
/// rot (float): angular component of desired 2D twist
/// trans (float): linear component of desired 2D twist
/// frac_vel (float): fraction to be used of the turtlebot3's maximum linear and angular velocities
/// max_lin_vel_ (float): turtlebot3's maximum linear velocity
/// max_ang_vel_ (float): turtlebot3's maximum rotational velocity
/// frequency (float): loop rate for this operation
///
/// PUBLISHES:
/// publishes Twist to cmd_vel
///
/// FUNCTIONS:
/// start_callback (bool): callback for start service, which calls the set_pose and fake/set_pose services, then sets the appripriate flags to begin movement
/// timerCallback (void): deterministically publishes Twist message based on desired commands according to loop logic
///
///  SERVICES:
///  start: calls set_pose service and intiates either 20 rotations, or a 2 meter traversal

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

// Set these higher than feasible so turtle doesn't start moving at launch
int full_rots = 10000000;
int full_trans = 1000000;

float rot = 0;
float trans = 0;

bool startCallback(nuturtle_robot::Rotation::Request& req, nuturtle_robot::Rotation::Response& res)
{
  /// \brief start service callback, calls set_pose service and intiates either 20 rotations, or a 2 meter traversal
  /// \param rot_trans (bool): False - Rotation, True - Translation
  /// \param rot_direction (bool): False - CW, True - CCW
  /// \param trans_direction (bool): False - Forward, True - Backwards
  /// \returns result (bool): True or False.
  request = req;

  start_called = true;

  res.result = true;

  ROS_INFO("SERVICE CALLED");

  return res.result;
}

void timerCallback(const ros::TimerEvent&, const ros::Publisher& vel_pub)
{
  /// \brief deterministic timer to publish Twist messages to cmd_vel
  /// \param ros::TimerEvent: ensures loop is consistent (deterministic)
  /// \param vel_pub (ros::Publisher): publisher object used to send messages to cmd_Vel
  /// \returns publishes Twist to cmd_vel

  if (full_rots < 20 && full_trans < 10)
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
  float frac_vel;
  float frequency = 110.;
  float max_lin_vel_;
  float max_ang_vel_;

  ros::init(argc, argv, "rotation"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("frac_vel", frac_vel);
  nh.getParam("/tran_vel_max", max_lin_vel_);
  nh.getParam("/rot_vel_max", max_ang_vel_);

  // Determine how many timer counts per revolution (rot) or sequence (trans)
  float abs_rot = max_ang_vel_ * frac_vel;
  rotation_count = std::round((2. * rigid2d::PI) / (abs_rot / frequency));

  float abs_trans = max_lin_vel_ * frac_vel;
  translation_count = std::round( 2. / ((abs_trans/ frequency) * 10.));

  // Init Service Client
  // SetPose Client
  ros::ServiceClient set_pose_client = nh.serviceClient<rigid2d::SetPose>("set_pose");
  ros::service::waitForService("set_pose", -1);
  // setup service parameters
  rigid2d::SetPose set_pose;
  set_pose.request.x = 0;
  set_pose.request.y = 0;
  set_pose.request.theta = 0;
  // reset pose to zero
  set_pose_client.call(set_pose);
  // Fake Client
  ros::ServiceClient set_pose_client_fake = nh.serviceClient<rigid2d::SetPose>("fake/set_pose");
  ros::service::waitForService("fake/set_pose", -1);
  set_pose_client_fake.call(set_pose);

  // Init Publisher
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // Init ROSCPP Timer and pass vel_pub to it with std::bind and _1 placeholder for passing timer event when ready
  ros::Timer timer = nh.createTimer(ros::Duration(1. / frequency), std::bind(&timerCallback, std::placeholders::_1, vel_pub));

  // Init Service Server
  ros::ServiceServer start_server = nh.advertiseService("start", startCallback);

  // Main While
  while (ros::ok())
  {
    ros::spinOnce();

    if (start_called)
    {
      ros::service::waitForService("set_pose", -1);
      ros::service::waitForService("fake/set_pose", -1);
      // Call setpose service
      set_pose_client.call(set_pose);
      set_pose_client_fake.call(set_pose);
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