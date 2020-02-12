/// \file
/// \brief
///
/// PARAMETERS:
///
/// PUBLISHES:
/// SUBSCRIBES:
///   /cmd_vel (geometry_msgs::Twist): subscriber, which records the commanded twist
///
/// FUNCTIONS:
///   vel_callback (void): callback for /cmd_vel subscriber, which records the commanded twist

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/JointState.h>

#include<string>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"

// GLOBAL VARS
rigid2d::WheelVelocities w_vel;
rigid2d::WheelVelocities w_vel_measured;
rigid2d::WheelVelocities w_ang;
rigid2d::DiffDrive driver;
double frequency = 60;
bool vel_flag = false;
bool sensor_flag = false;
float max_lin_vel_ = 0;
float max_ang_vel_ = 0;
float motor_rot_max_ = 0;
float encoder_ticks_per_rev_ = 0;

void vel_callback(const geometry_msgs::Twist &tw)
{

  // Cap Angular Twist
  float ang_vel = tw.angular.z;
  if (ang_vel > max_ang_vel_)
  {
    ang_vel = max_ang_vel_;
  } else if (ang_vel < - max_ang_vel_)
  {
    ang_vel = -max_ang_vel_;
  }
  // Cap Linear Twist
  float lin_vel = tw.linear.x;
  if (lin_vel > max_lin_vel_)
  {
    lin_vel = max_lin_vel_;
  } else if (lin_vel < - max_lin_vel_)
  {
    lin_vel = -max_lin_vel_;
  }

  rigid2d::Twist2D Vb(ang_vel, lin_vel, tw.linear.y);
  // Get Wheel Velocities
  w_vel = driver.twistToWheels(Vb);

  // Cap Wheel Velocities
  if (w_vel.ul > motor_rot_max_)
  {
    w_vel.ul = motor_rot_max_;
  } else if (w_vel.ul < - motor_rot_max_)
  {
    w_vel.ul = - motor_rot_max_;
  }

  if (w_vel.ur > motor_rot_max_)
  {
    w_vel.ur = motor_rot_max_;
  } else if (w_vel.ur < - motor_rot_max_)
  {
    w_vel.ur = - motor_rot_max_;
  }

  // Now convert wheel vel to integers from -265 to 265
  float m = (265. - - 265.) / (motor_rot_max_ * 2.);
  float b = (265. - motor_rot_max_ * m);

  w_vel.ul = w_vel.ul * m + b;
  w_vel.ur = w_vel.ur * m + b;

  vel_flag = true;
}

void sensor_callback(const nuturtlebot::SensorData &sns)
{
  w_ang.ul = sns.left_encoder;
  w_ang.ur = sns.right_encoder;

  // Now convert encoder values (0-4096 to 0-2pi)
  float m = (2 * rigid2d::PI) / (encoder_ticks_per_rev_);
  float b = (2 * rigid2d::PI - encoder_ticks_per_rev_ * m);

  w_ang.ul = w_ang.ul * m + b; 
  w_ang.ur = w_ang.ur * m + b;

  // Normalize Encoder Values
  w_ang.ul = rigid2d::normalize_angle(w_ang.ul);
  w_ang.ur = rigid2d::normalize_angle(w_ang.ur);

  // Get wheel velocities based on encoder data
  w_vel_measured = driver.updateOdometry(w_ang.ul, w_ang.ur);

  sensor_flag = true;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  // Vars
  std::string o_fid_, b_fid_, wl_fid_, wr_fid_;
  float wbase_, wrad_;

  ros::init(argc, argv, "turtle_interface"); // register the node on ROS
  ros::NodeHandle nh_("~"); // PRIVATE handle to ROS
  ros::NodeHandle nh; // PUBLIC handle to ROS
  // Parameters
  // Private
  nh_.getParam("left_wheel_joint", wl_fid_);
  nh_.getParam("right_wheel_joint", wr_fid_);
  // Public
  nh.getParam("/wheel_base", wbase_);
  nh.getParam("/wheel_radius", wrad_);
  nh.getParam("/tran_vel_max", max_lin_vel_);
  nh.getParam("/rot_vel_max", max_ang_vel_);
  nh.getParam("/motor_rot_max", motor_rot_max_);
  nh.getParam("/encoder_ticks_per_rev", encoder_ticks_per_rev_);
  // Set Driver Wheel Base and Radius
  driver.set_static(wbase_, wrad_);

  // Init Subscriber
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1, vel_callback);
  ros::Subscriber sensor_sub = nh.subscribe("sensor_data", 1, sensor_callback);
  // Init Publisher
  ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher wvel_pub = nh.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 1);

  // Init Time
  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate rate(frequency);
  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();
    current_time = ros::Time::now();

    if (sensor_flag == true)
    {
      sensor_msgs::JointState js;

      js.header.stamp = current_time;

      // js stores vectors, so we push back the name corresp. to left wheel joint
      js.name.push_back(wl_fid_);
      // then we insert the left wheel encoder value
      js.position.push_back(w_ang.ul);
      js.velocity.push_back(w_vel_measured.ul);

      // repeat with right wheel. Note order must be consistent between name pushback and
      // encoder value pushback
      js.name.push_back(wr_fid_);
      js.position.push_back(w_ang.ur);
      js.velocity.push_back(w_vel_measured.ur);

      // now publish
      // std::cout << w_ang.ul << std::endl;
      // std::cout << w_ang.ur << std::endl;
      js_pub.publish(js);

      sensor_flag = false;
    }

    if (vel_flag == true)
    {
      nuturtlebot::WheelCommands wc;

      wc.left_velocity = std::round(w_vel.ul);
      wc.right_velocity = std::round(w_vel.ur);

      wvel_pub.publish(wc);

      vel_flag = false;
    }
    rate.sleep();

  }

  return 0;
}