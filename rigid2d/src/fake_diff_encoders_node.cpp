/// \file
/// \brief Main: Publishes Odometry messages
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVICES:

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/JointState.h>

#include<string>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

// GLOBAL VARS
rigid2d::WheelVelocities w_ang;
rigid2d::DiffDrive driver;

void vel_callback(const geometry_msgs::Twist &tw)
{ //ConstPtr is a smart pointer which knows to de-allocate memory
  rigid2d::Twist2D Vb(tw.angular.z, tw.linear.x, tw.linear.y);
  driver.feedforward(Vb);
  // Note this is WheelVelocities type but actually contains
  // wheel angles
  w_ang = driver.get_ang();

  w_ang.ul = rigid2d::normalize_encoders(w_ang.ul);
  w_ang.ur = rigid2d::normalize_encoders(w_ang.ur);
}

int main(int argc, char** argv)
/// The Main Function ///
{
  // Vars
  std::string o_fid_, b_fid_, wl_fid_, wr_fid_;
  float wbase_, wrad_;

  ros::init(argc, argv, "fake_diff_encoders_node"); // register the node on ROS
  ros::NodeHandle nh("~"); // get a handle to ROS
  // Init Private Parameters
  nh.param<std::string>("left_wheel_joint", wl_fid_, "left_wheel_axle");
  nh.param<std::string>("right_wheel_joint", wr_fid_, "right_wheel_axle");
  nh.param<float>("/wheel_base", wbase_, 1.5);
  nh.param<float>("/wheel_radius", wrad_, 0.5);
  // Set Driver Wheel Base and Radius
  driver.set_static(1.5, 0.5);

  // Init Subscriber
  ros::Subscriber vel_sub = nh.subscribe("/turtle1/cmd_vel", 1, vel_callback);
  // Init Publisher
  ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();

    sensor_msgs::JointState js;

    // js stores vectors, so we push back the name corresp. to left wheel joint
    js.name.push_back(wl_fid_);
    // then we insert the left wheel encoder value
    js.position.push_back(w_ang.ul);

    // repeat with right wheel. Note order must be consistent between name pushback and
    // encoder value pushback
    js.name.push_back(wr_fid_);
    js.position.push_back(w_ang.ur);

    // now publish
    std::cout << w_ang.ul << std::endl;
    std::cout << w_ang.ur << std::endl;

    js_pub.publish(js);

  }

  return 0;
}