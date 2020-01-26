/// \file
/// \brief Main: Publishes Odometry messages
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVICES:

#include <ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include<string>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

// GLOBAL VARS
float wl_enc = 0;
float wr_enc = 0;
rigid2d::Twist2D Vb;
rigid2d::WheelVelocities w_vel;
rigid2d::DiffDrive driver;
rigid2d::Pose2D pose;
bool callback_flag = false;

void js_callback(const sensor_msgs::JointState::ConstPtr &js)
{
  //ConstPtr is a smart pointer which knows to de-allocate memory
  wl_enc = js->position.at(0);
  // wl_enc = rigid2d::normalize_encoders(js->position.at(0));
  wr_enc = js->position.at(1);
  // wr_enc = rigid2d::normalize_encoders(js->position.at(1));
	w_vel = driver.updateOdometry(wl_enc, wr_enc);
	// ROS_INFO("wheel vel")
	Vb = driver.wheelsToTwist(w_vel);
	pose = driver.get_pose();
  // Print Wheel Angles
	// std::cout << driver;
  callback_flag = true;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  // Vars
  std::string o_fid_, b_fid_, wl_fid_, wr_fid_;
  float wbase_, wrad_, frequency;

  ros::init(argc, argv, "odometer_node"); // register the node on ROS
  ros::NodeHandle nh("~"); // get a handle to ROS
  // Init Private Parameters
  nh.param<std::string>("odom_frame_id", o_fid_, "odom");
  nh.param<std::string>("body_frame_id", b_fid_, "base_footprint");
  nh.param<std::string>("left_wheel_joint", wl_fid_, "left_wheel_axle");
  nh.param<std::string>("right_wheel_joint", wr_fid_, "right_wheel_axle");
  nh.param<float>("/wheel_base", wbase_, 1.5);
  nh.param<float>("/wheel_radius", wrad_, 0.5);
  frequency = 60;
  // Set Driver Wheel Base and Radius
  driver.set_static(1.5, 0.5);

  // Init Subscriber
  ros::Subscriber js_sub = nh.subscribe("/joint_states", 1, js_callback);
  // Init Publisher
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  // Init Transform Broadcaster
  tf::TransformBroadcaster odom_broadcaster;

  // Init Time
  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate rate(frequency);

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();
  	current_time = ros::Time::now();

  	// Update and Publish Odom Transform
  	// Init Tf
    if (callback_flag == true)
    {
      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header.stamp = current_time;
      odom_tf.header.frame_id = o_fid_;
      odom_tf.child_frame_id = b_fid_;
      // Pose
      odom_tf.transform.translation.x = pose.x;
      odom_tf.transform.translation.y = pose.y;
      odom_tf.transform.translation.z = 0;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);
      odom_tf.transform.rotation = odom_quat;
      // Send the Transform
      odom_broadcaster.sendTransform(odom_tf);

      // Update and Publish Odom Msg
      // Init Msg
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = o_fid_;
      // Pose
      odom.pose.pose.position.x = pose.x;
      odom.pose.pose.position.y = pose.y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;
      // Twist
      odom.child_frame_id = b_fid_;
      odom.twist.twist.linear.x = Vb.v_x;
      odom.twist.twist.linear.y = Vb.v_y;
      odom.twist.twist.angular.z = Vb.w_z;
      // Publish the Message
      odom_pub.publish(odom);

      callback_flag = false;
    }

    rate.sleep();
  }

  return 0;
}