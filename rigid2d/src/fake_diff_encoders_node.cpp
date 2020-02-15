/// \file
/// \brief Main: Publishes Fake Encoder Messages to /joint_states
///
/// PARAMETERS:
///   wl_fid_ (string): the left wheel joint name in diff_drive.urdf.xacro of nuturtle_desctipion pkg
///   wr_fid_ (string): the right wheel joint name in diff_drive.urdf.xacro of nuturtle_desctipion pkg
///   wbase_ (float): wheel base of modeled diff drive robot
///   wrad_ (float): wheel radius of modeled diff drive robot
///   frequency (int): frequency of control loop.
///   callback_flag (bool): specifies whether to send a new transform (only when new pose is read)
///
///   pose (rigid2d::Pose2D): modeled diff drive robot pose based on read wheel encoder angles
///   wl_enc (float): left wheel encoder angles
///   wr_enc (float): right wheel encoder angles
///   driver (rigid2d::DiffDrive): model of the diff drive robot
///   Vb (rigid2d::Twist2D): read from cmd_vel subscriber
///   w_ang (rigid2d::WheelVelocities): wheel angles used to calculate ddrive robot twist (overloaded struct)
///
///   js (sensor_msgs::JointState): used to publish simulated wheel encoder readings to /joint_states topic
///
/// PUBLISHES:
///   js (sensor_msgs::JointState): publishes joint state message containing left and right wheel angles
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

// GLOBAL VARS
rigid2d::WheelVelocities w_ang;
rigid2d::DiffDrive driver;
double frequency = 60;
bool callback_flag = false;

void vel_callback(const geometry_msgs::Twist &tw)
{ 
  /// \brief cmd_vel subscriber callback. Records commanded twist
  ///
  /// \param tw (geometry_msgs::Twist): the commanded linear and angular velocity
  /// \returns w_ang (rigid2d::WheelVelocities): the left and right wheel angles
  /** 
  * This function runs every time we get a geometry_msgs::Twist message on the "cmd_vel" topic.
  * We generally use the const <message>ConstPtr &msg syntax to prevent our node from accidentally
  * changing the message, in the case that another node is also listening to it.
  */
  //ConstPtr is a smart pointer which knows to de-allocate memory
  // Scale Twist2D to loop rate of turtle_way_node
  rigid2d::Twist2D Vb(tw.angular.z / (double)frequency, tw.linear.x / (double)frequency,\
                      tw.linear.y / (double)frequency);
  driver.feedforward(Vb);
  // Note this is WheelVelocities type but actually contains
  // wheel angles
  w_ang = driver.get_ang();

  // w_ang.ul = rigid2d::normalize_encoders(w_ang.ul);
  // w_ang.ur = rigid2d::normalize_encoders(w_ang.ur);

  callback_flag = true;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  // Vars
  std::string o_fid_, b_fid_, wl_fid_, wr_fid_;
  float wbase_, wrad_;

  ros::init(argc, argv, "fake_diff_encoders_node"); // register the node on ROS
  ros::NodeHandle nh_("~"); // PRIVATE handle to ROS
  ros::NodeHandle nh; // PUBLIC handle to ROS
  // Parameters
  // Private
  nh_.getParam("left_wheel_joint", wl_fid_);
  nh_.getParam("right_wheel_joint", wr_fid_);
  // Public
  nh.getParam("/wheel_base", wbase_);
  nh.getParam("/wheel_radius", wrad_);
  // Set Driver Wheel Base and Radius
  driver.set_static(wbase_, wrad_);

  // Init Subscriber
  ros::Subscriber vel_sub = nh.subscribe("turtle1/cmd_vel", 1, vel_callback);
  // Init Publisher
  ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // Init Time
  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate rate(frequency);
  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();
    current_time = ros::Time::now();

    if (callback_flag)
    {
      sensor_msgs::JointState js;

      js.header.stamp = current_time;

      // js stores vectors, so we push back the name corresp. to left wheel joint
      js.name.push_back(wl_fid_);
      // then we insert the left wheel encoder value
      js.position.push_back(w_ang.ul);

      // repeat with right wheel. Note order must be consistent between name pushback and
      // encoder value pushback
      js.name.push_back(wr_fid_);
      js.position.push_back(w_ang.ur);

      // now publish
      // std::cout << w_ang.ul << std::endl;
      // std::cout << w_ang.ur << std::endl;
      js_pub.publish(js);

      callback_flag = false;

    }
    rate.sleep();

  }

  return 0;
}