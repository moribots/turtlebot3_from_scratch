/// \file
/// \brief Main: Publishes Odometry messages for diff drive robot with Extended Kalman Filter SLAM
///
/// PARAMETERS:
///   o_fid_ (string): parent frame ID for the published tf transform
///   o_fid_ (string): child frame ID for the published tf transform
///   wbase_ (float): wheel base of modeled diff drive robot
///   wrad_ (float): wheel radius of modeled diff drive robot
///   frequency (int): frequency of control loop.
///   callback_flag (bool): specifies whether to send a new transform (only when new pose is read)
///
///   pose (rigid2d::Pose2D): modeled diff drive robot pose based on read wheel encoder angles
///   wl_enc (float): left wheel encoder angles
///   wr_enc (float): right wheel encoder angles
///   driver (rigid2d::DiffDrive): model of the diff drive robot
///   Vb (rigid2d::Twist2D): read from driver instances to publish to odom message
///   w_vel (rigid2d::WheelVelocities): wheel velocities used to calculate ddrive robot twist
///
///   odom_tf (geometry_msgs::TransformStamped): odometry frame transform used to update RViz sim
///   odom (nav_msgs::Odometry): odometry message containing pose and twist published to odom topic
///
/// PUBLISHES:
///   odom (nav_msgs::Odometry): publishes odometry message containing pose(x,y,z) and twist(lin,ang)
/// SUBSCRIBES:
///   /joint_states (sensor_msgs::JointState), which records the ddrive robot's joint states
///
/// FUNCTIONS:
///   js_callback (void): callback for /joint_states subscriber, which records the ddrive robot's joint states
///   set_poseCallback (bool): callback for set_pose service, which resets the robot's pose in the tf tree

#include <ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rigid2d/SetPose.h"

#include<string>

#include "nuslam/landmarks.hpp"
#include "nuslam/ekf.hpp"
#include "nuslam/TurtleMap.h"

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

// GLOBAL VARS
float wl_enc = 0;
float wr_enc = 0;
float ekf_wl_enc = 0;
float ekf_wr_enc = 0;
rigid2d::Twist2D Vb;
rigid2d::WheelVelocities w_vel;
rigid2d::Pose2D reset_pose;
rigid2d::DiffDrive driver;
rigid2d::DiffDrive ekf_driver;
bool callback_flag = false;
bool landmark_flag = false;
bool odom_flag = false;
bool service_flag = false;
// EKF object
nuslam::EKF ekf;
std::vector<double> radii;
std::vector<double> x_pts;
std::vector<double> y_pts;

void js_callback(const sensor_msgs::JointState::ConstPtr &js)
{
  /// \brief /joint_states subscriber callback. Records left and right wheel angles
  ///
  /// \param js (sensor_msgs::JointState): the left and right wheel joint angles
  /// \returns pose (rigid2d::Pose2D): modeled diff drive robot pose based on read wheel encoder angles
  /** 
  * This function runs every time we get a sensor_msgs::JointState message on the "/joint_states" topic.
  * We generally use the const <message>ConstPtr &msg syntax to prevent our node from accidentally
  * changing the message, in the case that another node is also listening to it.
  */
  //ConstPtr is a smart pointer which knows to de-allocate memory
  wl_enc = js->position.at(0);
  ekf_wl_enc += wl_enc;
  // wl_enc = rigid2d::normalize_encoders(js->position.at(0));
  wr_enc = js->position.at(1);
  ekf_wr_enc += wr_enc;
  // wr_enc = rigid2d::normalize_encoders(js->position.at(1));
	w_vel = driver.updateOdometry(wl_enc, wr_enc);
	// ROS_INFO("wheel vel")
  // Get Twist for EKF
	Vb = driver.wheelsToTwist(w_vel);
  // Print Wheel Angles
	// std::cout << driver;

  callback_flag = true;
  odom_flag = true;
}

void landmark_callback(const nuslam::TurtleMap &map)
{
  std::vector<nuslam::Point> measurements;
  // Convert map to vector of Points
  // Map data has x,y relative to robot, so no change needed
  for (long unsigned int i = 0; i < map.radii.size(); i++)
  {
    rigid2d::Vector2D map_pose = rigid2d::Vector2D(map.x_pts.at(i), map.y_pts.at(i));
    nuslam::Point map_point = nuslam::Point(map_pose);
    measurements.push_back(map_point);
    // std::cout << "\nPOINT: (" << map_point.pose.x << "," << map_point.pose.y << ")" << std::endl;
  }

  // Perform prediction step of EKF here using twist
  if (odom_flag)
  {
    rigid2d::WheelVelocities ekf_w_vel = ekf_driver.updateOdometry(ekf_wl_enc, ekf_wr_enc);
    rigid2d::Twist2D ekf_Vb = ekf_driver.wheelsToTwist(ekf_w_vel);
    ekf.predict(ekf_Vb);
    ekf_wl_enc = 0;
    ekf_wr_enc = 0;
  }

  // Perform measurement update step of EKF here
  ekf.msr_update(measurements);

  // Return Map
  std::vector<nuslam::Point> map_state = ekf.return_map();

  // Now, return landmarks radii x, and y positions each in a separate vector
  radii.clear();
  x_pts.clear();
  y_pts.clear();
  for (auto iter = map_state.begin(); iter != map_state.end(); iter++)
  {
    radii.push_back(0.1);
    // std::cout << "RADIUS: " << iter->return_radius() << std::endl;
    x_pts.push_back(iter->pose.x);
    y_pts.push_back(iter->pose.y);
  }

  landmark_flag = true;
  callback_flag = true;
  odom_flag = false;
}

bool set_poseCallback(rigid2d::SetPose::Request& req, rigid2d::SetPose::Response& res)
/// \brief set_pose service callback. Sets the turtlebot's pose belief to desired value.
///
/// \param x (float32): desired x pose.
/// \param y (float32): desired y pose.
/// \param theta (float32): desired theta pose.
/// \returns result (bool): True or False.
{
  // Update pose to match service request
  reset_pose.x = req.x;
  reset_pose.y = req.y;
  reset_pose.theta = req.theta;

  // Set Result to true
  res.result = true;

  service_flag = true;
  callback_flag = true;

  return res.result;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  // Vars
  std::string o_fid_, b_fid_;
  std::string frame_id_ = "map";
  float wbase_, wrad_, frequency;
  double max_range_ = 1;
  double x_noise = 1e-20;
  double y_noise = 1e-20;
  double theta_noise = 1e-20;
  double range_noise = 1e-20;
  double bearing_noise = 1e-20;

  ros::init(argc, argv, "odometer_node"); // register the node on ROS
  ros::NodeHandle nh_("~"); // PRIVATE handle to ROS
  ros::NodeHandle nh; // PUBLIC handle to ROS
  // Init Private Parameters
  nh_.getParam("odom_frame_id", o_fid_);
  nh_.getParam("body_frame_id", b_fid_);
  // Init Global Parameters
  nh.getParam("/wheel_base", wbase_);
  nh.getParam("/wheel_radius", wrad_);
  // Filter for max detection radius
  nh.getParam("max_range", max_range_);
  // Get Noise Params
  nh.getParam("x_noise", x_noise);
  nh.getParam("y_noise", y_noise);
  nh.getParam("theta_noise", theta_noise);
  nh.getParam("range_noise", range_noise);
  nh.getParam("bearing_noise", bearing_noise);

  // For Landmark Pub
  nh_.getParam("landmark_frame_id", frame_id_);

  // Freq
  frequency = 30.0;
  // Set Driver Wheel Base and Radius
  driver.set_static(wbase_, wrad_);
  ekf_driver.set_static(wbase_, wrad_);

  // Init Service Server
  ros::ServiceServer set_pose_server = nh.advertiseService("set_pose", set_poseCallback);
  // Init Subscriber
  ros::Subscriber js_sub = nh.subscribe("joint_states", 1, js_callback);
  ros::Subscriber lnd_sub = nh.subscribe("landmarks_node/landmarks", 1, landmark_callback);
  // Init Publisher
  ros::Publisher odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher lnd_pub = nh_.advertise<nuslam::TurtleMap>("landmarks", 1);
  // Init Transform Broadcaster
  tf2_ros::TransformBroadcaster odom_broadcaster;

  // Initialize EKF class with robot state, vector of 12 landmarks at 0,0,0, and noise
  std::vector<nuslam::Point> map_state_(12, nuslam::Point());
  nuslam::Pose2D xyt_noise_var = nuslam::Pose2D(x_noise, y_noise, theta_noise);
  nuslam::RangeBear rb_noise_var_ = nuslam::RangeBear(range_noise, bearing_noise);
  ekf = nuslam::EKF(driver.get_pose(), map_state_, xyt_noise_var, rb_noise_var_, max_range_);

  // Init Time
  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate rate(frequency);

  // Main While
  while (ros::ok())
  {
    // NOTE: All callbacks will be executed before rest of main
    // so if both odom and landmark update are available, both will
    // execute before rest of main loop. Order cannot be guaranteed however
  	ros::spinOnce();
  	current_time = ros::Time::now();

  	// Update and Publish Odom Transform
  	// Init Tf
    if (service_flag == true)
    {
      // Reset Driver Pose
      driver.reset(reset_pose);
      ekf.reset_pose(reset_pose);
      ROS_DEBUG("Reset Pose:");
      ROS_DEBUG("pose x: %f", driver.get_pose().x);
      ROS_DEBUG("pose y: %f", driver.get_pose().y);
      ROS_DEBUG("pose theta: %f", driver.get_pose().theta);
      service_flag = false;
    }

    if (callback_flag)
    {
    // SLAM Node publishes Tmo = map->odom
    // To get this, we do Tmo = Tmb * Tob.inv
    // Where Tmb = map->base and Tob = odom->base
    rigid2d::Pose2D odom_pose = driver.get_pose();
    rigid2d::Pose2D ekf_pose = ekf.return_pose();

    // Construct Tmb
    rigid2d::Vector2D Vmb = rigid2d::Vector2D(ekf_pose.x, ekf_pose.y);
    rigid2d::Transform2D Tmb = rigid2d::Transform2D(Vmb, ekf_pose.theta);
    // Construct Tob
    rigid2d::Vector2D Vob = rigid2d::Vector2D(odom_pose.x, odom_pose.y);
    rigid2d::Transform2D Tob = rigid2d::Transform2D(Vob, odom_pose.theta);
    // Now find Tmo
    rigid2d::Transform2D Tmo = Tmb * Tob.inv();
    rigid2d::Transform2DS TmoS = Tmo.displacement();

    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    ROS_DEBUG("body_frame_id %s", b_fid_.c_str());
    ROS_DEBUG("odom_frame_id %s", o_fid_.c_str());
    odom_tf.header.frame_id = o_fid_;
    odom_tf.child_frame_id = b_fid_;
    // Pose
    odom_tf.transform.translation.x = TmoS.x;
    odom_tf.transform.translation.y = TmoS.y;
    odom_tf.transform.translation.z = 0;
    // use tf2 to create transform
    tf2::Quaternion q;
    q.setRPY(0, 0, TmoS.theta);
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);
    odom_tf.transform.rotation = odom_quat;
    // Send the Transform
    odom_broadcaster.sendTransform(odom_tf);

    // Update and Publish Odom Msg
    // Init Msg
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = o_fid_;
    // Pose
    odom.pose.pose.position.x = ekf_pose.x;
    odom.pose.pose.position.y = ekf_pose.y;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion ekf_q;
    ekf_q.setRPY(0, 0, TmoS.theta);
    geometry_msgs::Quaternion ekf_quat = tf2::toMsg(ekf_q);
    odom.pose.pose.orientation = ekf_quat;
    // Twist
    odom.child_frame_id = b_fid_;
    odom.twist.twist.linear.x = Vb.v_x;
    odom.twist.twist.linear.y = Vb.v_y;
    odom.twist.twist.angular.z = Vb.w_z;
    // Publish the Message
    odom_pub.publish(odom);
    callback_flag = false;

    // Publish Map State
    if (landmark_flag)
    {
      nuslam::TurtleMap belief_map;
      belief_map.radii = radii;
      belief_map.x_pts = x_pts;
      belief_map.y_pts = y_pts;
      belief_map.header.stamp = ros::Time::now();
      belief_map.header.frame_id = frame_id_;
      lnd_pub.publish(belief_map);
      landmark_flag = false;
    }
    }

    rate.sleep();
  }

  return 0;
}