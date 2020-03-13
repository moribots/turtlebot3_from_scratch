/// \file
/// \brief Publishes attributes of discovered landmarks straight from gazebo (no noise)
///
/// PARAMETERS:
///
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:
/// SERVICES:

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <math.h>
#include <string>
#include <vector>
#include <boost/iterator/zip_iterator.hpp>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <functional>  // To use std::bind
#include <algorithm>  // to use std::find_if

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "tsim/PoseError.h"


// Global Vars
bool gazebo_callback_flag = false;
bool odom_callback_flag = false;
bool slam_callback_flag = false;
std::string robot_name = "diff_drive";
std::vector<geometry_msgs::PoseStamped> gazebo_poses;
std::vector<geometry_msgs::PoseStamped> odom_poses;
std::vector<geometry_msgs::PoseStamped> slam_poses;
std::string frame_id_ = "map";


void gazebo_callback(const gazebo_msgs::ModelStates &model)
{
  // First, find current Diff Drive Robot Pose
  auto dd_it = std::find(model.name.begin(), model.name.end(), robot_name);
  auto dd_index = std::distance(model.name.begin(), dd_it);
  geometry_msgs::Pose dd_pose = model.pose.at(dd_index);

  geometry_msgs::PoseStamped ps;

  ps.header.frame_id = frame_id_;
  ps.header.stamp = ros::Time::now();
  ps.pose = dd_pose;

  // Append to poses vector to publish Path msg
  gazebo_poses.push_back(ps);

  gazebo_callback_flag = true;
}

void odom_callback(const nav_msgs::Odometry &odom)
{

  geometry_msgs::PoseStamped ps;

  ps.header.frame_id = frame_id_;
  ps.header.stamp = ros::Time::now();
  ps.pose = odom.pose.pose;

  // Append to poses vector to publish Path msg
  odom_poses.push_back(ps);

  odom_callback_flag = true;
}

void slam_callback(const nav_msgs::Odometry &slam)
{

  geometry_msgs::PoseStamped ps;

  ps.header.frame_id = frame_id_;
  ps.header.stamp = ros::Time::now();
  ps.pose = slam.pose.pose;

  // Append to poses vector to publish Path msg
  slam_poses.push_back(ps);

  slam_callback_flag = true;
}


int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: visualizer");

  double frequency = 60.0;

  ros::init(argc, argv, "visualizer"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("frequency", frequency);
  nh_.getParam("path_frame_id", frame_id_);

  nav_msgs::Path path;

  // Publish Path data wrt this frame
  path.header.frame_id = frame_id_;

  // Init Publishers
  // Path Publishers
  ros::Publisher gzb_path_pub = nh_.advertise<nav_msgs::Path>("gazebo_path", 1);
  ros::Publisher odom_path_pub = nh_.advertise<nav_msgs::Path>("odom_path", 1);
  ros::Publisher slam_path_pub = nh_.advertise<nav_msgs::Path>("slam_path", 1);
  // Pose Error Publishers
  ros::Publisher odom_err_pub = nh_.advertise<tsim::PoseError>("odom_err", 1);
  ros::Publisher slam_err_pub = nh_.advertise<tsim::PoseError>("slam_err", 1);

  // Init ModelState Subscriber - only calls back if gazebo launched - used to publish ground truth path
  ros::Subscriber gzb_sub = nh.subscribe("/gazebo/model_states", 1, gazebo_callback);

  // Init odom subscriber to publish path according to odometry
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);

  // Init slam subscriber to publish path according to odometry
  ros::Subscriber slam_sub = nh.subscribe("slam/odom", 1, slam_callback);

  ros::Rate rate(frequency);

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();

    if (gazebo_callback_flag)
    {
      path.header.stamp = ros::Time::now();
      // Populate using growing vector of poses
      path.poses = gazebo_poses;
      gzb_path_pub.publish(path);
      gazebo_callback_flag = false;
    }

    if (odom_callback_flag)
    {
      path.header.stamp = ros::Time::now();
      // Populate using growing vector of poses
      path.poses = odom_poses;
      odom_path_pub.publish(path);
      odom_callback_flag = false;
    }

    if (slam_callback_flag)
    {
      path.header.stamp = ros::Time::now();
      // Populate using growing vector of poses
      path.poses = slam_poses;
      slam_path_pub.publish(path);
      slam_callback_flag = false;
    }

    if (gazebo_poses.size() > 0 and odom_poses.size() > 0 and slam_poses.size() > 0)
    {
      // Current Gazebo Pose
      geometry_msgs::Pose gazebo_pose = gazebo_poses.back().pose;
      auto roll = 0.0, pitch = 0.0, gazebo_yaw = 0.0;
      tf2::Quaternion quat(gazebo_pose.orientation.x,\
                           gazebo_pose.orientation.y,\
                           gazebo_pose.orientation.z,\
                           gazebo_pose.orientation.w);
      tf2::Matrix3x3 mat(quat);
      mat.getRPY(roll, pitch, gazebo_yaw);    
  

      // Current Odom Pose
      geometry_msgs::Pose odom_pose = odom_poses.back().pose;
      auto odom_yaw = 0.0;
      quat = tf2::Quaternion(odom_pose.orientation.x,\
                             odom_pose.orientation.y,\
                             odom_pose.orientation.z,\
                             odom_pose.orientation.w);
      mat = tf2::Matrix3x3(quat);
      mat.getRPY(roll, pitch, odom_yaw);
    

      // Current SLAM Pose
      geometry_msgs::Pose slam_pose = slam_poses.back().pose;
      auto slam_yaw = 0.0;
      quat = tf2::Quaternion(slam_pose.orientation.x,\
                             slam_pose.orientation.y,\
                             slam_pose.orientation.z,\
                             slam_pose.orientation.w);
      mat = tf2::Matrix3x3(quat);
      mat.getRPY(roll, pitch, slam_yaw);

      // Publish pose error between odom-gazebo
      tsim::PoseError odom_err;
      odom_err.x_error = fabs(gazebo_pose.position.x - odom_pose.position.x);
      odom_err.y_error = fabs(gazebo_pose.position.y - odom_pose.position.y);
      odom_err.theta_error = fabs(fabs(gazebo_yaw) - fabs(odom_yaw));
      odom_err_pub.publish(odom_err);

      // Publish pose error between slam-gazebo
      tsim::PoseError slam_err;
      slam_err.x_error = fabs(gazebo_pose.position.x - slam_pose.position.x);
      slam_err.y_error = fabs(gazebo_pose.position.y - slam_pose.position.y);
      slam_err.theta_error = fabs(fabs(gazebo_yaw) - fabs(slam_yaw));
      slam_err_pub.publish(slam_err);
    }

    rate.sleep();
  }

  return 0;
}