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


// Global Vars
bool gazebo_callback_flag = false;
bool odom_callback_flag = false;
std::string robot_name = "diff_drive";
std::vector<geometry_msgs::PoseStamped> gazebo_poses;
std::vector<geometry_msgs::PoseStamped> odom_poses;
std::vector<geometry_msgs::PoseStamped> slam_poses;
std::string frame_id_ = "odom";


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
  ros::Publisher gzb_path_pub = nh_.advertise<nav_msgs::Path>("gazebo_path", 1);
  ros::Publisher odom_path_pub = nh_.advertise<nav_msgs::Path>("odom_path", 1);

  // Init ModelState Subscriber - only calls back if gazebo launched - used to publish ground truth path
  ros::Subscriber gzb_sub = nh.subscribe("/gazebo/model_states", 1, gazebo_callback);

  // Init odom subscriber to publish path according to odometry
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);

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



    rate.sleep();
  }

  return 0;
}