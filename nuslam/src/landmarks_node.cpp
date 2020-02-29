/// \file
/// \brief Interprets LaserScan data and detects and publishes attributes of discovered landmarks
///
/// PARAMETERS:
///
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:
/// SERVICES:

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>
#include <string>
#include <vector>
#include <boost/iterator/zip_iterator.hpp>

#include "nuslam/landmarks.hpp"
#include "nuslam/TurtleMap.h"

#include <functional>  // To use std::bind


// Global Vars
double threshold_ = 0.5;
bool callback_flag = false;
nuslam::TurtleMap map;


void scan_callback(const sensor_msgs::LaserScan &lsr)
{ 
  /// \brief 
  ///
  /// \param

  std::vector<nuslam::Landmark> landmarks;

  // First, clear the vector of landmarks upon receiving a scan
  // landmarks.clear();

  // Useful LaserScan info: range_min/max, angle_min/max, time/angle_increment, scan_time, ranges[]

  // Create New Cluster (points which potentially form a landmark)
  // Threshold is used to evaluate whether a point belongs in a Cluster
  nuslam::Landmark cluster(threshold_);

  // Flag to indicate success of cluster expansion
  bool success = false;

  // Loop across ranges[]
  for (long unsigned int i = 0; i < lsr.ranges.size(); i++)
  {

    if (lsr.ranges.at(i) >= lsr.range_min && lsr.ranges.at(i) <= lsr.range_max)
    {

      // Wrap calculated bearing between -pi and pi
      double bearing = rigid2d::normalize_angle(i * lsr.angle_increment);

      // Store point's range and bearing
      nuslam::RangeBear rb(lsr.ranges.at(i), bearing);
      // Create Point from LaserScan
      nuslam::Point point(rb);
      // Evaluate Point to see if it fits within the cluster. If it is the first point, it always fits
      success = cluster.evaluate_point(point);

      if (!success)
        // Indicates that cluster has been completed
      {
        // Append cluster to vector of landmarks
        landmarks.push_back(cluster);

        // Re-initialize cluster and add the current point to it
        cluster = nuslam::Landmark(threshold_);

        // bool is irrelevant here since new cluster has no points yet
        success = cluster.evaluate_point(point);

      }

    }
  }

  // Next, we check first and last clusters to see if they can be merged
  nuslam::Landmark first_cluster = landmarks.at(0);
  nuslam::Landmark last_cluster = landmarks.back();


  // Now, since vectors allow pop_back to delete last element, we will compare
  // the first Point of first_cluster to last Point of last_cluster.
  // Then, if they are to be merged, we merge them into first_cluster and pop last_cluster
  success = last_cluster.evaluate_point(first_cluster.points.back());

  if (success)
  {
    // Insert Points of last_cluster into first_cluster
    landmarks.at(0).points.insert(landmarks.at(0).points.end(),
                                  landmarks.back().points.begin(),
                                  landmarks.back().points.end());
    // Pop last_cluster from landmarks
    landmarks.pop_back();
  }

  // Next, we eliminate all clusters with less than 3 points in them
  // std::vector<Landmark>::iterator iter = landmarks.begin();
  for (auto iter = landmarks.begin(); iter != landmarks.end();)
  {
    // Check if cluster contains less than 3 points
    if (iter->points.size() < 3)
    {
      // Erase this element from the vector
      // This will  erase the current element from the
      // vector and have the next loop's iter point at what would have
      // been the next element
      iter = landmarks.erase(iter);
    } else {
       iter++;
    }
  }

  // Finally, we perform circle detection for each cluster
  for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++)
  {
    iter->fit_circle();
    // ROS_INFO("FITTING CIRCLE");
  }

  // Now, return landmarks radii x, and y positions each in a separate vector
  std::vector<double> radii;
  std::vector<double> x_pts;
  std::vector<double> y_pts;

  // iter = landmarks.begin();
  int c = 0;
  for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++)
  {
    radii.push_back(iter->return_radius());
    x_pts.push_back(iter->return_coords().pose.x);
    y_pts.push_back(iter->return_coords().pose.y);
    c++;
  }

  ROS_INFO("FOUND %d CLUSTERS", c);

  // Now, publish
  map.radii = radii;
  map.x_pts = x_pts;
  map.y_pts = y_pts;

  callback_flag = true;
}


int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: landmarks");

  double frequency = 60.0;

  ros::init(argc, argv, "landmarks"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("threshold", threshold_);
  nh_.getParam("frequency", frequency);

  // Init Publishers
  ros::Publisher landmark_pub = nh_.advertise<nuslam::TurtleMap>("landmarks", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Init LaserScan Subscriber
  ros::Subscriber lsr_sub = nh.subscribe("/scan", 1, scan_callback);

  ros::Rate rate(frequency);

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();

    if (callback_flag)
    {
      landmark_pub.publish(map);
      callback_flag = false;
    }

    rate.sleep();
  }

  return 0;
}