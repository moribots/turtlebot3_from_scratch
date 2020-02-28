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

#include <functional>  // to use std::bind

// Global Vars
double threshold_ = 0;

namespace nuslam {


void scanCallback(const sensor_msgs::LaserScan &lsr, std::vector<Landmark> &landmarks, const ros::Publisher &landmark_pub)
{ 
  /// \brief 
  ///
  /// \param

  // First, clear the vector of landmarks upon receiving a scan
  landmarks.clear();

  // Useful LaserScan info: range_min/max, angle_min/max, time/angle_increment, scan_time, ranges[]

  // Create New Cluster (points which potentially form a landmark)
  // Threshold is used to evaluate whether a point belongs in a Cluster
  Landmark cluster(threshold_);

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
      RangeBear rb(lsr.ranges.at(i), bearing);
      // Create Point from LaserScan
      Point point(rb);
      // Evaluate Point to see if it fits within the cluster. If it is the first point, it always fits
      success = cluster.evaluate_point(point);

      if (!success)
        // Indicates that cluster has been completed
      {
        // Append cluster to vector of landmarks
        landmarks.push_back(cluster);

        // Re-initialize cluster and add the current point to it
        cluster = Landmark(threshold_);

        // bool is irrelevant here since new cluster has no points yet
        success = cluster.evaluate_point(point);

      }

    }
  }

  // Next, we check first and last clusters to see if they can be merged
  Landmark first_cluster = landmarks.at(0);
  Landmark last_cluster = landmarks.back();

  // Now, since vectors allow pop_back to delete last element, we will compare
  // the first Point of first_cluster to last Point of last_cluster.
  // Then, if they are to be merged, we merge them into first_cluster and pop last_cluster
  success = last_cluster.evaluate_point(first_cluster.points.back());

  if (success)
  {
    // Insert Points of last_cluster into first_cluster
    landmarks.at(0).points.insert(landmarks.at(0).points.end(),
                                  last_cluster.points.begin(),
                                  last_cluster.points.end());
    // Pop last_cluster from landmarks
    landmarks.pop_back();
  }

  // Next, we eliminate all clusters with less than 3 points in them
  // std::vector<Landmark>::iterator iter = landmarks.begin();
  for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++)
  {
    // Check if cluster contains less than 3 points
    if (iter->points.size() < 3)
    {
      // Erase this element from the vector
      // Notice iter is decremented after being passed to erase, but before
      // erase is executed. This will  erase the current element from the
      // vector and have the next loop's iter point at what would have
      // been the next element
      iter = landmarks.erase(iter--);
    }
  }

  // Finally, we perform circle detection for each cluster

  // Now, return landmarks radii x, and y positions each in a separate vector
  std::vector<double> radii;
  std::vector<double> x_pts;
  std::vector<double> y_pts;

  // iter = landmarks.begin();
  for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++)
  {
    radii.push_back(iter->return_radius());
    x_pts.push_back(iter->return_coords().pose.x);
    y_pts.push_back(iter->return_coords().pose.y);
  }

  // Now, publish
  TurtleMap map;
  map.radii = radii;
  map.x_pts = x_pts;
  map.y_pts = y_pts;

  landmark_pub.publish(map);
}


int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: landmarks");

  double frequency;

  ros::init(argc, argv, "landmarks_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("threshold", threshold_);
  nh_.getParam("frequency", frequency);

  // Landmark list
  std::vector<Landmark> landmarks;

  // Init Publishers
  ros::Publisher landmark_pub = nh.advertise<TurtleMap>("landmarks", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Init LaserScan Subscriber
  ros::Subscriber lsr_sub = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, std::bind(&scanCallback,
                                                                             std::placeholders::_1,
                                                                             landmarks,
                                                                             landmark_pub));

  // Init Marker
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  int marker_id_counter = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.
  // This is a 6DOF pose wrt frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = threshold_;
  marker.scale.y = threshold_;
  marker.scale.z = threshold_;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.5f;
  marker.color.g = 0.0f;
  marker.color.b = 0.5f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(); // persistent

  // Init Time
  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate rate(frequency);

  bool started_cycle = false;

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();
    current_time = ros::Time::now();


    rate.sleep();
  }

  return 0;
}

}