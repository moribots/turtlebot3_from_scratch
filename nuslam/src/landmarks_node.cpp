/// \file
/// \brief Interprets LaserScan data and detects and publishes attributes of discovered landmarks
///
/// PARAMETERS:
///   threshold (double): used to determine whether two points from LaserScan belong to one cluster
///   callback_flag (bool): specifies whether to publish landmarks based on callback trigger
///   pc (sensor_msgs::PointCloud): contains interpreted pointcloud which is published for debugging purposes
///   map (nuslam::TurtleMap): stores lists of x,y coordinates and radii of detected landmarks
///   frequency (double): frequency of control loop.
///   frame_id_ (string): frame ID of discovered landmarks (in this case, relative to base_scan)
///
/// PUBLISHES:
///   landmarkks (nuslam::TurtleMap): publishes TurtleMap message containing landmark coordinates (x,y) and radii
///   pointcloud (sensor_msgs::PointCloud): publishes PointCloud for visualization in RViz for debugging purposees
///
/// SUBSCRIBES:
///   /scan (sensor_msgs::LaserScan), which contains data with which it is possible to extract range,bearing measurements
///
/// FUNCTIONS:
///   scan_callback (void): callback for /scan subscriber, which processes LaserScan data and detects landmarks

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <math.h>
#include <string>
#include <vector>
#include <boost/iterator/zip_iterator.hpp>

#include "nuslam/landmarks.hpp"
#include "nuslam/TurtleMap.h"

#include <functional>  // To use std::bind


// Global Vars
double threshold_ = 0.15;
bool callback_flag = false;
nuslam::TurtleMap map;
// Create Point Cloud
sensor_msgs::PointCloud pc;


void scan_callback(const sensor_msgs::LaserScan &lsr)
{ 
  /// \brief forms clusters from LaserScan data and fits circles to
  /// them before assessing whether or not they are landmarks (or walls)
  /// \param sensor_msgs::LaserScan, which contains data with
  /// which it is possible to extract range,bearing measurements

  std::vector<nuslam::Landmark> landmarks;
  // landmarks.clear();

  // Clear Point Cloud
  pc.points.clear();
  // Useful LaserScan info: range_min/max, angle_min/max, time/angle_increment, scan_time, ranges[]

  // Create New Cluster (points which potentially form a landmark)
  // Threshold is used to evaluate whether a point belongs in a Cluster
  nuslam::Landmark cluster(threshold_);

  // Flag to indicate success of cluster expansion
  bool success = false;

  // Bearing
  double bearing = lsr.angle_min;

  // Loop across ranges[]
  for (long unsigned int i = 0; i < lsr.ranges.size(); i++)
  {

    if (lsr.ranges.at(i) >= lsr.range_min && lsr.ranges.at(i) <= lsr.range_max)
    {

      // Wrap Angle
      if (bearing > lsr.angle_max && lsr.angle_max >= 0)
      {
        bearing = lsr.angle_min;
      } else if ((bearing < lsr.angle_max && lsr.angle_max < 0))
      {
        bearing = lsr.angle_min;
      }

      // bearing = rigid2d::normalize_angle(bearing);

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

      // Bias angle by minimum scan angle
      bearing += lsr.angle_increment;

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
    if (iter->points.size() <= 3)
    {
      // Erase this element from the vector
      // This will  erase the current element from the
      // vector and have the next loop's iter point at what would have
      // been the next element
      iter = landmarks.erase(iter);
    } else {
       // Populate Point Cloud
       for (auto pt_iter = iter->points.begin(); pt_iter < iter->points.end(); pt_iter++)
       {
        geometry_msgs::Point32 p32;
        p32.z = 0.05;
        p32.x = pt_iter->pose.x;
        p32.y = pt_iter->pose.y;
        pc.points.push_back(p32);
       }
       iter++;
    }
  }

  // Next, we classify the cluster into CIRCLE or NOT_CIRCLE and discard
  // the clusters which do not meet the classificiation.
  // NOTE: DO THIS BEFORE CIRCLE FIT SINCE WE DISCARD WALLS ANYWAY AND THEY
  // SLOW THINGS DOWN
  // for (auto iter = landmarks.begin(); iter != landmarks.end();)
  // {
  //   bool is_circle = iter->classify_circle();

  //   if (!is_circle)
  //     // If the cluster is not a circle
  //   {
  //       // Erase this element from the vector
  //       // This will  erase the current element from the
  //       // vector and have the next loop's iter point at what would have
  //       // been the next element
  //       iter = landmarks.erase(iter);
  //   } else {
  //      iter++;
  //   }
  // }

  // Finally, we perform circle detection for each cluster
  for (auto iter = landmarks.begin(); iter < landmarks.end(); iter++)
  {
    iter->fit_circle();
    // ROS_INFO("FITTING CIRCLE");
    // std::cout << "RADIUS: " << iter->return_radius() << std::endl;
  }

  // Now filter by radius
  for (auto iter = landmarks.begin(); iter < landmarks.end();)
  {

    if (iter->return_radius() > 0.1)
      // If the cluster is not a circle
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

  // Now, return landmarks radii x, and y positions each in a separate vector
  std::vector<double> radii;
  std::vector<double> x_pts;
  std::vector<double> y_pts;

  // iter = landmarks.begin();
  int c = 0;
  for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++)
  {
    radii.push_back(iter->return_radius());
    // std::cout << "RADIUS: " << iter->return_radius() << std::endl;
    x_pts.push_back(iter->return_coords().pose.x);
    y_pts.push_back(iter->return_coords().pose.y);
    // std::cout << "POS: (" << iter->return_coords().pose.x << ", " << iter->return_coords().pose.y << ")" << std::endl;
    c++;
  }

  // ROS_INFO("FOUND %d CLUSTERS", c);

  // Now, populate
  map.radii.clear();
  map.radii = radii;
  map.x_pts.clear();
  map.x_pts = x_pts;
  map.y_pts.clear();
  map.y_pts = y_pts;

  callback_flag = true;
}


int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: landmarks");

  double frequency = 60.0;
  std::string frame_id_ = "base_scan";

  ros::init(argc, argv, "landmarks"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("threshold", threshold_);
  nh_.getParam("frequency", frequency);
  nh_.getParam("landmark_frame_id", frame_id_);

  // Publish TurtleMap data wrt this frame
  map.header.frame_id = frame_id_;

  // Init Publishers
  ros::Publisher landmark_pub = nh_.advertise<nuslam::TurtleMap>("landmarks", 1);

  ros::Publisher pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud>("pointcloud", 1);

  // Init LaserScan Subscriber
  ros::Subscriber lsr_sub = nh.subscribe("/scan", 1, scan_callback);

  ros::Rate rate(frequency);

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();

    if (callback_flag)
    {
      map.header.stamp = ros::Time::now();
      landmark_pub.publish(map);
      pc.header.stamp = ros::Time::now();
      pc.header.frame_id = frame_id_;
      pointcloud_pub.publish(pc);
      callback_flag = false;
    }

    rate.sleep();
  }

  return 0;
}