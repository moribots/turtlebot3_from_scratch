/// \file
/// \brief Receives TurtleMap data and publishes markers to RViz which indicate landmark estimations
///
/// PARAMETERS:
///   callback_flag (bool): specifies whether to draw landmarks from based on callback trigger
///   global_map (nuslam::TurtleMap): stores lists of x,y coordinates and radii of landmarks to publish
///   frequency (double): frequency of control loop.
///   color (string): "gazebo", "scan", or "slam" determines color and size of markers for clarity
///
/// PUBLISHES:
///   scan/marker (visualization_msgs::Marker): publishes markers to indicate detected landmark positions
///
/// SUBSCRIBES:
///   /landmarks_node/landmarks (nuslam::TurtleMap), stores lists of x,y coordinates and radii of detected landmarks
///
/// FUNCTIONS:
///   mapCallback (void): callback for /landmarks_node/landmarks subscriber, which stores TurtleMap data for exraction

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
#include <visualization_msgs/Marker.h>

// GLOBAL VARS
bool callback_flag = false;
nuslam::TurtleMap global_map;

void mapCallback(const nuslam::TurtleMap &map)
{
  /// \brief store TurtleMap data for converting into marker coordinates
  /// \param nuslam::TurtleMap, which stores lists of x,y coordinates and radii of detected landmarks
  global_map = map;

  // std::cout << "NUM OF CLUSTERS: " << global_map.radii.size() << std::endl;
  
  callback_flag = true;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: draw_map");
  // Vars
  double frequency = 60.0;
  std::string color = "scan";

  ros::init(argc, argv, "draw_map_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("frequency", frequency);
  nh_.getParam("color", color);

  // Init Marker Publisher
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("scan/marker", 1);

  // Init Marker
  visualization_msgs::Marker marker;
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.5f;
  marker.color.g = 0.0f;
  marker.color.b = 0.5f;
  marker.color.a = 1.0;
  if (color == "scan")
  {
    // Heighest Marker
    marker.scale.z = 1.0;
    marker.color.r = 0.5f;
    marker.color.g = 0.0f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0;
  } else if (color == "gazebo")
  {
    // Lowest Marker
    marker.scale.z = 0.25;
    marker.color.r = 0.96f;
    marker.color.g = 0.475f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
  } else if (color == "slam")
  {
    // Mid Marker
    marker.scale.z = 0.5;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
  }

  // Init Map Subscriber
  ros::Subscriber lsr_sub = nh.subscribe("landmarks_node/landmarks", 1, mapCallback);

  ros::Rate rate(frequency);

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();

    if (callback_flag)
    {
      // Sync headers
      marker.header.frame_id = global_map.header.frame_id;
      marker.header.stamp = ros::Time::now();

      // Populate Marker information for each landmark
      // std::cout << "------------------------------------------" << std::endl;
      for (long unsigned int i = 0; i < global_map.radii.size(); i++)
      {
        marker.id = i;
        // Set the pose of the marker.
        // This is a 6DOF pose wrt frame/time specified in the header
        marker.pose.position.x = global_map.x_pts.at(i);
        marker.pose.position.y = global_map.y_pts.at(i);
        marker.pose.position.z = marker.scale.z / 2.0; // height
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = global_map.radii.at(i);
        marker.scale.y = global_map.radii.at(i);
        marker.lifetime = ros::Duration(0.5);
        // std::cout << "POS: (" << global_map.x_pts.at(i) << ", " << global_map.y_pts.at(i) << ")" << std::endl;
        marker_pub.publish(marker);

      }
      callback_flag = false;
    }

    rate.sleep();
  }

  return 0;
}