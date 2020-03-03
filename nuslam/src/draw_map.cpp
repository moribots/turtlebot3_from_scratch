/// \file
/// \brief TurtleMap data and publish markers to RViz which indicate landmark estimations
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
#include <visualization_msgs/Marker.h>

// GLOBAL VARS
bool callback_flag = false;
nuslam::TurtleMap global_map;

void mapCallback(const nuslam::TurtleMap &map)
{
  global_map = map;
  
  callback_flag = true;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: draw_map");
  // Vars
  double frequency = 60.0;

  ros::init(argc, argv, "draw_map_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("frequency", frequency);

  // Init Marker Publisher
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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
      for (long unsigned int i = 0; i < global_map.radii.size(); i++)
      {
        marker.id = i;
        // Set the pose of the marker.
        // This is a 6DOF pose wrt frame/time specified in the header
        marker.pose.position.x = global_map.x_pts.at(i);
        marker.pose.position.y = global_map.y_pts.at(i);
        marker.pose.position.z = 0.0; // height
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = global_map.radii.at(i);
        marker.scale.y = global_map.radii.at(i);
        marker.scale.z = 1.0;
        marker.lifetime = ros::Duration();
        // std::cout << "POS: (" << global_map.x_pts.at(i) << ", " << global_map.y_pts.at(i) << ")" << std::endl;
        marker_pub.publish(marker);

      }
      callback_flag = false;
    }

    rate.sleep();
  }

  return 0;
}