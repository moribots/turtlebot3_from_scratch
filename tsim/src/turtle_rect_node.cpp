/// \file
/// \brief Makes turtle in turtlesim move in a rectangular trajectory.
///
/// PARAMETERS:
///     x (int): x coordinate for lower left corner of rectangle
///     y (int): y coordinate for lower left corner of rectangle
///     width (int): width of rectangle
///     height (int): height of rectangle
///		trans_vel (int): translational velocity of robot
///		rot_vel (int): rotational velocity of robot
///		frequency(int): frequency of control loop
/// PUBLISHES:
///     topic_name (topic_type): description of topic
/// SUBSCRIBES:
///     topic_name (topic_type): description of the topic
/// SERVICES:
///     service_name (service_type): description of the service

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include "tsim/turtle_rect.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_rect_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  turtle_rect::TurtleRect turtle;  // instatiate our class object
  // SLEEP ONCE to make sure position syncs in the beginning
  ros::Duration(1).sleep();
  turtle.control();

  ros::spin();

  return 0;
}