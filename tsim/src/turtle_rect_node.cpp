/// \file
/// \brief Main: makes turtle in turtlesim move in a rectangular trajectory.
///
/// PARAMETERS:
///     x (int): x coordinate for lower left corner of rectangle.
///     y (int): y coordinate for lower left corner of rectangle.
///     width (int): width of rectangle.
///     height (int): height of rectangle.
///		trans_vel (int): translational velocity of robot.
///		rot_vel (int): rotational velocity of robot.
///		frequency(int): frequency of control loop.
///		threshold(float): specifies when the target pose has been reached.
/// PUBLISHES:
///     turtle1/cmd_vel (Twist): publishes a twist with linear (x) and angular (z) velocities to command turtle
///		pose_error (PoseError): publishes pose error in x, y, and theta for plotting
/// SUBSCRIBES:
///     turtle1/pose (Pose): feceives the x, y, and theta position of the turtle from turtlesim.
/// SERVICES:
///     traj_reset (Empty): teleports the turtle back to (x,y) with 0 heading, and without leaving a pen trace.
///							also resets the predicted pose estimate to these values.
///
///   taken from https://magiccvs.byu.edu/wiki/#!ros_tutorials/c++_node_class.md

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include "tsim/turtle_rect.h"

int main(int argc, char** argv)
/// The Main Function ///
{
  ros::init(argc, argv, "turtle_rect_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  turtle_rect::TurtleRect turtle;  // instatiate our class object
  // SLEEP ONCE to make sure position syncs in the beginning
  ros::Duration(1).sleep();
  turtle.control();

  // ros::spin();

  return 0;
}