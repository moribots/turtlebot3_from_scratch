#ifndef TURTLE_RECT_H
#define TURTLE_RECT_H

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <ros/console.h>

// taken from https://magiccvs.byu.edu/wiki/#!ros_tutorials/c++_node_class.md

namespace turtle_rect
{

class TurtleRect
{

public:

  TurtleRect();

  // function which publishes Twists to cmd_vel
  void control();

private:

  //***************** NODE HANDLES ***************//
  // end private parameters with _ (best practice)
  // using ros namespace ros::
  ros::NodeHandle nh_;         // public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; // private node handle for pulling parameter values from the parameter server

  //***************** PUBLISHERS AND SUBSCRIBERS ***************//
  ros::Subscriber pose_subscriber_;
  // will end up getting hooked up to the callback for the Pose message

  ros::Publisher vel_publisher_;
  // will publish the to cmd_vel

  //***************** SERVICE CLIENTS ***************//
  ros::ServiceClient pen_client_;
  // turns the turtle's pen on and off

  ros::ServiceClient tele_client_;
  // teleports the turtle

  //***************** PARAMETERS ***************//
  double threshold_;
  // a parameter we get from the ROS server, in this case the value below which
  // we consider the turtle as not moving.  This is basically a class variable at this point,
  // but it is distinct from the other class variables, so we separate them here.

  // service parameter modifiers
  turtlesim::SetPen pen_srv_;
  turtlesim::TeleportAbsolute tele_srv_;

  // turtle_rect.yaml
  int x_;
  int y_;
  int width_;
  int height_;
  int trans_vel_;
  int rot_vel_;
  int frequency_;

  // x,y,heading of turtle
  float x_pos_;
  float y_pos_;
  float head_;

  // state machine variables
  // True when correct position or heading has been achieved
  bool done_flag_ = false;
  // True when angular velocity should be applied, linear otherwise
  bool lin_ang_flag_ = true;
  // Counter which resets to zero once all rectangle vertices have been reached
  int count_vertex_ = 1;

  // movement variable
  geometry_msgs::Twist twist_;

  //***************** STATE VARIABLES ***************//
  // in this node, we don't have any variables.  Often though, we need to remember
  // things between loops, so we could create variables here to hold those values

  //***************** CALLBACKS ***************//
  void poseCallback(const turtlesim::PoseConstPtr &msg);
  // this function will get called every time ROS "spins"
  // and there is a Pose message in the queue.

  //***************** FUNCTIONS ***************//
  // Helper functions.

  // helper function to actuate the turtle
  void move(const float &goal_x, const float &goal_y, const float &goal_head);
};

} // namespace turtle_rect

#endif // TurtleRect_H