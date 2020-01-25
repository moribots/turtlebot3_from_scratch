/// \file
/// \brief Main: Makes turtlesim modeled as diff drive robot follow trajectory specified by user inputs
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVICES:

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tsim/PoseError.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>

#include <math.h>
#include <string>
#include <vector>
#include <boost/iterator/zip_iterator.hpp>

// #include "rigid2d/rigid2d.hpp"
// #include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"

using namespace rigid2d;

// GLOBAL VARS
Pose2D pose;

void poseCallback(const turtlesim::PoseConstPtr &tw)
{ //ConstPtr is a smart pointer which knows to de-allocate memory
  pose.x = tw->x;
  pose.y = tw->y;
  pose.theta = tw->theta;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: turtle_way");
  // Vars
  std::vector<float> wpts_x, wpts_y;
  float threshold;
  int frequency;

  ros::init(argc, argv, "turtle_way_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  // Init Private Parameters
  nh.getParam("waypoint_x", wpts_x);
  nh.getParam("waypoint_y", wpts_y);
  nh.param<float>("threshold", threshold, 0.08);
  nh.param<int>("frequency", frequency, 60);

  // Setup Waypoint instance using wpts_x and wpts_y
  std::vector<Vector2D> waypoints_;
  // below could be more efficient but I coudlnt' get it to work
  // Iterating over two loops simultaneously using iterator
  // std::vector<float>::iterator iter_x = wpts_x.begin();
  // std::vector<float>::iterator iter_y = wpts_y.begin();
  // for (iter_x, iter_y; iter_x != wpts_x.end() && iter_y != wpts_y.end(); ++iter_x, ++iter_y)
  // {
  //   // create Vector2D using params and assign to increasing-size waypoints_ vector
  //   Vector2D v(*iter_x, *iter_y);
  //   waypoints_.push_back(v);
  // }
  for (long unsigned int i = 0; i < wpts_x.size(); i++)
  {
    Vector2D v(wpts_x.at(i), wpts_y.at(i));
    waypoints_.push_back(v);
  }
  // Now init Waypoint instance
  Waypoints waypoints(waypoints_);

  // Init driver and set wheelbase and wheel radius
  DiffDrive driver;
  Pose2D init_pose(waypoints_.at(0).x, waypoints_.at(0).y, 0);
  // Set initial pose
  driver.reset(init_pose);
  driver.set_static(1.5, 0.5);

  // Init Publishers
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  ros::Publisher pose_err_pub = nh.advertise<tsim::PoseError>("pose_error", 1);

  // Init Pose Subscriber
  ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);

  // Init Time
  ros::Time current_time;
  current_time = ros::Time::now();

  // Init Service Clients
  // Pen Client
  ros::ServiceClient pen_client = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
  // setup pen parameters
  turtlesim::SetPen pen_srv;
  pen_srv.request.r = 255;
  pen_srv.request.g = 255;
  pen_srv.request.b = 255;
  pen_srv.request.width = 1;
  // off at first
  pen_srv.request.off = 1;
  // Teleport Client
  ros::ServiceClient tele_client = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
  // setup teleport paramters
  turtlesim::TeleportAbsolute tele_srv;
  tele_srv.request.x = waypoints_.at(0).x;
  tele_srv.request.y = waypoints_.at(0).y;
  tele_srv.request.theta = 0;
  // Remove pen, teleport, and re-add pen. Make sure services are available first.
  // http://docs.ros.org/electric/api/roscpp/html/namespaceros_1_1service.html
  ros::service::waitForService("turtle1/set_pen", -1);
  ros::service::waitForService("turtle1/teleport_absolute", -1);
  // turn pen off
  pen_client.call(pen_srv);
  // teleport
  tele_client.call(tele_srv);
  // turn pen on
  pen_srv.request.off = 0;
  pen_client.call(pen_srv);
  ROS_INFO("TURTLE TELEPORTED, READY TO LOOP");

  ros::Rate rate(frequency);

  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();
    current_time = ros::Time::now();

    // Compute next required twist
    Twist2D Vb = waypoints.nextWaypoint(driver);
    Twist2D Vb_turtle = Vb;
    // std::cout << Vb;
    // Scale twist by frequency
    Vb.w_z /= (float)frequency;
    Vb.v_x /= (float)frequency;
    // Update internal model of robot motion
    driver.feedforward(Vb);

    // Publish twist
    geometry_msgs::Twist tw;
    tw.linear.x = Vb_turtle.v_x;
    tw.linear.y = Vb_turtle.v_y;
    tw.linear.z = 0;
    tw.angular.x = 0;
    tw.angular.y = 0;
    tw.angular.z = Vb_turtle.w_z;
    vel_pub.publish(tw);

    // Publish pose error
    Pose2D driver_pose = driver.get_pose();
    // std::cout << driver_pose.x << "\t" << driver_pose.y << "\t" << driver_pose.theta << std::endl;
    float theta_err = abs(abs(pose.theta) - abs(driver_pose.theta));
    float x_err = abs(pose.x - driver_pose.x);
    float y_err = abs(pose.y - driver_pose.y);

    tsim::PoseError pose_error;
    pose_error.x_error = x_err;
    pose_error.y_error = y_err;
    pose_error.theta_error = theta_err;
    pose_err_pub.publish(pose_error);

    rate.sleep();
  }

  return 0;
}