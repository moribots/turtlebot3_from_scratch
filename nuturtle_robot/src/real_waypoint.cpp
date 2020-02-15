/// \file
/// \brief Makes turtlesim modeled as Diff Drive robot follow a user-specified trajectory in real_waypoint.yaml
///
/// PARAMETERS:
///
/// PUBLISHES:
/// SUBSCRIBES:
///
/// FUNCTIONS:

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include "rigid2d/SetPose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>

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
bool move = false;
bool callback_flag = false;

void odomCallback(const nav_msgs::Odometry &odom)
{ 
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;
  auto roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Quaternion quat(odom.pose.pose.orientation.x,\
                       odom.pose.pose.orientation.y,\
                       odom.pose.pose.orientation.z,\
                       odom.pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);
  pose.theta = yaw;
  callback_flag = true;
}

bool startCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  move = true;

  ROS_INFO("START WAYPOINTS SERVICE CALLED");

  return true;
}

bool stopCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  move = false;

  ROS_INFO("STOP WAYPOINTS SERVICE CALLED");

  return true;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: real_waypoint");
  // Vars
  std::vector<float> wpts_x, wpts_y;
  float frequency, P_h_, P_l_, w_z_max_, v_x_max_,\
        threshold_, frac_vel;

  ros::init(argc, argv, "real_waypoint_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("frac_vel", frac_vel);
  nh_.getParam("waypoint_x", wpts_x);
  nh_.getParam("waypoint_y", wpts_y);
  nh_.getParam("threshold", threshold_);
  nh_.getParam("frequency", frequency);
  nh_.getParam("P_h", P_h_);
  nh_.getParam("P_l", P_l_);
  nh.getParam("rot_vel_max", w_z_max_);
  nh.getParam("tran_vel_max", v_x_max_);

  // Mult max vels by frac_vel
  w_z_max_ *= frac_vel;
  v_x_max_ *= frac_vel;

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
  Waypoints waypoints(waypoints_, P_h_, P_l_,\
                      w_z_max_, v_x_max_, threshold_);
  Pose2D init_pose(waypoints_.at(0).x, waypoints_.at(0).y, 0);
  // Call SetPose service with init pose
  // Init Service Client
  // SetPose Client
  ros::ServiceClient set_pose_client = nh.serviceClient<rigid2d::SetPose>("set_pose");
  ros::service::waitForService("set_pose", -1);
  // setup service parameters
  rigid2d::SetPose set_pose;
  set_pose.request.x = init_pose.x;
  set_pose.request.y = init_pose.y;
  set_pose.request.theta = init_pose.theta;
  // reset pose to zero
  set_pose_client.call(set_pose);

  // Init Publishers
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Init Marker
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  int marker_id_counter = 0;
  marker.id = marker_id_counter; // overwrite next marker by setting id=0
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

  // Init Odom Subscriber
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);

  // Init Start Waypoints Server
  ros::ServiceServer start_server = nh_.advertiseService("start", startCallback);
  // Init Stop Waypoint Server
  ros::ServiceServer stop_server = nh_.advertiseService("stop", stopCallback);

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

    if (move)
    {
      // Restart markers if one waypoint cycle done.
      if (fabs(waypoints_.at(0).x - pose.x) <= threshold_ \
          && fabs(waypoints_.at(0).y - pose.y) <= threshold_\
          && started_cycle)
      {
        marker_id_counter = 0; // Oevrwrite first marker

        // If heading from wpt -1 to wpt 0, then we have
        // completed one cycle
        // float final_head = atan2(waypoints_.at(0).y- waypoints_.back().y,
        //                          waypoints_.at(0).x - waypoints_.back().x);
        move = false; // one cycle complete
        started_cycle = false;
        ROS_INFO("ONE CYCLE COMPLETE.");
      }

      // Cycle has begun if first waypoint crossed
      if (fabs(waypoints_.at(1).x - pose.x) <= threshold_ \
          && fabs(waypoints_.at(1).y - pose.y) <= threshold_)
      {
        started_cycle = true;
      }

      // Publish marker
      marker.header.stamp = ros::Time::now();
      marker.id = marker_id_counter;
      marker_pub.publish(marker);
      marker_id_counter++; // to avoid overwriting

      // Publish twist only if received pose
      if (callback_flag)
        {
        // Compute next required twist
        Twist2D Vb = waypoints.nextWaypoint(pose);

        // Publish twist
        geometry_msgs::Twist tw;
        tw.linear.x = Vb.v_x;
        tw.linear.y = Vb.v_y;
        tw.linear.z = 0;
        tw.angular.x = 0;
        tw.angular.y = 0;
        tw.angular.z = Vb.w_z;
        vel_pub.publish(tw);
        }
     }
    rate.sleep();
  }

  return 0;
}