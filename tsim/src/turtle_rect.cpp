#include "tsim/turtle_rect.h"

// taken from https://magiccvs.byu.edu/wiki/#!ros_tutorials/c++_node_class.md

namespace turtle_rect
{

TurtleRect::TurtleRect() :
  nh_(ros::NodeHandle()),             /* This is an initialization list */
  nh_private_(ros::NodeHandle("~"))   /* The Node Handle can access the node and its attributes */
{
  // THIS IS THE CLASS CONSTRUCTOR //

  //***************** RETREIVE PARAMS ***************//
  nh_private_.param<double>("threshold", threshold_, 0.0001);
  // This will pull the "threshold" parameter from the ROS server, and store it in the threshold_ variable.
  // If no value is specified on the ROS param server, then the default value of 0.0001 will be applied

  nh_private_.param<int>("x", x_, 1);
  nh_private_.param<int>("y", y_, 1);
  nh_private_.param<int>("width", width_, 1);
  nh_private_.param<int>("height", height_, 1);
  nh_private_.param<int>("trans_vel", trans_vel_, 1);
  nh_private_.param<int>("rot_vel", rot_vel_, 1);
  nh_private_.param<int>("frequency", frequency_, 1);

  //***************** NODE HANDLES ***************//
  pose_subscriber_ = nh_.subscribe("turtle1/pose", 10, &TurtleRect::poseCallback, this);
  // This connects the poseCallback function with the reception of a Pose message on the "turtle1/pose" topic
  // ROS will essentially call the poseCallback function every time it receives a message on that topic.
  // 1 is the queue size.
  // 'this' is a class pointer.

  vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  // This connects a std_msgs::Bool message on the "is_moving" topic. 1 is the queue size.

  pen_client_ = nh_.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

  // setup pen parameters
  pen_srv_.request.r = 255;
  pen_srv_.request.g = 255;
  pen_srv_.request.b = 255;
  pen_srv_.request.width = 1;
  // off at first
  pen_srv_.request.off = 1;

  tele_client_ = nh_.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

  // setup teleport paramters
  tele_srv_.request.x = x_;
  tele_srv_.request.y = y_;
  tele_srv_.request.theta = 0;

  // Remove pen, teleport, and re-add pen. Make sure services are available first.
  // http://docs.ros.org/electric/api/roscpp/html/namespaceros_1_1service.html
  ros::service::waitForService("turtle1/set_pen", 5);
  ros::service::waitForService("turtle1/teleport_absolute", 5);
  // turn pen off
  pen_client_.call(pen_srv_);
  // teleport
  tele_client_.call(tele_srv_);
  // turn pen on
  pen_srv_.request.off = 0;
  pen_client_.call(pen_srv_);
}

void TurtleRect::poseCallback(const turtlesim::PoseConstPtr &msg)
// This function runs every time we get a turtlesim::Pose message on the "turtle1/pose" topic.
// We generally use the const <message>ConstPtr &msg syntax to prevent our node from accidentally
// changing the message, in the case that another node is also listening to it.
{
  x_pos_ = msg->x;
  y_pos_ = msg->y;
  head_ = msg->theta;
}

void TurtleRect::move(const int &goal_x, const int &goal_y, const int &goal_head)
{

  switch(lin_ang_flag_)
  {
    case true:
      // Check if need to move lin or ang
      if (-threshold_ <= goal_head - head_ <= threshold_)
      {
        lin_ang_flag_ = false;
      } else {
        // Move Angularly
        twist_.linear.x = 0;
        twist_.linear.y = 0;
        twist_.linear.z = 0;
        twist_.angular.x = 0;
        twist_.angular.y = 0;
        twist_.angular.z = rot_vel_;

        // Publish
        vel_publisher_.publish(twist_);
      }

      break;

    case false:
      // Check if need to move lin or ang
      if (-threshold_ <= goal_y - y_pos_ <= threshold_ \
          && -threshold_ <= goal_head - head_ <= threshold_)
      {
        lin_ang_flag_ = true;
      } else {
        // Move Linearly
        twist_.linear.x = trans_vel_;
        twist_.linear.y = 0;
        twist_.linear.z = 0;
        twist_.angular.x = 0;
        twist_.angular.y = 0;
        twist_.angular.z = 0;

        // Publish
        vel_publisher_.publish(twist_);
      }

      break;
  }

}

void TurtleRect::control()
{
  ros::Rate rate(frequency_);

  // decalre goal position variables
  int goal_x = 0;
  int goal_y = 0;
  int goal_head = 0;

  switch(count_vertex_)
  {

    case 0:
      // vertex 1
      goal_head = 3.14159;
      goal_x = x_;
      goal_y = y_;

      // function to move (pass in goals), move fcn decides if lin or ang
      // updates vertex done flag
      // 'this' is a pointer to the TurtleRect class
      this->move(goal_x, goal_y, goal_head);

      if (-threshold_ <= goal_x - x_pos_ <= threshold_ \
        && -threshold_ <= goal_y - y_pos_ <= threshold_ \
        && -threshold_ <= goal_head - head_ <= threshold_)
      {
        done_flag_ = true;
      }

      if (done_flag_)
      {
        count_vertex_ = 1;
        done_flag_ = false;
        lin_ang_flag_ = true;
      }
      break;

    case 1:
      // vertex 2
      goal_head = -3.14159 / 2.0;
      goal_x = x_ + width_;
      goal_y = y_;

      this->move(goal_x, goal_y, goal_head);

      if (-threshold_ <= goal_x - x_pos_ <= threshold_ \
        && -threshold_ <= goal_y - y_pos_ <= threshold_ \
        && -threshold_ <= goal_head - head_ <= threshold_)
      {
        done_flag_ = true;
      }

      if (done_flag_)
      {
        count_vertex_ = 2;
        done_flag_ = false;
        lin_ang_flag_ = true;
      }
      break;

    case 2:
      // vertex 3
      goal_head = 0;
      goal_x = x_ + width_;
      goal_y = y_ + height_;

      this->move(goal_x, goal_y, goal_head);

      if (-threshold_ <= goal_x - x_pos_ <= threshold_ \
        && -threshold_ <= goal_y - y_pos_ <= threshold_ \
        && -threshold_ <= goal_head - head_ <= threshold_)
      {
        done_flag_ = true;
      }

      if (done_flag_)
      {
        count_vertex_ = 3;
        done_flag_ = false;
        lin_ang_flag_ = true;
      }
      break;

    case 3:
      // vertex 4
      goal_head = 3.14159 / 2.0;
      goal_x = x_;
      goal_y = y_ + height_;

      this->move(goal_x, goal_y, goal_head);

      if (-threshold_ <= goal_x - x_pos_ <= threshold_ \
        && -threshold_ <= goal_y - y_pos_ <= threshold_ \
        && -threshold_ <= goal_head - head_ <= threshold_)
      {
        done_flag_ = true;
      }

      if (done_flag_)
      {
        count_vertex_ = 0;
        done_flag_ = false;
        lin_ang_flag_ = true;
      }
      break;

    default:
      // reset count_vertex to zero by default
      count_vertex_ = 0;

  }

  ros::spinOnce();
  rate.sleep();

}

} // namespace turtle_rect