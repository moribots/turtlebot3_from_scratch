/// \file
/// \brief Class Constructor for TurtleRect.
///
/// PARAMETERS:
///   x_ (int): x coordinate for lower left corner of rectangle.
///   y_ (int): y coordinate for lower left corner of rectangle.
///   width_ (int): width of rectangle.
///   height_ (int): height of rectangle.
///   trans_vel_ (int): translational velocity of robot.
///   rot_vel_ (int): rotational velocity of robot.
///   frequency_ (int): frequency of control loop.
///   threshold_ (float): specifies when the target pose has been reached.
///
///   goal_x (float): target turtle position in x.
///   goal_y (float): target turtle position in y.
///   goal_head (float): target turtle position in theta.
///
///   x_pos_ (float): turtle position in x read from turtle1/pose topic.
///   y_pos_ (float): turtle position in y read from turtle1/pose topic.
///   head_ (float): turtle position in theta read from turtle1/pose topic.
///
///   x_o_ (float): turtle position in x predicted using forward model propagation.
///   y_o_ (float): turtle position in y predicted using forward model propagation.
///   head_o_ (float): turtle position in theta predicted using forward model propagation.
///
///   x_error_ (float): turtle position error in x between read and predicted values.
///   y_error_ (float): turtle position error in y between read and predicted values.
///   theta_error_ (float): turtle position error in theta between read and predicted values.
///
///   pose_error_ (PoseError): custom message that stores x_error, y_error and theta_error.
///   twist_ (Twist): used to publish linear and angular velocities to turtle1/cmd_vel.
///
///   done_flag_ (bool): true when correct position or heading has been achieved in loop.
///   lin_ang_flag_ (bool): true when angular velocity should be applied, linear otherwise.
///   count)vertex_ (int): counter which resets to zero once all rectangle vertices have been reached.
///
/// FUNCTIONS:
///   traj_resetCallback (bool): callback for traj_reset service, which teleports turtle back to initial config.
///   poseCallback (void): callback for turtle1/pose subscriber, which records the turtle's pose for use elsewhere.
///   move (void): helper function which publishes Twist messages to turtle1/cmd_vel to actuate the turtle.
///   predict(void): helper function which forward propagates the open-loop model and publishes PoseError to pose_error.
///   control(void): main class method. Houses state machine and calls helper function to perform trajectory and plot.
///
///   taken from https://magiccvs.byu.edu/wiki/#!ros_tutorials/c++_node_class.md

#include "tsim/turtle_rect.h"

namespace turtle_rect
{

TurtleRect::TurtleRect() :
  nh_(ros::NodeHandle()),             /* This is an initialization list */
  nh_private_(ros::NodeHandle("~"))   /* The Node Handle can access the node and its attributes */
{
  // THIS IS THE CLASS CONSTRUCTOR //

  //***************** RETREIVE PARAMS ***************//
  nh_private_.param<float>("threshold", threshold_, 0.08);
  // This will pull the "threshold" parameter from the ROS server, and store it in the threshold_ variable.
  // If no value is specified on the ROS param server, then the default value of 0.0001 will be applied

  nh_private_.param<int>("x", x_, 3);
  nh_private_.param<int>("y", y_, 2);
  nh_private_.param<int>("width", width_, 4);
  nh_private_.param<int>("height", height_, 5);
  nh_private_.param<int>("trans_vel", trans_vel_, 2);
  nh_private_.param<int>("rot_vel", rot_vel_, 1);
  nh_private_.param<int>("frequency", frequency_, 100);

  // print parameters
  ROS_INFO("x: %d", x_);
  ROS_INFO("y: %d", y_);
  ROS_INFO("width: %d", width_);
  ROS_INFO("height: %d", height_);
  ROS_INFO("trans_vel: %d", trans_vel_);
  ROS_INFO("rot_vel: %d", rot_vel_);
  ROS_INFO("frequency: %d", frequency_);

  //***************** CUSTOM SERVER **************//
  traj_reset_server_ = nh_.advertiseService("traj_reset", &TurtleRect::traj_resetCallback, this);
  // trasj_reset service will teleport turtle to bottom left position and resume trajectory

  //***************** NODE HANDLES ***************//
  pose_subscriber_ = nh_.subscribe("turtle1/pose", 1, &TurtleRect::poseCallback, this);
  // This connects the poseCallback function with the reception of a Pose message on the "turtle1/pose" topic
  // ROS will essentially call the poseCallback function every time it receives a message on that topic.
  // 1 is the queue size.
  // 'this' is a class pointer.

  vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  // This connects a geometry_msgs::Twist message on the "turtle1/cmd_vel" topic. 1 is the queue size.

  pose_error_publisher_ = nh_.advertise<tsim::PoseError>("pose_error", 1);

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
  ros::service::waitForService("turtle1/set_pen", -1);
  ros::service::waitForService("turtle1/teleport_absolute", -1);
  ros::service::waitForService("traj_reset", -1);
  
  // turn pen off
  pen_client_.call(pen_srv_);
  // teleport
  tele_client_.call(tele_srv_);
  // turn pen on
  pen_srv_.request.off = 0;
  pen_client_.call(pen_srv_);

  // predicted turtle pose
  x_o_ = x_;
  y_o_ = y_;
  head_o_ = 0;

  // sleep
  ros::Duration(1).sleep();

}

bool TurtleRect::traj_resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
/// \brief traj_reset service callback. Teleports turtle1 to original config.
///
/// \param x_ (int): x coordinate for lower left corner of rectangle.
/// \param y_ (int): y coordinate for lower left corner of rectangle.
/// \returns x_o_ (float): turtle position in x predicted using forward model propagation.
/// \returns y_o_ (float): turtle position in y predicted using forward model propagation.
/// \returns head_o_ (float): turtle position in theta predicted using forward model propagation.
/// \returns publishes Twist.
/**
 * Teleports the turtle back to its initial configuration (x_, y_, 0), equates the predicted turtle
 * position to this value and publishes all-zero Twist to ensure synchronization.
 * Turns pen off during operation, and then back on subsequently.
 */
{
  // turn pen off
  pen_srv_.request.off = 1;
  pen_client_.call(pen_srv_);
  // teleport
  tele_client_.call(tele_srv_);
  // turn pen on
  pen_srv_.request.off = 0;
  pen_client_.call(pen_srv_);

  // Reset state machine
  count_vertex_ = 1;
  done_flag_ = false;
  lin_ang_flag_ = true;

  ROS_INFO("TURTLE RESET");

  // reset predicton
  x_o_ = x_;
  y_o_ = y_;
  head_o_ = 0;

  twist_.linear.x = 0;
  twist_.linear.y = 0;
  twist_.linear.z = 0;
  twist_.angular.x = 0;
  twist_.angular.y = 0;
  twist_.angular.z = 0;

  // Publish
  vel_publisher_.publish(twist_);
  ros::Duration(0.5).sleep();

  return true;
}

void TurtleRect::poseCallback(const turtlesim::PoseConstPtr &msg)
/// \brief turtle1/pose subscriber callback. Records turtle1 pose (x, y, theta)
///
/// \param msg (Pose): turtle1's pose in x, y, and theta.
/// \returns x_pos_ (float): turtle position in x read from turtle1/pose topic.
/// \returns y_pos_ (float): turtle position in y read from turtle1/pose topic.
/// \returns head_ (float): turtle position in theta read from turtle1/pose topic.
/** 
* This function runs every time we get a turtlesim::Pose message on the "turtle1/pose" topic.
* We generally use the const <message>ConstPtr &msg syntax to prevent our node from accidentally
* changing the message, in the case that another node is also listening to it.
*/
{
  x_pos_ = msg->x;
  y_pos_ = msg->y;
  head_ = msg->theta;
}

void TurtleRect::move(const float &goal_x, const float &goal_y, const float &goal_head)
/// \brief Actuates the turtle by publishing Twists to turtle1/cmd_vel.
///
/// \param goal_x (float): target turtle position in x.
/// \param goal_y (float): target turtle position in y.
/// \param goal_head (float): target turtle position in theta.
/// \param lin_ang_flag (bool): true when angular velocity should be applied, linear otherwise.
/// \returns publishes Twist.
/** 
* This function sends Twist commands to turtle1/cmd_vel. It sends either linear or angular Twists
* depending on the flag provided, which changes depending on the turtle's goal pose, whereby changes
* in goal pose take precedence with respect to their theta component.
*/
{
  // ROS_DEBUG("MOVE LOOP");

  switch(lin_ang_flag_)
  {
    case true:
      // ROS_DEBUG("CHECK ANG");
      // Check if need to move lin or ang
      if (abs(goal_head - head_) <= threshold_ / 3.0)
      {
        lin_ang_flag_ = false;
        // ROS_DEBUG("VAL: %f", goal_head - head_ - threshold_);
        // ROS_DEBUG("THRESH: %f", threshold_ / 3.0);
      } else {
        // ROS_DEBUG("ANG");
        // ROS_DEBUG("head: %f \t goal_head: %f", head_, goal_head);
        // Move Angularly
        twist_.linear.x = 0;
        twist_.linear.y = 0;
        twist_.linear.z = 0;
        twist_.angular.x = 0;
        twist_.angular.y = 0;
        twist_.angular.z = rot_vel_;

        // Predict turtle pos assuming perfect commands
        // and publish to "pose_error" topic as PoseError msg
        this->predict();

        // Publish
        vel_publisher_.publish(twist_);
      }

      break;

    case false:
      ROS_DEBUG("CHECK LIN");
      // Check if need to move lin or ang
      if (abs(goal_y - y_pos_) <= threshold_ \
          && abs(goal_x - x_pos_) <= threshold_)
      {
        lin_ang_flag_ = true;
      } else {
        // ROS_DEBUG("LIN");
        // ROS_INFO("test y: %d", abs(goal_y - y_pos_) <= threshold_);
        // ROS_INFO("test x: %d", abs(goal_x - x_pos_) <= threshold_);
        // ROS_INFO("TEST y: %f", abs(goal_y - y_pos_));
        // ROS_INFO("TEST x: %f", abs(goal_x - x_pos_));
        // ROS_INFO("THRESH: %f", abs(threshold_));

        // ROS_DEBUG("x: %f \t goal_x: %f", x_pos_, goal_x);
        // ROS_DEBUG("y: %f \t goal_y: %f", y_pos_, goal_y);
        // Move Linearly
        twist_.linear.x = trans_vel_;
        twist_.linear.y = 0;
        twist_.linear.z = 0;
        twist_.angular.x = 0;
        twist_.angular.y = 0;
        twist_.angular.z = 0;

        // Predict turtle pos assuming perfect commands
        // and publish to "pose_error" topic as PoseError msg
        this->predict();

        // Publish
        vel_publisher_.publish(twist_);
      }

      break;
  }

}

void TurtleRect::predict()
/// \brief Predicts where the turtle would be had it followed commands precisely and publishes resultant error to pose_error.
///
/// \param x_pos_ (float): turtle position in x read from turtle1/pose topic.
/// \param y_pos_ (float): turtle position in y read from turtle1/pose topic.
/// \param head_ (float): turtle position in theta read from turtle1/pose topic.
/// \returns x_o_ (float): turtle position in x predicted using forward model propagation.
/// \returns y_o_ (float): turtle position in y predicted using forward model propagation.
/// \returns head_o_ (float): turtle position in theta predicted using forward model propagation.
/// \returns publishes pose_error (PoseError) to pose_error topic.
/** 
* This function forward-propagates the turtle with the actual published Twists and compares the resultant
* position to that actually recorded from turtle1/pose. Next, it calculates the resultant error, ensuring
* that the theta values are angle-wrapped, and publishes said error to the pose_error topic.
*/
{
  x_o_ += twist_.linear.x * cos(head_o_) * 1.0 / (float)frequency_;
  y_o_ += twist_.linear.x * sin(head_o_) * 1.0 / (float)frequency_;
  head_o_ += twist_.angular.z * 1.0 / (float)frequency_;

  // wrap to -PI to PI
  head_o_ = std::fmod(head_o_ + PI, 2 * PI);
  if (head_o_ < 0)
  {
    head_o_ += 2 * PI;
  }
  head_o_ -= M_PI;

  theta_error_ = abs(abs(head_o_) - abs(head_));
  if (theta_error_ > 10)
  {
    theta_error_ = 0;
  }
  x_error_ = abs(x_o_ - x_pos_);
  // ROS_INFO("X_OPEN: %f \t X_SUB: %f", x_o_, x_pos_);
  y_error_ = abs(y_o_ - y_pos_);
  // ROS_INFO("Y_OPEN: %f \t Y_SUB: %f", y_o_, y_pos_);

  pose_error_.x_error = x_error_;
  pose_error_.y_error = y_error_;
  pose_error_.theta_error = theta_error_;

  pose_error_publisher_.publish(pose_error_);
}

void TurtleRect::control()
/// \brief Main class method; implements state machine to trace rectangular trajectory and plot.
///
/// \param done_flag_ (bool): true when correct position or heading has been achieved in loop.
/// \param lin_ang_flag_ (bool): true when angular velocity should be applied, linear otherwise.
/// \param count)vertex_ (int): counter which resets to zero once all rectangle vertices have been reached.
/// \param x_pos_ (float): turtle position in x read from turtle1/pose topic.
/// \param y_pos_ (float): turtle position in y read from turtle1/pose topic.
/// \param head_ (float): turtle position in theta read from turtle1/pose topic.
/// \param goal_x (float): target turtle position in x.
/// \param goal_y (float): target turtle position in y.
/// \param goal_head (float): target turtle position in theta.
/// \returns calls helper function and implements state machine for node.
/** 
* This function implements a four-case state machine, guiding the turtle in turtlesim to trace a rectangular
* trajectory by calling various class helper methods and implementing a counter, a vertex completion flag, and
* a flag to indicate whether linear or angular velocity commands should be given.
*/
{
  while (ros::ok())
  {
  
  ros::Rate rate(frequency_);

  // ROS_DEBUG("CONTROL LOOP");

  // decalre goal position variables
  float goal_x = 0;
  float goal_y = 0;
  float goal_head = 0;

  switch(count_vertex_)
  {

    case 0:
      // vertex 1
      ROS_DEBUG("CASE0");
      goal_head = - PI / 2;
      goal_x = x_;
      goal_y = y_;

      // function to move (pass in goals), move fcn decides if lin or ang
      // updates vertex done flag
      // 'this' is a pointer to the TurtleRect class
      this->move(goal_x, goal_y, goal_head);

      if (abs(goal_x - x_pos_) <= threshold_ \
        && abs(goal_y - y_pos_) <= threshold_ \
        && abs(goal_head - head_) <= threshold_ / 3.0)
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
    ROS_DEBUG("CASE1");
      goal_head = 0;
      goal_x = x_ + width_;
      goal_y = y_;

      this->move(goal_x, goal_y, goal_head);

      if (abs(goal_x - x_pos_) <= threshold_ \
        && abs(goal_y - y_pos_) <= threshold_ \
        && abs(goal_head - head_) <= threshold_ / 3.0)
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
    ROS_DEBUG("CASE2");
      goal_head = PI / 2.0;
      goal_x = x_ + width_;
      goal_y = y_ + height_;

      this->move(goal_x, goal_y, goal_head);

      if (abs(goal_x - x_pos_) <= threshold_ \
        && abs(goal_y - y_pos_) <= threshold_ \
        && abs(goal_head - head_) <= threshold_ / 3.0)
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
    ROS_DEBUG("CASE3");
      goal_head = PI;
      goal_x = x_;
      goal_y = y_ + height_;

      this->move(goal_x, goal_y, goal_head);

      if (abs(goal_x - x_pos_) <= threshold_ \
        && abs(goal_y - y_pos_) <= threshold_ \
        && abs(goal_head - head_) <= threshold_ / 3.0)
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
}

} // namespace turtle_rect