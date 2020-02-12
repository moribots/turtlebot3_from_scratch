#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/JointState.h>

#include<string>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
// Bring in gtest
#include <gtest/gtest.h>

// Global Vars
bool vel_flag = false;
nuturtlebot::WheelCommands wcmd_test;
bool sensor_flag = true;
rigid2d::WheelVelocities w_ang;
rigid2d::WheelVelocities w_vel_measured;

// joint_states callback
void js_callback(const sensor_msgs::JointState &js)
{
  w_ang.ul = js.position[0];
  w_ang.ur = js.position[1];
  w_vel_measured.ul = js.velocity[0];
  w_vel_measured.ur = js.velocity[1];
  sensor_flag = true;
}

// /wheel_cmd callback
void wcmd_callback(const nuturtlebot::WheelCommands &wcmd)
{
  wcmd_test = wcmd;
  vel_flag = true;
}

// Testing cmd_vel --> wheel cmds
TEST(TurtleInterface, SensorToJoints)
{
	ros::NodeHandle nh;

	// Init Subscriber
	ros::Subscriber js_sub = nh.subscribe("/joint_states", 1, js_callback);

	// TEST - /sensor_data publisher
	ros::Publisher sns_pub = nh.advertise<nuturtlebot::SensorData>("/sensor_data", 1, true);
	nuturtlebot::SensorData test_sensor;

	test_sensor.left_encoder = 100;
	test_sensor.right_encoder = 100;

	w_ang.ul = 0;
	w_ang.ur = 0;
	w_vel_measured.ul = 0;
	w_vel_measured.ur = 0;

	// Wait until subcriber callback works
	sensor_flag = false;
	while(!sensor_flag)
	{
	ros::spinOnce();
	sns_pub.publish(test_sensor);
	}

	ASSERT_NEAR(w_ang.ul, 0.15339790821170141, 1e-3);
	ASSERT_NEAR(w_ang.ur, 0.15339790821170141, 1e-3);
	ASSERT_NEAR(w_vel_measured.ul, 0.15339790821170141, 1e-3);
	ASSERT_NEAR(w_vel_measured.ur, 0.15339790821170141, 1e-3);
}

// Testing cmd_vel --> wheel cmds
TEST(TurtleInterface, LinearWheelCmd)
{
	ros::NodeHandle nh;

	// Init Subscriber
	ros::Subscriber wcmd_sub = nh.subscribe("/wheel_cmd", 1, wcmd_callback);

	// TEST - cmd_vel publisher
	ros::Publisher cmdv_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	geometry_msgs::Twist test_twist;

	test_twist.linear.x = 0.1; // 0.1 m/s linear velocity to publish

	wcmd_test.left_velocity = 0;
	wcmd_test.right_velocity = 0;

	// Wait until subcriber callback works
	vel_flag = false;
	while(!vel_flag)
	{
	ros::spinOnce();
	cmdv_pub.publish(test_twist);

	// // TEST - wheel_cmd subscriber
	// boost::shared_ptr<nuturtlebot::WheelCommands const> sharedWcmdtest;
	// sharedWcmdtest = ros::topic::waitForMessage<nuturtlebot::WheelCommands>("/wheel_cmd", nh, ros::Duration(0.5));
	// if(sharedWcmdtest != NULL){
	// wcmd_test = *sharedWcmdtest;
	// vel_flag = true;
	// }

	}

	ASSERT_EQ(wcmd_test.left_velocity, 126);
	ASSERT_EQ(wcmd_test.right_velocity, 126);
}

// Testing cmd_vel --> wheel cmds
TEST(TurtleInterface, RotWheelCmd)
{
	ros::NodeHandle nh;

	// Init Subscriber
	ros::Subscriber wcmd_sub = nh.subscribe("/wheel_cmd", 1, wcmd_callback);

	// TEST - cmd_vel publisher
	ros::Publisher cmdv_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	geometry_msgs::Twist test_twist;

	test_twist.angular.z = 1; // 1 m/s rotational velocity to publish

	wcmd_test.left_velocity = 0;
	wcmd_test.right_velocity = 0;

	// Wait until subcriber callback works
	vel_flag = false;
	while(!vel_flag)
	{
	ros::spinOnce();
	cmdv_pub.publish(test_twist);

	}

	ASSERT_EQ(wcmd_test.left_velocity, -101);
	ASSERT_EQ(wcmd_test.right_velocity, 101);
}

// Testing cmd_vel --> wheel cmds
TEST(TurtleInterface, MixedWheelCmd)
{
	ros::NodeHandle nh;

	// Init Subscriber
	ros::Subscriber wcmd_sub = nh.subscribe("/wheel_cmd", 1, wcmd_callback);

	// TEST - cmd_vel publisher
	ros::Publisher cmdv_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	geometry_msgs::Twist test_twist;

	test_twist.angular.z = 1; // 1 m/s rotational velocity to publish
	test_twist.linear.x = 0.1; // 0.1 m/s linear velocity to publish

	wcmd_test.left_velocity = 0;
	wcmd_test.right_velocity = 0;

	// Wait until subcriber callback works
	vel_flag = false;
	while(!vel_flag)
	{
	ros::spinOnce();
	cmdv_pub.publish(test_twist);

	}

	ASSERT_EQ(wcmd_test.left_velocity, 25);
	ASSERT_EQ(wcmd_test.right_velocity, 227);
}

int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtle_interface_test");
    return RUN_ALL_TESTS();
}

