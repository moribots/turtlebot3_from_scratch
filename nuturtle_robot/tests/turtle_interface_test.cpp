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

// Testing cmd_vel --> wheel cmds
TEST(TurtleInterface, LinearWheelCmd)
{
	ros::NodeHandle nh;

	// TEST - cmd_vel publisher
	ros::Publisher cmdv_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	geometry_msgs::Twist test_twist;

	test_twist.linear.x = 0.1; // 0.1 m/s linear velocity to publish

	nuturtlebot::WheelCommands wcmd_test;

	// Wait until subcriber callback works
	bool vel_flag = false;
	while(!vel_flag)
	{
	ros::spinOnce();
	cmdv_pub.publish(test_twist);

	// TEST - wheel_cmd subscriber
	boost::shared_ptr<nuturtlebot::WheelCommands const> sharedWcmdtest;
	sharedWcmdtest = ros::topic::waitForMessage<nuturtlebot::WheelCommands>("/wheel_cmd", nh);
	if(sharedWcmdtest != NULL){
	wcmd_test = *sharedWcmdtest;
	vel_flag = true;
	}

	}

	ASSERT_EQ(wcmd_test.left_velocity, 21);
	ASSERT_EQ(wcmd_test.right_velocity, 21);
}

int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtle_interface_test");
    // ros::init(argc, argv, "test_node_rigid2d");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

