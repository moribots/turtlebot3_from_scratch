#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

TEST(rigid2d_lib, VectorIO)
{
	// make sure input and output are read correctly
	rigid2d::Vector2D v;

	std::string input = "1 1";
	std::stringstream in_s(input);

	std::string output = "[1, 1]\n";
	std::stringstream out_s;

	in_s >> v;
	out_s << v;

	// compare raw input to in_s input
	ASSERT_EQ(in_s.str(), input);
	// compare raw output to out_s input
	ASSERT_EQ(out_s.str(), output);
}

TEST(rigid2d_lib, VectorAdd)
{
	// +=
	rigid2d::Vector2D v1(1, 2);
	rigid2d::Vector2D v2(1, 2);
	rigid2d::Vector2D v3 = v1; // temp to store v1
	rigid2d::Vector2D v4 = v2; // temp to store v2

	v1 += v2;
	v2 += v3;

	std::stringstream out_1;
	std::stringstream out_2;

	out_1 << v1;
	out_2 << v2;

	ASSERT_EQ(out_1.str(), out_2.str());

	// +
	v3 = v1 + v2;
	v4 = v2 + v1;

	std::stringstream out_3;
	std::stringstream out_4;
	out_3 << v3;
	out_4 << v4;

	ASSERT_EQ(out_3.str(), out_4.str());
}

TEST(rigid2d_lib, VectorSubtract)
{
	// -=
	rigid2d::Vector2D v1(1, 2);
	rigid2d::Vector2D v2(1, 2);
	rigid2d::Vector2D v3 = v1; // temp to store v1
	rigid2d::Vector2D v4 = v2; // temp to store v2

	v1 -= v2;
	v2 -= v3;

	std::stringstream out_1;
	std::stringstream out_2;

	out_1 << v1;
	out_2 << v2;

	ASSERT_EQ(out_1.str(), out_2.str());

	// -
	v3 = v1 - v2;
	v4 = v2 - v1;

	std::stringstream out_3;
	std::stringstream out_4;
	out_3 << v3;
	out_4 << v4;

	ASSERT_EQ(out_3.str(), out_4.str());
}

TEST(rigid2d_lib, VectorScale)
{
	// *=
	rigid2d::Vector2D v1(1, 2);
	rigid2d::Vector2D v2;
	double scalar = 2.34;

	v2 = v1 * scalar; 
	v1 *= scalar;

	std::stringstream out_1;
	std::stringstream out_2;

	out_1 << v1;
	out_2 << v2;

	ASSERT_EQ(out_1.str(), out_2.str());
}

TEST(rigid2d_lib, VectorNormalization)
{
	rigid2d::Vector2D v1(1, 1); // unit vector with init
	rigid2d::Vector2D v2; // unit vector without init
	v2.x = 1;
	v2.y = 1;
	v2.normalize();

	ASSERT_FLOAT_EQ(v1.x, v2.x);

	ASSERT_FLOAT_EQ(v1.x, v2.x); // check that x1 matches x2
	ASSERT_FLOAT_EQ(v1.y, v2.y); // check that y1 matches y2
	ASSERT_FLOAT_EQ(v1.norm_x, v2.norm_x); // check that norm x1 matches norm x2
	ASSERT_FLOAT_EQ(v1.norm_y, v2.norm_y); // check that norm y1 matches norm y2

	ASSERT_FLOAT_EQ(v1.norm_x, sqrt(2)/2); // test norm based on my calc
	ASSERT_FLOAT_EQ(v1.norm_y, sqrt(2)/2); // test norm based on my calc
}

TEST(rigid2d_lib, VectorLength)
{
	rigid2d::Vector2D v(1,1);
	double length = rigid2d::length(v);
	ASSERT_FLOAT_EQ(length, 1.4142135);
}

TEST(rigid2d_lib, VectorDistance)
{
	rigid2d::Vector2D v1(1,1);
	rigid2d::Vector2D v2(2,2);
	double distance = rigid2d::distance(v1, v2);
	ASSERT_FLOAT_EQ(distance, 1.4142135);
}

TEST(rigid2d_lib, VectorAngle)
{
	rigid2d::Vector2D v(1,1);
	double angle = rigid2d::angle(v);
	ASSERT_FLOAT_EQ(angle, rigid2d::PI / 4);
}

TEST(rigid2d_lib, TransformIO)
{
	// make sure input and output are read correctly
	rigid2d::Transform2D T;

	std::string input = "90 1 1";
	std::stringstream in_s(input);

	std::string output = "dtheta (degrees): 90	dx: 1	dy: 1\n";
	std::stringstream out_s;

	in_s >> T;
	out_s << T;

	// compare raw input to in_s input
	ASSERT_EQ(in_s.str(), input);
	// compare raw output to out_s input
	ASSERT_EQ(out_s.str(), output);
}

TEST(rigid2d_lib, TransformVector)
{
	rigid2d::Transform2D Tac;
	std::string input = "90 -1 3";
	std::stringstream in_s(input);
	in_s >> Tac;

	rigid2d::Vector2D vc(3, 3);
	rigid2d::Vector2D va = Tac(vc);

	std::stringstream out_1;
	out_1 << va;
	std::stringstream out_2("[-4, 6]\n");

	ASSERT_EQ(out_1.str(), out_2.str());
}

TEST(rigid2d_lib, TransformInv)
{
	rigid2d::Transform2D Tac;
	std::string input = "90 -1 3";
	std::stringstream in_s(input);
	in_s >> Tac;

	rigid2d::Transform2D Tca = Tac.inv();

	std::stringstream out_1;
	out_1 << Tca;
	std::stringstream out_2("dtheta (degrees): -90	dx: -3	dy: -1\n");

	ASSERT_EQ(out_1.str(), out_2.str());
}

TEST(rigid2d_lib, TransformIntegrateTwist)
{
	// w_z = 0
	rigid2d::Transform2D Tac;
	std::string input1 = "90 -1 3";
	std::stringstream in_s1(input1);
	in_s1 >> Tac;

	rigid2d::Twist2D tw;
	std::string input2 = "0 1 1";
	std::stringstream in_s2(input2);
	in_s2 >> tw;

	// Integrate
	rigid2d::Transform2D Tac_twisted = Tac.integrateTwist(tw);

	std::string output1 = "dtheta (degrees): 90	dx: -2	dy: 4\n";
	std::stringstream out_s1;
	out_s1 << Tac_twisted;

	// compare raw output to out_s input
	ASSERT_EQ(out_s1.str(), output1);

	// w_z != 0

	rigid2d::Twist2D tw2;
	std::string input3 = "1 1 1";
	std::stringstream in_s3(input3);
	in_s3 >> tw2;

	// Integrate
	Tac_twisted = Tac.integrateTwist(tw2);

	std::string output2 = "dtheta (degrees): 147.296\tdx: -2.30117\tdy: 3.38177\n";
	std::stringstream out_s2;
	out_s2 << Tac_twisted;

	// compare raw output to out_s input
	ASSERT_EQ(out_s2.str(), output2);

	// Zero Twist
	rigid2d::Twist2D tw3;
	std::string input4 = "0 0 0";
	std::stringstream in_s4(input4);
	in_s4 >> tw3;

	// Integrate
	Tac_twisted = Tac.integrateTwist(tw3);

	std::string output3 = "dtheta (degrees): 90\tdx: -1\tdy: 3\n";
	std::stringstream out_s3;
	out_s3 << Tac_twisted;

	// compare raw output to out_s input
	ASSERT_EQ(out_s3.str(), output3);
}

TEST(rigid2d_lib, TransformDisplacement)
{
	rigid2d::Transform2D Tac;
	std::string input = "90 -1 3";
	std::stringstream in_s(input);
	in_s >> Tac;

	rigid2d::Transform2DS Tdisp = Tac.displacement();

	ASSERT_FLOAT_EQ(Tdisp.theta, rigid2d::PI / 2);
	ASSERT_FLOAT_EQ(Tdisp.x, -1);
	ASSERT_FLOAT_EQ(Tdisp.y, 3);
}

TEST(rigid2d_lib, TransformCompose)
{
	rigid2d::Transform2D Tab;
	std::string input1 = "90 1 1";
	std::stringstream in_s1(input1);
	in_s1 >> Tab;

	rigid2d::Transform2D Tbc;
	std::string input2 = "0 2 2";
	std::stringstream in_s2(input2);
	in_s2 >> Tbc;

	rigid2d::Transform2D Tac = Tab * Tbc;

	std::stringstream out_1;
	out_1 << Tac;
	std::stringstream out_2("dtheta (degrees): 90	dx: -1	dy: 3\n");

	ASSERT_EQ(out_1.str(), out_2.str());
}

TEST(rigid2d_lib, TwistIO)
{
	// make sure input and output are read correctly
	rigid2d::Twist2D tw;

	std::string input = "1 1 1";
	std::stringstream in_s(input);

	std::string output = "w_z (rad/s): 1	v_x (m/s): 1	v_y (m/s): 1\n";
	std::stringstream out_s;

	in_s >> tw;
	out_s << tw;

	// compare raw input to in_s input
	ASSERT_EQ(in_s.str(), input);
	// compare raw output to out_s input
	ASSERT_EQ(out_s.str(), output);
}

TEST(rigid2d_lib, TwistConvert)
{
	rigid2d::Transform2D Tac;
	std::string input = "90 -1 3";
	std::stringstream in_s(input);
	in_s >> Tac;

	rigid2d::Twist2D twc(1, 2, 2);
	rigid2d::Twist2D twa = twc.convert(Tac);

	std::stringstream out_1;
	out_1 << twa;
	std::stringstream out_2("w_z (rad/s): 1	v_x (m/s): 1	v_y (m/s): 3\n");

	ASSERT_EQ(out_1.str(), out_2.str());
}

TEST(diff_drive, TwistToWheels)
{
	rigid2d::Twist2D Vb(1, 0, 0);
	rigid2d::DiffDrive driver;

	// Rotation Test
	rigid2d::WheelVelocities vel = driver.twistToWheels(Vb);
	ASSERT_NEAR(vel.ul, -25, 1e-6);
  	ASSERT_NEAR(vel.ur, 25, 1e-6);

  	// Translation Test
  	Vb.reassign(0, 1, 0);
  	vel = driver.twistToWheels(Vb);
  	ASSERT_NEAR(vel.ul, 50, 1e-6);
  	ASSERT_NEAR(vel.ur, 50, 1e-6);

  	// Mixed Motion Test
  	Vb.reassign(1, 1, 0);
  	vel = driver.twistToWheels(Vb);
  	ASSERT_NEAR(vel.ul, 25, 1e-6);
  	ASSERT_NEAR(vel.ur, 75, 1e-6);

  	// Zero Test
  	Vb.reassign(0, 0, 0);
  	vel = driver.twistToWheels(Vb);
  	ASSERT_NEAR(vel.ul, 0, 1e-6);
  	ASSERT_NEAR(vel.ur, 0, 1e-6);
}

TEST(diff_drive, WheelsToTwist)
{
	rigid2d::WheelVelocities vel(10, 10);
	rigid2d::DiffDrive driver;

	// Translation Test
	rigid2d::Twist2D Vb = driver.wheelsToTwist(vel);
	ASSERT_NEAR(Vb.w_z, 0, 1e-6);
  	ASSERT_NEAR(Vb.v_x, 10, 1e-6);
  	ASSERT_NEAR(Vb.v_y, 0, 1e-6);

  	// Rotation Test
	vel.ul = -10;
	vel.ur = 10;
	Vb = driver.wheelsToTwist(vel);
	ASSERT_NEAR(Vb.w_z, 20, 1e-6);
  	ASSERT_NEAR(Vb.v_x, 0, 1e-6);
  	ASSERT_NEAR(Vb.v_y, 0, 1e-6);

	// Mixed Motion Test
	vel.ul = 0;
	vel.ur = 10;
	Vb = driver.wheelsToTwist(vel);
	ASSERT_NEAR(Vb.w_z, 10, 1e-6);
  	ASSERT_NEAR(Vb.v_x, 5, 1e-6);
  	ASSERT_NEAR(Vb.v_y, 0, 1e-6);

  	// Zero Test
  	vel.ul = 0;
	vel.ur = 0;
	Vb = driver.wheelsToTwist(vel);
	ASSERT_NEAR(Vb.w_z, 0, 1e-6);
  	ASSERT_NEAR(Vb.v_x, 0, 1e-6);
  	ASSERT_NEAR(Vb.v_y, 0, 1e-6);

}

TEST(diff_drive, UpdateOdometry)
{
	// Translation Test
	rigid2d::WheelVelocities vel;
	rigid2d::DiffDrive driver;
	// both wheels rotate 2pi
	double left_wheel = 2 * rigid2d::PI;
	double right_wheel = 2 * rigid2d::PI;
	vel = driver.updateOdometry(left_wheel, right_wheel);
	rigid2d::Pose2D pose = driver.get_pose();
	ASSERT_NEAR(vel.ul, 6.28319, 1e-3);
	ASSERT_NEAR(vel.ur, 6.28319, 1e-3);
	ASSERT_NEAR(pose.theta, 0, 1e-3);
	ASSERT_NEAR(pose.x, 6.28319, 1e-3);
	ASSERT_NEAR(pose.y, 0, 1e-3);

	// Rotation Test
	// reset driver
	rigid2d::Pose2D pose_reset;
	driver.reset(pose_reset);
	left_wheel = -rigid2d::PI/4;
	right_wheel = rigid2d::PI/4;
	vel = driver.updateOdometry(left_wheel, right_wheel);
	pose = driver.get_pose();
	ASSERT_NEAR(vel.ul, -0.785398, 1e-3);
	ASSERT_NEAR(vel.ur, 0.785398, 1e-3);
	ASSERT_NEAR(pose.theta, 1.5708, 1e-3);
	ASSERT_NEAR(pose.x, 0, 1e-3);
	ASSERT_NEAR(pose.y, 0, 1e-3);

	// Mixed Motion Test
	// reset driver
	driver.reset(pose_reset);
	left_wheel = 0;
	right_wheel = rigid2d::PI / 4;
	vel = driver.updateOdometry(left_wheel, right_wheel);
	pose = driver.get_pose();
	ASSERT_NEAR(vel.ul, 0, 1e-3);
	ASSERT_NEAR(vel.ur, 0.785398, 1e-3);
	ASSERT_NEAR(pose.theta, 0.785398, 1e-3);
	ASSERT_NEAR(pose.x, 0.353553, 1e-3);
	ASSERT_NEAR(pose.y, 0.146447, 1e-3);

	// Zero Vel Test
	// reset driver
	driver.reset(pose_reset);
	left_wheel = 0;
	right_wheel = 0;
	vel = driver.updateOdometry(left_wheel, right_wheel);
	pose = driver.get_pose();
	ASSERT_NEAR(vel.ul, 0, 1e-3);
	ASSERT_NEAR(vel.ur, 0, 1e-3);
	ASSERT_NEAR(pose.theta, 0, 1e-3);
	ASSERT_NEAR(pose.x, 0, 1e-3);
	ASSERT_NEAR(pose.y, 0, 1e-3);
}

TEST(diff_drive, Feedforward)
{
	rigid2d::DiffDrive driver;
	rigid2d::Twist2D Vb(0, 1, 0);

	// Translation Test
	driver.feedforward(Vb);
	rigid2d::Pose2D pose = driver.get_pose();
	ASSERT_NEAR(pose.theta, 0, 1e-3);
	ASSERT_NEAR(pose.x, 1, 1e-3);
	ASSERT_NEAR(pose.y, 0, 1e-3);

	// Rotation Test
	// reset driver
	rigid2d::Pose2D pose_reset;
	driver.reset(pose_reset);
	Vb.reassign(0, 1, 0);
	driver.feedforward(Vb);
	pose = driver.get_pose();
	ASSERT_NEAR(pose.theta, 0, 1e-3);
	ASSERT_NEAR(pose.x, 1, 1e-3);
	ASSERT_NEAR(pose.y, 0, 1e-3);

	// Mixed Motion Test
	// reset driver
	driver.reset(pose_reset);
	Vb.reassign(rigid2d::PI / 4, 1, 0);
	driver.feedforward(Vb);
	pose = driver.get_pose();
	ASSERT_NEAR(pose.theta, 0.785398, 1e-3);
	ASSERT_NEAR(pose.x, 0.900316, 1e-3);
	ASSERT_NEAR(pose.y, 0.372923, 1e-3);

	// Zero Test
	// reset driver
	driver.reset(pose_reset);
	Vb.reassign(0, 0, 0);
	driver.feedforward(Vb);
	pose = driver.get_pose();
	ASSERT_NEAR(pose.theta, 0, 1e-3);
	ASSERT_NEAR(pose.x, 0, 1e-3);
	ASSERT_NEAR(pose.y, 0, 1e-3);
}

int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "test_node_rigid2d");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}