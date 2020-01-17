#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"

TEST(rigid2d_lib, VectorIO)
{
	// make sure input and output are read correctly
	rigid2d::Vector2D v;

	std::string input = "1 1";
	std::stringstream in_s(input);

	std::string output = "[1, 1]\n";
	std::stringstream out_s(output);

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
	std::stringstream out_s(output);

	in_s >> T;
	out_s << T;

	// compare raw input to in_s input
	ASSERT_EQ(in_s.str(), input);
	// compare raw output to out_s input
	ASSERT_EQ(out_s.str(), output);
}

TEST(rigid2d_lib, TransformVector)
{
}

TEST(rigid2d_lib, TransformInv)
{
}

TEST(rigid2d_lib, TransformIntegrateTwist)
{
}

TEST(rigid2d_lib, TransformDisplacement)
{
}

TEST(rigid2d_lib, TransformCompose)
{
}

TEST(rigid2d_lib, TwistIO)
{
	// make sure input and output are read correctly
	rigid2d::Twist2D tw;

	std::string input = "1 1 1";
	std::stringstream in_s(input);

	std::string output = "w_z (rad/s): 1	v_x (m/s): 1	v_y (m/s): 1\n";
	std::stringstream out_s(output);

	in_s >> tw;
	out_s << tw;

	// compare raw input to in_s input
	ASSERT_EQ(in_s.str(), input);
	// compare raw output to out_s input
	ASSERT_EQ(out_s.str(), output);
}

TEST(rigid2d_lib, TwistConvert)
{
}

int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "test_node_rigid2d");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}