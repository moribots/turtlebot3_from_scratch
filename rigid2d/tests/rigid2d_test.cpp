#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"

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

	std::string output = "dtheta (degrees): 90	dx: -2	dy: 4\n";
	std::stringstream out_s;
	out_s << Tac_twisted;

	// compare raw output to out_s input
	ASSERT_EQ(out_s.str(), output);
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

int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "test_node_rigid2d");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}