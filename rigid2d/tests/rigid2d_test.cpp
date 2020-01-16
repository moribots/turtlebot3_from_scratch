#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"

TEST(rigid2d_lib, Test_Tester)
{
    ASSERT_EQ(3, 3); // change this to 2 to force a fail
    // There are many other ASSERT (fatal) and EXPECT (non-fatal, tests continue)
    // macros in google test
    // When first setting up tests, I like to write one that fails,
    // to ensure it is being run at the appropriate time
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

	ASSERT_FLOAT_EQ(v1.norm_x, 1.41421356237309/2.0); // test norm based on my calc
	ASSERT_FLOAT_EQ(v1.norm_y, 1.41421356237309/2.0); // test norm based on my calc
}

int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "test_node_rigid2d");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}