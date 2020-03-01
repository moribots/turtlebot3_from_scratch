#include <gtest/gtest.h>
#include "nuslam/landmarks.hpp"

namespace nuslam
{

TEST(landmarks, CircleFitting)
{
	double test_threshold = 1e-4;

	double cluster_threshold = 1e6; // large threshold to fit any circle

	// TEST 1 INPUTS: {(1, 7), (2, 6), (5, 8), (7, 7), (9, 5), (3, 7)}
	//		  OUTPUTS: center (4.615482, 2.807354) radius 4.8725

	// Construct Cluster
	Landmark cluster_1(cluster_threshold);
	// Append points to cluster
	rigid2d::Vector2D pose_1_a(1.0, 7.0);
	Point point_1_a(pose_1_a);
	cluster_1.evaluate_point(point_1_a);
	rigid2d::Vector2D pose_1_b(2.0, 6.0);
	Point point_1_b(pose_1_b);
	cluster_1.evaluate_point(point_1_b);
	rigid2d::Vector2D pose_1_c(5.0, 8.0);
	Point point_1_c(pose_1_c);
	cluster_1.evaluate_point(point_1_c);
	rigid2d::Vector2D pose_1_d(7.0, 7.0);
	Point point_1_d(pose_1_d);
	cluster_1.evaluate_point(point_1_d);
	rigid2d::Vector2D pose_1_e(9.0, 5.0);
	Point point_1_e(pose_1_e);
	cluster_1.evaluate_point(point_1_e);
	rigid2d::Vector2D pose_1_f(3.0, 7.0);
	Point point_1_f(pose_1_f);
	cluster_1.evaluate_point(point_1_f);

	std::cout << "CLUSTER 1 SIZE: " << cluster_1.points.size() << std::endl;

	// Now fit circle
	cluster_1.fit_circle();

	// Now evaluate centre and radius
	ASSERT_NEAR(cluster_1.return_coords().pose.x, 4.615482, test_threshold);
	ASSERT_NEAR(cluster_1.return_coords().pose.y, 2.807354, test_threshold);
	ASSERT_NEAR(cluster_1.return_radius(), 4.8725, test_threshold);


	// TEST 2 INPUTS: {(-1, 0), (-0.3, -0.06), (0.3, 0.1), (1, 0)}
	//		  OUTPUTS: center (0.4908357, -22.15212) radius 22.17979

	// Construct Cluster
	Landmark cluster_2(cluster_threshold);
	// Append points to cluster
	rigid2d::Vector2D pose_2_a(-1.0, 0.0);
	Point point_2_a(pose_2_a);
	cluster_2.evaluate_point(point_2_a);
	rigid2d::Vector2D pose_2_b(-0.3, -0.06);
	Point point_2_b(pose_2_b);
	cluster_2.evaluate_point(point_2_b);
	rigid2d::Vector2D pose_2_c(0.3, 0.1);
	Point point_2_c(pose_2_c);
	cluster_2.evaluate_point(point_2_c);
	rigid2d::Vector2D pose_2_d(1.0, 0.0);
	Point point_2_d(pose_2_d);
	cluster_2.evaluate_point(point_2_d);

	std::cout << "CLUSTER 2 SIZE: " << cluster_2.points.size() << std::endl;

	// Now fit circle
	cluster_2.fit_circle();

	// Now evaluate centre and radius
	ASSERT_NEAR(cluster_2.return_coords().pose.x, 0.4908357, test_threshold);
	ASSERT_NEAR(cluster_2.return_coords().pose.y, -22.15212, test_threshold);
	ASSERT_NEAR(cluster_2.return_radius(), 22.17979, test_threshold);

}

}

int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "test_node_rigid2d");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}