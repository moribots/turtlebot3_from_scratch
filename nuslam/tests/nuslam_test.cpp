#include <gtest/gtest.h>
#include "nuslam/landmarks.hpp"
#include "nuslam/ekf.hpp"
#include "rigid2d/diff_drive.hpp"

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
	// a
	rigid2d::Vector2D pose_1_a(1.0, 7.0);
	Point point_1_a(pose_1_a);
	cluster_1.evaluate_point(point_1_a);
	// b
	rigid2d::Vector2D pose_1_b(2.0, 6.0);
	Point point_1_b(pose_1_b);
	cluster_1.evaluate_point(point_1_b);
	// c
	rigid2d::Vector2D pose_1_c(5.0, 8.0);
	Point point_1_c(pose_1_c);
	cluster_1.evaluate_point(point_1_c);
	// d
	rigid2d::Vector2D pose_1_d(7.0, 7.0);
	Point point_1_d(pose_1_d);
	cluster_1.evaluate_point(point_1_d);
	// e
	rigid2d::Vector2D pose_1_e(9.0, 5.0);
	Point point_1_e(pose_1_e);
	cluster_1.evaluate_point(point_1_e);
	// f
	rigid2d::Vector2D pose_1_f(3.0, 7.0);
	Point point_1_f(pose_1_f);
	cluster_1.evaluate_point(point_1_f);

	// std::cout << "CLUSTER 1 SIZE: " << cluster_1.points.size() << std::endl;

	// Now fit circle
	cluster_1.fit_circle();

	// Now evaluate centre and radius
	ASSERT_NEAR(cluster_1.return_coords().pose.x, 4.615482, test_threshold);
	ASSERT_NEAR(cluster_1.return_coords().pose.y, 2.807354, test_threshold);
	ASSERT_NEAR(cluster_1.return_radius(), 4.8275, test_threshold);


	// TEST 2 INPUTS: {(-1, 0), (-0.3, -0.06), (0.3, 0.1), (1, 0)}
	//		  OUTPUTS: center (0.4908357, -22.15212) radius 22.17979

	// Construct Cluster
	Landmark cluster_2(cluster_threshold);
	// Append points to cluster
	// a
	rigid2d::Vector2D pose_2_a(-1.0, 0.0);
	Point point_2_a(pose_2_a);
	cluster_2.evaluate_point(point_2_a);
	// b
	rigid2d::Vector2D pose_2_b(-0.3, -0.06);
	Point point_2_b(pose_2_b);
	cluster_2.evaluate_point(point_2_b);
	// c
	rigid2d::Vector2D pose_2_c(0.3, 0.1);
	Point point_2_c(pose_2_c);
	cluster_2.evaluate_point(point_2_c);
	// d
	rigid2d::Vector2D pose_2_d(1.0, 0.0);
	Point point_2_d(pose_2_d);
	cluster_2.evaluate_point(point_2_d);

	// std::cout << "CLUSTER 2 SIZE: " << cluster_2.points.size() << std::endl;

	// Now fit circle
	cluster_2.fit_circle();

	// Now evaluate centre and radius
	ASSERT_NEAR(cluster_2.return_coords().pose.x, 0.4908357, test_threshold);
	ASSERT_NEAR(cluster_2.return_coords().pose.y, -22.15212, test_threshold);
	ASSERT_NEAR(cluster_2.return_radius(), 22.17979, test_threshold);

}

TEST(slam, Prediction)
{
	rigid2d::DiffDrive driver;
	// Set Driver Wheel Base and Radius
	double wbase_ = 0.16;
	double wrad_ = 0.033;
	double max_range_ = 3.5;
	driver.set_static(wbase_, wrad_);

	// Initialize EKF class with robot state, vector of 12 landmarks at 0,0,0, and noise
	double x_noise = 1e-10;
	double y_noise = 1e-10;
	double theta_noise = 1e-10;
	double range_noise = 1e-10;
	double bearing_noise = 1e-10;
	std::vector<nuslam::Point> map_state_(12, nuslam::Point());
	nuslam::Pose2D xyt_noise_var = nuslam::Pose2D(x_noise, y_noise, theta_noise);
	nuslam::RangeBear rb_noise_var_ = nuslam::RangeBear(range_noise, bearing_noise);
	nuslam::EKF ekf1 = nuslam::EKF(driver.get_pose(), map_state_, xyt_noise_var, rb_noise_var_, max_range_);
	rigid2d::Pose2D xyt_noise_mean;

	// Translation Test
	rigid2d::Twist2D Vb(0, 1, 0);
	ekf1.predict(Vb, xyt_noise_mean);
	rigid2d::Pose2D pose = ekf1.return_pose();
	ASSERT_NEAR(pose.theta, 0, 1e-3);
	ASSERT_NEAR(pose.x, 1, 1e-3);
	ASSERT_NEAR(pose.y, 0, 1e-3);

	// // Rotation Test
	// // reset driver
	// rigid2d::Pose2D pose_reset;
	// driver.reset(pose_reset);
	// Vb.reassign(0, 1, 0);
	// driver.feedforward(Vb);
	// pose = driver.get_pose();
	// ASSERT_NEAR(pose.theta, 0, 1e-3);
	// ASSERT_NEAR(pose.x, 1, 1e-3);
	// ASSERT_NEAR(pose.y, 0, 1e-3);

	// // Mixed Motion Test
	// // reset driver
	// driver.reset(pose_reset);
	// Vb.reassign(rigid2d::PI / 4, 1, 0);
	// driver.feedforward(Vb);
	// pose = driver.get_pose();
	// ASSERT_NEAR(pose.theta, 0.785398, 1e-3);
	// ASSERT_NEAR(pose.x, 0.900316, 1e-3);
	// ASSERT_NEAR(pose.y, 0.372923, 1e-3);

	// // Zero Test
	// // reset driver
	// driver.reset(pose_reset);
	// Vb.reassign(0, 0, 0);
	// driver.feedforward(Vb);
	// pose = driver.get_pose();
	// ASSERT_NEAR(pose.theta, 0, 1e-3);
	// ASSERT_NEAR(pose.x, 0, 1e-3);
	// ASSERT_NEAR(pose.y, 0, 1e-3);

}

}

int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "test_node_rigid2d");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}