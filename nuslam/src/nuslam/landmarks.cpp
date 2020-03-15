#include "nuslam/landmarks.hpp"

namespace nuslam
{
	using rigid2d::Vector2D;

	// RangeBear
	RangeBear::RangeBear()
	{
		range = 0;
		bearing = 0;
	}

	RangeBear::RangeBear(const double & range_, const double & bearing_)
	{
		range = range_;
		bearing = bearing_;
	}

	// Point
	Point::Point()
	{
		range_bear = RangeBear();
		pose = Vector2D();
		init = false;

		index = 0;
		seen_count = 0;
	}

	Point::Point(const Vector2D & pose_)
	{
		pose = pose_;

		range_bear = cartesianToPolar(pose);

		init = false;

		index = 0;
		seen_count = 0;
	}

	Point::Point(const RangeBear & range_bear_)
	{
		range_bear = range_bear_;

		pose = polarToCartesian(range_bear);

		init = false;

		index = 0;
		seen_count = 0;
	}

	// Landmark
	Landmark::Landmark()
	{
		radius = 0;
		coords = Point();
		// empty vector
		std::vector<Point> p;
		points = p;

		threshold = 0.05; // 5 cm
	}

	Landmark::Landmark(const double & threshold_)
	{
		radius = 0;
		coords = Point();
		// empty vector
		std::vector<Point> p;
		points = p;

		threshold = threshold_;
	}

	Landmark::Landmark(const double & radius_, const Point & coords_, const std::vector<Point> points_, const double & threshold_)
	{
		radius = radius_;
		coords = coords_;
		points = points_;
		threshold = threshold_;
	}

	bool Landmark::evaluate_point(const Point & point_)
	{
		bool added = false;

		if (points.empty())
		{
			points.push_back(point_);
			added = true;
		} else {

			// double abs_x = pow(point_.pose.x - points.back().pose.x, 2);
			// double abs_y = pow(point_.pose.y - points.back().pose.y, 2);
			// double abs_dist = sqrt(abs_x + abs_y);

			// We know points are at angle incrememnts, so we only need to
			// compare range
			double abs_dist = fabs(point_.range_bear.range - points.back().range_bear.range);

			if (abs_dist <= threshold)
			{
				points.push_back(point_);
				added = true;
			}

		}

		return added;

	}

	std::vector<Point> Landmark::return_points()
	{
		return points;
	}

	Point Landmark::return_coords()
	{
		return coords;
	}

	double Landmark::return_radius()
	{
		return radius;
	}

	double Landmark::fit_circle()
	{
		auto n = std::size(points);
		// Step 1: Compute x,y coordinates of the centroid of n data points
		// (xc1, yc1), ...(xcn, yxcn) --> (mean of x,y coords of points)
		// TODO: FIGURE OUT HOW TO ACCESS Point::pose::x,y
		std::vector<rigid2d::Vector2D> poses;
		poses.reserve(points.size());
		// Syntax below to insert Point::pose elments of points into poses vector
		std::transform
		(
		    points.begin(), points.end(), std::back_inserter(poses),
		    [](const Point& p) { return p.pose; }
		);

		// Syntax below to compute mean
		float mean_x  = std::accumulate(poses.begin(), poses.end(), 0.0, \
                            std::bind(std::plus<double>(), std::placeholders::_1,
                            std::bind(&rigid2d::Vector2D::x, std::placeholders::_2))) / static_cast<double>(n);
		float mean_y  = std::accumulate(poses.begin(), poses.end(), 0.0, \
                            std::bind(std::plus<double>(), std::placeholders::_1,
                            std::bind(&rigid2d::Vector2D::y, std::placeholders::_2))) / static_cast<double>(n);
		// std::cout << "X MEAN: " << mean_x << std::endl;

	    // Step 2: Shift the coordinates so the centroid is at the origin
	    std::vector<double> z;
	    z.reserve(points.size());
	    for (auto iter = points.begin(); iter != points.end(); iter++)
		{
			iter->pose.x -= mean_x;
			iter->pose.y -= mean_y;

			// Step 3: compute zi = xi^2 + yi^2
			double zi = pow(iter->pose.x, 2) + pow(iter->pose.y, 2);
			z.push_back(zi);
		}

		// Step 4: Compute mean of z
	    double mean_z = std::accumulate(z.begin(), z.end(), 0.0) / static_cast<double>(n);
	    // std::cout << "z mean: \n" << mean_z << std::endl;

	    // Step 5: form data matrix from n data points
	    // Create Matrix typedef
	    // Dynamic says we don't know matrix size for rows, but columns = 4
	    typedef Eigen::Matrix<double, Eigen::Dynamic, 4> ClusterMat;
	    // reserving storage
	    ClusterMat Z = ClusterMat::Zero(n, 4);
	    // or use r as counter in the loop
	    Eigen::Index r = 0;
	    // range-based for-loop via iterator
	    // p is automatically of type of whatever's inside the vector points
		for (auto & p : points)
		{
		    Z.row(r) << z.at(r), p.pose.x, p.pose.y, 1.0; // fill matrix one row per iteration, then increment row
		    r++;
		}
		// std::cout << "Z: \n" << Z << std::endl;

		// Step 6: Data Matrix M = (1/n) Z.T Z
		// auto M = Z.transpose() * Z / static_cast<double>(n);

		// Step 7: Constraint Matrix H for 'Hyperaccurate algebraic fit'
		ClusterMat H = ClusterMat::Zero(4, 4);
		H.row(0) << 8.0 * mean_z, 0.0, 0.0, 2.0;
		H.row(1) << 0.0, 1.0, 0.0, 0.0;
		H.row(2) << 0.0, 0.0, 1.0, 0.0;
		H.row(3) << 2.0, 0.0, 0.0, 0.0;

		// Step 8: H Inv
		ClusterMat H_inv = ClusterMat::Zero(4, 4);
		H_inv.row(0) << 0.0, 0.0, 0.0, 0.5;
		H_inv.row(1) << 0.0, 1.0, 0.0, 0.0;
		H_inv.row(2) << 0.0, 0.0, 1.0, 0.0;
		H_inv.row(3) << 0.5, 0, 0, - 2.0 * mean_z;

		// Step 9: Singular Value Decomposition of Z
		// ThinV returns a diagonal vector suitable for constructing
		// a square sigma matrix

		// Using BDCSVD faster for large matrices
		Eigen::BDCSVD<ClusterMat> Z_svd(Z,  Eigen::DecompositionOptions::ComputeFullV);

		// Using jacobiSvd (accurate, fast for small matrices)
		// Eigen::JacobiSVD<ClusterMat> Z_svd(Z,  Eigen::DecompositionOptions::ComputeFullV);

		if (n < 10)
		{
			// Using jacobiSvd (accurate, fast for small matrices)
			Eigen::JacobiSVD<ClusterMat> Z_svd(Z,  Eigen::DecompositionOptions::ComputeFullV);
		}
		// If the smallest singular value is <10^-12 then A is the 4th column of V
		// Note that singular value vector is returned in decreasing order
		// Initialize A Vector of doubles
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 1);
		// ROS_INFO("COMPARING Z_svd");
		// ROS_INFO("-------------------------------------");
		// std::cout << "V DIAG: \n" << Z_svd.singularValues() << std::endl;
		// std::cout << "n SIZE: " << n << std::endl; 
		if (Z_svd.singularValues()(3) < 1e-12)
		{
			// ROS_INFO("LESS THRESH");
			// Step 10: A is the 4th column of the V matrix
			A = Z_svd.matrixV().col(3);
		} else {
			// ROS_INFO("GTR THRESH");
			// std::cout << "V DIAG: \n" << Z_svd.singularValues() << std::endl;
			// Step 11:
			// Construct sigma matrix (diagonal with singular values)
			Eigen::MatrixXd S = Z_svd.singularValues().asDiagonal();
			// std::cout << "S: \n" << S << std::endl;

			// Y Matrix = V * Sigma * V^T
			// std::cout << "V SIZE: " << Z_svd.matrixV().size() << std::endl;
			// std::cout << "SIGMA SIZE: " << S.size() << std::endl;
			Eigen::MatrixXd Y = Z_svd.matrixV() * S * Z_svd.matrixV().transpose();
			// std::cout << "Y: \n" << Y << std::endl;

			// Q Matrix = Y * H_inv * Y
			Eigen::MatrixXd Q = Y * H_inv * Y;
			// std::cout << "Q: \n" << Q << std::endl;

			// Find Eigenvalues and Eigenvectors of Q via constructor
			Eigen::SelfAdjointEigenSolver<ClusterMat> es(Q);
			// A_eigvect is the eigenvector corresponding to the smallest positive
			// eigenvalue of Q
			// Store min positive eigenvalue
			int min_pos_counter = 0;
			bool min_found = false;

			// std::cout << "EIGEINVALUES: \n" << es.eigenvalues() << std::endl;

			while (!min_found)
			{
				if (min_pos_counter > 3)
				{
					std::cout << "NO MINIMUM EIGENVALUE FOUND GREATER THAN ZERO." << std::endl;

				} else if (es.eigenvalues()(min_pos_counter) > 0)
				{
					min_found = true;
				} else {
					min_pos_counter++;
				}
			}
			
			// The eigenvector corresponding to the minimum positive eigenvalue
			Eigen::MatrixXd A_eigvect = es.eigenvectors().col(min_pos_counter);
			// std::cout << "A_eigvect: \n" << A_eigvect << std::endl;

			// Solve for A = Y_inv * A_star
			A = Y.completeOrthogonalDecomposition().solve(A_eigvect);
			// std::cout << "A: \n" << A << std::endl;
		}

		// Step 12: eqn of circle is (x - a)^2 + (y - b)^2 = R^2
		auto a = -A(1) / (2.0 * A(0));
		auto b = -A(2) / (2.0 * A(0));
		auto R = sqrt((pow(A(1), 2) + pow(A(2), 2) - (4.0 * A(0) * A(3))) / (4.0 * pow(A(0), 2)));
		// std::cout << "A MATRIX: \n" << A << std::endl;

		// std::cout << "a: \n" << a << std::endl;
		// std::cout << "b: \n" << b << std::endl;
		// std::cout << "r: \n" << R << std::endl;

		// Step 13: We shifted our coordinate system, so actual centroid is at
		// a + mean_x, b + mean_y
		// Store Cluster Parameters (coords(x,y) and radius)
		coords.pose.x = a + mean_x;
		coords.pose.y = b + mean_y;
		radius = R;

		// Step 14: Calculate Root-Mean-Squared-Error of the fit
		auto sum = 0;
		for (auto iter = points.begin(); iter != points.end(); iter++)
		{
			sum += pow((pow(iter->pose.x - a, 2) + pow(iter->pose.y - b, 2) - pow(R, 2) ), 2);
		}

		auto rms_err = sqrt(sum / static_cast<double>(n));

		return rms_err;
	}


	bool Landmark::classify_circle()
	{
		bool is_circle = false;
		// Store endpoints of cluster arc
		Vector2D p_first = points.at(0).pose;
		Vector2D p_last = points.back().pose;

		std::vector<double> angles;

		// Step 1: Store the angle ^P_FIRST|P_X|P_LAST in a vector of angles
		// Below syntax to iterate from 1th element to penultimate element
		for (auto iter = std::next(points.begin()); iter != std::prev(points.end()); iter++)
		{
			// Using Law of Cosines to find angle
			// First, find the three lengths:
			// P_FIRST <--> P_LAST
			double p_first_last = sqrt(pow(p_first.x - p_last.x, 2) + pow(p_first.y - p_last.y, 2));

			// std::cout << "P FIRST LAST: " << p_first_last << std::endl;

			// P_FIRST <--> P_X
			double p_first_x = sqrt(pow(p_first.x - iter->pose.x, 2) + pow(p_first.y - iter->pose.y, 2));

			// std::cout << "P FIRST X: " << p_first_x << std::endl;

			// P_X <--> P_LAST
			double p_x_last = sqrt(pow(iter->pose.x - p_last.x, 2) + pow(iter->pose.y - p_last.y, 2));

			// std::cout << "P X LAST: " << p_x_last << std::endl;

			// Next, find angle and append to vector
			double angle = acos((pow(p_first_last, 2) - pow(p_first_x, 2) - pow(p_x_last, 2)) / (- 2.0 * p_first_x * p_x_last));
			// std::cout << "ANGLE: " << angle << std::endl;
			angles.push_back(angle);

		}

		// Step 2: compute the mean and standard deviation of all the angles
		double mean_angle = std::accumulate(angles.begin(), angles.end(), 0.0) / static_cast<double>(angles.size());
		double total = 0.0;
		// for (auto iter = angles.begin(); iter != angles.end(); iter++)
		// {
		// 	total += pow(*iter - mean_angle, 2);
		// }
		std::for_each (angles.begin(), angles.end(), [&](const double ang)
		{
		    total += pow(ang - mean_angle, 2);
		});

		double std_dev = sqrt(total / static_cast<double>(angles.size()));

		// Step 3: If std_dev is below 0.15 radians, and the mean_angle is between 90 and 135 degrees, we have a circle
		// Convert Mean Angle to Radians
		mean_angle *= 180.0 / rigid2d::PI;

		// std::cout << "STD DEV: " << std_dev << "\t MEAN: " << mean_angle << std::endl;

		if (std_dev < 0.5 && mean_angle >= 10.0 && mean_angle <= 170.0)
		{
			is_circle = true;
		}

		return is_circle;
	}

	// Helper Functions
	RangeBear cartesianToPolar(const Vector2D & pose)
	{
		double range = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
		double bear = rigid2d::normalize_angle(atan2(pose.y, pose.x));
		RangeBear rb(range, bear);
		return rb;
	}

	Vector2D polarToCartesian(const RangeBear & range_bear)
	{
		Vector2D pose;

		pose.x = range_bear.range * cos(range_bear.bearing);
		pose.y = range_bear.range * sin(range_bear.bearing);

		return pose;

	}

}