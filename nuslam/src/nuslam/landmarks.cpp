#include "nuslam/landmarks.hpp"
#include <Eigen/Dense>

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
	}

	Point::Point(const Vector2D & pose_)
	{
		pose = pose_;

		range_bear = cartesianToPolar(pose);
	}

	Point::Point(const RangeBear & range_bear_)
	{
		range_bear = range_bear_;

		pose = polarToCartesian(range_bear);
	}

	Point::Point(const RangeBear & range_bear_, const Vector2D & pose_)
	{
		range_bear = range_bear_;
		pose = pose_;
	}

	// Landmark
	Landmark::Landmark()
	{
		radius = 0;
		coords = Point();
		// empty vector
		std::vector<Point> p;
		points = p;

		threshold = 0.5;
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

			double abs_x = pow(point_.pose.x - points.back().pose.x, 2);
			double abs_y = pow(point_.pose.y - points.back().pose.y, 2);
			double abs_dist = sqrt(abs_x + abs_y);

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
		// Step 1: Compute x,y coordinates of the centroid of n data points
		// (xc1, yc1), ...(xcn, yxcn) --> (mean of x,y coords of points)
		float mean_x = std::accumulate(std::begin(points), std::end(points),
									   0.0) (const Point & p) {return p.pose.x;} / std::size(points);
		float mean_y = std::accumulate(std::begin(points), std::end(points),
									   0.0 (const Point & p) {return p.pose.y;}) / std::size(points);
		// std::vector<double> radii;
		//std::vector<double> x_pts;
		//std::vector<double> y_pts;

	    // Step 2: Shift the coordinates so the centroid is at the origin
	    std::vector<double> z;
	    for (auto iter = points.begin(); iter != points.end(); iter++)
		{
			iter->pose.x -= mean_x;
			iter->pose.y -= mean_y;

			// Step 3: compute zi = xi^2 + yi^2
			z.push_back((pow(iter->pose.x, 2) + pow(iter->pose.y, 2)));
		}

		// Step 4: Compute mean of z
	    double mean_z = std::accumulate(std::begin(z), std::end(z), 0.0) / std::size(z);

	    // Step 5: form data matrix from n data points
	    // Create Matrix typedef
	    // Dynamic says we don't know matrix size for rows, but columns = 4
	    typedef Eigen::Matrix<double, Eigen::Dynamic, 4> ClusterMat;
	    auto n = std::size(z);
	    // reserving storage
	    ClusterMat Z(n, 4);
	    // or use r as counter in the loop
	    Eigen::Index r = 0;
	    // range-based for-loop via iterator
	    // p is automatically of type of whatever's inside the vector points
		for (auto & p : points)
		{
		    Z.row(r++) << z.at(r), p.pose.x, p.pose.y, 1; // fill matrix one row per iteration, then increment row
		}

		// Step 6: Data Matrix M = (1/n) Z.T Z
		auto M = Z.transpose() * Z / n;

		// Step 7: Constraint Matrix H for 'Hyperaccurate algebraic fit'
		ClusterMat H(4, 4);
		H.row(0) = 8 * mean_z, 0, 0, 2;
		H.row(1) = 0, 1, 0, 0;
		H.row(2) = 0, 0, 1, 0;
		H.row(3) = 2, 0, 0, 0;

		// Step 8: H Inv
		ClusterMat H_inv(4, 4);
		H_inv.row(0) = 0, 0, 0, 0.5;
		H_inv.row(1) = 0, 1, 0, 0;
		H_inv.row(2) = 0, 0, 1, 0;
		H_inv.row(3) = 0.5, 0, 0, - 2 * mean_z;

		// TODO Step 9: Singular Value Decomposition of Z
		// Using jacobiSvd (accurate, fast for small matrices)
		// ThinV returns a diagonal vector suitable for constructing
		// a square sigma matrix
		JacobiSVD<ClusterMat> Z_svd(Z, ComputeThinV)

		// If the smallest singular value is <10^-12 then A is the 4th column of V
		// Note that singular value vector is returned in decreasing order
		if (Z_svd.singularValues().back() < pow(10.0, -12))
		{
			// Step 10: A is the 4th column of the V matrix
			auto A = Z_svd.matrixV().column(3);
		} else {
			// Step 11:
			// Construct sigma matrix (diagonal with singular values)
			ClusterMat S(n, 4);
			S << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
			S(0, 0) = Z_svd.singularValues().at(0);
			S(1, 1) = Z_svd.singularValues().at(1);
			S(2, 2) = Z_svd.singularValues().at(2);
			S(3, 3) = Z_svd.singularValues().at(3);

			// Y Matrix = V * Sigma * V^T
			auto Y = Z_svd.matrixV() * S * Z_svd.matrixV().transpose();

			// Q Matrix = Y * H_inv * Y
			auto Q = Y * H_inv * Y;

			Eigen::SelfAdjointEigenSolver<ClusterMat> es;
			// Find Eigenvalues and Eigenvectors of Q
			auto ev = es.compute(Q);
			// A_star is the eigenvector corresponding to the smallest positive
			// eigenvalue of Q
			// Store min positive eigenvalue
			double min_pos = 0.0;
			auto min_pos_counter = 0;
			for (auto iter = ev.eigenvalues().begin(); iter != ev.eigenvalues().end(); iter++)
			{
				if (*iter > 0)
				{
					min_pos = std::min(*iter, min_pos);
					min_pos_counter = iter - ev.eigenvalues().begin();
				}

			}
			// The eigenvector corresponding to the minimum positive eigenvalue
			A_eigvect = ev.eigenvectors().col(min_pos_counter);

			// Solve for A
			auto A = A_star * Y.inverse();
		}

		// Step 12: eqn of circle is (x - a)^2 + (y - b)^2 = R^2
		auto a = -A(2) / (2 * A(1));
		auto b = -A(3) / (2 * A(1));
		auto R = sqrt((pow(A(2), 2) + pow(A(3), 2) - (4 * A(1) * A(4))) / (4 * pow(A(4), 2)));

		// Step 13: We shifted our coordinate system, so actual centroid is at
		// a + mean_x, b + mean_y
		// Store Cluster Parameters (coords(x,y) and radius)
		coords.x = a + mean_x;
		coords.y = a + mean_y;
		radius = R;

		// Step 14: Calculate Root-Mean-Squared-Error of the fit
		auto sum = 0;
		for (auto iter = points.begin(); iter != points.end(); iter++)
		{
			sum += pow((pow(iter->pose.x - a, 2) + pow(iter->pose.y - b, 2) - pow(R, 2) ), 2);
		}

		rms_err = sqrt(sum / n);

		return rms_err;
	}


	void Landmark::detect_circle()
	{

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
		double x = range_bear.range * cos(range_bear.bearing);
		double y = range_bear.range * sin(range_bear.bearing);

		Vector2D pose(x, y);

		return pose;

	}

}