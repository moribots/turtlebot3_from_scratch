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

		threshold = 0.1;
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

			double abs_x = pow(point_.pose.x - points.end()->pose.x, 2);
			double abs_y = pow(point_.pose.y - points.end()->pose.y, 2);
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

	// Helper Functions
	RangeBear cartesianToPolar(const Vector2D & pose)
	{
		double range = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
		double bear = atan2(pose.y, pose.x);
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