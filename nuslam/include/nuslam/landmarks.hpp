#ifndef LANDMARKS_INCLUDE_GUARD_HPP
#define LANDMARKS_INCLUDE_GUARD_HPP
/// \file
/// \brief Library Landmarks landmark detection and classification.
#include "rigid2d/rigid2d.hpp"

namespace nuslam
{
    // Used to store landmark and laserpoint poses
    using rigid2d::Vector2D;

    struct RangeBear
    // Struct to store range and bearing in m and rad/s respectively
    {
        double range, bearing;

        // \brief constructor for RangeBear with no inputs, initializes to zero
        RangeBear();

        // \brief constructor for RangeBear with inputs
        RangeBear(const double & range_, const double & bearing_);
    };

    struct Point
    // Laser Scan Point
    {
        RangeBear range_bear;
        Vector2D pose;

        // \brief constructor for Point with no inputs, initializes all to zero
        Point();


        // \brief constructor for Point with cartesian inputs
        Point(const Vector2D & pose);

        // \brief constructor for Point with polar inputs
        Point(const RangeBear & range_bear_);

        // \brief constructor for Point with all inputs
        Point(const RangeBear & range_bear_, const Vector2D & pose);
    };

    /// \brief create a Landmark with pose relative to turtlebot3
    class Landmark
    {
    public:
        /// \brief the default constructor creates a landmark with 0 radius and 0,0 pose and default threshold
        Landmark();

        /// \brief the default constructor creates a landmark user-specified threshold
        /// \param
        Landmark(const double & threshold_);

        /// \brief construct Landmark object with inputs
        /// \param
        /// \param
        Landmark(const double & radius_, const Point & coords_, const std::vector<Points> points_, const double & threshold_);


        /// \brief check if a Point belongs to this Landmark, and add it to points if so
        /// \param
        /// \returns boolean which indicates whether point has been added to cluster.
        /// if so, continue to next point in /scan, otherwise, turn this point into its
        /// own new cluster
        bool evaluate_point(const Point & point_);

        /// \brief return the Points that make up this landmark
        /// \param
        /// \returns
        std::vector<Point> return_points();

        /// \brief return the centre of the landmark (range, bearing, x, y) through Point struct
        /// \param
        /// \returns
        Point return_coords();

        /// \brief return the radius of the landmark
        /// \param
        /// \returns
        double return_radius();


        // TODO: Detect Circle from cluster and set x,y,radius,range,bearing of Landmark

    private:
        // Landmark radius
        double radius
        // Cartesian and Polar coordinates of Landmark
        Point coords;
        // Points which make up Landmark
        std::vector<Point> points;
        // Threshold for evaluating Points
        double threshold;
    };

    /// \brief gets polar coordinates from cartesian coodinates
    /// \param
    /// \returns 
    RangeBear cartesianToPolar(const Vector2D & pose);

    /// \brief gets cartesian coordinates from polar coodinates
    /// \param
    /// \returns 
    Vector2D polarToCartesian(const RangeBear & range_bear);

}

#endif
