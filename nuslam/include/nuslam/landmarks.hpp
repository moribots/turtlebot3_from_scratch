#ifndef LANDMARKS_INCLUDE_GUARD_HPP
#define LANDMARKS_INCLUDE_GUARD_HPP
/// \file
/// \brief Library Landmarks landmark detection and classification.
#include <rigid2d/rigid2d.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Eigenvalues>
#include <numeric>
#include <functional>
#include <ros/ros.h>

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
        // Data Association Vars
        bool init; // New/Old Landmark
        int index; // Index in Landmark List
        int seen_count; // criterion for adding to map state

        // \brief constructor for Point with no inputs, initializes all to zero
        Point();

        // \brief constructor for Point with cartesian inputs
        Point(const Vector2D & pose_);

        // \brief constructor for Point with polar inputs
        Point(const RangeBear & range_bear_);
    };

    /// \brief create a Landmark with pose relative to turtlebot3
    class Landmark
    {
    public:
        /// \brief the default constructor creates a landmark with 0 radius and 0,0 pose and default threshold
        Landmark();

        /// \brief the default constructor creates a landmark user-specified threshold
        /// \param threshold: euclidean distance below which to consider two LIDAR points as belonging to one cluster
        Landmark(const double & threshold_);

        /// \brief construct Landmark object with inputs 
        /// \param radius: radius of detected landmark
        /// \param coords_: cartesian coordinates of detected landmark
        /// \param points_: vector of Point, each of which is a part of the cluster constituing this landmark
        /// \param threshold: euclidean distance below which to consider two LIDAR points as belonging to one cluster
        Landmark(const double & radius_, const Point & coords_, const std::vector<Point> points_, const double & threshold_);


        /// \brief check if a Point belongs to this Landmark, and add it to points if so
        /// \param point: Point struct containing x,y coords as well as ra
        /// \returns boolean which indicates whether point has been added to cluster.
        /// if so, continue to next point in /scan, otherwise, turn this point into its
        /// own new cluster
        bool evaluate_point(const Point & point_);

        /// \brief return the Points that make up this landmark
        /// \returns vector of Point
        std::vector<Point> return_points();

        /// \brief return the centre of the landmark (range, bearing, x, y) through Point struct
        /// \returns Point struct containing x,y coords as well as range,bear measurement
        Point return_coords();

        /// \brief return the radius of the landmark
        /// \returns landmark radius
        double return_radius();


        /// \brief Fit Circle from cluster and set x,y,radius,range,bearing of Landmark
        /// \returns root-mean-squared error of circle fit
        double fit_circle();

        /// \brief Classify Circle from cluster and set x,y,radius,range,bearing of Landmark
        /// based on Inscribed Angle Theorem for the Arc of a Circle
        /// \returns a boolean to indicate whether the cluster is a circle
        bool classify_circle();

        // Points which make up Landmark
        std::vector<Point> points;

    private:
        // Landmark radius
        double radius;
        // Cartesian and Polar coordinates of Landmark
        Point coords;
        // Threshold for evaluating Points
        double threshold;
    };

    /// \brief gets polar coordinates from cartesian coodinates
    /// \param pose containing x and y position relative to robot
    /// \returns RangeBear struct containing range and bearing
    RangeBear cartesianToPolar(const Vector2D & pose);

    /// \brief gets cartesian coordinates from polar coodinates
    /// \param RangeBear struct containing range and bearing
    /// \returns  Vector2D containing cartesian coordinates
    Vector2D polarToCartesian(const RangeBear & range_bear);

}

#endif
