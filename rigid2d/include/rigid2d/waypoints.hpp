#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library DiffDrive robot kinematics and odometry.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include<iostream>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

namespace rigid2d
{
    class Waypoints
    {
    public:
        /// \brief the default constructor creates a vectour of four waypoints in a rectangle
        Waypoints();

        /// \brief create a Waypoints instance by specifying the
        /// waypoints to visit in a vector
        /// \param std::vector<rigid2d::Vector2D> - the waypoints to visit
        Waypoints(std::vector<rigid2d::Vector2D> waypoints_);

        /// \brief computes the required Twist2D to reach the next waypoint,
        /// whether linear or angular, and pushes waypoints back for cyclical
        /// motion when necessary
        /// \param DiffDrive - instance of robot for which we compute a Twist2D
        /// \returns Twist2D required to follow the current waypoint
        rigid2d::Twist2D nextWaypoint(const rigid2d::DiffDrive && driver);

    private:
        std::vector<rigid2d::Vector2D> waypoints;
        bool counter;
        bool lin_ang;

    };
}

#endif
