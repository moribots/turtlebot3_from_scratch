#ifndef WAYPOINTS_INCLUDE_GUARD_HPP
#define WAYPOINTS_INCLUDE_GUARD_HPP
/// \file
/// \brief Library DiffDrive robot kinematics and odometry.
// #include "rigid2d/rigid2d.hpp"- implicitly included
#include "rigid2d/diff_drive.hpp"
#include <vector>
#include <algorithm> // to use std::rotate

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
        Waypoints(const std::vector<Vector2D> & waypoints_);

        /// \brief computes the required Twist2D to reach the next waypoint,
        /// whether linear or angular, and pushes waypoints back for cyclical
        /// motion when necessary
        /// \param DiffDrive - instance of robot for which we compute a Twist2D
        /// \returns Twist2D required to follow the current waypoint
        Twist2D nextWaypoint(DiffDrive & driver);

    private:
        std::vector<Vector2D> waypoints;
        int done;
        int lin_ang;

    };
}

#endif
