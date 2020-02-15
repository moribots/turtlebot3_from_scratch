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

        /// \brief create a Waypoints instance by specifying the
        /// waypoints to visit in a vector, the proportional controller
        /// gains, the maximum twist thresholds and the pose threshold
        /// \param std::vector<rigid2d::Vector2D> - the waypoints to visit
        /// \param (double) P_h - proportional control for heading
        /// \param (double) P_h - proportional control for cartesian position
        /// \param (double) w_z_max - maximum angular velocity
        /// \param (double) v_x_max - maximum linear velocity
        Waypoints(const std::vector<Vector2D> & waypoints_, const double & P_h_, const double & P_l_,\
                  const double & w_z_max_, const double & v_x_max_, const double & threshold_);

        /// \brief computes the required Twist2D to reach the next waypoint,
        /// whether linear or angular, and pushes waypoints back for cyclical
        /// motion when necessary
        /// \param DiffDrive - instance of robot for which we compute a Twist2D
        /// \returns Twist2D required to follow the current waypoint
        Twist2D nextWaypoint(const Pose2D & pose);

    private:
        std::vector<Vector2D> waypoints;
        int done;
        int lin_ang;
        double P_h;
        double P_l;
        double w_z_max;
        double v_x_max;
        double threshold;

    };
}

#endif
