#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library DiffDrive robot kinematics and odometry.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include<iostream>
#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{
    // wraps encoder angles from 0 to 2PI
    constexpr double normalize_encoders(double rad)
    {
        double min = 0;
        double max = 2 * PI - min;
        rad -= min;
        return rad - (std::floor(rad / max) * max) + min;
    }

    static_assert(almost_equal(normalize_encoders(deg2rad(370)), (deg2rad(10))), "normalize_encoders failed");
    static_assert(almost_equal((deg2rad(350)), normalize_encoders(deg2rad(-10))), "normalize_encoders failed");

    /// \brief A 2-Dimensional Pose
    struct Pose2D
    {
    double x;
    double y;
    double theta;

    // \brief constructor for Pose2D with no inputs, creates a zero Pose
    Pose2D();

    // \brief constructor for Pose2D with inputs
    Pose2D(double x_, double y_, double theta_);
    };

    /// \brief Wheel Velocities (rad/s)
    struct WheelVelocities
    {
    double ul;
    double ur;

    // \brief constructor for WheelVelocities with no inputs, initialized to zero
    WheelVelocities();

    // \brief constructor for WheelVelocities with inputs
    WheelVelocities(double ul_, double ur_);
    };

        /// \brief create a DiffDrive model
        class DiffDrive
    {
    friend class Waypoints;
    public:
        /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
        DiffDrive();

        /// \brief create a DiffDrive model by specifying the Pose, and geometry
        ///
        /// \param Pose2D - the current position of the robot
        /// \param wheel_base - the distance between the wheel centers
        /// \param wheel_radius - the raidus of the wheels
        DiffDrive(rigid2d::Pose2D pose_, double wheel_base_, double wheel_radius_);

        /// \brief determine the wheel velocities required to make the robot
        ///        move with the desired linear and angular velocities
        /// \param twist - the desired twist in the body frame of the robot
        /// \returns - the wheel velocities to use
        /// \throws std::exception if Twist has y component.
        rigid2d::WheelVelocities twistToWheels(rigid2d::Twist2D Vb);

        /// \brief determine the body twist of the robot from its wheel velocities
        /// \param vel - the velocities of the wheels, assumed to be held constant
        ///  for one time unit
        /// \returns twist in the original body frame of the robot
        rigid2d::Twist2D wheelsToTwist(rigid2d::WheelVelocities vel);

        /// \brief Update the robot's odometry based on the current encoder readings
        /// \param left - the left encoder angle (in radians)
        /// \param right - the right encoder angle (in radians)
        /// \return the velocities of each wheel, assuming that they have been
        /// constant since the last call to updateOdometry
        rigid2d::WheelVelocities updateOdometry(double left, double right);

        /// \brief update the odometry of the diff drive robot, assuming that
        /// it follows the given body twist for one time  unit
        /// \param Vb - the twist command to send to the robot
        void feedforward(rigid2d::Twist2D Vb);

        /// \brief get the current pose of the robot
        rigid2d::Pose2D get_pose();

        /// \brief get the current wheel angles (overloading WheelVelocities struct)
        rigid2d::WheelVelocities get_ang();

        /// \brief set DiffDrive instance's static parameters such as wheel base and radius
        void set_static(double wheel_base_, double wheel_radius_);

        /// \brief get the wheel speeds, based on the last encoder update
        rigid2d::WheelVelocities wheelVelocities() const;

        /// \brief reset the robot to the given position/orientation with 0 vel
        void reset(rigid2d::Pose2D pos);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description.
        /// friend tag allows non-member functions to access private params.
        friend std::ostream & operator<<(std::ostream & os, const DiffDrive & dd);

    private:
        double wheel_base, wheel_radius;
        rigid2d::WheelVelocities wheel_vel;
        double wl_ang, wr_ang;
        rigid2d::Pose2D pose;

    };

    /// \brief should print a human readable version of the twist:
    /// An example output:
    /// wl: (rad), wr: (rad)
    /// \param os - an output stream
    /// \param dd - the diffdrive wheel angles to print
    std::ostream & operator<<(std::ostream & os, const DiffDrive & driver);

}

#endif
