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
    /// \throws std::exception
    rigid2d::WheelVelocities twistToWheels();

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
    rigid2d::WheelVelocities updateOdometry();

    /// \brief update the odometry of the diff drive robot, assuming that
    /// it follows the given body twist for one time  unit
    /// \param cmd - the twist command to send to the robot
    void feedforward();

    /// \brief get the current pose of the robot
    rigid2d::Pose2D pose();

    /// \brief get the wheel speeds, based on the last encoder update
    rigid2d::WheelVelocities wheelVelocities() const;

    /// \brief reset the robot to the given position/orientation
    void reset(rigid2d::Pose2D ps);
private:
    double wheel_base, wheel_radius;
    rigid2d::WheelVelocities wheel_vel;
    double wl_ang, wr_ang;
    rigid2d::Pose2D pose;

};

#endif
