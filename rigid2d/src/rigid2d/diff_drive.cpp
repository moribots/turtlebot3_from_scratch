#include "rigid2d/diff_drive.hpp"
#include <iostream>

namespace rigid2d
{

// Pose2D
Pose2D::Pose2D()
{
    x = 0;
    y = 0;
    theta = 0;
}

Pose2D::Pose2D(double x_, double y_, double theta_)
{
    x = x_;
    y = y_;
    theta = theta_;
}

//WheelVelocities
WheelVelocities::WheelVelocities()
{
    ul = 0;
    ur = 0;
}

WheelVelocities::WheelVelocities(double ul_, double ur_)
{
    ul = ul_;
    ur = ur_;
}

DiffDrive::DiffDrive()
{
	rigid2d::Pose2D pose();
	wheel_base = 0.1;
	wheel_radius = 0.025;
	rigid2d::WheelVelocities wheel_vel();

}

DiffDrive::DiffDrive(rigid2d::Pose2D pose_, double wheel_base_, double wheel_radius_)
{
	rigid2d::Pose2D pose - pose_;
	wheel_base = wheel_base_;
	wheel_radius = wheel_radius_;
	rigid2d::WheelVelocities wheel_vel();
}

}