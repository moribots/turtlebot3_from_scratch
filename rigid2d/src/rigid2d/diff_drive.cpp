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
	wheel_base = 1.0;
	wheel_radius = 0.02;
	rigid2d::WheelVelocities wheel_vel();
}

DiffDrive::DiffDrive(rigid2d::Pose2D pose_, double wheel_base_, double wheel_radius_)
{
	pose = pose_;
	wheel_base = wheel_base_;
	wheel_radius = wheel_radius_;
	rigid2d::WheelVelocities wheel_vel();
}


WheelVelocities DiffDrive::twistToWheels(rigid2d::Twist2D Vb)
{
	if (Vb.v_y != 0)
	{
		throw std::invalid_argument("Twist cannot have a y velocity component for the DiffDrive robot.");
		return wheel_vel;
	} else {
	WheelVelocities wheel_vel(((-Vb.w_z * wheel_base / 2) + Vb.v_x) / wheel_radius,
							  ((Vb.w_z * wheel_base / 2) + Vb.v_x) / wheel_radius);
	return wheel_vel;
	}
}

rigid2d::Twist2D DiffDrive::wheelsToTwist(rigid2d::WheelVelocities vel)
{
	rigid2d::Twist2D twist((-vel.ul + vel.ur) / wheel_base, (vel.ul + vel.ur) / 2, 0);
	return twist;
}

rigid2d::WheelVelocities DiffDrive::updateOdometry(double left, double right)
{
	// Update Wheel Velocities
	wheel_vel.ul = left - wl_ang;
	wheel_vel.ur = right - wr_ang;

	// Update Wheel Angles
	wl_ang += left;
	wl_ang = normalize_encoders(wl_ang);
	wr_ang += right;
	wl_ang = normalize_encoders(wr_ang);

	// Now call feedforward to update odometry
	rigid2d::Twist2D Vb = DiffDrive::wheelsToTwist(wheel_vel);
	DiffDrive::feedforward(Vb);

	return wheel_vel;
}

void DiffDrive::feedforward(rigid2d::Twist2D Vb)
{
	// Update odometry by calculating Tbb' = exp(Vb)

	// Now integrate Twist to get Tbb', first create Transform2D
	rigid2d::Transform2D Tb(pose.theta, cos(pose.theta), sin(pose.theta), pose.x, pose.y);
	rigid2d::Transform2D Tbbp = Tb.integrateTwist(Vb);
	// Use Transform2DS to return private Transform2D params
	rigid2d::Transform2DS TbbpS = Tbbp.displacement();
	// Update Pose
	pose.theta = TbbpS.theta;
	pose.x = TbbpS.x;
	pose.y = TbbpS.y;
}

rigid2d::Pose2D DiffDrive::get_pose()
{
	return pose;
}

void DiffDrive::set_static(double wheel_base_, double wheel_radius_)
{
	wheel_base = wheel_base_;
	wheel_radius = wheel_radius_;
}

rigid2d::WheelVelocities DiffDrive::wheelVelocities() const
{
	return wheel_vel;
}

void DiffDrive::reset(rigid2d::Pose2D pos)
{
	pose = pos;
	wheel_vel = rigid2d::WheelVelocities();
	wl_ang = 0;
	wr_ang = 0;
}
																																			
}