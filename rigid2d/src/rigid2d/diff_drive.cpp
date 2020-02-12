#include "rigid2d/diff_drive.hpp"

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
	// Eqn 1.1 and 1.2 in Kinematics_Derivation.pdf
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
	// pg 547 Modern Robotics
	// Eqn 2 in Kinematics_Derivation.pdf
	rigid2d::Twist2D twist(wheel_radius * (-vel.ul + vel.ur) / wheel_base,\
						   wheel_radius * (vel.ul + vel.ur) / 2,\
						   0);
	return twist;
}

rigid2d::WheelVelocities DiffDrive::updateOdometry(double left, double right)
{
	// Update Wheel Velocities
	wheel_vel.ul = normalize_angle(left - wl_ang);
	wheel_vel.ur = normalize_angle(right - wr_ang);

	// Update Wheel Angles
	// left = normalize_encoders(left);
	wl_ang = normalize_angle(left);
	// std::cout << left << std::endl;
	// right = normalize_encoders(right);
	wr_ang = normalize_angle(right);
	// std::cout << right << std::endl;

	// Now call feedforward to update odometry
	rigid2d::Twist2D Vb = DiffDrive::wheelsToTwist(wheel_vel);

	// Same thing as feedforward fcn...
	// Update odometry by calculating Tbb' = exp(Vb)
	// Now integrate Twist to get Tbb', first create Transform2D
	rigid2d::Transform2D Tb(pose.theta, cos(pose.theta), sin(pose.theta), pose.x, pose.y);
	rigid2d::Transform2D Tbbp = Tb.integrateTwist(Vb);
	// Use Transform2DS to return private Transform2D params
	rigid2d::Transform2DS TbbpS = Tbbp.displacement();
	// Update Pose and Wrap between +- PI
	pose.theta = normalize_angle(TbbpS.theta);
	pose.x = TbbpS.x;
	pose.y = TbbpS.y;

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
	// Update Pose and Wrap between +- PI
	pose.theta = normalize_angle(TbbpS.theta);
	pose.x = TbbpS.x;
	pose.y = TbbpS.y;
	// Update Wheel Velocities
	wheel_vel = DiffDrive::twistToWheels(Vb);
	wheel_vel.ul = normalize_angle(wheel_vel.ul);
	wheel_vel.ur = normalize_angle(wheel_vel.ur);
	// Update Wheel Angles
	wl_ang += wheel_vel.ul;
	wl_ang = normalize_angle(wl_ang);
	// wl_ang = normalize_encoders(wl_ang);
	wr_ang += wheel_vel.ur;
	wr_ang = normalize_angle(wr_ang);
	// wr_ang = normalize_encoders(wr_ang);

}

rigid2d::Pose2D DiffDrive::get_pose()
{
	return pose;
}

rigid2d::WheelVelocities DiffDrive::get_ang()
{
	rigid2d::WheelVelocities w_ang(wl_ang, wr_ang);
	return w_ang;
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
	// wl_ang = 0;
	// wr_ang = 0;
}

std::ostream & operator<<(std::ostream & os, const rigid2d::DiffDrive & driver)
{
	os << "wl (rad): " << driver.wl_ang << "\t" << "wr: " << driver.wr_ang << "\n";

	return os;
}
																																			
}