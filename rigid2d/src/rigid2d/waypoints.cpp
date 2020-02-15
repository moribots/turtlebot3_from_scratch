#include "rigid2d/waypoints.hpp"
#include <ros/ros.h>

namespace rigid2d
{
	Waypoints::Waypoints()
	{
		// Waypoint 0
		Vector2D w0(3.0, 2.0);
		// Waypoint 1
		Vector2D w1(7.0, 2.0);
		// Waypoint 2
		Vector2D w2(7.0, 7.0);
		// Waypoint 3
		Vector2D w3(3.0, 7.0);
		std::vector<Vector2D> v{w0, w1, w2, w3};
		waypoints = v;

		done = 0;
		lin_ang = 0;

		P_h = 20;
		P_l = 10;

		w_z_max = 0.5;
		v_x_max = 0.5;

		threshold = 0.05;
	}

	Waypoints::Waypoints(const std::vector<Vector2D> & waypoints_)
	{
		waypoints = waypoints_;
		done = 0;
		lin_ang = 0;
		P_h = 20;
		P_l = 10;
		w_z_max = 0.5;
		v_x_max = 0.5;
		threshold = 0.05;
	}

	Waypoints::Waypoints(const std::vector<Vector2D> & waypoints_,\
						 const double & P_h_, const double & P_l_,\
	                     const double & w_z_max_,\
	                     const double & v_x_max_,\
	                     const double & threshold_)
	{
		waypoints = waypoints_;
		done = 0;
		lin_ang = 0;
		P_h = P_h_;
		P_l = P_l_;
		w_z_max = w_z_max_;
		v_x_max = v_x_max_;
		threshold = threshold_;
	}

	Twist2D Waypoints::nextWaypoint(const Pose2D & pose)
	{
		Twist2D Vb(0, 0, 0);
		double goal_x = waypoints.at(0).x;
		double goal_y = waypoints.at(0).y;
		double goal_head = atan2(goal_y - pose.y, goal_x - pose.x);
		// done 0: compute Twist2D
		// done 1: select next waypoint
		switch(done)
		{
		case 0:
		// lin_ang 0: angular motion
		// lin_ang 1: linear motion
		switch(lin_ang)
		{
			case 0:
			// goal_x = waypoints.at(0).x;
			// goal_y = waypoints.at(0).y;
			// goal_head = atan2(goal_y - pose.y, goal_x - pose.x);
			// check if heading appropriate
			if ((fabs(goal_head - pose.theta) <= threshold / 2.0))
			{
				// std::cout << "ANG DONE" << std::endl;
				lin_ang = 1;
			} else {
				// std::cout << "H ERR: " << fabs(goal_head - pose.theta) << "\tTHRESH: " << threshold / 2.0 << std::endl;
				// std::cout << "GOAL H: " << goal_head << "\tCURRENT H: "<< pose.theta << std::endl;
				// calculate angular twist
				double h_err = goal_head - pose.theta;
				// Wrap head error to avoid unecessary turns
				if (h_err > PI) {h_err -= 2. * PI;}
				if (h_err < -PI) {h_err += 2. * PI;}
				double w_z = P_h * h_err;
				// Cap vel
				if (w_z > w_z_max) {w_z = w_z_max;}
				else if (w_z < -w_z_max) {w_z = -w_z_max;}
				Twist2D Vb(w_z, 0, 0);
				return Vb;
			}
			break;

			case 1:
			// Check if goal reached
			if (fabs(goal_y - pose.y) <= threshold \
          	&& fabs(goal_x - pose.x) <= threshold)
      		{
      		// std::cout << "LIN DONE" << std::endl;
        	lin_ang = 0;
        	// done = 1;
      		} else if ((fabs(goal_head - pose.theta) > threshold / 2.0)){
      			// Double check for appropriate heading
      			lin_ang = 0;
      		} else {
      			// calculate translational twist
      			double d_err = sqrt(pow(goal_x - pose.x, 2) + pow(goal_y - pose.y, 2));
				double v_x = P_l * d_err;
				// Cap vel
				if (v_x > v_x_max) {v_x = v_x_max;}
				else if (v_x < -v_x_max) {v_x = -v_x_max;}
				Twist2D Vb(0, v_x, 0);
				return Vb;
      		}
			break;
		}

		if (fabs(goal_x - pose.x) <= threshold \
		&& fabs(goal_y - pose.y) <= threshold \
		&& fabs(goal_head - pose.theta) <= threshold / 2.0)
		{
		done = 1;
		lin_ang = 0;
		}
		break;

		case 1:
		std::cout << "WAYPOINT REACHED: [" << waypoints.at(0).x << ", " << waypoints.at(0).y << "]" << std::endl;
		// rotate performs the following ex: {0, 1, 2, 3} --> {1, 2, 3, 0}
		// subsequent calls: {2, 3, 0, 1} --> {3, 0, 1, 2} --> {0, 1, 2, 3}
		std::rotate(waypoints.begin(), waypoints.begin() + 1, waypoints.end() );
		std::cout << "---------------------------------------------------------------------" << std::endl;
		std::cout << "NEXT WAYPOINT: [" << waypoints.at(0).x << ", " << waypoints.at(0).y << "]" << std::endl;
		// set done flag to 0
		done = 0;
		lin_ang = 0;
		break;

		default:
		// reset done flag to 0
		lin_ang = 0;

		}
	return Vb;
	}																												
}