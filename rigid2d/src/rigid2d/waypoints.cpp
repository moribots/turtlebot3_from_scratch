#include "rigid2d/waypoints.hpp"

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
	}

	Waypoints::Waypoints(const std::vector<Vector2D> & waypoints_)
	{
		waypoints = waypoints_;
		done = 0;
		lin_ang = 0;
	}

	Twist2D Waypoints::nextWaypoint(DiffDrive & driver)
	{
		Twist2D Vb(0, 0, 0);
		float threshold = 0.05;
		float goal_x = waypoints.at(0).x;
		float goal_y = waypoints.at(0).y;
		float goal_head = atan2(goal_y - driver.pose.y, goal_x - driver.pose.x);
		// Wrap driver pose theta from -PI to PI
		driver.pose.theta = normalize_angle(driver.pose.theta);
		// std::cout << "GOAL H: " << goal_head << "\tCURRENT H: "<< driver.pose.theta << \
		// "\tGOAL X: " << goal_x << "\tGOAL Y: " << goal_y << std::endl;
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
			// goal_head = atan2(goal_y - driver.pose.y, goal_x - driver.pose.x);
			// check if heading appropriate
			if ((fabs(goal_head - driver.pose.theta) <= threshold / 2.0))
			{
				// std::cout << "H ERR: " << fabs(goal_head - driver.pose.theta) << "\tTHRESH: " << threshold << std::endl;
				// std::cout << "GOAL H: " << goal_head << "\tCURRENT H: "<< driver.pose.theta << std::endl;
				// std::cout << "ANG DONE" << std::endl;
				lin_ang = 1;
			} else {
				// std::cout << "GOAL H: " << goal_head << "\tCURRENT H: "<< driver.pose.theta << std::endl;
				// calculate angular twist
				float h_err = goal_head - driver.pose.theta;
				// Wrap head error to avoid unecessary turns
				if (h_err >= PI) {h_err -= 2 * PI;}
				if (h_err <= -PI) {h_err += 2 * PI;}
				float P_h = 20;
				float w_z = P_h * h_err;
				// Cap vel
				if (w_z > 0.5) {w_z = 0.5;}
				else if (w_z < 0.5) {w_z = -0.5;}
				Twist2D Vb(w_z, 0, 0);
				return Vb;
			}
			break;

			case 1:
			// Check if goal reached
			if (fabs(goal_y - driver.pose.y) <= threshold \
          	&& fabs(goal_x - driver.pose.x) <= threshold)
      		{
      		// std::cout << "LIN DONE" << std::endl;
        	lin_ang = 0;
        	// done = 1;
      		} else if ((fabs(goal_head - driver.pose.theta) > threshold / 2.0)){
      			// Double check for appropriate heading
      			lin_ang = 0;
      		} else {
      			// calculate translational twist
      			float d_err = sqrt(pow(goal_x - driver.pose.x, 2) + pow(goal_y - driver.pose.y, 2));
				float P_l = 10;
				float v_x = P_l * d_err;
				// Cap vel
				if (v_x > 0.5) {v_x = 0.5;}
				else if (v_x < 0.5) {v_x = -0.5;}
				Twist2D Vb(0, v_x, 0);
				return Vb;
      		}
			break;
		}

		if (fabs(goal_x - driver.pose.x) <= threshold \
		&& fabs(goal_y - driver.pose.y) <= threshold \
		&& fabs(goal_head - driver.pose.theta) <= threshold / 2.0)
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