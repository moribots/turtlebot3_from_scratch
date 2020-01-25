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

	Twist2D Waypoints::nextWaypoint(const DiffDrive & driver)
	{
		Twist2D Vb(0, 0, 0);
		float threshold = 0.08;
		float goal_x = waypoints[0].x;
		float goal_y = waypoints[0].y;
		float goal_head = atan2(goal_y - driver.pose.y, goal_x- driver.pose.x);
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
			// check if heading appropriate
			if ((abs(goal_head - driver.pose.theta) <= threshold / 3.0))
			{
				lin_ang = 1;
			} else {
				// calculate angular twist
				float h_err = goal_head - driver.pose.theta;
				float P_h = 3;
				float w_z = P_h * h_err;
				if (w_z > 0.5) {w_z = 0.5;}
				Twist2D Vb(w_z, 0, 0);
			}
			break;

			case 1:
			// Check if goal reached
			if (abs(goal_y - driver.pose.y) <= threshold \
          	&& abs(goal_x - driver.pose.x) <= threshold)
      		{
        	lin_ang = 0;
        	done = 1;
      		} else if (!(abs(goal_head - driver.pose.theta) <= threshold / 3.0)){
      			// Double check for appropriate heading
      			lin_ang = 0;
      		} else {
      			// calculate translational twist
      			float d_err = sqrt(pow(goal_x - driver.pose.x, 2) + pow(goal_y - driver.pose.y, 2));
				float P_l = 3;
				float v_x = P_l * d_err;
				if (v_x > 0.5) {v_x = 0.5;}
				Twist2D Vb(0, v_x, 0);
      		}
			break;
		}

		if (abs(goal_x - driver.pose.x) <= threshold \
		&& abs(goal_y - driver.pose.y) <= threshold \
		&& abs(goal_head - driver.pose.theta) <= threshold / 3.0)
		{
		done = 1;
		}
		break;

		case 1:
		// rotate performs the following ex: {0, 1, 2, 3} --> {1, 2, 3, 0}
		// subsequent calls: {2, 3, 0, 1} --> {3, 0, 1, 2} --> {0, 1, 2, 3}
		std::rotate(waypoints.begin(), waypoints.begin() + 1, waypoints.end() );
		// set done flag to 0
		done = 0;
		break;

		default:
		// reset done flag to 0
		done = 0;

		}
	return Vb;
	}																												
}