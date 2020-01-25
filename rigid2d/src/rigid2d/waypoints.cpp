#include "rigid2d/waypoints.hpp"
#include <iostream>

namespace rigid2d
{
	Waypoints()
	{
		// Waypoint 0
		rigid2d::Vector2D w0(3.0, 2.0);
		// Waypoint 1
		rigid2d::Vector2D w1(7.0, 2.0);
		// Waypoint 2
		rigid2d::Vector2D w2(7.0, 7.0);
		// Waypoint 3
		rigid2d::Vector2D w3(3.0, 7.0);
		waypoints = {w0, w1, w2, w3};

		done = false;
		lin_ang = false;
	}

	Waypoints(std::vector<rigid2d::Vector2D> waypoints_)
	{
		waypoints = waypoints_;
		done = false;
		lin_ang = false;
	}

	rigid2d::Twist2D nextWaypoint(const rigid2d::DiffDrive && driver)
	{
		threshold = 0.01;
		// done true: compute Twist2D
		// done false: select next waypoint
		switch(done)
		{

		case false:
		// wpt 0
		// !TODO!: atan2 using driver pos and waypoint
		// calculate heading based on atan2(dy,dx);
		float goal_x = waypoint[0].x;
		float goal_y = waypoint[0].y;
		float goal_head = atan2(goal_y - driver.pose.y, goal_x- driver.pose.x);

		// lin_ang true: angular motion
		// lin_ang false: linear motion
		switch(lin_ang)
		{
			case true:
			// check if heading appropriate
			if ((abs(goal_head - driver.pose.theta) <= threshold / 3.0))
			{
				lin_ang = false;
			} else {
				// calculate angular twist
			}
			break;

			case false:
			// Check if goal reached
			if (abs(goal_y - driver.pose.y) <= threshold \
          	&& abs(goal_x - driver.pose.x) <= threshold)
      		{
        	lin_ang = true;
        	done = true;
      		} else {
      			// calculate translational twist
      		}
			break;
		}

		if (abs(goal_x - driver.pose.x) <= threshold \
		&& abs(goal_y - driver.pose.y) <= threshold \
		&& abs(goal_head - driver.pose.theta) <= threshold / 3.0)
		{
		done = true;
		}
		break;

		case true:
		// rotate performs the following ex: {0, 1, 2, 3} --> {1, 2, 3, 0}
		// subsequent calls: {2, 3, 0, 1} --> {3, 0, 1, 2} --> {0, 1, 2, 3}
		std::rotate(waypoints.begin(), waypoints.begin() + 1, waypoints.end() );
		// set done flag to false
		done = false;
		break;

		default:
		// reset done flag to false
		done = false;

		}

	}																												
}