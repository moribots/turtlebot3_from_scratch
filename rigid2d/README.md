# Package: rigid2d
Author: Maurice Rahme

## Package Summary

This package is an implementation of 2D Lie Group Operations and Differential Drive Kinematics based on [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics) by Kevin Lynch. Visit [my website](https://moribots.github.io/project/ekfslam) for details

## rigid2d.hpp/cpp

2D Lie Group operations Library. Includes Transform, Vector, and Twist operations. Run `rigid2d_node.cpp` to test it for yourself.

## diff_drive.hpp/cpp

Differential Drive Kinematics Library. Run `odometer_node.cpp` for odometry estimates based on wheel angles. Run `fake_diff_encoders_node.cpp` to publish fake wheel angles to `/joint_states`, read by `odometer_node.cpp`.

## waypoints.hpp/cpp

Waypoint navigation Library with Proportional Bang-Bang control. 


