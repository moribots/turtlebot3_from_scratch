# ME495 Sensing, Navigation, and Machine Learning
# Package: tsim
Author: Maurice Rahme

## Package Summary

![tsimpic](media/traj_rect.png)

This package makes the turtle in turtlesim trace a rectangular trajectory while showing a plot of the absolute position error (x, y, theta) between the pure-feedforward and the actual turtle position as read from the `turtle1/pose` topic. It also has a service, `traj_reset`, which teleports the turtle back to its initial configuration whenever called.

Resultant Simulation:

![tsim](media/traj_rect.gif)

## Launch Instructions
To launch the main node without showing the position error plot, run: `roslaunch tsim trect.launch`.

To launch the main node while showing the position error plot, run: `roslaunch tsim trect.launch plot_gui:=1`.

## turtle_rect_node.cpp
This is the executable node, which initialises the node, creates a `Node Handle`, and includes the `TurtleRect` class to make the turtle in turtlesim trace a rectangular trajectory while showing a plot of the absolute position error (x, y, theta) by calling its public `control` method, making it loop indefinitely until it is interrupted.

## turtle_rect.cpp
This is the Class Constructor for `TurtleRect` containing the following methods:

* `traj_resetCallback (bool)`: callback for `traj_reset service`, which teleports turtle back to initial config.
* `poseCallback (void)`: callback for `turtle1/pose` subscriber, which records the turtle's pose for use elsewhere.
* `move (void)`: helper function which publishes `Twist` messages to `turtle1/cmd_vel` to actuate the turtle.
* `predict(void)`: helper function which forward propagates the open-loop model and publishes `PoseError` to `pose_error`.
* `control(void)`: main class method. Houses state machine and calls helper function to perform trajectory and plot.

## turtle_rect.h
Header file for the `TurtleRect` class.

## trect.launch
Calls the `roaming_turtle` node from `turtlesim`, as well as the `turtle_rect_node` node from this package, and gives the user an option to show the error plot using the `plot_gui` argument, which defaults to `False`.

## turtle_rect.yaml
Contains the parameters for executing the rectangular trajectory.

* `x (int)`: x coordinate for lower left corner of rectangle.
* `y (int)`: y coordinate for lower left corner of rectangle.
* `width (int)`: width of rectangle.
* `height (int)`: height of rectangle.
* `trans_vel (int)`: translational velocity of robot.
* `rot_vel (int)`: rotational velocity of robot.
* `frequency (int)`: frequency of control loop.
* `threshold (float)`: specifies when the target pose has been reached.

## Resultant Plot

![plot](media/plot.png)

Note that the plot increases in drift over time as the forward propagated controls would result in several superimposed but slanted rectangular trajectories (as you saw from Josh ealier today) were it not for the feedback control implemented here to stop and start the linear and angular velocity commands. The error plot hence descibres the difference between pure feedforward control and the implementation done here. Calling the `traj_reset` service resets this error to zero temporarily, before re-commencing the trajectory and ensuing in drift as seen below.

![plot](media/reset.png)



