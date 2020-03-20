#ifndef _TURTLE_DRIVE_PLUGIN_HH_
#define _TURTLE_DRIVE_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"
#include <ros/ros.h>
#include <functional> 

#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/rigid2d.hpp"

namespace gazebo
{
  class TurtleDrivePlugin : public ModelPlugin
  {
    /// \brief Constructor for TurtleDrivePlugin. Models real Turtlebot interface for use in Gazebo
    public: 

        /// \brief empty constructor for plugin
        TurtleDrivePlugin();

        /// \brief callback to convert wheel commands (+- 256) to velocities and applies them to gazebo model
        /// \param nuturtlebot::WheelCommands: integer value between +-256 to indicate wheel speed
        void wheel_cmdCallback(const nuturtlebot::WheelCommands &wc);

        /// \brief Loads the plugin and uses the physics::ModelPtr object to manipulate model physics and
        /// the sdf::ElementPtr to read user-specified parameters such as joint names and ros topics
        /// \param physics::ModelPtr: pointer to manipulate model physics (set joint speeds/torques etc)
        /// \param sdf::ElementPtr: pointer to read user input from .gazebo.xacro file to set plugin parameters
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:

        /// \brief Plugin equivalent of ross:Spin() where model updates happen according to callbacks or otherwise
        /// in this case, sets the wheel joint velocities and torques (always max specified) according to wheel_cmdCallback
        /// using physics::ModelPtr
        void OnUpdate();

        std::vector<event::ConnectionPtr> connections;

        physics::ModelPtr model;
        std::vector<physics::JointPtr> joints;

        ros::NodeHandle nh;
        // std::unique_ptr<ros::NodeHandle> nh;
        transport::NodePtr node;
        ros::Subscriber WheelCmdSub;
        ros::Publisher SensorDataPub;
        // transport::SubscriberPtr WheelCmdSub;
        // transport::PublisherPtr SensorDataPub;

        int sensor_frequency_, encoder_ticks_per_rev_, motor_pwr_max_;
        std::string wheel_cmd_topic_, sensor_data_topic_;
        double motor_rot_max_, motor_torque_max_, update_period_;
        double desired_left_velocity, desired_right_velocity;
        common::Time last_update_time_;
        bool wheel_cmd_flag_;

  };
}

#endif