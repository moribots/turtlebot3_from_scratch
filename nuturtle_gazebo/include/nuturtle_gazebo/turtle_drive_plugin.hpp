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
  class GAZEBO_VISIBLE TurtleDrivePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: 

        TurtleDrivePlugin();

        void wheel_cmdCallback(const nuturtlebot::WheelCommands &wc);

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:

        void OnUpdate();

        std::vector<event::ConnectionPtr> connections;

        physics::ModelPtr model;
        std::vector<physics::JointPtr> joints;

        transport::NodePtr node;
        transport::SubscriberPtr WheelCmdSub;
        transport::PublisherPtr SensorDataPub;

        int sensor_frequency_, encoder_ticks_per_rev_, motor_pwr_max_;
        std::string wheel_cmd_topic_, sensor_data_topic_;
        double motor_rot_max_, motor_torque_max_, update_period_;
        double desired_left_velocity, desired_right_velocity;
        common::Time last_update_time_;
        bool wheel_cmd_flag_;

  };
}

#endif