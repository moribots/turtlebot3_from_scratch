#ifndef _TURTLE_DRIVE_PLUGIN_HH_
#define _TURTLE_DRIVE_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"

#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/rigid2d.hpp"

namespace gazebo
{
  class GAZEBO_VISIBLE TurtleDrivePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: TurtleDrivePlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private: void OnUpdate();

    private: void OnWheelCmdMsg(const nuturtlebot::WheelCommands & wc)

    private: std::vector<event::ConnectionPtr> connections;

    private: physics::ModelPtr model;
    private: std::vector<physics::JointPtr> joints;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr WheelCmdSub;
    private: transport::PublisherPtr SensorDataPub;

    private: int sensor_frequency_, encoder_ticks_per_rev_, motor_pwr_max_;
    private: std::string wheel_cmd_topic_, sensor_data_topic_;
    private: double motor_rot_max_, motor_torque_max_, update_period_;
    private: double desired_left_velocity, desired_right_velocity;
    private: common::Time last_update_time_;
  };
}

#endif