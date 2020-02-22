#ifndef _TURTLE_DRIVE_PLUGIN_HH_
#define _TURTLE_DRIVE_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE TurtleDrivePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: TurtleDrivePlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: void OnVelMsg(ConstPosePtr &_msg);

    private: std::vector<event::ConnectionPtr> connections;

    private: physics::ModelPtr model;
    private: physics::LinkPtr chassis;
    private: std::vector<physics::JointPtr> joints;
    private: physics::JointPtr gasJoint, brakeJoint;
    private: physics::JointPtr steeringJoint;

    private: math::Vector3 velocity;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr velSub;

    private: double frontPower, rearPower;
    private: double maxSpeed;
    private: double wheelRadius;

    private: double steeringRatio;
    private: double tireAngleRange;
    private: double maxGas, maxBrake;

    private: double aeroLoad;
    private: double swayForce;
  };
}
#endif