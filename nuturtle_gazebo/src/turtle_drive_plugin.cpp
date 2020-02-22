#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/turtle_drive_plugin.hh"


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(TurtleDrivePlugin)

/////////////////////////////////////////////////
TurtleDrivePlugin::TurtleDrivePlugin()
{
  this->joints.resize(2);
}

// Load Plugin
void TurtleDrivePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // see http://gazebosim.org/tutorials?tut=ros_plugins Accessed 02/09/2020
  // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized."
              "Unable to load plugin. Load the Gazebo system plugin"
              "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
    return;
  }

  ROS_INFO("Loading TurtleDrive Plugin...")

  // Get Model object to manipulate model physics
  this->model = _model;
  this->physics = this->model->GetWorld()->GetPhysicsEngine();

  // Get Joints (left and right wheel axles)
  this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("left_wheel_joint"));
  if (!this->joints[0])
  {
    gzerr << "Unable to find left_wheel_joint joint ["
          << _sdf->GetElement("left_wheel_joint") << "]\n";
    return;
  }

  this->joints[1] = this->model->GetJoint(
      _sdf->Get<std::string>("right_wheel_joint"));

  if (!this->joints[1])
  {
    gzerr << "Unable to find right_wheel_joint joint ["
          << _sdf->GetElement("right_wheel_joint") << "]\n";
    return;
  }

  // Get Elements from Plugin Call in .gazebo.xacro

  // Get Sensor Frequency (Encoders)
  this->sensor_frequency_ = 200; // default at 200Hz
    if (!_sdf->HasElement("sensor_frequency")) {
      ROS_WARN("TurtleDrive Plugin missing <sensor_frequency>, defaulting to \"%s\"",
          this->sensor_frequency_.c_str());
    } else {
      this->sensor_frequency_ = _sdf->GetElement("sensor_frequency")->Get<int>();
    }

  // Get Wheel Command Topic
  this->wheel_cmd_topic_ = "nuturtlebot/WheelCommands"; // default
    if (!_sdf->HasElement("wheel_cmd_topic")) {
      ROS_WARN("TurtleDrive Plugin missing <wheel_cmd_topic>, defaulting to \"%s\"",
          this->wheel_cmd_topic_.c_str());
    } else {
      this->wheel_cmd_topic_ = _sdf->GetElement("wheel_cmd_topic")->Get<std::string>();
    }

  // Get Sensor Data Topic
  this->sensor_data_topic_ = "nuturtlebot/SensorData"; // default
    if (!_sdf->HasElement("sensor_data_topic")) {
      ROS_WARN("TurtleDrive Plugin missing <sensor_data_topic>, defaulting to \"%s\"",
          this->sensor_data_topic_.c_str());
    } else {
      this->sensor_data_topic_ = _sdf->GetElement("sensor_data_topic")->Get<std::string>();
    }

  // Get Encoder Ticks Per Rev
  this->encoder_ticks_per_rev_ = 4096; // default at 4096
    if (!_sdf->HasElement("encoder_ticks_per_rev")) {
      ROS_WARN("TurtleDrive Plugin missing <encoder_ticks_per_rev>, defaulting to \"%s\"",
          this->encoder_ticks_per_rev_.c_str());
    } else {
      this->encoder_ticks_per_rev_ = _sdf->GetElement("encoder_ticks_per_rev")->Get<int>();
    }

  // Get Max Motor Velocity in rad/s
  this->motor_rot_max_ = 6.35492; // default at 6.35492
    if (!_sdf->HasElement("motor_rot_max")) {
      ROS_WARN("TurtleDrive Plugin missing <motor_rot_max>, defaulting to \"%s\"",
          this->motor_rot_max_.c_str());
    } else {
      this->motor_rot_max_ = _sdf->GetElement("motor_rot_max")->Get<double>();
    }

  // Get Max Motor Power (command to send ints to ESC for output between +- 265)
  this->motor_pwr_max_ = 265; // default at 265
    if (!_sdf->HasElement("motor_pwr_max")) {
      ROS_WARN("TurtleDrive Plugin missing <motor_pwr_max>, defaulting to \"%s\"",
          this->motor_pwr_max_.c_str());
    } else {
      this->motor_pwr_max_ = _sdf->GetElement("motor_pwr_max")->Get<double>();
    }

  // Get Max Motor Torque (stall) in Nm
  this->motor_torque_max_ = 1.5; // default at 1.5
    if (!_sdf->HasElement("motor_torque_max")) {
      ROS_WARN("TurtleDrive Plugin missing <motor_torque_max>, defaulting to \"%s\"",
          this->motor_torque_max_.c_str());
    } else {
      this->motor_torque_max_ = _sdf->GetElement("motor_torque_max")->Get<double>();
    }

  // Now set up wheel joints using max velocity and torque attributes
  // Note this method only accepts doubles
  this->joints[0]->SetParam("fmax", 0, this->motor_torque_max_);
  this->joints[0]->SetParam("vel", 0, this->motor_rot_max_);

  this->joints[1]->SetParam("fmax", 0, this->motor_torque_max_);
  this->joints[1]->SetParam("vel", 0, this->motor_rot_max_);

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&TurtleDrivePlugin::OnUpdate, this)));

  // This is the Gazebo version of a nove (see gazebo_tools_test.cpp in plen_ros package)
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  // Subscribe to Wheel Cmd topic
  this->WheelCmdSub = this->node->Subscribe(this->wheel_cmd_topic_,
                                            &TurtleDrivePlugin::OnWheelCmdMsg, this);

  ROS_INFO("TurtleDrive Plugin Loaded!")
}

/////////////////////////////////////////////////
void TurtleDrivePlugin::Init()
{
}

/////////////////////////////////////////////////
void TurtleDrivePlugin::OnUpdate()
{
  
}

/////////////////////////////////////////////////
void TurtleDrivePlugin::OnWheelCmdMsg(ConstPosePtr &/*_msg*/)
{
}
