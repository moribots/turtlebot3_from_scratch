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
  // this->physics = this->model->GetWorld()->GetPhysicsEngine();

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
      this->motor_pwr_max_ = _sdf->GetElement("motor_pwr_max")->Get<int>();
    }

  // Get Max Motor Torque (stall) in Nm
  this->motor_torque_max_ = 1.5; // default at 1.5
    if (!_sdf->HasElement("motor_torque_max")) {
      ROS_WARN("TurtleDrive Plugin missing <motor_torque_max>, defaulting to \"%s\"",
          this->motor_torque_max_.c_str());
    } else {
      this->motor_torque_max_ = _sdf->GetElement("motor_torque_max")->Get<double>();
    }

  // Now set up wheel joints using max torque attributes
  // Note this method only accepts doubles
  this->joints[0]->SetParam("fmax", 0, this->motor_torque_max_);
  this->joints[1]->SetParam("fmax", 0, this->motor_torque_max_);

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&TurtleDrivePlugin::OnUpdate, this)));

  // This is the Gazebo version of a nove (see gazebo_tools_test.cpp in plen_ros package)
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  // Subscribe to Wheel Cmd topic
  this->WheelCmdSub = this->node->Subscribe(this->wheel_cmd_topic_,
                                            &TurtleDrivePlugin::OnWheelCmdMsg, this);

  // Sensor Data Publisher
  this->SensorDataPub = this->node->Advertise<nuturtlebot::SensorData>(this->sensor_data_topic);

  // Update Rate Parameters
  // Initialize update rate stuff
  if ( this->this->sensor_frequency_ > 0.0 )
  {
    this->update_period_ = 1.0 / this->sensor_frequency_;
  }
  else
  {
    this->update_period_ = 0.0;
  }
  this->last_update_time_ = this->model->GetWorld()->GetSimTime();

  ROS_INFO("TurtleDrive Plugin Loaded!")
}

// Gazebo Update
void TurtleDrivePlugin::OnUpdate()
{
  // ONLY PERFORM UPDATE IF UPDATE PERIOD HAS PASSED
  common::Time current_time = this->model->GetWorld()->GetSimTime();
  double seconds_since_last_update = (current_time - this->last_update_time_ ).Double();

  if (seconds_since_last_update >= this->update_period_ )
  {
    // Set Last Update Time
    this->last_update_time_ = this->model->GetWorld()->GetSimTime();


    // Get Wheel Joint Positions and Publish
    // First, convert from 0-2pi to 0-encoder_ticks_per_rev
    double m = (this->encoder_ticks_per_rev_) / (2.0  * rigid2d::PI);
    double b = (this->encoder_ticks_per_rev_ - 2.0 * rigid2d::PI * m);
    nuturtlebot::SensorData sns;
    sns.left_encoder = this->joints[0]->Position() * m + b;
    sns.right_encoder = this->joints[1]->Position() * m + b;
    this->SensorDataPub->publish(sns);

    // If new wheel command received, set axle velocities
    if (this->wheel_cmd_flag)
    {
      this->joints[0]->SetParam("vel", 0, this->desired_left_velocity);
      this->joints[0]->SetParam("fmax", 0, this->motor_torque_max_);
      this->joints[1]->SetParam("vel", 0, this->desired_right_velocity);
      this->joints[1]->SetParam("fmax", 0, this->motor_torque_max_);
      // Reset Flag
      this->wheel_cmd_flag= false;
    }

  }
  
}

// Wheel Command Callback
void TurtleDrivePlugin::OnWheelCmdMsg(const nuturtlebot::WheelCommands & wc)
{
  // Convert wheel commands (+- 256) to velocities
  double m = (this->motor_rot_max_ * 2.0) / (this->motor_pwr_max_ * 2.0)
  double b = (this->motor_rot_max_ - this->motor_pwr_max_ * m);

  double this->desired_left_velocity =  left_velocity * m + b;
  double this->desired_right_velocity = right_velocity * m + b;

  // Now cap wheel velocities if excessive
  // Cap High
  if (this->desired_left_velocity > this->motor_rot_max_)
  {
    this->desired_left_velocity = this->motor_rot_max_;
  } else if (this->desired_left_velocity < - this->motor_rot_max_)
  {
    this->desired_left_velocity = - this->motor_rot_max_;
  }
  // Cap Low
  if (this->desired_right_velocity > this->motor_rot_max_)
  {
    this->desired_right_velocity = this->motor_rot_max_;
  } else if (this->desired_right_velocity < - this->motor_rot_max_)
  {
    this->desired_right_velocity = - this->motor_rot_max_;
  }

  // Flag to indicate we can publish new wheel commands
  this->wheel_cmd_flag = true;
}
