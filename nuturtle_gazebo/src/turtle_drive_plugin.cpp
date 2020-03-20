#include "nuturtle_gazebo/turtle_drive_plugin.hpp"


gazebo::TurtleDrivePlugin::TurtleDrivePlugin()
{
}

// Load Plugin
void gazebo::TurtleDrivePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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

  ROS_INFO("Loading TurtleDrive Plugin...");

  // Diff Drive Robot only has 2 joints
  joints.resize(2);

  // Inititialize joint velocity variables and publish flag
  desired_right_velocity = 0.0;
  desired_left_velocity = 0.0;
  wheel_cmd_flag_ = false;

  // Get Model object to manipulate model physics
  model = _model;
  // physics = model->GetWorld()->GetPhysicsEngine();

  // Get Joints (left and right wheel axles)
  joints[0] = model->GetJoint(_sdf->Get<std::string>("left_wheel_joint"));
  if (!joints[0])
  {
    gzerr << "Unable to find left_wheel_joint joint ["
          << _sdf->GetElement("left_wheel_joint") << "]\n";
    return;
  }

  joints[1] = model->GetJoint(
      _sdf->Get<std::string>("right_wheel_joint"));

  if (!joints[1])
  {
    gzerr << "Unable to find right_wheel_joint joint ["
          << _sdf->GetElement("right_wheel_joint") << "]\n";
    return;
  }

  // Get Elements from Plugin Call in .gazebo.xacro

  // Get Sensor Frequency (Encoders)
  sensor_frequency_ = 200; // default at 200Hz
    if (!_sdf->HasElement("sensor_frequency")) {
      ROS_WARN("TurtleDrive Plugin missing <sensor_frequency>, using default");
    } else {
      sensor_frequency_ = _sdf->GetElement("sensor_frequency")->Get<int>();
    }

  // Get Wheel Command Topic
  wheel_cmd_topic_ = "nuturtlebot/WheelCommands"; // default
    if (!_sdf->HasElement("wheel_cmd_topic")) {
      ROS_WARN("TurtleDrive Plugin missing <wheel_cmd_topic>, using default");
    } else {
      wheel_cmd_topic_ = _sdf->GetElement("wheel_cmd_topic")->Get<std::string>();
    }

  // Get Sensor Data Topic
  sensor_data_topic_ = "nuturtlebot/SensorData"; // default
    if (!_sdf->HasElement("sensor_data_topic")) {
      ROS_WARN("TurtleDrive Plugin missing <sensor_data_topic>, using default");
    } else {
      sensor_data_topic_ = _sdf->GetElement("sensor_data_topic")->Get<std::string>();
    }

  // Get Encoder Ticks Per Rev
  encoder_ticks_per_rev_ = 4096; // default at 4096
    if (!_sdf->HasElement("encoder_ticks_per_rev")) {
      ROS_WARN("TurtleDrive Plugin missing <encoder_ticks_per_rev>, using default");
    } else {
      encoder_ticks_per_rev_ = _sdf->GetElement("encoder_ticks_per_rev")->Get<int>();
    }

  // Get Max Motor Velocity in rad/s
  motor_rot_max_ = 6.35492; // default at 6.35492
    if (!_sdf->HasElement("motor_rot_max")) {
      ROS_WARN("TurtleDrive Plugin missing <motor_rot_max>, using default");
    } else {
      motor_rot_max_ = _sdf->GetElement("motor_rot_max")->Get<double>();
    }

  // Get Max Motor Power (command to send ints to ESC for output between +- 265)
  motor_pwr_max_ = 265; // default at 265
    if (!_sdf->HasElement("motor_pwr_max")) {
      ROS_WARN("TurtleDrive Plugin missing <motor_pwr_max>, using default");
    } else {
      motor_pwr_max_ = _sdf->GetElement("motor_pwr_max")->Get<int>();
    }

  // Get Max Motor Torque (stall) in Nm
  motor_torque_max_ = 1.5; // default at 1.5
    if (!_sdf->HasElement("motor_torque_max")) {
      ROS_WARN("TurtleDrive Plugin missing <motor_torque_max>, using default");
    } else {
      motor_torque_max_ = _sdf->GetElement("motor_torque_max")->Get<double>();
    }

  // Now set up wheel joints using max torque attributes
  // Note this method only accepts doubles
  joints[0]->SetParam("vel", 0, 0.0);
  joints[0]->SetParam("fmax", 0, motor_torque_max_);
  joints[1]->SetParam("vel", 0, 0.0);
  joints[1]->SetParam("fmax", 0, motor_torque_max_);

  connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&gazebo::TurtleDrivePlugin::OnUpdate, this)));

  // This is the Gazebo version of a node (see gazebo_tools_test.cpp in plen_ros package)
  node = gazebo::transport::NodePtr(new transport::Node());

  node->Init(model->GetWorld()->Name());

  // Subscribe to Wheel Cmd topic
  // WheelCmdSub = node->Subscribe<nuturtlebot::WheelCommands>(wheel_cmd_topic_, 1,
  //                                           &gazebo::TurtleDrivePlugin::wheel_cmdCallback, this);

  WheelCmdSub = nh.subscribe(wheel_cmd_topic_, 1,
                                            &gazebo::TurtleDrivePlugin::wheel_cmdCallback, this);
  // // Sensor Data Publisher
  // SensorDataPub = node->Advertise<nuturtlebot::SensorData>(sensor_data_topic_, 1);
  // SensorDataPub->WaitForConnection();
  SensorDataPub = nh.advertise<nuturtlebot::SensorData>(sensor_data_topic_, 1);

  // Update Rate Parameters
  // Initialize update rate stuff
  if ( sensor_frequency_ > 0.0 )
  {
    update_period_ = 1.0 / sensor_frequency_;
  }
  else
  {
    update_period_ = 0.0;
  }
  last_update_time_ = model->GetWorld()->SimTime();

  ROS_INFO("TurtleDrive Plugin Loaded!");
}

// Gazebo Update
void gazebo::TurtleDrivePlugin::OnUpdate()
{
  // ONLY PERFORM UPDATE IF UPDATE PERIOD HAS PASSED
  gazebo::common::Time current_time = model->GetWorld()->SimTime();
  double seconds_since_last_update = (current_time - last_update_time_ ).Double();

  if (seconds_since_last_update >= update_period_)
  {
    // Set Last Update Time
    last_update_time_ = model->GetWorld()->SimTime();


    // Get Wheel Joint Positions and Publish
    // First, convert from 0-2pi to 0-encoder_ticks_per_rev
    double m = (encoder_ticks_per_rev_) / (2.0  * rigid2d::PI);
    double b = (encoder_ticks_per_rev_ - 2.0 * rigid2d::PI * m);
    nuturtlebot::SensorData sns;
    sns.left_encoder = joints[0]->Position() * m + b;
    sns.right_encoder = joints[1]->Position() * m + b;
    SensorDataPub.publish(sns);

    // If new wheel command received, set axle velocities
    if (wheel_cmd_flag_)
    {
      joints[0]->SetParam("vel", 0, desired_left_velocity);
      joints[0]->SetParam("fmax", 0, motor_torque_max_);
      joints[1]->SetParam("vel", 0, desired_right_velocity);
      joints[1]->SetParam("fmax", 0, motor_torque_max_);
      // Reset Flag
      wheel_cmd_flag_ = false;
    }

  }
  
}

// Wheel Command Callback
void gazebo::TurtleDrivePlugin::wheel_cmdCallback(const nuturtlebot::WheelCommands & wc)
{
  // Convert wheel commands (+- 256) to velocities
  double m = (motor_rot_max_ * 2.0) / (motor_pwr_max_ * 2.0);
  double b = (motor_rot_max_ - motor_pwr_max_ * m);

  desired_left_velocity =  wc.left_velocity * m + b;
  desired_right_velocity = wc.right_velocity * m + b;

  // Now cap wheel velocities if excessive
  // Cap High
  if (desired_left_velocity > motor_rot_max_)
  {
    desired_left_velocity = motor_rot_max_;
  } else if (desired_left_velocity < - motor_rot_max_)
  {
    desired_left_velocity = - motor_rot_max_;
  }
  // Cap Low
  if (desired_right_velocity > motor_rot_max_)
  {
    desired_right_velocity = motor_rot_max_;
  } else if (desired_right_velocity < - motor_rot_max_)
  {
    desired_right_velocity = - motor_rot_max_;
  }

  // Flag to indicate we can publish new wheel commands
  wheel_cmd_flag_ = true;
}

GZ_REGISTER_MODEL_PLUGIN(gazebo::TurtleDrivePlugin)
