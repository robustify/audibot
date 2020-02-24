#include <audibot_gazebo/AudibotInterfacePlugin.h>

namespace gazebo {

AudibotInterfacePlugin::AudibotInterfacePlugin() {
  target_angle_ = 0.0;
  brake_cmd_ = 0.0;
  throttle_cmd_ = 0.0;
  gear_cmd_ = DRIVE;
  current_steering_angle_ = 0.0;
  rollover_ = false;
}

void AudibotInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  // Gazebo initialization
  steer_fl_joint_ = model->GetJoint("steer_fl");
  steer_fr_joint_ = model->GetJoint("steer_fr");
  wheel_rl_joint_ = model->GetJoint("wheel_rl");
  wheel_rr_joint_ = model->GetJoint("wheel_rr");
  wheel_fl_joint_ = model->GetJoint("wheel_fl");
  wheel_fr_joint_ = model->GetJoint("wheel_fr");
  footprint_link_ = model->GetLink("base_footprint");

  // Load SDF parameters
  if (sdf->HasElement("pubTf")) {
    sdf->GetElement("pubTf")->GetValue()->Get(pub_tf_);
  } else {
    pub_tf_ = false;
  }

  if (sdf->HasElement("robotName")) {
    sdf::ParamPtr sdf_robot_name = sdf->GetElement("robotName")->GetValue();
    if (sdf_robot_name) {
      sdf_robot_name->Get(robot_name_);
    } else {
      robot_name_ = std::string("");
    }
  } else {
    robot_name_ = std::string("");
  }

  if (sdf->HasElement("pubTf")) {
    sdf->GetElement("pubTf")->GetValue()->Get(pub_tf_);
  } else {
    pub_tf_ = false;
  }

  if (sdf->HasElement("tfFreq")) {
    sdf->GetElement("tfFreq")->GetValue()->Get(tf_freq_);
  } else {
    tf_freq_ = 100.0;
  }

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AudibotInterfacePlugin::OnUpdate, this, _1));

  steer_fl_joint_->SetParam("fmax", 0, 99999.0);
  steer_fr_joint_->SetParam("fmax", 0, 99999.0);

  // ROS initialization
  n_ = new ros::NodeHandle(robot_name_);

  sub_steering_cmd_ = n_->subscribe("steering_cmd", 1, &AudibotInterfacePlugin::recvSteeringCmd, this);
  sub_brake_cmd_ = n_->subscribe("brake_cmd", 1, &AudibotInterfacePlugin::recvBrakeCmd, this);
  sub_throttle_cmd_ = n_->subscribe("throttle_cmd", 1, &AudibotInterfacePlugin::recvThrottleCmd, this);
  sub_gear_cmd_ = n_->subscribe("gear_cmd", 1, &AudibotInterfacePlugin::recvGearCmd, this);

  pub_twist_ = n_->advertise<geometry_msgs::TwistStamped> ("twist", 1);
  pub_gear_state_ = n_->advertise<std_msgs::UInt8> ("gear_state", 1);
  feedback_timer_ = n_->createTimer(ros::Duration(0.02), &AudibotInterfacePlugin::feedbackTimerCallback, this);

  if (pub_tf_) {
    tf_timer_ = n_->createTimer(ros::Duration(1.0 / tf_freq_), &AudibotInterfacePlugin::tfTimerCallback, this);
  }
}

void AudibotInterfacePlugin::OnUpdate(const common::UpdateInfo& info) {
  if (last_update_time_ == common::Time(0)) {
    last_update_time_ = info.simTime;
    return;
  }

  twistStateUpdate();
  driveUpdate();
  steeringUpdate(info);
  dragUpdate();
}

void AudibotInterfacePlugin::twistStateUpdate() {
#if GAZEBO_MAJOR_VERSION >= 9
  world_pose_ = footprint_link_->WorldPose();
  twist_.linear.x = footprint_link_->RelativeLinearVel().X();
  twist_.angular.z = footprint_link_->RelativeAngularVel().Z();
  rollover_ = (fabs(world_pose_.Rot().X()) > 0.2 || fabs(world_pose_.Rot().Y()) > 0.2);
#else
  world_pose_ = footprint_link_->GetWorldPose();
  twist_.linear.x = footprint_link_->GetRelativeLinearVel().x;
  twist_.angular.z = footprint_link_->GetRelativeAngularVel().z;
  rollover_ = (fabs(world_pose_.rot.x) > 0.2 || fabs(world_pose_.rot.y) > 0.2);
#endif
}

void AudibotInterfacePlugin::driveUpdate() {
  // Stop wheels if vehicle is rolled over
  if (rollover_) {
    stopWheels();
    return;
  }

  // Brakes have precedence over throttle
  ros::Time current_stamp = ros::Time::now();
  if ((brake_cmd_ > 0) && ((current_stamp - brake_stamp_).toSec() < 0.25)) {
    double brake_torque_factor = 1.0;
    if (twist_.linear.x < -0.1) {
      brake_torque_factor = -1.0;
    } else if (twist_.linear.x < 0.1) {
      brake_torque_factor = 1.0 + (twist_.linear.x - 0.1) / 0.1;
    }

    setAllWheelTorque(-brake_torque_factor * brake_cmd_);
  } else {
    if ((current_stamp - throttle_stamp_).toSec() < 0.25) {
      double throttle_torque;
      if (gear_cmd_ == DRIVE) {
        throttle_torque = throttle_cmd_ * 4000.0 - 40.1 * twist_.linear.x;
        if (throttle_torque < 0.0) {
          throttle_torque = 0.0;
        }
      } else { // REVERSE
        throttle_torque = -throttle_cmd_ * 4000.0 - 250.0 * twist_.linear.x;
        if (throttle_torque > 0.0) {
          throttle_torque = 0.0;
        }
      }
      setRearWheelTorque(throttle_torque);
    }
  }
}

void AudibotInterfacePlugin::steeringUpdate(const common::UpdateInfo& info) {
  double time_step = (info.simTime - last_update_time_).Double();
  last_update_time_ = info.simTime;

  // Arbitrarily set maximum steering rate to 800 deg/s
  const double max_rate = 800.0 * M_PI / 180.0 / AUDIBOT_STEERING_RATIO;
  double max_inc = time_step * max_rate;

  if ((target_angle_ - current_steering_angle_) > max_inc) {
    current_steering_angle_ += max_inc;
  } else if ((target_angle_ - current_steering_angle_) < -max_inc) {
    current_steering_angle_ -= max_inc;
  }

  // Compute Ackermann steering angles for each wheel
  double t_alph = tan(current_steering_angle_);
  double left_steer = atan(AUDIBOT_WHEELBASE * t_alph / (AUDIBOT_WHEELBASE - 0.5 * AUDIBOT_TRACK_WIDTH * t_alph));
  double right_steer = atan(AUDIBOT_WHEELBASE * t_alph / (AUDIBOT_WHEELBASE + 0.5 * AUDIBOT_TRACK_WIDTH * t_alph));

#if GAZEBO_MAJOR_VERSION >= 9
  steer_fl_joint_->SetParam("vel", 0, 100.0 * (left_steer - steer_fl_joint_->Position(0)));
  steer_fr_joint_->SetParam("vel", 0, 100.0 * (right_steer - steer_fr_joint_->Position(0)));
#else
  steer_fl_joint_->SetParam("vel", 0, 100.0 * (left_steer - steer_fl_joint_->GetAngle(0).Radian()));
  steer_fr_joint_->SetParam("vel", 0, 100.0 * (right_steer - steer_fr_joint_->GetAngle(0).Radian()));
#endif
}

void AudibotInterfacePlugin::dragUpdate() {
  // Apply rolling resistance and aerodynamic drag forces
  double rolling_resistance_torque = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS * GRAVITY_ACCEL;
  double drag_force = AERO_DRAG_COEFF * twist_.linear.x * twist_.linear.x;
  double drag_torque = drag_force * WHEEL_RADIUS; // Implement aerodynamic drag as a torque disturbance

  if (twist_.linear.x > 0.0) {
    setAllWheelTorque(-rolling_resistance_torque);
    setAllWheelTorque(-drag_torque);
  } else {
    setAllWheelTorque(rolling_resistance_torque);
    setAllWheelTorque(drag_torque);
  }
}

void AudibotInterfacePlugin::setAllWheelTorque(double torque) {
  wheel_rl_joint_->SetForce(0, 0.25 * torque);
  wheel_rr_joint_->SetForce(0, 0.25 * torque);
  wheel_fl_joint_->SetForce(0, 0.25 * torque);
  wheel_fr_joint_->SetForce(0, 0.25 * torque);
}

void AudibotInterfacePlugin::setRearWheelTorque(double torque) {
  wheel_rl_joint_->SetForce(0, 0.5 * torque);
  wheel_rr_joint_->SetForce(0, 0.5 * torque);
}

void AudibotInterfacePlugin::stopWheels() {
  wheel_fl_joint_->SetForce(0, -1000.0 * wheel_fl_joint_->GetVelocity(0));
  wheel_fr_joint_->SetForce(0, -1000.0 * wheel_fr_joint_->GetVelocity(0));
  wheel_rl_joint_->SetForce(0, -1000.0 * wheel_rl_joint_->GetVelocity(0));
  wheel_rr_joint_->SetForce(0, -1000.0 * wheel_rr_joint_->GetVelocity(0));
}

void AudibotInterfacePlugin::recvSteeringCmd(const std_msgs::Float64ConstPtr& msg) {
  if (!std::isfinite(msg->data)) {
    target_angle_ = 0.0;
    return;
  }

  target_angle_ = msg->data / AUDIBOT_STEERING_RATIO;
  if (target_angle_ > AUDIBOT_MAX_STEER_ANGLE) {
    target_angle_ = AUDIBOT_MAX_STEER_ANGLE;
  } else if (target_angle_ < -AUDIBOT_MAX_STEER_ANGLE) {
    target_angle_ = -AUDIBOT_MAX_STEER_ANGLE;
  }
}

void AudibotInterfacePlugin::recvBrakeCmd(const std_msgs::Float64ConstPtr& msg) {
  brake_cmd_ = msg->data;
  if (brake_cmd_ < 0) {
    brake_cmd_ = 0;
  } else if (brake_cmd_ > MAX_BRAKE_TORQUE) {
    brake_cmd_ = MAX_BRAKE_TORQUE;
  }
  brake_stamp_ = ros::Time::now();
}

void AudibotInterfacePlugin::recvThrottleCmd(const std_msgs::Float64ConstPtr& msg) {
  throttle_cmd_ = msg->data;
  if (throttle_cmd_ < 0.0) {
    throttle_cmd_ = 0.0;
  } else if (throttle_cmd_ > 1.0) {
    throttle_cmd_ = 1.0;
  }
  throttle_stamp_ = ros::Time::now();
}

void AudibotInterfacePlugin::recvGearCmd(const std_msgs::UInt8ConstPtr& msg) {
  if (msg->data > REVERSE) {
    ROS_WARN("Invalid gear command received [%u]", msg->data);
  } else {
    gear_cmd_ = msg->data;
  }
}

void AudibotInterfacePlugin::feedbackTimerCallback(const ros::TimerEvent& event) {
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.frame_id = tf::resolve(robot_name_, footprint_link_->GetName());
  twist_msg.header.stamp = event.current_real;
  twist_msg.twist = twist_;
  pub_twist_.publish(twist_msg);

  std_msgs::UInt8 gear_state_msg;
  gear_state_msg.data = gear_cmd_;
  pub_gear_state_.publish(gear_state_msg);
}

void AudibotInterfacePlugin::tfTimerCallback(const ros::TimerEvent& event) {
  tf::StampedTransform t;
  t.frame_id_ = "world";
  t.child_frame_id_ = tf::resolve(robot_name_, footprint_link_->GetName());
  t.stamp_ = event.current_real;
#if GAZEBO_MAJOR_VERSION >= 9
  t.setOrigin(tf::Vector3(world_pose_.Pos().X(), world_pose_.Pos().Y(), world_pose_.Pos().Z()));
  t.setRotation(tf::Quaternion(world_pose_.Rot().X(), world_pose_.Rot().Y(), world_pose_.Rot().Z(), world_pose_.Rot().W()));
#else
  t.setOrigin(tf::Vector3(world_pose_.pos.x, world_pose_.pos.y, world_pose_.pos.z));
  t.setRotation(tf::Quaternion(world_pose_.rot.x, world_pose_.rot.y, world_pose_.rot.z, world_pose_.rot.w));
#endif
  br_.sendTransform(t);
}

void AudibotInterfacePlugin::Reset() {
}

AudibotInterfacePlugin::~AudibotInterfacePlugin() {
  n_->shutdown();
  delete n_;
}


}
