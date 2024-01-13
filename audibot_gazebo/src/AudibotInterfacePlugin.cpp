#include <audibot_gazebo/AudibotInterfacePlugin.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>

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

  //Have to use rclcpp::get_logger("AudibotInterfacePlugin") as the node handle is not yet initialized
  if(!rclcpp::ok()){
    RCLCPP_FATAL(rclcpp::get_logger("AudibotInterfacePlugin"), "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package");
  }


  world_ = model->GetWorld();
  RCLCPP_INFO(rclcpp::get_logger("AudibotInterfacePlugin"), "The audibot plugin is loading!");

  // Gazebo initialization
  steer_fl_joint_ = model->GetJoint("steer_fl_joint");
  steer_fr_joint_ = model->GetJoint("steer_fr_joint");
  wheel_rl_joint_ = model->GetJoint("wheel_rl_joint");
  wheel_rr_joint_ = model->GetJoint("wheel_rr_joint");
  wheel_fl_joint_ = model->GetJoint("wheel_fl_joint");
  wheel_fr_joint_ = model->GetJoint("wheel_fr_joint");
  footprint_link_ = model->GetLink("base_footprint");

  // Load SDF parameters
  pub_tf_ = sdf->Get<bool>("pub_tf", false).first;
  robot_name_ = sdf->Get<std::string>("robot_name", "").first;
  tf_freq_ = std::max(1.0, sdf->Get<double>("tf_freq", 100.0).first);
  tf_timer_thres_ = (int)(1e3 / tf_freq_);

  model_name_ = model->GetName().substr(0, model->GetName().find("::"));

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AudibotInterfacePlugin::Update, this));

  steer_fl_joint_->SetParam("fmax", 0, 99999.0);
  steer_fr_joint_->SetParam("fmax", 0, 99999.0);

  // ROS initialization
  node_handle_ = std::make_shared<rclcpp::Node>("control", model_name_);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

  sub_steering_cmd_ = node_handle_->create_subscription<std_msgs::msg::Float64>("steering_cmd", 1, std::bind(&AudibotInterfacePlugin::recvSteeringCmd, this, std::placeholders::_1));
  sub_brake_cmd_ = node_handle_->create_subscription<std_msgs::msg::Float64>("brake_cmd", 1, std::bind(&AudibotInterfacePlugin::recvBrakeCmd, this, std::placeholders::_1));
  sub_throttle_cmd_ = node_handle_->create_subscription<std_msgs::msg::Float64>("throttle_cmd", 1, std::bind(&AudibotInterfacePlugin::recvThrottleCmd, this, std::placeholders::_1));
  sub_gear_cmd_ = node_handle_->create_subscription<std_msgs::msg::UInt8>("gear_cmd", 1, std::bind(&AudibotInterfacePlugin::recvGearCmd, this, std::placeholders::_1));

  pub_twist_ = node_handle_->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 1);
  pub_gear_state_ = node_handle_->create_publisher<std_msgs::msg::UInt8>("gear_state", 1);
  pub_odom_= node_handle_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  pub_steering_ = node_handle_->create_publisher<std_msgs::msg::Float64>("steering_state", 1);

  feedback_timer_count_ = 0;
  tf_timer_count_ = 0;

  if (robot_name_.empty()) {
    frame_id_ = footprint_link_->GetName();
  } else {
    frame_id_ = robot_name_ + "/" + footprint_link_->GetName();
  }

  executor_->add_node(node_handle_);
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AudibotInterfacePlugin::Update, this));

  RCLCPP_INFO(node_handle_->get_logger(), "The audiobot plugin finished loading!");

}

void AudibotInterfacePlugin::Update() {

  common::Time sim_time = world_->SimTime();
  double dt = (sim_time - last_time).Double();
  if (dt == 0.0) return;

  executor_->spin_some(std::chrono::milliseconds(100));
  twistStateUpdate();
  driveUpdate();
  steeringUpdate(dt);
  dragUpdate();

  if (tf_timer_count_++ >= tf_timer_thres_) {
    tf_timer_count_ = 0;
    tfTimerCallback();
  }

  if (feedback_timer_count_++ >= 20) {
    feedback_timer_count_ = 0;
    feedbackTimerCallback();
  }

  // save last time stamp
  last_time = sim_time;
}

void AudibotInterfacePlugin::twistStateUpdate() {
  world_pose_ = footprint_link_->WorldPose();
  twist_.linear.x = footprint_link_->RelativeLinearVel().X();
  twist_.angular.z = footprint_link_->RelativeAngularVel().Z();
  rollover_ = (fabs(world_pose_.Rot().X()) > 0.2 || fabs(world_pose_.Rot().Y()) > 0.2);
}

void AudibotInterfacePlugin::driveUpdate() {
  // Stop wheels if vehicle is rolled over
  if (rollover_) {
    stopWheels();
    return;
  }

  // Brakes have precedence over throttle
  if ((brake_cmd_ > 0) && ((last_time - brake_stamp_).Double() < 0.25)) {
    double brake_torque_factor = 1.0;
    if (twist_.linear.x < -0.1) {
      brake_torque_factor = -1.0;
    } else if (twist_.linear.x < 0.1) {
      brake_torque_factor = 1.0 + (twist_.linear.x - 0.1) / 0.1;
    }

    setAllWheelTorque(-brake_torque_factor * brake_cmd_);
  } else {
    if ((last_time - throttle_stamp_).Double() < 0.25) {
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

void AudibotInterfacePlugin::steeringUpdate(double time_step) {
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

  steer_fl_joint_->SetParam("vel", 0, 100.0 * (left_steer - steer_fl_joint_->Position(0)));
  steer_fr_joint_->SetParam("vel", 0, 100.0 * (right_steer - steer_fr_joint_->Position(0)));
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

void AudibotInterfacePlugin::recvSteeringCmd(const std_msgs::msg::Float64::ConstSharedPtr msg) {
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

void AudibotInterfacePlugin::recvBrakeCmd(const std_msgs::msg::Float64::ConstSharedPtr msg) {
  brake_cmd_ = msg->data;
  if (brake_cmd_ < 0) {
    brake_cmd_ = 0;
  } else if (brake_cmd_ > MAX_BRAKE_TORQUE) {
    brake_cmd_ = MAX_BRAKE_TORQUE;
  }
  brake_stamp_ = last_time;
}

void AudibotInterfacePlugin::recvThrottleCmd(const std_msgs::msg::Float64::ConstSharedPtr msg) {
  throttle_cmd_ = msg->data;
  if (throttle_cmd_ < 0.0) {
    throttle_cmd_ = 0.0;
  } else if (throttle_cmd_ > 1.0) {
    throttle_cmd_ = 1.0;
  }
  throttle_stamp_ = last_time;
}

void AudibotInterfacePlugin::recvGearCmd(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  if (msg->data > REVERSE) {
    RCLCPP_WARN(node_handle_->get_logger(), "Invalid gear command received [%u]", msg->data);
  } else {
    gear_cmd_ = msg->data;
  }
}

void AudibotInterfacePlugin::feedbackTimerCallback() {
  auto current_ros_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(last_time);
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.frame_id = frame_id_;
  twist_msg.header.stamp = current_ros_time;
  twist_msg.twist = twist_;
  pub_twist_->publish(twist_msg);

  std_msgs::msg::UInt8 gear_state_msg;
  gear_state_msg.data = gear_cmd_;
  pub_gear_state_->publish(gear_state_msg);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.header.stamp = current_ros_time;
  odom_msg.twist.twist = twist_;
  odom_msg.pose.pose.position.x = world_pose_.Pos().X();
  odom_msg.pose.pose.position.y = world_pose_.Pos().Y();
  odom_msg.pose.pose.position.z = world_pose_.Pos().Z();
  odom_msg.pose.pose.orientation.x = world_pose_.Rot().X();
  odom_msg.pose.pose.orientation.y = world_pose_.Rot().Y();
  odom_msg.pose.pose.orientation.z = world_pose_.Rot().Z();
  odom_msg.pose.pose.orientation.w = world_pose_.Rot().W();
  pub_odom_->publish(odom_msg);

  std_msgs::msg::Float64 steering;
  steering.data = AUDIBOT_STEERING_RATIO * current_steering_angle_;
  pub_steering_->publish(steering);
}

void AudibotInterfacePlugin::tfTimerCallback() {
  auto current_ros_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(last_time);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = frame_id_;
  t.header.stamp = current_ros_time;
  t.transform.translation.x = world_pose_.Pos().X();
  t.transform.translation.y = world_pose_.Pos().Y();
  t.transform.translation.z = world_pose_.Pos().Z();
  t.transform.rotation.w = world_pose_.Rot().W();
  t.transform.rotation.x = world_pose_.Rot().X();
  t.transform.rotation.y = world_pose_.Rot().Y();
  t.transform.rotation.z = world_pose_.Rot().Z();
  tf_broadcaster_->sendTransform(t);
}

void AudibotInterfacePlugin::Reset() {}

AudibotInterfacePlugin::~AudibotInterfacePlugin() {}

} // namespace gazebo
