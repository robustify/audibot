#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::placeholders;

namespace gazebo {

// Kinematics parameters
#define AUDIBOT_STEERING_RATIO      17.3  // Ratio between steering wheel angle and tire angle
#define AUDIBOT_LOCK_TO_LOCK_REVS   3.2   // Number of steering wheel turns to go from lock to lock
#define AUDIBOT_MAX_STEER_ANGLE     (M_PI * AUDIBOT_LOCK_TO_LOCK_REVS / AUDIBOT_STEERING_RATIO)
#define AUDIBOT_WHEELBASE           2.65  // Distance between front and rear axles
#define AUDIBOT_TRACK_WIDTH         1.638 // Distance between front wheels

// Drag parameters
#define ROLLING_RESISTANCE_COEFF  0.01
#define AERO_DRAG_COEFF           0.35
#define GRAVITY_ACCEL             9.81
#define VEHICLE_MASS              1700.0
#define WHEEL_RADIUS              0.36
#define MAX_BRAKE_TORQUE          8000.0

// Gear states
enum { DRIVE = 0, REVERSE = 1 };

class AudibotInterfacePlugin : public ModelPlugin {
public:
  AudibotInterfacePlugin();
  virtual ~AudibotInterfacePlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  //virtual void LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf);  
  virtual void Update();
  // void UpdateDynamics(double dt);
  // void UpdateState(double dt);  
  virtual void Reset();

private:
  void feedbackTimerCallback();
  void tfTimerCallback();
  void recvSteeringCmd(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void recvThrottleCmd(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void recvBrakeCmd(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void recvGearCmd(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void twistStateUpdate();
  void driveUpdate();
  void steeringUpdate(double time_step);
  void dragUpdate();
  void stopWheels();
  void setAllWheelTorque(double torque);
  void setRearWheelTorque(double torque);

  //gazebo_ros::Node::SharedPtr ros_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_handle_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gear_state_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_steering_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_steering_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_throttle_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_brake_cmd_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_cmd_;
  // ros::Subscriber sub_model_states_;
  int feedback_timer_count_;
  int tf_timer_count_;
  int tf_timer_thres_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::Twist twist_;
  bool rollover_;
  ignition::math::Pose3d world_pose_;
  event::ConnectionPtr update_connection_;
  
  /// \brief The parent World
  physics::WorldPtr world_;

  physics::JointPtr steer_fl_joint_;
  physics::JointPtr steer_fr_joint_;
  physics::JointPtr wheel_rl_joint_;
  physics::JointPtr wheel_rr_joint_;
  physics::JointPtr wheel_fl_joint_;
  physics::JointPtr wheel_fr_joint_;
  physics::LinkPtr footprint_link_;
  std::string frame_id_;

  /// \brief save last_time
  common::Time last_time;

  std::string model_name_;

  // SDF parameters
  std::string robot_name_;
  bool pub_tf_;
  double tf_freq_;

  // Steering values
  double right_angle_;
  double left_angle_;
  double target_angle_;
  double current_steering_angle_;

  // Brakes
  double brake_cmd_;
  common::Time brake_stamp_;

  // Throttle
  double throttle_cmd_;
  common::Time throttle_stamp_;

  // Gear
  uint8_t gear_cmd_;
};

GZ_REGISTER_MODEL_PLUGIN(AudibotInterfacePlugin)

}
