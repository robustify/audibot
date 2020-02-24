#ifndef AUDIBOTINTERFACEPLUGIN_H
#define AUDIBOTINTERFACEPLUGIN_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/TwistStamped.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

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
  virtual void Reset();

private:
  void feedbackTimerCallback(const ros::TimerEvent& event);
  void tfTimerCallback(const ros::TimerEvent& event);
  void OnUpdate(const common::UpdateInfo& info);
  void recvSteeringCmd(const std_msgs::Float64ConstPtr& msg);
  void recvThrottleCmd(const std_msgs::Float64ConstPtr& msg);
  void recvBrakeCmd(const std_msgs::Float64ConstPtr& msg);
  void recvGearCmd(const std_msgs::UInt8ConstPtr& msg);
  void twistStateUpdate();
  void driveUpdate();
  void steeringUpdate(const common::UpdateInfo& info);
  void dragUpdate();
  void stopWheels();
  void setAllWheelTorque(double torque);
  void setRearWheelTorque(double torque);

  ros::NodeHandle* n_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_gear_state_;
  ros::Subscriber sub_steering_cmd_;
  ros::Subscriber sub_throttle_cmd_;
  ros::Subscriber sub_brake_cmd_;
  ros::Subscriber sub_gear_cmd_;
  ros::Subscriber sub_model_states_;
  ros::Timer feedback_timer_;
  ros::Timer tf_timer_;

  tf::TransformBroadcaster br_;
  geometry_msgs::Twist twist_;
  bool rollover_;
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d world_pose_;
#else
  gazebo::math::Pose world_pose_;
#endif
  event::ConnectionPtr update_connection_;
  physics::JointPtr steer_fl_joint_;
  physics::JointPtr steer_fr_joint_;
  physics::JointPtr wheel_rl_joint_;
  physics::JointPtr wheel_rr_joint_;
  physics::JointPtr wheel_fl_joint_;
  physics::JointPtr wheel_fr_joint_;
  physics::LinkPtr footprint_link_;
  common::Time last_update_time_;

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
  ros::Time brake_stamp_;

  // Throttle
  double throttle_cmd_;
  ros::Time throttle_stamp_;

  // Gear
  uint8_t gear_cmd_;
};

GZ_REGISTER_MODEL_PLUGIN(AudibotInterfacePlugin)

}

#endif // AUDIBOTINTERFACEPLUGIN_H
