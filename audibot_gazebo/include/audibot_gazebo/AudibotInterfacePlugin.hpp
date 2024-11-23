#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>

using namespace gz;
using namespace sim;

namespace audibot_gazebo {

    class AudibotSpeedControl {
        public:
            AudibotSpeedControl() {}
            void reset() { int_throttle_ = 0.0; }
            void update(const double& speed_cmd, const double& speed_meas, const double& dt, double& throttle_cmd, double& brake_cmd) {
                if (!std::isfinite(speed_meas)) {
                    brake_cmd = 0.0;
                    throttle_cmd = 0.0;
                    return;
                }

                double speed_error = speed_cmd - std::abs(speed_meas);
                brake_cmd = std::clamp(-BRAKE_GAIN * speed_error, 0.0, 4000.0);
                int_throttle_ = std::clamp(int_throttle_ + dt * THROTTLE_KI * speed_error, 0.0, 0.3);
                throttle_cmd = std::clamp(THROTTLE_KP * speed_error + int_throttle_, 0.0, 1.0);
            }

        private:
            static constexpr double THROTTLE_KP = 0.2;
            static constexpr double THROTTLE_KI = 0.1;
            static constexpr double BRAKE_GAIN = 3000.0;

            double int_throttle_ = 0.0;
    };

    class AudibotInterfacePlugin
    : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
    {
        public:
            AudibotInterfacePlugin();
            ~AudibotInterfacePlugin() override;

            void Configure(const Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf, EntityComponentManager & _ecm, EventManager & _eventMgr) override;

            void PreUpdate( const UpdateInfo & _info, EntityComponentManager & _ecm) override;

            void PostUpdate(const UpdateInfo & _info, const EntityComponentManager & _ecm) override;

        private:
            // Sim interaction
            Model model_;
            Entity steer_fl_joint_;
            Entity steer_fr_joint_;
            Entity wheel_rl_joint_;
            Entity wheel_rr_joint_;
            Entity wheel_fl_joint_;
            Entity wheel_fr_joint_;
            Entity world_entity_;
            transport::Node node_;
            transport::Node::Publisher pub_twist_;
            transport::Node::Publisher pub_pose_;
            transport::Node::Publisher pub_gnss_heading_;
            static constexpr double TWIST_SAMPLE_TIME = 0.01;
            static constexpr double GNSS_HEADING_SAMPLE_TIME = 0.02;

            // Persistent internal values
            uint64_t twist_pub_stamp_ = 0;
            uint64_t heading_pub_stamp_ = 0;
            double current_bicycle_angle_ = 0.0;
            bool first_update_ = true;
            uint64_t current_time_ = 0;
            bool rollover_ = false;
            math::Pose3d last_vehicle_pose_;
            double current_speed_ = 0.0;
            msgs::Pose_V posev_msg_;
            bool publish_ground_truth_pose_;
            bool publish_gnss_heading_;
            double world_heading_offset_;

            // Kinematics parameters
            static constexpr double AUDIBOT_STEERING_RATIO =      17.3;  // Ratio between steering wheel angle and tire angle
            static constexpr double AUDIBOT_LOCK_TO_LOCK_REVS =   3.2;   // Number of steering wheel turns to go from lock to lock
            static constexpr double AUDIBOT_MAX_STEER_ANGLE =     (M_PI * AUDIBOT_LOCK_TO_LOCK_REVS / AUDIBOT_STEERING_RATIO);
            static constexpr double AUDIBOT_WHEELBASE =           2.67;  // Distance between front and rear axles
            static constexpr double AUDIBOT_TRACK_WIDTH =         1.638; // Distance between front wheels
            static constexpr double AUDIBOT_MAX_STEER_RATE =      800.0 * M_PI / 180.0 / AUDIBOT_STEERING_RATIO;
            static constexpr double AUDIBOT_MAX_SPEED =           130.0 * 0.44704;

            // Drag parameters
            static constexpr double ROLLING_RESISTANCE_COEFF =  0.01;
            static constexpr double AERO_DRAG_COEFF =           0.35;
            static constexpr double GRAVITY_ACCEL =             9.81;
            static constexpr double VEHICLE_MASS =              1700.0;
            static constexpr double WHEEL_RADIUS =              0.36;
            static constexpr double MAX_BRAKE_TORQUE =          5000.0;

            // Longitudinal control inputs
            uint64_t speed_cmd_stamp_ = 0;
            uint64_t brake_cmd_stamp_ = 0;
            uint64_t throttle_cmd_stamp_ = 0;
            double speed_cmd_ = 0.0;
            double brake_cmd_ = 0.0;
            double throttle_cmd_ = 0.0;
            AudibotSpeedControl speed_control_;
            void recvThrottleCmd(const gz::msgs::Double& msg);
            void recvBrakeCmd(const gz::msgs::Double& msg);
            void recvSpeedCmd(const gz::msgs::Double& msg);

            // Steering wheel angle command
            uint64_t steering_cmd_stamp_ = 0;
            double target_bicycle_angle_ = 0.0;
            void recvSteeringCmd(const gz::msgs::Double& msg);

            // Gear shift command
            enum class AudibotGear : uint8_t {
                DRIVE = 0,
                REVERSE = 1
            };
            AudibotGear gear_cmd_ = AudibotGear::DRIVE;
            void recvGearCmd(const gz::msgs::UInt32& msg);

            bool isTimeout(const uint64_t& stamp);
    };

}