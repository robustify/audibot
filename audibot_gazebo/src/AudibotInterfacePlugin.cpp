#include <audibot_gazebo/AudibotInterfacePlugin.hpp>

#include "gz/sim/World.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointEffortLimitsCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/World.hh>

using namespace gz;
using namespace gz::sim;
using namespace systems;

namespace audibot_gazebo {
    math::Pose3d worldPose(const Entity &_entity, const EntityComponentManager &_ecm) {
        auto poseComp = _ecm.Component<components::Pose>(_entity);
        if (nullptr == poseComp) {
            gzwarn << "Trying to get world pose from entity [" << _entity
                    << "], which doesn't have a pose component" << std::endl;
            return math::Pose3d();
        }

        // work out pose in world frame
        math::Pose3d pose = poseComp->Data();
        auto p = _ecm.Component<components::ParentEntity>(_entity);
        while (p) {
            // get pose of parent entity
            auto parentPose = _ecm.Component<components::Pose>(p->Data());
            if (!parentPose)
                break;
            // transform pose
            pose = parentPose->Data() * pose;
            // keep going up the tree
            p = _ecm.Component<components::ParentEntity>(p->Data());
        }
        return pose;
    }

    AudibotInterfacePlugin::AudibotInterfacePlugin() {}
    AudibotInterfacePlugin::~AudibotInterfacePlugin() {}

    void AudibotInterfacePlugin::Configure(const Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          EntityComponentManager &_ecm,
                          EventManager &/*_eventMgr*/)
    {
        this->model_ = Model(_entity);
        if (!this->model_.Valid(_ecm)) {
            gzerr << "Audibot plugin should be attached to a model entity\n";
            return;
        }

        this->publish_ground_truth_pose_ = _sdf->Get<bool>("pub_tf", true).first;
        this->publish_gnss_heading_ = _sdf->Get<bool>("pub_heading", true).first;

        this->steer_fl_joint_ = this->model_.JointByName(_ecm, "steer_fl_joint");
        this->steer_fr_joint_ = this->model_.JointByName(_ecm, "steer_fr_joint");
        this->wheel_rl_joint_ = this->model_.JointByName(_ecm, "wheel_rl_joint");
        this->wheel_rr_joint_ = this->model_.JointByName(_ecm, "wheel_rr_joint");
        this->wheel_fl_joint_ = this->model_.JointByName(_ecm, "wheel_fl_joint");
        this->wheel_fr_joint_ = this->model_.JointByName(_ecm, "wheel_fr_joint");

        this->node_.Subscribe("/model/" + model_.Name(_ecm) + "/throttle_cmd", &AudibotInterfacePlugin::recvThrottleCmd, this);
        this->node_.Subscribe("/model/" + model_.Name(_ecm) + "/brake_cmd", &AudibotInterfacePlugin::recvBrakeCmd, this);
        this->node_.Subscribe("/model/" + model_.Name(_ecm) + "/speed_cmd", &AudibotInterfacePlugin::recvSpeedCmd, this);
        this->node_.Subscribe("/model/" + model_.Name(_ecm) + "/steering_cmd", &AudibotInterfacePlugin::recvSteeringCmd, this);
        this->node_.Subscribe("/model/" + model_.Name(_ecm) + "/gear_cmd", &AudibotInterfacePlugin::recvGearCmd, this);
        this->pub_twist_ = this->node_.Advertise<msgs::Twist>("/model/" + model_.Name(_ecm) + "/twist");

        if (this->publish_ground_truth_pose_) {
            this->pub_pose_ = this->node_.Advertise<msgs::Pose_V>("/model/" + model_.Name(_ecm) + "/pose");
        }

        if (this->publish_gnss_heading_) {
            pub_gnss_heading_ = this->node_.Advertise<msgs::Double>("/model/" + model_.Name(_ecm) + "/gnss_heading");
        }

        _ecm.SetComponentData<components::JointEffortLimitsCmd>(this->steer_fl_joint_, {gz::math::Vector2(-1e6, 1e6)});
        _ecm.SetComponentData<components::JointEffortLimitsCmd>(this->steer_fr_joint_, {gz::math::Vector2(-1e6, 1e6)});

        this->world_entity_ = _ecm.EntityByComponents(components::World());
        gz::sim::World world(this->world_entity_);
        if (world.SphericalCoordinates(_ecm)) {
            auto sphericalCoordinates = world.SphericalCoordinates(_ecm).value();
            this->world_heading_offset_ = sphericalCoordinates.HeadingOffset().Radian();
        }
    }

    void AudibotInterfacePlugin::recvThrottleCmd(const gz::msgs::Double& msg) {
        this->throttle_cmd_stamp_ = this->current_time_;
        if (std::isfinite(msg.data())) {
            throttle_cmd_ = std::clamp(msg.data(), 0.0, 1.0);
        } else {
            throttle_cmd_ = 0.0;
        }
    }

    void AudibotInterfacePlugin::recvBrakeCmd(const gz::msgs::Double& msg) {
        this->brake_cmd_stamp_ = this->current_time_;
        if (std::isfinite(msg.data())) {
            brake_cmd_ = std::clamp(msg.data(), 0.0, MAX_BRAKE_TORQUE);
        } else {
            brake_cmd_ = 0.0;
        }
    }

    void AudibotInterfacePlugin::recvSpeedCmd(const gz::msgs::Double& msg) {
        this->speed_cmd_stamp_ = this->current_time_;
        if (std::isfinite(msg.data())) {
            speed_cmd_ = std::clamp(msg.data(), 0.0, AUDIBOT_MAX_SPEED);
        } else {
            speed_cmd_ = 0.0;
        }
    }

    void AudibotInterfacePlugin::recvSteeringCmd(const gz::msgs::Double& msg) {
        this->steering_cmd_stamp_ = this->current_time_;
        if (std::isfinite(msg.data())) {
            target_bicycle_angle_ = std::clamp(msg.data() / AUDIBOT_STEERING_RATIO, -AUDIBOT_MAX_STEER_ANGLE, AUDIBOT_MAX_STEER_ANGLE);
        } else {
            target_bicycle_angle_ = 0.0;
        }
    }

    void AudibotInterfacePlugin::recvGearCmd(const gz::msgs::UInt32& msg) {
        switch (msg.data()) {
            case 0:
                this->gear_cmd_ = AudibotGear::DRIVE;
            break;
            case 1:
                this->gear_cmd_ = AudibotGear::REVERSE;
            break;
            default:
            gzwarn << "Gear shift ignored: unsupported gear enum " << (int)msg.data() << " (0 = DRIVE, 1 = REVERSE)" << std::endl;
            break;
        }
    }

    bool AudibotInterfacePlugin::isTimeout(const uint64_t& stamp) {
        return (1e-9 * (this->current_time_ - stamp)) > 0.1;
    }

    void AudibotInterfacePlugin::PreUpdate(const UpdateInfo & _info, EntityComponentManager & _ecm) {
        this->current_time_ = _info.realTime.count();
        bool wheel_speeds_valid = true;
        bool steering_positions_valid = true;

        auto rl_joint_vel = _ecm.Component<components::JointVelocity>(this->wheel_rl_joint_);
        if (!rl_joint_vel) {
            _ecm.CreateComponent(this->wheel_rl_joint_, components::JointVelocity());
            wheel_speeds_valid = false;
        }
        auto rr_joint_vel = _ecm.Component<components::JointVelocity>(this->wheel_rr_joint_);
        if (!rr_joint_vel) {
            _ecm.CreateComponent(this->wheel_rr_joint_, components::JointVelocity());
            wheel_speeds_valid = false;
        }
        auto fl_joint_vel = _ecm.Component<components::JointVelocity>(this->wheel_fl_joint_);
        if (!fl_joint_vel) {
            _ecm.CreateComponent(this->wheel_fl_joint_, components::JointVelocity());
            wheel_speeds_valid = false;
        }
        auto fr_joint_vel = _ecm.Component<components::JointVelocity>(this->wheel_fr_joint_);
        if (!fr_joint_vel) {
            _ecm.CreateComponent(this->wheel_fr_joint_, components::JointVelocity());
            wheel_speeds_valid = false;
        }
        auto fl_steer_pos = _ecm.Component<components::JointPosition>(this->steer_fl_joint_);
        if (!fl_steer_pos) {
            _ecm.CreateComponent(this->steer_fl_joint_, components::JointPosition());
            steering_positions_valid = false;
        }
        auto fr_steer_pos = _ecm.Component<components::JointPosition>(this->steer_fr_joint_);
        if (!fr_steer_pos) {
            _ecm.CreateComponent(this->steer_fr_joint_, components::JointPosition());
            steering_positions_valid = false;
        }

        if (_info.dt.count() == 0 || !wheel_speeds_valid || !steering_positions_valid || !std::isfinite(current_speed_)) {
            return;
        }
        double dt = 1e-9 * _info.dt.count();

        // Steering update
        if (isTimeout(this->steering_cmd_stamp_)) {
            this->target_bicycle_angle_ = 0.0;
        }
        double max_angle_inc = dt * AUDIBOT_MAX_STEER_RATE;
        if ((this->target_bicycle_angle_ - this->current_bicycle_angle_) > max_angle_inc) {
            this->current_bicycle_angle_ += max_angle_inc;
        } else if ((this->target_bicycle_angle_ - this->current_bicycle_angle_) < -max_angle_inc) {
            this->current_bicycle_angle_ -= max_angle_inc;
        }
        double t_alph = tan(this->current_bicycle_angle_);
        double left_steer = atan(AUDIBOT_WHEELBASE * t_alph / (AUDIBOT_WHEELBASE - 0.5 * AUDIBOT_TRACK_WIDTH * t_alph));
        double right_steer = atan(AUDIBOT_WHEELBASE * t_alph / (AUDIBOT_WHEELBASE + 0.5 * AUDIBOT_TRACK_WIDTH * t_alph));

        double left_steer_vel;
        if (!fl_steer_pos->Data().empty()) {
            left_steer_vel = 100.0 * (left_steer - fl_steer_pos->Data()[0]);
        } else {
            left_steer_vel = 0.0;
        }
        _ecm.SetComponentData<components::JointVelocityCmd>(this->steer_fl_joint_, {left_steer_vel});

        double right_steer_vel;
        if (!fr_steer_pos->Data().empty()) {
            right_steer_vel = 100.0 * (right_steer - fr_steer_pos->Data()[0]);
        } else {
            right_steer_vel = 0.0;
        }
        _ecm.SetComponentData<components::JointVelocityCmd>(this->steer_fr_joint_, {right_steer_vel});

        // Speed control update
        if (!isTimeout(this->speed_cmd_stamp_)) {
            // Override brake and throttle commands if there is a speed command
            speed_control_.update(speed_cmd_, current_speed_, dt, throttle_cmd_, brake_cmd_);
        } else {
            speed_control_.reset();
        }

        // Drivetrain update
        double rolling_resistance_torque = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS * GRAVITY_ACCEL;
        double drag_force = AERO_DRAG_COEFF * this->current_speed_ * this->current_speed_;
        double drag_torque = drag_force * WHEEL_RADIUS; // Implement aerodynamic drag as a torque disturbance

        double fl_torque = 0.25 * ((this->current_speed_ >= 0) ? -drag_torque : drag_torque);
        double fr_torque = 0.25 * ((this->current_speed_ >= 0) ? -drag_torque : drag_torque);
        double rl_torque = 0.25 * ((this->current_speed_ >= 0) ? -drag_torque : drag_torque);
        double rr_torque = 0.25 * ((this->current_speed_ >= 0) ? -drag_torque : drag_torque);

        double brake_torque_factor = 1.0;
        if (this->current_speed_ < -0.1) {
            brake_torque_factor = -1.0;
        } else if (this->current_speed_ < 0.1) {
            brake_torque_factor = 1.0 + (this->current_speed_ - 0.1) / 0.1;
        }

        if (this->rollover_) {
            // Stop wheels
            fl_torque  = -1000.0 * fl_joint_vel->Data()[0];
            fr_torque  = -1000.0 * fr_joint_vel->Data()[0];
            rl_torque  = -1000.0 * rl_joint_vel->Data()[0];
            rr_torque  = -1000.0 * rr_joint_vel->Data()[0];
        } else if (!isTimeout(this->speed_cmd_stamp_) || (!isTimeout(this->brake_cmd_stamp_) && !isTimeout(this->throttle_cmd_stamp_))) {
            // Brakes take precedence over throttle.
            if (brake_cmd_ > 0.0) {
                fl_torque -= (0.25 * brake_torque_factor * brake_cmd_);
                fr_torque -= (0.25 * brake_torque_factor * brake_cmd_);
                rl_torque -= (0.25 * brake_torque_factor * brake_cmd_);
                rr_torque -= (0.25 * brake_torque_factor * brake_cmd_);
            } else {
                double throttle_torque;
                if (gear_cmd_ == AudibotGear::DRIVE) {
                    throttle_torque = std::max(throttle_cmd_ * 4000.0 - 40.1 * this->current_speed_, 0.0);
                } else { // Reverse
                    throttle_torque = std::min(-throttle_cmd_ * 4000.0 - 250.0 * this->current_speed_, 0.0);
                }
                rl_torque += (0.5 * throttle_torque);
                rr_torque += (0.5 * throttle_torque);
            }
        }
        _ecm.SetComponentData<components::JointForceCmd>(this->wheel_fl_joint_, {fl_torque});
        _ecm.SetComponentData<components::JointForceCmd>(this->wheel_fr_joint_, {fr_torque});
        _ecm.SetComponentData<components::JointForceCmd>(this->wheel_rl_joint_, {rl_torque});
        _ecm.SetComponentData<components::JointForceCmd>(this->wheel_rr_joint_, {rr_torque});
    }

    void AudibotInterfacePlugin::PostUpdate(const UpdateInfo& _info, const EntityComponentManager &_ecm) {
        double dt = 1e-9 * _info.dt.count();
        const math::Pose3d vehicle_pose = worldPose(this->model_.Entity(), _ecm);
        this->rollover_ = (std::abs(vehicle_pose.Roll()) > 0.2 || std::abs(vehicle_pose.Pitch()) > 0.2);
        if (this->first_update_) {
            this->first_update_ = false;
            this->last_vehicle_pose_ = vehicle_pose;
            return;
        }

        double dx = vehicle_pose.Pos().X() - this->last_vehicle_pose_.Pos().X();
        double dy = vehicle_pose.Pos().Y() - this->last_vehicle_pose_.Pos().Y();
        double yaw = vehicle_pose.Rot().Yaw();
        double dyaw = yaw - this->last_vehicle_pose_.Rot().Yaw();

        this->current_speed_ = (dx * cosf(yaw) + dy * sinf(yaw)) / dt;
        double yvel = (-dx * sinf(yaw) + dy * cosf(yaw)) / dt;
        double yawvel = dyaw / dt;
        this->last_vehicle_pose_ = vehicle_pose;

        if ((1e-9 * (this->current_time_ - this->twist_pub_stamp_)) > TWIST_SAMPLE_TIME) {
            this->twist_pub_stamp_ = this->current_time_;
            msgs::Twist twist_msg;
            twist_msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
            twist_msg.mutable_linear()->set_x(this->current_speed_);
            twist_msg.mutable_linear()->set_y(yvel);
            twist_msg.mutable_linear()->set_z(0.0);
            twist_msg.mutable_angular()->set_x(0.0);
            twist_msg.mutable_angular()->set_y(0.0);
            twist_msg.mutable_angular()->set_z(yawvel);
            this->pub_twist_.Publish(twist_msg);

            if (this->publish_ground_truth_pose_) {
                msgs::Pose *pose_msg = nullptr;
                this->posev_msg_.Clear();
                pose_msg = this->posev_msg_.add_pose();
                GZ_ASSERT(pose_msg != nullptr, "Pose msg is null");

                auto header = pose_msg->mutable_header();
                header->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
                auto frame = header->add_data();
                frame->set_key("frame_id");
                frame->add_value("world");
                auto child_frame = header->add_data();
                child_frame->set_key("child_frame_id");
                child_frame->add_value("base_footprint");
                pose_msg->set_name("world_to_footprint");
                msgs::Set(pose_msg, vehicle_pose);
                this->pub_pose_.Publish(this->posev_msg_);
            }
        }

        if (((1e-9 * (this->current_time_ - this->heading_pub_stamp_)) > TWIST_SAMPLE_TIME) && this->publish_gnss_heading_) {
            this->heading_pub_stamp_ = this->current_time_;
            msgs::Double heading_msg;
            Set(&heading_msg, 90.0 - 180.0 / M_PI * (yaw + this->world_heading_offset_));
            this->pub_gnss_heading_.Publish(heading_msg);
        }
    }
}

// Register plugin
GZ_ADD_PLUGIN(audibot_gazebo::AudibotInterfacePlugin,
              System,
              audibot_gazebo::AudibotInterfacePlugin::ISystemConfigure,
              audibot_gazebo::AudibotInterfacePlugin::ISystemPreUpdate,
              audibot_gazebo::AudibotInterfacePlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(audibot_gazebo::AudibotInterfacePlugin, "gz::sim::systems::AudibotInterfacePlugin")