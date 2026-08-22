#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gazebo_sim_visualization/scene_contract.hpp"
#include "xgc2_robot_visualization/fs150_uav_visualizer.hpp"
#include "xgc2_robot_visualization/mecanum_ugv_visualizer.hpp"
#include "xgc2_robot_visualization/path_history.hpp"
#include "xgc2_robot_visualization/scout_ugv_visualizer.hpp"

#include <foxglove_msgs/SceneEntityDeletion.h>
#include <foxglove_msgs/SceneUpdate.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

geometry_msgs::Quaternion makeQuaternion(double x, double y, double z, double w) {
    geometry_msgs::Quaternion out;
    out.x = x;
    out.y = y;
    out.z = z;
    out.w = w;
    return out;
}

bool isFinite(double value) {
    return std::isfinite(value);
}

bool isFinite(const geometry_msgs::Pose& pose) {
    return isFinite(pose.position.x) && isFinite(pose.position.y) && isFinite(pose.position.z) &&
           isFinite(pose.orientation.x) && isFinite(pose.orientation.y) && isFinite(pose.orientation.z) &&
           isFinite(pose.orientation.w);
}

bool isFinite(const geometry_msgs::Twist& twist) {
    return isFinite(twist.linear.x) && isFinite(twist.linear.y) && isFinite(twist.linear.z) &&
           isFinite(twist.angular.x) && isFinite(twist.angular.y) && isFinite(twist.angular.z);
}

geometry_msgs::Quaternion normalize(const geometry_msgs::Quaternion& q) {
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        return makeQuaternion(0.0, 0.0, 0.0, 1.0);
    }
    return makeQuaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
}

geometry_msgs::Pose copyPose(const geometry_msgs::Pose& pose) {
    geometry_msgs::Pose out;
    out.position = pose.position;
    out.orientation = normalize(pose.orientation);
    return out;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const geometry_msgs::Quaternion normalized = normalize(q);
    const double siny_cosp = 2.0 * (normalized.w * normalized.z + normalized.x * normalized.y);
    const double cosy_cosp = 1.0 - 2.0 * (normalized.y * normalized.y + normalized.z * normalized.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

} // namespace

class GazeboAutoVisualizer {
  public:
    GazeboAutoVisualizer() : private_nh_("~") {
        private_nh_.param<std::string>("frame_id", frame_id_, "world");
        private_nh_.param<std::string>("model_states_topic", model_states_topic_, "/gazebo/model_states");
        private_nh_.param<std::string>("vrpn_pose_prefix", vrpn_pose_prefix_, "/vrpn_client_node");
        private_nh_.param<std::string>("tracked_uav_models", tracked_uav_models_csv_, "");
        private_nh_.param<std::string>("tracked_ugv_models", tracked_ugv_models_csv_, "");
        private_nh_.param<std::string>("tracked_fs150_models", tracked_fs150_models_csv_, "");
        private_nh_.param<std::string>("tracked_scout_models", tracked_scout_models_csv_, "");
        private_nh_.param<std::string>("tracked_mecanum_models", tracked_mecanum_models_csv_, "");
        private_nh_.param<std::string>("scene_update_topic", scene_update_topic_, "/xgc/scene");
        private_nh_.param<std::string>("transform_topic", transform_topic_, "/xgc/tf");
        if (!private_nh_.getParam("marker_color", marker_color_)) {
            throw std::runtime_error("marker_color is required");
        }
        scene_label_style_ = gazebo_sim_visualization::sceneLabelStyleFromMarkerColor(marker_color_);
        private_nh_.param("allow_auto_discovery", allow_auto_discovery_, true);
        private_nh_.param("track_ugv", track_ugv_, true);
        private_nh_.param("publish_markers", publish_markers_, true);
        private_nh_.param("publish_transforms", publish_transforms_, true);
        private_nh_.param("publish_scene_update", publish_scene_update_, false);
        private_nh_.param("publish_scene_paths", publish_scene_paths_, true);
        private_nh_.param("publish_rate", publish_rate_, 30.0);
        private_nh_.param("scene_publish_rate", scene_publish_rate_, 10.0);
        private_nh_.param("scene_path_publish_rate", scene_path_publish_rate_, 10.0);
        // The gradient is deliberate. A pose is what an operator reads, so it runs
        // at the timer rate. A trail has to stay attached to the robot drawing it
        // -- at 0.2 Hz the history visibly trailed the aircraft -- so it sits one
        // step down. A spinning propeller or wheel indicates the vehicle is
        // armed and moving; it is an animation, not a measurement, so it is last.
        private_nh_.param("joint_transform_publish_rate", joint_transform_publish_rate_, 5.0);
        private_nh_.param("path_publish_rate", path_publish_rate_, xgc2_robot_visualization::kDefaultPathPublishRateHz);
        private_nh_.param("path_limit", path_limit_, 0);
        private_nh_.param("path_history_duration", path_history_duration_sec_,
                          xgc2_robot_visualization::kDefaultPathHistoryDurationSec);
        private_nh_.param("vrpn_timeout", vrpn_timeout_sec_, 0.5);
        private_nh_.param("mavros_state_timeout", mavros_state_timeout_sec_, 2.0);
        private_nh_.param<std::string>("mavros_state_topic_suffix", mavros_state_topic_suffix_, "/mavros/state");
        private_nh_.param<std::string>("mavros_extended_state_topic_suffix", mavros_extended_state_topic_suffix_,
                                       "/mavros/extended_state");
        private_nh_.param("uav_rotor_speed_ground", uav_rotor_speed_ground_rad_s_, 25.0);
        private_nh_.param("uav_rotor_speed_transition", uav_rotor_speed_transition_rad_s_, 90.0);
        private_nh_.param("uav_rotor_speed_airborne", uav_rotor_speed_airborne_rad_s_, 60.0);
        private_nh_.param("ugv_motion_timeout", ugv_motion_timeout_sec_, 0.5);
        private_nh_.param<std::string>("ugv_cmd_vel_topic_suffix", ugv_cmd_vel_topic_suffix_, "/cmd_vel");
        private_nh_.param<std::string>("ugv_twist_topic_suffix", ugv_twist_topic_suffix_, "/twist");
        private_nh_.param("ugv_visual_wheel_radius", ugv_visual_wheel_radius_, 0.08);
        private_nh_.param("ugv_visual_track_width", ugv_visual_track_width_, 0.416);
        private_nh_.param("ugv_wheel_motion_deadband", ugv_wheel_motion_deadband_, 0.02);
        private_nh_.param("ugv_max_visual_wheel_speed_rad_s", ugv_max_visual_wheel_speed_rad_s_, 35.0);
        private_nh_.param("rotor_speed_rad_s", rotor_speed_rad_s_, 70.0);
        private_nh_.param("uav_mesh_scale", uav_mesh_scale_, 1.0);
        private_nh_.param("ugv_mesh_scale", ugv_mesh_scale_, 1.0);
        private_nh_.param("mecanum_mesh_scale", mecanum_mesh_scale_, 0.001);

        publish_rate_ = std::max(1.0, publish_rate_);
        scene_publish_rate_ = std::min(publish_rate_, std::max(1.0, scene_publish_rate_));
        scene_path_publish_rate_ = std::min(publish_rate_, std::max(0.1, scene_path_publish_rate_));
        joint_transform_publish_rate_ = std::min(publish_rate_, std::max(0.1, joint_transform_publish_rate_));
        path_publish_rate_ = std::max(1.0, path_publish_rate_);
        mavros_state_timeout_sec_ = std::max(0.0, mavros_state_timeout_sec_);
        ugv_motion_timeout_sec_ = std::max(0.0, ugv_motion_timeout_sec_);
        ugv_visual_wheel_radius_ = std::max(0.001, ugv_visual_wheel_radius_);
        ugv_visual_track_width_ = std::max(0.001, ugv_visual_track_width_);
        ugv_wheel_motion_deadband_ = std::max(0.0, ugv_wheel_motion_deadband_);
        ugv_max_visual_wheel_speed_rad_s_ = std::max(0.0, ugv_max_visual_wheel_speed_rad_s_);
        xgc2_robot_visualization::applyPathHistoryConfig(&path_publish_rate_, &path_history_duration_sec_,
                                                         &path_limit_);

        xgc2_robot_visualization::Fs150UavVisualizer::Config uav_config;
        uav_config.frame_id = frame_id_;
        uav_config.rotor_speed_rad_s = rotor_speed_rad_s_;
        uav_config.mesh_scale = uav_mesh_scale_;
        uav_config.path_publish_rate = path_publish_rate_;
        uav_config.path_history_duration_sec = path_history_duration_sec_;
        uav_config.path_limit = path_limit_;
        uav_visualizer_.reset(new xgc2_robot_visualization::Fs150UavVisualizer(uav_config));

        xgc2_robot_visualization::ScoutUgvVisualizer::Config ugv_config;
        ugv_config.frame_id = frame_id_;
        ugv_config.mesh_scale = ugv_mesh_scale_;
        ugv_config.path_publish_rate = path_publish_rate_;
        ugv_config.path_history_duration_sec = path_history_duration_sec_;
        ugv_config.path_limit = path_limit_;
        ugv_config.visual_wheel_radius = ugv_visual_wheel_radius_;
        ugv_config.visual_track_width = ugv_visual_track_width_;
        ugv_config.wheel_motion_deadband = ugv_wheel_motion_deadband_;
        ugv_config.max_visual_wheel_speed_rad_s = ugv_max_visual_wheel_speed_rad_s_;
        ugv_visualizer_.reset(new xgc2_robot_visualization::ScoutUgvVisualizer(ugv_config));

        xgc2_robot_visualization::MecanumUgvVisualizer::Config mecanum_config;
        mecanum_config.frame_id = frame_id_;
        mecanum_config.mesh_scale = mecanum_mesh_scale_;
        mecanum_config.path_publish_rate = path_publish_rate_;
        mecanum_config.path_history_duration_sec = path_history_duration_sec_;
        mecanum_config.path_limit = path_limit_;
        mecanum_visualizer_.reset(new xgc2_robot_visualization::MecanumUgvVisualizer(mecanum_config));

        const std::set<std::string> legacy_uav_models =
            gazebo_sim_visualization::parseModelNames(tracked_uav_models_csv_);
        const std::set<std::string> legacy_ugv_models =
            gazebo_sim_visualization::parseModelNames(tracked_ugv_models_csv_);
        configured_fs150_models_ = gazebo_sim_visualization::parseModelNames(tracked_fs150_models_csv_);
        configured_scout_models_ = gazebo_sim_visualization::parseModelNames(tracked_scout_models_csv_);
        configured_mecanum_models_ = gazebo_sim_visualization::parseModelNames(tracked_mecanum_models_csv_);
        if (!gazebo_sim_visualization::modelListsAreDisjoint(legacy_uav_models, legacy_ugv_models,
                                                             configured_fs150_models_, configured_scout_models_,
                                                             configured_mecanum_models_)) {
            throw std::runtime_error(
                "tracked FS150, Scout, and Mecanum model lists (including legacy aliases) must be disjoint");
        }
        configured_fs150_models_.insert(legacy_uav_models.begin(), legacy_uav_models.end());
        configured_scout_models_.insert(legacy_ugv_models.begin(), legacy_ugv_models.end());
        for (const std::string& name : configured_fs150_models_) {
            ensureTrackedModel(name, gazebo_sim_visualization::RobotModelKind::kFs150);
        }
        for (const std::string& name : configured_scout_models_) {
            ensureTrackedModel(name, gazebo_sim_visualization::RobotModelKind::kScout);
        }
        for (const std::string& name : configured_mecanum_models_) {
            ensureTrackedModel(name, gazebo_sim_visualization::RobotModelKind::kMecanum);
        }

        if (publish_markers_) {
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
        }
        if (publish_scene_update_) {
            scene_update_pub_ = nh_.advertise<foxglove_msgs::SceneUpdate>(scene_update_topic_, 1, true);
            scene_update_cadence_.reset(
                new gazebo_sim_visualization::SceneUpdateCadence(scene_publish_rate_, scene_path_publish_rate_));
            publishSceneReset();
        }
        joint_transform_cadence_.reset(new gazebo_sim_visualization::PublishCadence(joint_transform_publish_rate_));

        if (publish_transforms_) {
            transform_pub_ = nh_.advertise<tf2_msgs::TFMessage>(transform_topic_, 10, false);
            // Advertise the Fixed Frame parent on /tf via a single identity
            // child. Robot bodies stay on transform_topic_ (not /tf).
            if (transform_topic_ != "/tf") {
                tf_tree_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 10, false);
            }
        }

        model_states_sub_ = nh_.subscribe(model_states_topic_, 5, &GazeboAutoVisualizer::modelStatesCallback, this);
        publish_timer_ =
            nh_.createTimer(ros::Duration(1.0 / publish_rate_), &GazeboAutoVisualizer::publishCallback, this);
    }

  private:
    struct TrackedModel {
        std::string name;
        gazebo_sim_visualization::RobotModelKind kind;
        geometry_msgs::Pose gazebo_pose;
        geometry_msgs::Pose vrpn_pose;
        std::string vrpn_frame_id;
        bool has_gazebo_pose{false};
        bool has_vrpn_pose{false};
        bool has_mavros_state{false};
        bool mavros_armed{false};
        std::uint8_t mavros_landed_state{0};
        bool has_mavros_extended_state{false};
        ros::Time mavros_extended_state_stamp;
        ros::Subscriber mavros_extended_state_subscriber;
        ros::Time vrpn_stamp;
        ros::Time mavros_state_stamp;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::TwistStamped twist;
        bool has_cmd_vel{false};
        bool has_twist{false};
        ros::Time cmd_vel_stamp;
        ros::Time twist_stamp;
        ros::Subscriber vrpn_subscriber;
        ros::Subscriber mavros_state_subscriber;
        ros::Subscriber cmd_vel_subscriber;
        ros::Subscriber twist_subscriber;
    };

    std::map<std::string, TrackedModel>::iterator ensureTrackedModel(const std::string& name,
                                                                     gazebo_sim_visualization::RobotModelKind kind) {
        auto existing = models_.find(name);
        if (existing != models_.end()) {
            return existing;
        }

        TrackedModel model;
        model.name = name;
        model.kind = kind;
        model.vrpn_subscriber = subscribeVrpn(name);
        if (kind == gazebo_sim_visualization::RobotModelKind::kFs150) {
            model.mavros_state_subscriber = subscribeMavrosState(name);
            model.mavros_extended_state_subscriber = subscribeMavrosExtendedState(name);
        } else {
            model.cmd_vel_subscriber = subscribeUgvCmdVel(name);
            model.twist_subscriber = subscribeUgvTwist(name);
        }
        auto inserted = models_.emplace(name, std::move(model)).first;
        const char* kind_name = "scout";
        if (kind == gazebo_sim_visualization::RobotModelKind::kFs150) {
            kind_name = "fs150";
        } else if (kind == gazebo_sim_visualization::RobotModelKind::kMecanum) {
            kind_name = "mecanum";
        }
        ROS_INFO("[gazebo_auto_visualizer] Tracking %s model '%s'", kind_name, name.c_str());
        return inserted;
    }

    void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg) {
        for (std::size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i) {
            const std::string& name = msg->name[i];
            const gazebo_sim_visualization::RobotModelKind kind = gazebo_sim_visualization::selectRobotModelKind(
                name, configured_fs150_models_, configured_scout_models_, configured_mecanum_models_,
                allow_auto_discovery_, track_ugv_);
            if (kind == gazebo_sim_visualization::RobotModelKind::kNone) {
                continue;
            }
            if (!isFinite(msg->pose[i])) {
                ROS_WARN_THROTTLE(2.0,
                                  "[gazebo_auto_visualizer] Ignoring non-finite pose "
                                  "for model '%s'",
                                  name.c_str());
                continue;
            }
            auto it = ensureTrackedModel(name, kind);
            it->second.gazebo_pose = copyPose(msg->pose[i]);
            it->second.has_gazebo_pose = true;
        }
    }

    ros::Subscriber subscribeVrpn(const std::string& name) {
        const std::string topic = vrpn_pose_prefix_ + "/" + name + "/pose";
        return nh_.subscribe<geometry_msgs::PoseStamped>(topic, 10,
                                                         [this, name](const geometry_msgs::PoseStampedConstPtr& msg) {
                                                             vrpnPoseCallback(name, msg);
                                                         });
    }

    // Rotor speed reads the flight state machine; it does not model aerodynamics.
    // A few fixed tiers so an operator can tell parked from climbing from
    // cruising at a glance. Anything finer is a number nobody reads, recomputed
    // every frame for twelve aircraft.
    double rotorSpeedFor(const TrackedModel& model, const ros::Time& now) const {
        if (!model.has_mavros_extended_state ||
            now - model.mavros_extended_state_stamp > ros::Duration(mavros_state_timeout_sec_)) {
            return uav_rotor_speed_airborne_rad_s_;
        }
        switch (model.mavros_landed_state) {
        case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
            return uav_rotor_speed_ground_rad_s_;
        case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF:
        case mavros_msgs::ExtendedState::LANDED_STATE_LANDING:
            return uav_rotor_speed_transition_rad_s_;
        default:
            return uav_rotor_speed_airborne_rad_s_;
        }
    }

    ros::Subscriber subscribeMavrosExtendedState(const std::string& name) {
        const std::string topic = modelScopedTopic(name, mavros_extended_state_topic_suffix_, "/mavros/extended_state");
        return nh_.subscribe<mavros_msgs::ExtendedState>(topic, 10,
                                                         [this, name](const mavros_msgs::ExtendedStateConstPtr& msg) {
                                                             auto it = models_.find(name);
                                                             if (it == models_.end()) {
                                                                 return;
                                                             }
                                                             it->second.mavros_landed_state = msg->landed_state;
                                                             it->second.has_mavros_extended_state = true;
                                                             it->second.mavros_extended_state_stamp = ros::Time::now();
                                                         });
    }

    ros::Subscriber subscribeMavrosState(const std::string& name) {
        const std::string topic = modelScopedTopic(name, mavros_state_topic_suffix_, "/mavros/state");
        return nh_.subscribe<mavros_msgs::State>(topic, 10, [this, name](const mavros_msgs::StateConstPtr& msg) {
            mavrosStateCallback(name, msg);
        });
    }

    ros::Subscriber subscribeUgvCmdVel(const std::string& name) {
        const std::string topic = modelScopedTopic(name, ugv_cmd_vel_topic_suffix_, "/cmd_vel");
        return nh_.subscribe<geometry_msgs::Twist>(topic, 10, [this, name](const geometry_msgs::TwistConstPtr& msg) {
            ugvCmdVelCallback(name, msg);
        });
    }

    ros::Subscriber subscribeUgvTwist(const std::string& name) {
        const std::string topic = modelScopedTopic(name, ugv_twist_topic_suffix_, "/twist");
        return nh_.subscribe<geometry_msgs::TwistStamped>(topic, 10,
                                                          [this, name](const geometry_msgs::TwistStampedConstPtr& msg) {
                                                              ugvTwistCallback(name, msg);
                                                          });
    }

    std::string modelScopedTopic(const std::string& name, const std::string& suffix,
                                 const std::string& default_suffix) const {
        std::string normalized_suffix = suffix.empty() ? default_suffix : suffix;
        if (normalized_suffix.front() != '/') {
            normalized_suffix = "/" + normalized_suffix;
        }
        return "/" + name + normalized_suffix;
    }

    void vrpnPoseCallback(const std::string& name, const geometry_msgs::PoseStampedConstPtr& msg) {
        auto it = models_.find(name);
        if (it == models_.end() || !isFinite(msg->pose)) {
            return;
        }
        it->second.vrpn_pose = copyPose(msg->pose);
        it->second.vrpn_frame_id = msg->header.frame_id;
        it->second.vrpn_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        it->second.has_vrpn_pose = true;
    }

    void mavrosStateCallback(const std::string& name, const mavros_msgs::StateConstPtr& msg) {
        auto it = models_.find(name);
        if (it == models_.end()) {
            return;
        }
        it->second.mavros_armed = msg->armed;
        it->second.mavros_state_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        it->second.has_mavros_state = true;
    }

    void ugvCmdVelCallback(const std::string& name, const geometry_msgs::TwistConstPtr& msg) {
        auto it = models_.find(name);
        if (it == models_.end() || !isFinite(*msg)) {
            return;
        }
        it->second.cmd_vel = *msg;
        it->second.cmd_vel_stamp = ros::Time::now();
        it->second.has_cmd_vel = true;
    }

    void ugvTwistCallback(const std::string& name, const geometry_msgs::TwistStampedConstPtr& msg) {
        auto it = models_.find(name);
        if (it == models_.end() || !isFinite(msg->twist)) {
            return;
        }
        it->second.twist = *msg;
        it->second.twist_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        it->second.has_twist = true;
    }

    gazebo_sim_visualization::WorldEnuPoseSelection selectWorldPose(const TrackedModel& model,
                                                                    const ros::Time& now) const {
        gazebo_sim_visualization::WorldEnuPoseSource vrpn;
        vrpn.available = model.has_vrpn_pose;
        vrpn.pose = model.vrpn_pose;
        vrpn.stamp = model.vrpn_stamp;
        vrpn.frame_id = model.vrpn_frame_id;
        gazebo_sim_visualization::WorldEnuPoseSource gazebo;
        gazebo.available = model.has_gazebo_pose;
        gazebo.pose = model.gazebo_pose;
        gazebo.frame_id = frame_id_;
        return gazebo_sim_visualization::selectWorldEnuPose(vrpn, gazebo, now, vrpn_timeout_sec_);
    }

    bool rotorsActive(const TrackedModel& model, const ros::Time& now) const {
        if (!model.has_mavros_state || !model.mavros_armed) {
            return false;
        }
        return now - model.mavros_state_stamp <= ros::Duration(mavros_state_timeout_sec_);
    }

    bool isFresh(const ros::Time& stamp, const ros::Time& now, double timeout_sec) const {
        return !stamp.isZero() && now - stamp <= ros::Duration(timeout_sec);
    }

    bool twistFrameIsWorldFixed(const std::string& frame_id) const {
        std::string normalized = lower(frame_id);
        if (!normalized.empty() && normalized.front() == '/') {
            normalized.erase(normalized.begin());
        }
        std::string normalized_visual_frame = lower(frame_id_);
        if (!normalized_visual_frame.empty() && normalized_visual_frame.front() == '/') {
            normalized_visual_frame.erase(normalized_visual_frame.begin());
        }
        return !normalized.empty() && (normalized == normalized_visual_frame || normalized == "world" ||
                                       normalized == "map" || normalized == "odom");
    }

    xgc2_robot_visualization::UgvVisualState
    makeUgvVisualState(const TrackedModel& model, const geometry_msgs::Pose& pose, const ros::Time& now) const {
        xgc2_robot_visualization::UgvVisualState state;
        state.name = model.name;
        state.pose = pose;
        state.stamp = now;

        if (model.has_twist && isFresh(model.twist_stamp, now, ugv_motion_timeout_sec_)) {
            state.has_motion_hint = true;
            state.yaw_rate_rad_s = model.twist.twist.angular.z;
            if (twistFrameIsWorldFixed(model.twist.header.frame_id)) {
                const double yaw = yawFromQuaternion(pose.orientation);
                state.forward_velocity_m_s =
                    model.twist.twist.linear.x * std::cos(yaw) + model.twist.twist.linear.y * std::sin(yaw);
            } else {
                state.forward_velocity_m_s = model.twist.twist.linear.x;
            }
            return state;
        }

        if (model.has_cmd_vel && isFresh(model.cmd_vel_stamp, now, ugv_motion_timeout_sec_)) {
            state.has_motion_hint = true;
            state.forward_velocity_m_s = model.cmd_vel.linear.x;
            state.yaw_rate_rad_s = model.cmd_vel.angular.z;
        }
        return state;
    }

    xgc2_robot_visualization::MecanumVisualState
    makeMecanumVisualState(const TrackedModel& model, const geometry_msgs::Pose& pose, const ros::Time& now) const {
        xgc2_robot_visualization::MecanumVisualState state;
        state.name = model.name;
        state.pose = pose;
        state.stamp = now;

        if (model.has_twist && isFresh(model.twist_stamp, now, ugv_motion_timeout_sec_)) {
            state.has_motion_hint = true;
            state.yaw_rate_rad_s = model.twist.twist.angular.z;
            if (twistFrameIsWorldFixed(model.twist.header.frame_id)) {
                const double yaw = yawFromQuaternion(pose.orientation);
                state.forward_velocity_m_s =
                    model.twist.twist.linear.x * std::cos(yaw) + model.twist.twist.linear.y * std::sin(yaw);
                state.lateral_velocity_m_s =
                    -model.twist.twist.linear.x * std::sin(yaw) + model.twist.twist.linear.y * std::cos(yaw);
            } else {
                state.forward_velocity_m_s = model.twist.twist.linear.x;
                state.lateral_velocity_m_s = model.twist.twist.linear.y;
            }
            return state;
        }

        if (model.has_cmd_vel && isFresh(model.cmd_vel_stamp, now, ugv_motion_timeout_sec_)) {
            state.has_motion_hint = true;
            state.forward_velocity_m_s = model.cmd_vel.linear.x;
            state.lateral_velocity_m_s = model.cmd_vel.linear.y;
            state.yaw_rate_rad_s = model.cmd_vel.angular.z;
        }
        return state;
    }

    void publishSceneReset() {
        foxglove_msgs::SceneUpdate update;
        foxglove_msgs::SceneEntityDeletion deletion;
        deletion.timestamp = ros::Time::now();
        deletion.type = foxglove_msgs::SceneEntityDeletion::ALL;
        update.deletions.push_back(deletion);
        scene_update_pub_.publish(update);
    }

    void publishCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        gazebo_sim_visualization::SceneUpdateCadenceDecision scene_decision;
        if (publish_scene_update_) {
            scene_decision = scene_update_cadence_->take(now);
        }
        if (!publish_markers_ && !publish_transforms_ && !scene_decision.publish_label &&
            !scene_decision.publish_path) {
            return;
        }

        visualization_msgs::MarkerArray markers;
        std::vector<geometry_msgs::TransformStamped> transforms;
        foxglove_msgs::SceneUpdate scene_update;
        for (auto& entry : models_) {
            TrackedModel& model = entry.second;
            const gazebo_sim_visualization::WorldEnuPoseSelection world_pose = selectWorldPose(model, now);
            if (!world_pose.found) {
                continue;
            }
            const geometry_msgs::Pose* pose = &world_pose.pose;
            const std::size_t first_marker = markers.markers.size();
            if (model.kind == gazebo_sim_visualization::RobotModelKind::kFs150) {
                xgc2_robot_visualization::UavVisualState state;
                state.name = model.name;
                state.pose = *pose;
                state.rotors_active = rotorsActive(model, now);
                state.rotor_speed_rad_s = rotorSpeedFor(model, now);
                state.stamp = now;
                uav_visualizer_->append(state, &markers, &transforms);
            } else if (model.kind == gazebo_sim_visualization::RobotModelKind::kMecanum) {
                mecanum_visualizer_->append(makeMecanumVisualState(model, *pose, now), &markers, &transforms);
            } else {
                ugv_visualizer_->append(makeUgvVisualState(model, *pose, now), &markers, &transforms);
            }
            if (publish_scene_paths_ && scene_decision.publish_path) {
                gazebo_sim_visualization::appendSceneEntityPart(
                    model.kind, model.name, gazebo_sim_visualization::SceneEntityPart::kPath, markers, first_marker,
                    now, frame_id_, scene_label_style_, &scene_update);
            }
            if (scene_decision.publish_label) {
                // The label rides the scene cadence because its content -- a
                // name, a colour, a font size -- does not change. Where it is
                // drawn comes from its anchor transform instead, which is why
                // it can move at the pose rate without being retransmitted.
                gazebo_sim_visualization::appendSceneEntityPart(
                    model.kind, model.name, gazebo_sim_visualization::SceneEntityPart::kLabel, markers, first_marker,
                    now, frame_id_, scene_label_style_, &scene_update);
            }
        }

        if (publish_transforms_) {
            const geometry_msgs::TransformStamped root = gazebo_sim_visualization::worldFixedFrameRoot(frame_id_, now);
            if (tf_tree_pub_) {
                tf2_msgs::TFMessage world_frame;
                world_frame.transforms.push_back(root);
                tf_tree_pub_.publish(world_frame);
            }
            if (!transforms.empty()) {
                const bool publish_joints = joint_transform_cadence_->take(now);
                std::vector<geometry_msgs::TransformStamped> outgoing;
                outgoing.reserve(transforms.size() + 1);
                if (!tf_tree_pub_) {
                    outgoing.push_back(root);
                }
                for (const geometry_msgs::TransformStamped& transform : transforms) {
                    if (publish_joints || transform.header.frame_id == frame_id_) {
                        outgoing.push_back(transform);
                    }
                }
                if (!outgoing.empty()) {
                    tf2_msgs::TFMessage message;
                    message.transforms = std::move(outgoing);
                    transform_pub_.publish(message);
                }
            } else if (!tf_tree_pub_) {
                tf2_msgs::TFMessage world_frame;
                world_frame.transforms.push_back(root);
                transform_pub_.publish(world_frame);
            }
        }
        if (publish_markers_) {
            marker_pub_.publish(markers);
        }
        if (publish_scene_update_ && !scene_update.entities.empty()) {
            scene_update_pub_.publish(scene_update);
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::Publisher scene_update_pub_;
    ros::Subscriber model_states_sub_;
    ros::Timer publish_timer_;
    // Deliberately not tf2_ros::TransformBroadcaster: that publishes to the
    // global /tf, where a simulator already broadcasts the same frame names at
    // its own rate. Two publishers of one frame is a race, and forwarding /tf to
    // a viewer would carry the simulator's whole high-rate tree with it. This
    // node owns a topic instead -- the low-rate, product-owned view of the same
    // tree, which a physical fleet can publish just as well.
    ros::Publisher transform_pub_;
    ros::Publisher tf_tree_pub_;
    std::string transform_topic_;
    std::map<std::string, TrackedModel> models_;
    std::set<std::string> configured_fs150_models_;
    std::set<std::string> configured_scout_models_;
    std::set<std::string> configured_mecanum_models_;
    std::unique_ptr<xgc2_robot_visualization::Fs150UavVisualizer> uav_visualizer_;
    std::unique_ptr<xgc2_robot_visualization::ScoutUgvVisualizer> ugv_visualizer_;
    std::unique_ptr<xgc2_robot_visualization::MecanumUgvVisualizer> mecanum_visualizer_;
    std::unique_ptr<gazebo_sim_visualization::SceneUpdateCadence> scene_update_cadence_;
    gazebo_sim_visualization::SceneLabelStyle scene_label_style_;

    std::string frame_id_;
    std::string model_states_topic_;
    std::string vrpn_pose_prefix_;
    std::string mavros_extended_state_topic_suffix_;
    double uav_rotor_speed_ground_rad_s_{25.0};
    double uav_rotor_speed_transition_rad_s_{90.0};
    double uav_rotor_speed_airborne_rad_s_{60.0};
    double joint_transform_publish_rate_{5.0};
    std::unique_ptr<gazebo_sim_visualization::PublishCadence> joint_transform_cadence_;
    std::string tracked_uav_models_csv_;
    std::string tracked_ugv_models_csv_;
    std::string tracked_fs150_models_csv_;
    std::string tracked_scout_models_csv_;
    std::string tracked_mecanum_models_csv_;
    std::string marker_color_;
    std::string scene_update_topic_{"/xgc/scene"};
    std::string mavros_state_topic_suffix_{"/mavros/state"};
    std::string ugv_cmd_vel_topic_suffix_{"/cmd_vel"};
    std::string ugv_twist_topic_suffix_{"/twist"};
    double publish_rate_{30.0};
    double scene_publish_rate_{10.0};
    double scene_path_publish_rate_{2.0};
    double path_publish_rate_{xgc2_robot_visualization::kDefaultPathPublishRateHz};
    int path_limit_{0};
    double path_history_duration_sec_{xgc2_robot_visualization::kDefaultPathHistoryDurationSec};
    double vrpn_timeout_sec_{0.5};
    double mavros_state_timeout_sec_{2.0};
    double ugv_motion_timeout_sec_{0.5};
    double rotor_speed_rad_s_{70.0};
    double uav_mesh_scale_{1.0};
    double ugv_mesh_scale_{1.0};
    double mecanum_mesh_scale_{0.001};
    double ugv_visual_wheel_radius_{0.08};
    double ugv_visual_track_width_{0.416};
    double ugv_wheel_motion_deadband_{0.02};
    double ugv_max_visual_wheel_speed_rad_s_{35.0};
    bool allow_auto_discovery_{true};
    bool track_ugv_{true};
    bool publish_markers_{true};
    bool publish_transforms_{true};
    bool publish_scene_update_{false};
    bool publish_scene_paths_{true};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_auto_visualizer");
    GazeboAutoVisualizer visualizer;
    ros::spin();
    return 0;
}
