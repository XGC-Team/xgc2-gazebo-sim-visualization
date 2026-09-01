#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
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
#include "xgc2_robot_visualization/path_runtime.hpp"
#include "xgc2_robot_visualization/robot_description_runtime.hpp"
#include "xgc2_robot_visualization/robot_frames.hpp"
#include "xgc2_robot_visualization/scout_ugv_visualizer.hpp"

#include <foxglove_msgs/SceneEntityDeletion.h>
#include <foxglove_msgs/SceneUpdate.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

constexpr const char* kRosterEnvironment = "XGC2_ROBOT_VISUALIZATION_ROSTER";

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
        private_nh_.param<std::string>("tracked_fs150_models", tracked_fs150_models_csv_, "");
        private_nh_.param<std::string>("tracked_scout_models", tracked_scout_models_csv_, "");
        private_nh_.param<std::string>("tracked_mecanum_models", tracked_mecanum_models_csv_, "");
        private_nh_.param<std::string>("scene_update_topic", scene_update_topic_, "/xgc/scene");
        private_nh_.param<std::string>("transform_topic", transform_topic_, "/xgc/tf");
        if (!gazebo_sim_visualization::isWorldFixedFrame(frame_id_)) {
            throw std::runtime_error("frame_id must identify the world Fixed Frame");
        }
        if (!private_nh_.getParam("marker_color", marker_color_)) {
            throw std::runtime_error("marker_color is required");
        }
        scene_label_style_ = gazebo_sim_visualization::sceneLabelStyleFromMarkerColor(marker_color_);
        private_nh_.param("track_ugv", track_ugv_, true);
        private_nh_.param("publish_markers", publish_markers_, true);
        private_nh_.param("publish_transforms", publish_transforms_, true);
        private_nh_.param("publish_scene_update", publish_scene_update_, false);
        private_nh_.param("publish_scene_paths", publish_scene_paths_, true);
        private_nh_.param("publish_paths", publish_paths_, false);
        private_nh_.param("publish_rate", publish_rate_, 30.0);
        private_nh_.param("pose_transform_publish_rate", pose_transform_publish_rate_, 120.0);
        private_nh_.param("scene_publish_rate", scene_publish_rate_, 10.0);
        private_nh_.param("scene_path_publish_rate", scene_path_publish_rate_, 10.0);
        // The gradient is deliberate. A pose is what an operator reads, so it runs
        // at the timer rate. A trail has to stay attached to the robot drawing it
        // -- at 0.2 Hz the history visibly trailed the aircraft -- so it sits one
        // step down. A spinning propeller or wheel indicates the vehicle is
        // armed and moving. Its fastest visual tier is 90 rad/s, so the joint
        // transforms stay at the 30 Hz pose cadence instead of aliasing through
        // the old 5 Hz sampler and appearing to rotate slowly or backwards.
        private_nh_.param("joint_transform_publish_rate", joint_transform_publish_rate_, 30.0);
        private_nh_.param("path_publish_rate", path_publish_rate_, xgc2_robot_visualization::kDefaultPathPublishRateHz);
        private_nh_.param("path_limit", path_limit_, 0);
        private_nh_.param("path_history_duration", path_history_duration_sec_,
                          xgc2_robot_visualization::kDefaultPathHistoryDurationSec);
        private_nh_.param("canonical_pose_timeout", canonical_pose_timeout_sec_, 0.5);
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
        pose_transform_publish_rate_ = std::max(1.0, pose_transform_publish_rate_);
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

        configured_fs150_models_ = gazebo_sim_visualization::parseModelNames(tracked_fs150_models_csv_);
        configured_scout_models_ = gazebo_sim_visualization::parseModelNames(tracked_scout_models_csv_);
        configured_mecanum_models_ = gazebo_sim_visualization::parseModelNames(tracked_mecanum_models_csv_);
        if (!gazebo_sim_visualization::modelListsAreDisjoint(configured_fs150_models_, configured_scout_models_,
                                                             configured_mecanum_models_)) {
            throw std::runtime_error("tracked FS150, Scout, and Mecanum model lists must be disjoint");
        }
        for (const std::string& name : configured_fs150_models_) {
            ensureTrackedModel(name, gazebo_sim_visualization::RobotModelKind::kFs150);
        }
        for (const std::string& name : configured_scout_models_) {
            ensureTrackedModel(name, gazebo_sim_visualization::RobotModelKind::kScout);
        }
        for (const std::string& name : configured_mecanum_models_) {
            ensureTrackedModel(name, gazebo_sim_visualization::RobotModelKind::kMecanum);
        }
        configureFrozenRoster();
        subscribeCanonicalPoses();
        scene_ready_pub_ = nh_.advertise<std_msgs::Empty>("/xgc/robot_scene/ready", 1, true);

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
            pose_transform_timer_ = nh_.createTimer(
                ros::Duration(1.0 / pose_transform_publish_rate_),
                &GazeboAutoVisualizer::publishPoseTransformsCallback, this);
        }

        publish_timer_ =
            nh_.createTimer(ros::Duration(1.0 / publish_rate_), &GazeboAutoVisualizer::publishCallback, this);
    }

  private:
    struct TrackedModel {
        std::string name;
        // Experiment slot identity. Mesh/TF geometry continues to use `name`,
        // the scene-model identity (for example slot uav7 backed by model
        // ugv1). The slot namespace owns the selected pose topic: FS150 uses
        // MAVROS fused local pose, while ground robots use canonical pose.
        // Visible label class comes from `kind`, not either lowercase ID.
        std::string slot_name;
        std::string ros_namespace;
        gazebo_sim_visualization::RobotModelKind kind;
        gazebo_sim_visualization::CanonicalPoseSample canonical_pose;
        ros::Time last_pose_transform_stamp;
        bool has_mavros_state{false};
        bool mavros_armed{false};
        std::uint8_t mavros_landed_state{0};
        bool has_mavros_extended_state{false};
        ros::Time mavros_extended_state_stamp;
        ros::Subscriber mavros_extended_state_subscriber;
        ros::Time mavros_state_stamp;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::TwistStamped twist;
        bool has_cmd_vel{false};
        bool has_twist{false};
        ros::Time cmd_vel_stamp;
        ros::Time twist_stamp;
        ros::Subscriber pose_subscriber;
        ros::Subscriber mavros_state_subscriber;
        ros::Subscriber cmd_vel_subscriber;
        ros::Subscriber twist_subscriber;
    };

    struct PathPublisherRuntime {
        std::string topic;
        ros::Publisher publisher;
        std::unique_ptr<xgc2_robot_visualization::BoundedPathRuntime> history;
    };

    void configureFrozenRoster() {
        const char* raw_roster = std::getenv(kRosterEnvironment);
        if (raw_roster == nullptr) {
            if (publish_paths_) {
                throw std::runtime_error(std::string(kRosterEnvironment) +
                                         " is required when publish_paths is enabled");
            }
            return;
        }
        std::vector<xgc2_robot_visualization::RobotDescription> roster;
        std::string error;
        if (!xgc2_robot_visualization::readRobotVisualizationRoster(raw_roster, &roster, &error)) {
            throw std::runtime_error(error);
        }
        std::set<std::string> configured_models = configured_fs150_models_;
        configured_models.insert(configured_scout_models_.begin(), configured_scout_models_.end());
        configured_models.insert(configured_mecanum_models_.begin(), configured_mecanum_models_.end());
        std::set<std::string> topics;
        for (const xgc2_robot_visualization::RobotDescription& robot : roster) {
            if (robot.scene_model.empty()) {
                continue;
            }
            if (configured_models.count(robot.scene_model) == 0U) {
                throw std::runtime_error("Path roster scene model '" + robot.scene_model +
                                         "' is not tracked by the scene process");
            }
            auto tracked = models_.find(robot.scene_model);
            tracked->second.slot_name = robot.name;
            tracked->second.ros_namespace = robot.ros_namespace;
            if (!publish_paths_) {
                continue;
            }
            const std::string topic =
                xgc2_robot_visualization::namespacedPathTopic(robot.ros_namespace, robot.path_topic);
            if (!topics.insert(topic).second) {
                throw std::runtime_error("Path roster repeats topic '" + topic + "'");
            }
            PathPublisherRuntime runtime;
            runtime.topic = topic;
            runtime.publisher = nh_.advertise<nav_msgs::Path>(topic, 1, true);
            xgc2_robot_visualization::PathRuntimeConfig config;
            config.sample_rate_hz = path_publish_rate_;
            config.max_age_sec = path_history_duration_sec_;
            config.max_points = path_limit_;
            runtime.history.reset(new xgc2_robot_visualization::BoundedPathRuntime(frame_id_, config));
            path_publishers_.emplace(robot.scene_model, std::move(runtime));
        }
        if (publish_paths_ && path_publishers_.size() != configured_models.size()) {
            throw std::runtime_error("Path roster must map every tracked scene model exactly once");
        }
    }

    void subscribeCanonicalPoses() {
        for (auto& entry : models_) {
            TrackedModel& model = entry.second;
            const std::string topic =
                gazebo_sim_visualization::slotVisualizationPoseTopic(model.kind, model.ros_namespace);
            model.pose_subscriber = nh_.subscribe<geometry_msgs::PoseStamped>(
                topic, 10, [this, scene_model = model.name](const geometry_msgs::PoseStampedConstPtr& msg) {
                    canonicalPoseCallback(scene_model, msg);
                });
            ROS_INFO("[gazebo_auto_visualizer] Viewer pose for scene model '%s' slot '%s' is %s",
                     model.name.c_str(), model.slot_name.c_str(), topic.c_str());
        }
    }

    std::map<std::string, TrackedModel>::iterator ensureTrackedModel(const std::string& name,
                                                                     gazebo_sim_visualization::RobotModelKind kind) {
        auto existing = models_.find(name);
        if (existing != models_.end()) {
            return existing;
        }

        TrackedModel model;
        model.name = name;
        model.slot_name = name;
        model.ros_namespace = "/" + name;
        model.kind = kind;
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

    void canonicalPoseCallback(const std::string& name, const geometry_msgs::PoseStampedConstPtr& msg) {
        auto it = models_.find(name);
        if (it == models_.end() || !isFinite(msg->pose)) {
            return;
        }
        it->second.canonical_pose.available = true;
        it->second.canonical_pose.pose = copyPose(msg->pose);
        it->second.canonical_pose.stamp = msg->header.stamp;
        it->second.canonical_pose.frame_id = msg->header.frame_id;
    }

    void publishPoseTransformsCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        tf2_msgs::TFMessage message;
        std::vector<TrackedModel*> published;
        for (auto& entry : models_) {
            TrackedModel& model = entry.second;
            const gazebo_sim_visualization::CanonicalWorldPose pose = selectWorldPose(model, now);
            if (!pose.found || pose.stamp == model.last_pose_transform_stamp) {
                continue;
            }
            const auto transforms = gazebo_sim_visualization::canonicalRobotPoseTransforms(
                model.kind, model.name, pose.pose, pose.stamp, frame_id_);
            message.transforms.insert(message.transforms.end(), transforms.begin(), transforms.end());
            published.push_back(&model);
        }
        if (message.transforms.empty()) {
            return;
        }
        transform_pub_.publish(message);
        for (TrackedModel* model : published) {
            model->last_pose_transform_stamp = model->canonical_pose.stamp;
        }
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

    gazebo_sim_visualization::CanonicalWorldPose selectWorldPose(const TrackedModel& model,
                                                                 const ros::Time& now) const {
        return gazebo_sim_visualization::selectCanonicalWorldPose(model.canonical_pose, now,
                                                                  canonical_pose_timeout_sec_);
    }

    void publishPath(const TrackedModel& model, const gazebo_sim_visualization::CanonicalWorldPose& world_pose) {
        if (!publish_paths_) {
            return;
        }
        auto runtime = path_publishers_.find(model.name);
        if (runtime == path_publishers_.end()) {
            return;
        }
        if (!gazebo_sim_visualization::isWorldFixedFrame(world_pose.frame_id)) {
            ROS_WARN_THROTTLE(2.0,
                              "[gazebo_auto_visualizer] Refusing Path pose for '%s' in non-world frame '%s'",
                              model.name.c_str(), world_pose.frame_id.c_str());
            return;
        }
        if (runtime->second.history->append(world_pose.stamp, world_pose.pose)) {
            runtime->second.publisher.publish(runtime->second.history->message());
        }
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
        return !normalized.empty() && (normalized == normalized_visual_frame || normalized == "world");
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
            if (!publish_scene_paths_) {
                scene_decision.publish_label = !scene_labels_published_;
                scene_decision.publish_path = false;
            }
        }
        if (!publish_markers_ && !publish_transforms_ && !publish_paths_ && !scene_decision.publish_label &&
            !scene_decision.publish_path) {
            return;
        }

        visualization_msgs::MarkerArray markers;
        std::vector<geometry_msgs::TransformStamped> transforms;
        foxglove_msgs::SceneUpdate scene_update;
        std::size_t world_pose_count = 0U;
        for (auto& entry : models_) {
            TrackedModel& model = entry.second;
            const gazebo_sim_visualization::CanonicalWorldPose world_pose = selectWorldPose(model, now);
            if (!world_pose.found) {
                continue;
            }
            ++world_pose_count;
            publishPath(model, world_pose);
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
            gazebo_sim_visualization::applyRobotMarkerLabel(&markers, first_marker, model.kind, model.ros_namespace);
            if (publish_scene_paths_ && scene_decision.publish_path) {
                gazebo_sim_visualization::appendSceneEntityPart(
                    model.kind, model.slot_name, gazebo_sim_visualization::SceneEntityPart::kPath, markers,
                    first_marker, now, frame_id_, scene_label_style_, &scene_update);
            }
            if (scene_decision.publish_label) {
                // The label rides the scene cadence because its content -- a
                // name, a colour, a font size -- does not change. Where it is
                // drawn comes from its anchor transform instead, which is why
                // it can move at the pose rate without being retransmitted.
                gazebo_sim_visualization::appendSceneEntityPart(
                    model.kind, model.slot_name, gazebo_sim_visualization::SceneEntityPart::kLabel, markers,
                    first_marker, now, frame_id_, scene_label_style_, &scene_update);
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
                    // Body and upright label-anchor transforms follow each
                    // canonical pose callback. The timer owns only child joints.
                    if (publish_joints && transform.header.frame_id != frame_id_) {
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
        const bool complete_static_labels = !publish_scene_paths_ && scene_decision.publish_label &&
                                            world_pose_count == models_.size();
        if (publish_scene_update_ && !scene_update.entities.empty() &&
            (publish_scene_paths_ || complete_static_labels)) {
            scene_update_pub_.publish(scene_update);
            if (complete_static_labels) {
                scene_labels_published_ = true;
            }
        }
        if (!scene_ready_published_ && publish_transforms_ && publish_paths_ &&
            gazebo_sim_visualization::frozenVisualizationRosterReady(models_.size(), world_pose_count)) {
            scene_ready_pub_.publish(std_msgs::Empty());
            scene_ready_published_ = true;
            ROS_INFO_STREAM("published frozen visualization readiness for " << world_pose_count << " Robot(s)");
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::Publisher scene_update_pub_;
    ros::Publisher scene_ready_pub_;
    ros::Timer publish_timer_;
    ros::Timer pose_transform_timer_;
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
    std::map<std::string, PathPublisherRuntime> path_publishers_;
    std::set<std::string> configured_fs150_models_;
    std::set<std::string> configured_scout_models_;
    std::set<std::string> configured_mecanum_models_;
    std::unique_ptr<xgc2_robot_visualization::Fs150UavVisualizer> uav_visualizer_;
    std::unique_ptr<xgc2_robot_visualization::ScoutUgvVisualizer> ugv_visualizer_;
    std::unique_ptr<xgc2_robot_visualization::MecanumUgvVisualizer> mecanum_visualizer_;
    std::unique_ptr<gazebo_sim_visualization::SceneUpdateCadence> scene_update_cadence_;
    gazebo_sim_visualization::SceneLabelStyle scene_label_style_;

    std::string frame_id_;
    std::string mavros_extended_state_topic_suffix_;
    double uav_rotor_speed_ground_rad_s_{25.0};
    double uav_rotor_speed_transition_rad_s_{90.0};
    double uav_rotor_speed_airborne_rad_s_{60.0};
    double joint_transform_publish_rate_{30.0};
    std::unique_ptr<gazebo_sim_visualization::PublishCadence> joint_transform_cadence_;
    std::string tracked_fs150_models_csv_;
    std::string tracked_scout_models_csv_;
    std::string tracked_mecanum_models_csv_;
    std::string marker_color_;
    std::string scene_update_topic_{"/xgc/scene"};
    std::string mavros_state_topic_suffix_{"/mavros/state"};
    std::string ugv_cmd_vel_topic_suffix_{"/cmd_vel"};
    std::string ugv_twist_topic_suffix_{"/twist"};
    double publish_rate_{30.0};
    double pose_transform_publish_rate_{120.0};
    double scene_publish_rate_{10.0};
    double scene_path_publish_rate_{2.0};
    double path_publish_rate_{xgc2_robot_visualization::kDefaultPathPublishRateHz};
    int path_limit_{0};
    double path_history_duration_sec_{xgc2_robot_visualization::kDefaultPathHistoryDurationSec};
    double canonical_pose_timeout_sec_{0.5};
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
    bool track_ugv_{true};
    bool publish_markers_{true};
    bool publish_transforms_{true};
    bool publish_scene_update_{false};
    bool publish_scene_paths_{true};
    bool publish_paths_{false};
    bool scene_labels_published_{false};
    bool scene_ready_published_{false};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_auto_visualizer");
    GazeboAutoVisualizer visualizer;
    ros::spin();
    return 0;
}
