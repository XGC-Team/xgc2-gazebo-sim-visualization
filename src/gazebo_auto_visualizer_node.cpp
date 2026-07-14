#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "xgc2_robot_visualization/fs150_uav_visualizer.hpp"
#include "xgc2_robot_visualization/scout_ugv_visualizer.hpp"

#include <gazebo_msgs/ModelStates.h>
#include <foxglove_msgs/Color.h>
#include <foxglove_msgs/LinePrimitive.h>
#include <foxglove_msgs/ModelPrimitive.h>
#include <foxglove_msgs/SceneEntity.h>
#include <foxglove_msgs/SceneEntityDeletion.h>
#include <foxglove_msgs/SceneUpdate.h>
#include <foxglove_msgs/TextPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/transform_broadcaster.h>
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

bool isUavModel(const std::string& name) {
    static const std::regex pattern("^(uav|tello)[0-9]+$");
    return std::regex_match(name, pattern);
}

bool isUgvModel(const std::string& name) {
    static const std::regex pattern("^ugv[0-9]+$");
    const std::string normalized = lower(name);
    return std::regex_match(normalized, pattern) || normalized.find("scout") != std::string::npos;
}

std::string trim(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return std::string();
    }
    return value.substr(first, value.find_last_not_of(whitespace) - first + 1);
}

std::set<std::string> parseNames(const std::string& csv) {
    std::set<std::string> names;
    std::stringstream stream(csv);
    std::string item;
    while (std::getline(stream, item, ',')) {
        item = trim(item);
        if (!item.empty()) {
            names.insert(item);
        }
    }
    return names;
}

foxglove_msgs::Color copyColor(const std_msgs::ColorRGBA& source) {
    foxglove_msgs::Color color;
    color.r = source.r;
    color.g = source.g;
    color.b = source.b;
    color.a = source.a;
    return color;
}

} // namespace

class GazeboAutoVisualizer {
  public:
    GazeboAutoVisualizer() : private_nh_("~") {
        private_nh_.param<std::string>("frame_id", frame_id_, "world");
        private_nh_.param<std::string>("model_states_topic", model_states_topic_, "/gazebo/model_states");
        private_nh_.param<std::string>("vrpn_pose_prefix", vrpn_pose_prefix_, "/vrpn_client_node");
        private_nh_.param<std::string>("tracked_uav_models", tracked_uav_models_csv_, "");
        private_nh_.param<std::string>("scene_update_topic", scene_update_topic_, "/xgc/scene");
        private_nh_.param("allow_auto_discovery", allow_auto_discovery_, true);
        private_nh_.param("track_ugv", track_ugv_, true);
        private_nh_.param("publish_markers", publish_markers_, true);
        private_nh_.param("publish_transforms", publish_transforms_, true);
        private_nh_.param("publish_scene_update", publish_scene_update_, false);
        private_nh_.param("publish_rate", publish_rate_, 30.0);
        private_nh_.param("path_publish_rate", path_publish_rate_, 10.0);
        private_nh_.param("path_limit", path_limit_, 3000);
        private_nh_.param("path_history_duration", path_history_duration_sec_, 15.0);
        private_nh_.param("vrpn_timeout", vrpn_timeout_sec_, 0.5);
        private_nh_.param("mavros_state_timeout", mavros_state_timeout_sec_, 2.0);
        private_nh_.param<std::string>("mavros_state_topic_suffix", mavros_state_topic_suffix_, "/mavros/state");
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

        publish_rate_ = std::max(1.0, publish_rate_);
        path_publish_rate_ = std::max(1.0, path_publish_rate_);
        mavros_state_timeout_sec_ = std::max(0.0, mavros_state_timeout_sec_);
        ugv_motion_timeout_sec_ = std::max(0.0, ugv_motion_timeout_sec_);
        ugv_visual_wheel_radius_ = std::max(0.001, ugv_visual_wheel_radius_);
        ugv_visual_track_width_ = std::max(0.001, ugv_visual_track_width_);
        ugv_wheel_motion_deadband_ = std::max(0.0, ugv_wheel_motion_deadband_);
        ugv_max_visual_wheel_speed_rad_s_ = std::max(0.0, ugv_max_visual_wheel_speed_rad_s_);
        path_history_duration_sec_ = std::max(0.0, path_history_duration_sec_);
        if (path_history_duration_sec_ > 0.0) {
            path_limit_ = static_cast<int>(std::ceil(path_history_duration_sec_ * path_publish_rate_)) + 1;
        }
        path_limit_ = std::max(2, path_limit_);

        xgc2_robot_visualization::Fs150UavVisualizer::Config uav_config;
        uav_config.frame_id = frame_id_;
        uav_config.rotor_speed_rad_s = rotor_speed_rad_s_;
        uav_config.mesh_scale = uav_mesh_scale_;
        uav_config.path_publish_rate = path_publish_rate_;
        uav_config.path_limit = path_limit_;
        uav_visualizer_.reset(new xgc2_robot_visualization::Fs150UavVisualizer(uav_config));

        xgc2_robot_visualization::ScoutUgvVisualizer::Config ugv_config;
        ugv_config.frame_id = frame_id_;
        ugv_config.mesh_scale = ugv_mesh_scale_;
        ugv_config.path_publish_rate = path_publish_rate_;
        ugv_config.path_limit = path_limit_;
        ugv_config.visual_wheel_radius = ugv_visual_wheel_radius_;
        ugv_config.visual_track_width = ugv_visual_track_width_;
        ugv_config.wheel_motion_deadband = ugv_wheel_motion_deadband_;
        ugv_config.max_visual_wheel_speed_rad_s = ugv_max_visual_wheel_speed_rad_s_;
        ugv_visualizer_.reset(new xgc2_robot_visualization::ScoutUgvVisualizer(ugv_config));

        configured_uav_models_ = parseNames(tracked_uav_models_csv_);
        for (const std::string& name : configured_uav_models_) {
            ensureTrackedModel(name, ModelKind::kUav);
        }

        if (publish_markers_) {
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
        }
        if (publish_scene_update_) {
            scene_update_pub_ = nh_.advertise<foxglove_msgs::SceneUpdate>(scene_update_topic_, 1, true);
            publishSceneReset();
        }

        model_states_sub_ = nh_.subscribe(model_states_topic_, 5, &GazeboAutoVisualizer::modelStatesCallback, this);
        publish_timer_ =
            nh_.createTimer(ros::Duration(1.0 / publish_rate_), &GazeboAutoVisualizer::publishCallback, this);
    }

  private:
    enum class ModelKind { kUav, kUgv };

    struct TrackedModel {
        std::string name;
        ModelKind kind;
        geometry_msgs::Pose gazebo_pose;
        geometry_msgs::Pose vrpn_pose;
        bool has_gazebo_pose{false};
        bool has_vrpn_pose{false};
        bool has_mavros_state{false};
        bool mavros_armed{false};
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

    std::map<std::string, TrackedModel>::iterator ensureTrackedModel(const std::string& name, ModelKind kind) {
        auto existing = models_.find(name);
        if (existing != models_.end()) {
            return existing;
        }

        TrackedModel model;
        model.name = name;
        model.kind = kind;
        model.vrpn_subscriber = subscribeVrpn(name);
        if (kind == ModelKind::kUav) {
            model.mavros_state_subscriber = subscribeMavrosState(name);
        } else {
            model.cmd_vel_subscriber = subscribeUgvCmdVel(name);
            model.twist_subscriber = subscribeUgvTwist(name);
        }
        auto inserted = models_.emplace(name, std::move(model)).first;
        ROS_INFO("[gazebo_auto_visualizer] Tracking %s model '%s'",
                 kind == ModelKind::kUav ? "uav" : "ugv", name.c_str());
        return inserted;
    }

    void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg) {
        for (std::size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i) {
            const std::string& name = msg->name[i];
            const bool configured_uav = configured_uav_models_.count(name) != 0;
            const bool uav = configured_uav || (allow_auto_discovery_ && isUavModel(name));
            const bool ugv = track_ugv_ && isUgvModel(name);
            if (!uav && !ugv) {
                continue;
            }
            if (!isFinite(msg->pose[i])) {
                ROS_WARN_THROTTLE(2.0,
                                  "[gazebo_auto_visualizer] Ignoring non-finite pose "
                                  "for model '%s'",
                                  name.c_str());
                continue;
            }
            auto it = ensureTrackedModel(name, uav ? ModelKind::kUav : ModelKind::kUgv);
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

    ros::Subscriber subscribeMavrosState(const std::string& name) {
        const std::string topic = modelScopedTopic(name, mavros_state_topic_suffix_, "/mavros/state");
        return nh_.subscribe<mavros_msgs::State>(topic, 10,
                                                 [this, name](const mavros_msgs::StateConstPtr& msg) {
                                                     mavrosStateCallback(name, msg);
                                                 });
    }

    ros::Subscriber subscribeUgvCmdVel(const std::string& name) {
        const std::string topic = modelScopedTopic(name, ugv_cmd_vel_topic_suffix_, "/cmd_vel");
        return nh_.subscribe<geometry_msgs::Twist>(topic, 10,
                                                   [this, name](const geometry_msgs::TwistConstPtr& msg) {
                                                       ugvCmdVelCallback(name, msg);
                                                   });
    }

    ros::Subscriber subscribeUgvTwist(const std::string& name) {
        const std::string topic = modelScopedTopic(name, ugv_twist_topic_suffix_, "/twist");
        return nh_.subscribe<geometry_msgs::TwistStamped>(
            topic, 10, [this, name](const geometry_msgs::TwistStampedConstPtr& msg) {
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

    const geometry_msgs::Pose* selectPose(const TrackedModel& model) const {
        if (model.has_vrpn_pose && ros::Time::now() - model.vrpn_stamp <= ros::Duration(vrpn_timeout_sec_)) {
            return &model.vrpn_pose;
        }
        return model.has_gazebo_pose ? &model.gazebo_pose : nullptr;
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
        return !normalized.empty() &&
               (normalized == normalized_visual_frame || normalized == "world" || normalized == "map" ||
                normalized == "odom");
    }

    xgc2_robot_visualization::UgvVisualState makeUgvVisualState(const TrackedModel& model,
                                                                const geometry_msgs::Pose& pose,
                                                                const ros::Time& now) const {
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

    void publishSceneReset() {
        foxglove_msgs::SceneUpdate update;
        foxglove_msgs::SceneEntityDeletion deletion;
        deletion.timestamp = ros::Time::now();
        deletion.type = foxglove_msgs::SceneEntityDeletion::ALL;
        update.deletions.push_back(deletion);
        scene_update_pub_.publish(update);
    }

    void appendSceneEntity(const TrackedModel& model, const visualization_msgs::MarkerArray& markers,
                           std::size_t first_marker, const ros::Time& now,
                           foxglove_msgs::SceneUpdate* update) const {
        foxglove_msgs::SceneEntity entity;
        entity.timestamp = now;
        entity.frame_id = frame_id_;
        entity.id = "xgc2/px4/" + model.name;
        entity.lifetime = ros::Duration(0.0);
        entity.frame_locked = false;

        for (std::size_t index = first_marker; index < markers.markers.size(); ++index) {
            const visualization_msgs::Marker& marker = markers.markers[index];
            switch (marker.type) {
            case visualization_msgs::Marker::MESH_RESOURCE: {
                foxglove_msgs::ModelPrimitive primitive;
                primitive.pose = copyPose(marker.pose);
                primitive.scale = marker.scale;
                primitive.color = copyColor(marker.color);
                primitive.override_color = !marker.mesh_use_embedded_materials;
                primitive.url = marker.mesh_resource;
                entity.models.push_back(std::move(primitive));
                break;
            }
            case visualization_msgs::Marker::LINE_STRIP: {
                foxglove_msgs::LinePrimitive primitive;
                primitive.type = foxglove_msgs::LinePrimitive::LINE_STRIP;
                primitive.pose = copyPose(marker.pose);
                primitive.thickness = marker.scale.x;
                primitive.scale_invariant = false;
                primitive.points = marker.points;
                primitive.color = copyColor(marker.color);
                entity.lines.push_back(std::move(primitive));
                break;
            }
            case visualization_msgs::Marker::TEXT_VIEW_FACING: {
                foxglove_msgs::TextPrimitive primitive;
                primitive.pose = copyPose(marker.pose);
                primitive.billboard = true;
                primitive.font_size = marker.scale.z;
                primitive.scale_invariant = false;
                primitive.color = copyColor(marker.color);
                primitive.text = marker.text;
                entity.texts.push_back(std::move(primitive));
                break;
            }
            default:
                break;
            }
        }

        if (!entity.models.empty() || !entity.lines.empty() || !entity.texts.empty()) {
            update->entities.push_back(std::move(entity));
        }
    }

    void publishCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        visualization_msgs::MarkerArray markers;
        std::vector<geometry_msgs::TransformStamped> transforms;
        foxglove_msgs::SceneUpdate scene_update;
        for (auto& entry : models_) {
            TrackedModel& model = entry.second;
            const geometry_msgs::Pose* pose = selectPose(model);
            if (pose == nullptr) {
                continue;
            }
            if (model.kind == ModelKind::kUav) {
                const std::size_t first_marker = markers.markers.size();
                xgc2_robot_visualization::UavVisualState state;
                state.name = model.name;
                state.pose = *pose;
                state.rotors_active = rotorsActive(model, now);
                state.stamp = now;
                uav_visualizer_->append(state, &markers, &transforms);
                if (publish_scene_update_) {
                    appendSceneEntity(model, markers, first_marker, now, &scene_update);
                }
            } else {
                ugv_visualizer_->append(makeUgvVisualState(model, *pose, now), &markers, &transforms);
            }
        }

        if (publish_transforms_ && !transforms.empty()) {
            tf_broadcaster_.sendTransform(transforms);
        }
        if (publish_markers_) {
            marker_pub_.publish(markers);
        }
        if (publish_scene_update_) {
            scene_update_pub_.publish(scene_update);
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::Publisher scene_update_pub_;
    ros::Subscriber model_states_sub_;
    ros::Timer publish_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::map<std::string, TrackedModel> models_;
    std::set<std::string> configured_uav_models_;
    std::unique_ptr<xgc2_robot_visualization::Fs150UavVisualizer> uav_visualizer_;
    std::unique_ptr<xgc2_robot_visualization::ScoutUgvVisualizer> ugv_visualizer_;

    std::string frame_id_;
    std::string model_states_topic_;
    std::string vrpn_pose_prefix_;
    std::string tracked_uav_models_csv_;
    std::string scene_update_topic_{"/xgc/scene"};
    std::string mavros_state_topic_suffix_{"/mavros/state"};
    std::string ugv_cmd_vel_topic_suffix_{"/cmd_vel"};
    std::string ugv_twist_topic_suffix_{"/twist"};
    double publish_rate_{30.0};
    double path_publish_rate_{10.0};
    int path_limit_{3000};
    double path_history_duration_sec_{15.0};
    double vrpn_timeout_sec_{0.5};
    double mavros_state_timeout_sec_{2.0};
    double ugv_motion_timeout_sec_{0.5};
    double rotor_speed_rad_s_{70.0};
    double uav_mesh_scale_{1.0};
    double ugv_mesh_scale_{1.0};
    double ugv_visual_wheel_radius_{0.08};
    double ugv_visual_track_width_{0.416};
    double ugv_wheel_motion_deadband_{0.02};
    double ugv_max_visual_wheel_speed_rad_s_{35.0};
    bool allow_auto_discovery_{true};
    bool track_ugv_{true};
    bool publish_markers_{true};
    bool publish_transforms_{true};
    bool publish_scene_update_{false};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_auto_visualizer");
    GazeboAutoVisualizer visualizer;
    ros::spin();
    return 0;
}
