#include "gazebo_sim_visualization/scene_contract.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <foxglove_msgs/Color.h>
#include <foxglove_msgs/LinePrimitive.h>
#include <foxglove_msgs/ModelPrimitive.h>
#include <foxglove_msgs/SceneEntity.h>
#include <foxglove_msgs/TextPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace gazebo_sim_visualization {
namespace {

std::string trim(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return std::string();
    }
    return value.substr(first, value.find_last_not_of(whitespace) - first + 1);
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

bool isFs150Model(const std::string& name) {
    static const std::regex pattern("^(uav|tello)[0-9]+$");
    return std::regex_match(name, pattern);
}

bool isScoutModel(const std::string& name) {
    static const std::regex pattern("^ugv[0-9]+$");
    const std::string normalized = lower(name);
    return std::regex_match(normalized, pattern) || normalized.find("scout") != std::string::npos;
}

geometry_msgs::Quaternion normalizedQuaternion(const geometry_msgs::Quaternion& source) {
    const double norm =
        std::sqrt(source.x * source.x + source.y * source.y + source.z * source.z + source.w * source.w);
    geometry_msgs::Quaternion result;
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        result.w = 1.0;
        return result;
    }
    result.x = source.x / norm;
    result.y = source.y / norm;
    result.z = source.z / norm;
    result.w = source.w / norm;
    return result;
}

geometry_msgs::Pose copyPose(const geometry_msgs::Pose& source) {
    geometry_msgs::Pose result;
    result.position = source.position;
    result.orientation = normalizedQuaternion(source.orientation);
    return result;
}

foxglove_msgs::Color copyColor(const std_msgs::ColorRGBA& source) {
    foxglove_msgs::Color result;
    result.r = source.r;
    result.g = source.g;
    result.b = source.b;
    result.a = source.a;
    return result;
}

int hexDigit(char value) {
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    if (value >= 'a' && value <= 'f') {
        return value - 'a' + 10;
    }
    return -1;
}

double hexChannel(const std::string& value, std::size_t offset) {
    const int high = hexDigit(value[offset]);
    const int low = hexDigit(value[offset + 1U]);
    if (high < 0 || low < 0) {
        throw std::invalid_argument("marker color must use canonical lowercase #rrggbb syntax");
    }
    return static_cast<double>(high * 16 + low) / 255.0;
}

} // namespace

SceneLabelStyle sceneLabelStyleFromMarkerColor(const std::string& marker_color) {
    if (marker_color.size() != 7U || marker_color.front() != '#') {
        throw std::invalid_argument("marker color must use canonical lowercase #rrggbb syntax");
    }
    SceneLabelStyle style;
    style.font_size = 0.24;
    style.color.r = hexChannel(marker_color, 1U);
    style.color.g = hexChannel(marker_color, 3U);
    style.color.b = hexChannel(marker_color, 5U);
    style.color.a = 1.0;
    return style;
}

std::set<std::string> parseModelNames(const std::string& csv) {
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

bool modelListsAreDisjoint(const std::set<std::string>& legacy_uav_models,
                           const std::set<std::string>& legacy_ugv_models, const std::set<std::string>& fs150_models,
                           const std::set<std::string>& scout_models, const std::set<std::string>& mecanum_models) {
    std::set<std::string> seen;
    for (const std::set<std::string>* models :
         {&legacy_uav_models, &legacy_ugv_models, &fs150_models, &scout_models, &mecanum_models}) {
        for (const std::string& model : *models) {
            if (!seen.insert(model).second) {
                return false;
            }
        }
    }
    return true;
}

RobotModelKind selectRobotModelKind(const std::string& model_name, const std::set<std::string>& configured_fs150_models,
                                    const std::set<std::string>& configured_scout_models,
                                    const std::set<std::string>& configured_mecanum_models, bool allow_auto_discovery,
                                    bool track_ugv) {
    const bool configured_fs150 = configured_fs150_models.count(model_name) != 0U;
    const bool configured_scout = configured_scout_models.count(model_name) != 0U;
    const bool configured_mecanum = configured_mecanum_models.count(model_name) != 0U;
    if (static_cast<int>(configured_fs150) + static_cast<int>(configured_scout) + static_cast<int>(configured_mecanum) >
        1) {
        return RobotModelKind::kNone;
    }
    if (configured_fs150) {
        return RobotModelKind::kFs150;
    }
    if (configured_scout) {
        return track_ugv ? RobotModelKind::kScout : RobotModelKind::kNone;
    }
    if (configured_mecanum) {
        return track_ugv ? RobotModelKind::kMecanum : RobotModelKind::kNone;
    }
    if (!allow_auto_discovery) {
        return RobotModelKind::kNone;
    }
    if (isFs150Model(model_name)) {
        return RobotModelKind::kFs150;
    }
    if (track_ugv && isScoutModel(model_name)) {
        return RobotModelKind::kScout;
    }
    return RobotModelKind::kNone;
}

std::string sceneEntityID(RobotModelKind kind, const std::string& model_name) {
    switch (kind) {
    case RobotModelKind::kFs150:
        return "xgc2/px4/" + model_name;
    case RobotModelKind::kScout:
        return "xgc2/scout/" + model_name;
    case RobotModelKind::kMecanum:
        return "xgc2/mecanum/" + model_name;
    case RobotModelKind::kNone:
        break;
    }
    throw std::invalid_argument("scene entity requires a concrete robot model kind");
}

std::string sceneEntityPartID(RobotModelKind kind, const std::string& model_name, SceneEntityPart part) {
    const std::string robot_id = sceneEntityID(kind, model_name);
    return part == SceneEntityPart::kPath ? robot_id + "/path" : robot_id + "/label";
}

SceneUpdateCadence::SceneUpdateCadence(double label_publish_rate, double path_publish_rate)
    : label_publish_rate_(std::max(1.0, label_publish_rate)), path_publish_rate_(std::max(0.1, path_publish_rate)) {}

namespace {

// One drift-free rate gate, shared by every cadence in this file so a pose, a
// path trail and a joint animation all advance the same way.
bool takeRateGate(const ros::Time& now, double rate, bool* initialized, ros::Time* last_stamp) {
    if (initialized == nullptr || last_stamp == nullptr) {
        throw std::invalid_argument("scene update cadence gate state must be valid");
    }
    if (!*initialized || now < *last_stamp) {
        *initialized = true;
        *last_stamp = now;
        return true;
    }

    const double elapsed = (now - *last_stamp).toSec();
    const double period = 1.0 / rate;
    if (elapsed + 1.0e-9 < period) {
        return false;
    }

    // Advance by whole periods instead of assigning `now`. This avoids
    // accumulating timer quantisation drift when the source timer frequency is
    // not an integer multiple of the requested SceneUpdate rate.
    const double elapsed_periods = std::floor((elapsed + 1.0e-9) / period);
    *last_stamp += ros::Duration(elapsed_periods * period);
    return true;
}

} // namespace

bool SceneUpdateCadence::takeGate(const ros::Time& now, double rate, bool* initialized, ros::Time* last_stamp) {
    return takeRateGate(now, rate, initialized, last_stamp);
}

PublishCadence::PublishCadence(double publish_rate) : publish_rate_(publish_rate > 0.0 ? publish_rate : 1.0) {}

bool PublishCadence::take(const ros::Time& now) {
    return takeRateGate(now, publish_rate_, &initialized_, &last_stamp_);
}

SceneUpdateCadenceDecision SceneUpdateCadence::take(const ros::Time& now) {
    SceneUpdateCadenceDecision decision;
    decision.publish_label = takeGate(now, label_publish_rate_, &label_initialized_, &last_label_stamp_);
    decision.publish_path = takeGate(now, path_publish_rate_, &path_initialized_, &last_path_stamp_);
    return decision;
}

namespace {

bool markerBelongsToPart(const visualization_msgs::Marker& marker, SceneEntityPart part) {
    static const std::string path_suffix = "_actual_path";
    const bool is_path = marker.type == visualization_msgs::Marker::LINE_STRIP &&
                         marker.ns.size() >= path_suffix.size() &&
                         marker.ns.compare(marker.ns.size() - path_suffix.size(), path_suffix.size(), path_suffix) == 0;
    const bool is_label = marker.type == visualization_msgs::Marker::TEXT_VIEW_FACING;
    return part == SceneEntityPart::kPath ? is_path : is_label;
}

// A label is anchored rather than positioned: its entity names the robot's own
// label frame and asks the viewer to follow it. Every other part is drawn in
// world coordinates exactly where the message placed it.
bool partFollowsItsFrame(const SceneEntityPart* part) {
    return part != nullptr && *part == SceneEntityPart::kLabel;
}

void appendSceneEntityImpl(RobotModelKind kind, const std::string& entity_id,
                           const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                           const ros::Time& timestamp, const std::string& frame_id, const SceneEntityPart* part,
                           const SceneLabelStyle& label_style, foxglove_msgs::SceneUpdate* update) {
    if (update == nullptr || first_marker > markers.markers.size()) {
        throw std::invalid_argument("scene entity output and marker range must be valid");
    }
    if (kind == RobotModelKind::kNone) {
        throw std::invalid_argument("scene entity requires a concrete robot model kind");
    }
    foxglove_msgs::SceneEntity entity;
    entity.timestamp = timestamp;
    entity.frame_id = frame_id;
    entity.id = entity_id;
    entity.lifetime = ros::Duration(0.0);
    entity.frame_locked = false;
    if (partFollowsItsFrame(part)) {
        // Take the frame from the marker itself: the visualizer that built it
        // is the one that knows which anchor it belongs to, and a caller
        // passing the world frame for every part cannot know that.
        for (std::size_t index = first_marker; index < markers.markers.size(); ++index) {
            if (markerBelongsToPart(markers.markers[index], *part)) {
                entity.frame_id = markers.markers[index].header.frame_id;
                entity.frame_locked = true;
                break;
            }
        }
    }

    for (std::size_t index = first_marker; index < markers.markers.size(); ++index) {
        const visualization_msgs::Marker& marker = markers.markers[index];
        if (part != nullptr && !markerBelongsToPart(marker, *part)) {
            continue;
        }
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
            primitive.font_size = label_style.font_size;
            primitive.scale_invariant = false;
            primitive.color = label_style.color;
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

} // namespace

void appendSceneEntity(RobotModelKind kind, const std::string& model_name,
                       const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                       const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                       foxglove_msgs::SceneUpdate* update) {
    appendSceneEntityImpl(kind, sceneEntityID(kind, model_name), markers, first_marker, timestamp, frame_id, nullptr,
                          label_style, update);
}

void appendSceneEntityPart(RobotModelKind kind, const std::string& model_name, SceneEntityPart part,
                           const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                           const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                           foxglove_msgs::SceneUpdate* update) {
    appendSceneEntityImpl(kind, sceneEntityPartID(kind, model_name, part), markers, first_marker, timestamp, frame_id,
                          &part, label_style, update);
}

namespace {

std::string normalizedFrameLabel(const std::string& frame_id) {
    std::string normalized;
    normalized.reserve(frame_id.size());
    for (unsigned char character : frame_id) {
        if (character == ' ' || character == '\t' || character == '\r' || character == '\n') {
            continue;
        }
        normalized.push_back(static_cast<char>(std::tolower(character)));
    }
    if (!normalized.empty() && normalized.front() == '/') {
        normalized.erase(normalized.begin());
    }
    const std::size_t slash = normalized.rfind('/');
    if (slash != std::string::npos && slash + 1 < normalized.size()) {
        normalized = normalized.substr(slash + 1);
    }
    return normalized;
}

bool isFreshWorldSource(const WorldEnuPoseSource& source, const ros::Time& now, double timeout_sec) {
    if (!source.available) {
        return false;
    }
    if (timeout_sec <= 0.0) {
        return true;
    }
    if (source.stamp.isZero()) {
        return false;
    }
    return now - source.stamp <= ros::Duration(timeout_sec);
}

} // namespace

bool isWorldFixedFrame(const std::string& frame_id) {
    return normalizedFrameLabel(frame_id) == "world";
}

WorldEnuPoseSelection selectWorldEnuPose(const WorldEnuPoseSource& vrpn, const WorldEnuPoseSource& gazebo,
                                         const WorldEnuPoseSource& local_pose, const WorldEnuPoseSource& tf_pose,
                                         const ros::Time& now, double vrpn_timeout_sec,
                                         double local_pose_timeout_sec, double tf_pose_timeout_sec) {
    WorldEnuPoseSelection selected;
    if (isFreshWorldSource(vrpn, now, vrpn_timeout_sec) && isWorldFixedFrame(vrpn.frame_id)) {
        selected.found = true;
        selected.pose = vrpn.pose;
        selected.stamp = vrpn.stamp;
        selected.frame_id = vrpn.frame_id;
        selected.source = "vrpn";
        return selected;
    }
    if (gazebo.available && isWorldFixedFrame(gazebo.frame_id)) {
        selected.found = true;
        selected.pose = gazebo.pose;
        selected.stamp = gazebo.stamp;
        selected.frame_id = gazebo.frame_id;
        selected.source = "gazebo";
        return selected;
    }
    if (isFreshWorldSource(local_pose, now, local_pose_timeout_sec) && isWorldFixedFrame(local_pose.frame_id)) {
        selected.found = true;
        selected.pose = local_pose.pose;
        selected.stamp = local_pose.stamp;
        selected.frame_id = local_pose.frame_id;
        selected.source = "local-pose";
        return selected;
    }
    if (isFreshWorldSource(tf_pose, now, tf_pose_timeout_sec) && isWorldFixedFrame(tf_pose.frame_id)) {
        selected.found = true;
        selected.pose = tf_pose.pose;
        selected.stamp = tf_pose.stamp;
        selected.frame_id = tf_pose.frame_id;
        selected.source = "tf";
        return selected;
    }
    return selected;
}

geometry_msgs::TransformStamped worldFixedFrameRoot(const std::string& frame_id, const ros::Time& stamp) {
    geometry_msgs::TransformStamped origin;
    origin.header.stamp = stamp;
    origin.header.frame_id = frame_id;
    origin.child_frame_id = kWorldFixedFrameRootChild;
    origin.transform.rotation.w = 1.0;
    return origin;
}

} // namespace gazebo_sim_visualization
