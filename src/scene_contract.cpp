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

bool isUavModel(const std::string& name) {
    static const std::regex pattern("^(uav|tello)[0-9]+$");
    return std::regex_match(name, pattern);
}

bool isUgvModel(const std::string& name) {
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

} // namespace

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

bool modelListsAreDisjoint(const std::set<std::string>& uav_models, const std::set<std::string>& ugv_models) {
    for (const std::string& model : uav_models) {
        if (ugv_models.count(model) != 0U) {
            return false;
        }
    }
    return true;
}

RobotModelKind selectRobotModelKind(const std::string& model_name, const std::set<std::string>& configured_uav_models,
                                    const std::set<std::string>& configured_ugv_models, bool allow_auto_discovery,
                                    bool track_ugv) {
    const bool configured_uav = configured_uav_models.count(model_name) != 0U;
    const bool configured_ugv = configured_ugv_models.count(model_name) != 0U;
    if (configured_uav && configured_ugv) {
        return RobotModelKind::kNone;
    }
    if (configured_uav) {
        return RobotModelKind::kUav;
    }
    if (configured_ugv) {
        return track_ugv ? RobotModelKind::kUgv : RobotModelKind::kNone;
    }
    if (!allow_auto_discovery) {
        return RobotModelKind::kNone;
    }
    if (isUavModel(model_name)) {
        return RobotModelKind::kUav;
    }
    if (track_ugv && isUgvModel(model_name)) {
        return RobotModelKind::kUgv;
    }
    return RobotModelKind::kNone;
}

std::string sceneEntityID(RobotModelKind kind, const std::string& model_name) {
    switch (kind) {
    case RobotModelKind::kUav:
        return "xgc2/px4/" + model_name;
    case RobotModelKind::kUgv:
        return "xgc2/scout/" + model_name;
    case RobotModelKind::kNone:
        break;
    }
    throw std::invalid_argument("scene entity requires a concrete robot model kind");
}

void appendSceneEntity(RobotModelKind kind, const std::string& model_name,
                       const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                       const ros::Time& timestamp, const std::string& frame_id, foxglove_msgs::SceneUpdate* update) {
    if (update == nullptr || first_marker > markers.markers.size()) {
        throw std::invalid_argument("scene entity output and marker range must be valid");
    }
    foxglove_msgs::SceneEntity entity;
    entity.timestamp = timestamp;
    entity.frame_id = frame_id;
    entity.id = sceneEntityID(kind, model_name);
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

} // namespace gazebo_sim_visualization
