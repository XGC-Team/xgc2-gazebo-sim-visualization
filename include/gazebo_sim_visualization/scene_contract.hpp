#pragma once

#include <cstddef>
#include <set>
#include <string>

#include <foxglove_msgs/Color.h>
#include <foxglove_msgs/SceneUpdate.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

namespace gazebo_sim_visualization {

// Each concrete vehicle family owns a rendering path. Legacy uav/ugv parameter
// names remain aliases only; new callers must provide the concrete lists so a
// future airframe cannot silently inherit the FS150 mesh.
enum class RobotModelKind { kNone, kFs150, kScout, kMecanum };

// SceneEntity updates replace the complete entity with the same ID. Keep the
// high-rate robot geometry and the lower-rate path in separate entities so a
// pose update cannot erase a path which was intentionally not retransmitted.
enum class SceneEntityPart { kRobot, kPath };

struct SceneUpdateCadenceDecision {
    bool publish_robot{false};
    bool publish_path{false};
};

struct SceneLabelStyle {
    double font_size{0.0};
    foxglove_msgs::Color color;
};

SceneLabelStyle sceneLabelStyleFromMarkerColor(const std::string& marker_color);

// PublishCadence is one rate gate. It exists so a publisher can give each kind
// of fact its own cadence -- a pose, a path trail and a rotor animation are not
// equally urgent -- without every caller reimplementing drift-free gating.
class PublishCadence {
  public:
    explicit PublishCadence(double publish_rate);

    bool take(const ros::Time& now);

  private:
    double publish_rate_;
    bool initialized_{false};
    ros::Time last_stamp_;
};

class SceneUpdateCadence {
  public:
    SceneUpdateCadence(double robot_publish_rate, double path_publish_rate);

    SceneUpdateCadenceDecision take(const ros::Time& now);

  private:
    bool takeGate(const ros::Time& now, double rate, bool* initialized, ros::Time* last_stamp);

    double robot_publish_rate_;
    double path_publish_rate_;
    bool robot_initialized_{false};
    bool path_initialized_{false};
    ros::Time last_robot_stamp_;
    ros::Time last_path_stamp_;
};

std::set<std::string> parseModelNames(const std::string& csv);

bool modelListsAreDisjoint(const std::set<std::string>& legacy_uav_models,
                           const std::set<std::string>& legacy_ugv_models,
                           const std::set<std::string>& fs150_models,
                           const std::set<std::string>& scout_models,
                           const std::set<std::string>& mecanum_models);

RobotModelKind selectRobotModelKind(const std::string& model_name, const std::set<std::string>& configured_fs150_models,
                                    const std::set<std::string>& configured_scout_models,
                                    const std::set<std::string>& configured_mecanum_models,
                                    bool allow_auto_discovery, bool track_ugv);

std::string sceneEntityID(RobotModelKind kind, const std::string& model_name);

std::string sceneEntityPartID(RobotModelKind kind, const std::string& model_name, SceneEntityPart part);

void appendSceneEntity(RobotModelKind kind, const std::string& model_name,
                       const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                       const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                       foxglove_msgs::SceneUpdate* update);

void appendSceneEntityPart(RobotModelKind kind, const std::string& model_name, SceneEntityPart part,
                           const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                           const ros::Time& timestamp, const std::string& frame_id,
                           const SceneLabelStyle& label_style, foxglove_msgs::SceneUpdate* update);

} // namespace gazebo_sim_visualization
