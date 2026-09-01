#pragma once

#include <cstddef>
#include <set>
#include <string>

#include <foxglove_msgs/Color.h>
#include <foxglove_msgs/SceneUpdate.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

namespace gazebo_sim_visualization {

// Each concrete vehicle family owns a rendering path. Callers must provide the
// concrete lists so a future airframe cannot silently inherit the FS150 mesh.
enum class RobotModelKind { kNone, kFs150, kScout, kMecanum };

// SceneEntity updates replace the complete entity with the same ID. Keep the
// high-rate robot geometry and the lower-rate path in separate entities so a
// pose update cannot erase a path which was intentionally not retransmitted.
//
// The label is separate for a different reason. A trail is drawn where the
// message said it was; the label is anchored to a frame and drawn wherever that
// frame currently is, and one entity names exactly one frame.
//
// Robot geometry is not here at all. Every mesh this scene used to ship is a
// link of the robot's own URDF, which the viewer loads once from a parameter
// and places from transforms -- so shipping it again, in world coordinates, at
// the scene cadence, drew each vehicle twice: once smoothly from the
// description and once two-per-second on top of it.
enum class SceneEntityPart { kPath, kLabel };

struct SceneUpdateCadenceDecision {
    bool publish_label{false};
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
    SceneUpdateCadence(double label_publish_rate, double path_publish_rate);

    SceneUpdateCadenceDecision take(const ros::Time& now);

  private:
    bool takeGate(const ros::Time& now, double rate, bool* initialized, ros::Time* last_stamp);

    double label_publish_rate_;
    double path_publish_rate_;
    bool label_initialized_{false};
    bool path_initialized_{false};
    ros::Time last_label_stamp_;
    ros::Time last_path_stamp_;
};

std::set<std::string> parseModelNames(const std::string& csv);

bool modelListsAreDisjoint(const std::set<std::string>& fs150_models, const std::set<std::string>& scout_models,
                           const std::set<std::string>& mecanum_models);

RobotModelKind selectRobotModelKind(const std::string& model_name, const std::set<std::string>& configured_fs150_models,
                                    const std::set<std::string>& configured_scout_models,
                                    const std::set<std::string>& configured_mecanum_models, bool track_ugv);

std::string sceneEntityID(RobotModelKind kind, const std::string& model_name);

std::string sceneEntityPartID(RobotModelKind kind, const std::string& model_name, SceneEntityPart part);

// Experiment slot identity for the canonical world pose: /<namespace>/pose.
// Offset and hybridSource are already applied by the upstream projection.
std::string canonicalSlotPoseTopic(const std::string& ros_namespace);

// Replace only the operator-facing text generated for one Robot. The concrete
// robot kind owns the class word (FS150 -> UAV, Scout/Mecanum -> UGV), while
// the canonical lowercase /uavN or /ugvN namespace owns N. This also keeps a
// mixed-scene Scout in slot /uav7 visibly identified as UGV 7 without changing
// its ROS interface or its scene-model marker/frame identity.
void applyRobotMarkerLabel(visualization_msgs::MarkerArray* markers, std::size_t first_marker, RobotModelKind kind,
                           const std::string& ros_namespace);

void appendSceneEntity(RobotModelKind kind, const std::string& model_name,
                       const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                       const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                       foxglove_msgs::SceneUpdate* update);

void appendSceneEntityPart(RobotModelKind kind, const std::string& model_name, SceneEntityPart part,
                           const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                           const ros::Time& timestamp, const std::string& frame_id, const SceneLabelStyle& label_style,
                           foxglove_msgs::SceneUpdate* update);

// Body/path poses come only from the slot canonical pose already in the product
// Fixed Frame (world ENU z-up). map/odom/NED are not converted here. There is
// no source priority and no fallback among VRPN, Gazebo, MAVROS, or TF.
bool isWorldFixedFrame(const std::string& frame_id);

struct CanonicalPoseSample {
    bool available{false};
    geometry_msgs::Pose pose;
    ros::Time stamp;
    std::string frame_id;
};

struct CanonicalWorldPose {
    bool found{false};
    geometry_msgs::Pose pose;
    ros::Time stamp;
    std::string frame_id;
};

// Accept the single canonical world-ENU pose, or nothing. timeout_sec is a
// freshness window on that one sample; it does not select a substitute source.
CanonicalWorldPose selectCanonicalWorldPose(const CanonicalPoseSample& pose, const ros::Time& now, double timeout_sec);

// Identity child of the Fixed Frame published on /tf so RViz's Fixed Frame
// (the parent) exists in the TF tree. Displays consume the parent, not the child.
constexpr const char* kWorldFixedFrameRootChild = "xgc_origin";
geometry_msgs::TransformStamped worldFixedFrameRoot(const std::string& frame_id, const ros::Time& stamp);

// Runtime readiness is a whole frozen-roster fact. A reset SceneUpdate or the
// first Robot pose must not make a multi-Robot viewer ready while siblings are
// still absent from its transform and Path trees.
bool frozenVisualizationRosterReady(std::size_t tracked_models, std::size_t world_poses);

} // namespace gazebo_sim_visualization
