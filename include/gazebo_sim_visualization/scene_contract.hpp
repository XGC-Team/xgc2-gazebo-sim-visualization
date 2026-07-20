#pragma once

#include <cstddef>
#include <set>
#include <string>

#include <foxglove_msgs/SceneUpdate.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

namespace gazebo_sim_visualization {

enum class RobotModelKind { kNone, kUav, kUgv };

std::set<std::string> parseModelNames(const std::string& csv);

bool modelListsAreDisjoint(const std::set<std::string>& uav_models, const std::set<std::string>& ugv_models);

RobotModelKind selectRobotModelKind(const std::string& model_name, const std::set<std::string>& configured_uav_models,
                                    const std::set<std::string>& configured_ugv_models, bool allow_auto_discovery,
                                    bool track_ugv);

std::string sceneEntityID(RobotModelKind kind, const std::string& model_name);

void appendSceneEntity(RobotModelKind kind, const std::string& model_name,
                       const visualization_msgs::MarkerArray& markers, std::size_t first_marker,
                       const ros::Time& timestamp, const std::string& frame_id, foxglove_msgs::SceneUpdate* update);

} // namespace gazebo_sim_visualization
