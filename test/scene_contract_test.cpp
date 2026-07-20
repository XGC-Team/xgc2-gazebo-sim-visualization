#include "gazebo_sim_visualization/scene_contract.hpp"

#include <set>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <visualization_msgs/Marker.h>

namespace gazebo_sim_visualization {
namespace {

TEST(SceneContract, ImmutableListsSelectOnlyConfiguredModels) {
    const std::set<std::string> uavs{"uav1", "uav2"};
    const std::set<std::string> ugvs{"ugv1"};

    EXPECT_EQ(selectRobotModelKind("uav1", uavs, ugvs, false, true), RobotModelKind::kUav);
    EXPECT_EQ(selectRobotModelKind("ugv1", uavs, ugvs, false, true), RobotModelKind::kUgv);
    EXPECT_EQ(selectRobotModelKind("uav9", uavs, ugvs, false, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("ugv9", uavs, ugvs, false, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("ground_plane", uavs, ugvs, false, true), RobotModelKind::kNone);
}

TEST(SceneContract, ModelListsMustBeDisjoint) {
    EXPECT_TRUE(modelListsAreDisjoint({"uav1"}, {"ugv1"}));
    EXPECT_FALSE(modelListsAreDisjoint({"uav1", "shared"}, {"shared", "ugv1"}));
}

TEST(SceneContract, ScoutMarkersBecomeScoutSceneEntity) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://scout_description/meshes/base_link.dae";
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);

    foxglove_msgs::SceneUpdate update;
    appendSceneEntity(RobotModelKind::kUgv, "ugv1", markers, 0U, ros::Time(42, 0), "world", &update);

    ASSERT_EQ(update.entities.size(), 1U);
    EXPECT_EQ(update.entities[0].id, "xgc2/scout/ugv1");
    EXPECT_EQ(update.entities[0].frame_id, "world");
    ASSERT_EQ(update.entities[0].models.size(), 1U);
    EXPECT_EQ(update.entities[0].models[0].url, marker.mesh_resource);
}

TEST(SceneContract, EntityIDsRemainKindScoped) {
    EXPECT_EQ(sceneEntityID(RobotModelKind::kUav, "uav1"), "xgc2/px4/uav1");
    EXPECT_EQ(sceneEntityID(RobotModelKind::kUgv, "ugv1"), "xgc2/scout/ugv1");
    EXPECT_THROW(sceneEntityID(RobotModelKind::kNone, "unknown"), std::invalid_argument);
}

} // namespace
} // namespace gazebo_sim_visualization

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
