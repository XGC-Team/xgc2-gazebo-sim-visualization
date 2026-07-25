#include "gazebo_sim_visualization/scene_contract.hpp"

#include <set>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <ros/serialization.h>
#include <visualization_msgs/Marker.h>

namespace gazebo_sim_visualization {
namespace {

SceneLabelStyle blackLabelStyle() {
    return sceneLabelStyleFromMarkerColor("#000000");
}

TEST(SceneContract, MarkerColorIsStrictAndProducesOpaqueTextWithUnifiedFontSize) {
    const SceneLabelStyle black = blackLabelStyle();
    EXPECT_DOUBLE_EQ(black.font_size, 0.24);
    EXPECT_DOUBLE_EQ(black.color.r, 0.0);
    EXPECT_DOUBLE_EQ(black.color.g, 0.0);
    EXPECT_DOUBLE_EQ(black.color.b, 0.0);
    EXPECT_DOUBLE_EQ(black.color.a, 1.0);

    const SceneLabelStyle custom = sceneLabelStyleFromMarkerColor("#1a2b3c");
    EXPECT_NEAR(custom.color.r, 26.0 / 255.0, 1.0e-12);
    EXPECT_NEAR(custom.color.g, 43.0 / 255.0, 1.0e-12);
    EXPECT_NEAR(custom.color.b, 60.0 / 255.0, 1.0e-12);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("000000"), std::invalid_argument);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#FFFFFF"), std::invalid_argument);
    EXPECT_THROW(sceneLabelStyleFromMarkerColor("#00000000"), std::invalid_argument);
}

TEST(SceneContract, FS150ScoutAndMecanumLabelsUseOneSceneStyle) {
    const RobotModelKind kinds[] = {
        RobotModelKind::kFs150,
        RobotModelKind::kScout,
        RobotModelKind::kMecanum,
    };
    const char* names[] = {"uav1", "ugv1", "mecanum1"};
    const SceneLabelStyle style = sceneLabelStyleFromMarkerColor("#102030");

    for (std::size_t index = 0U; index < 3U; ++index) {
        visualization_msgs::Marker label;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.pose.orientation.w = 1.0;
        label.scale.z = index == 2U ? 0.18 : 0.32;
        label.color.r = 1.0;
        label.color.g = index == 2U ? 0.85 : 1.0;
        label.color.b = index == 2U ? 0.25 : 1.0;
        label.color.a = 1.0;
        label.text = names[index];
        visualization_msgs::MarkerArray markers;
        markers.markers.push_back(label);

        foxglove_msgs::SceneUpdate update;
        appendSceneEntity(kinds[index], names[index], markers, 0U, ros::Time(42, 0), "world", style, &update);
        ASSERT_EQ(update.entities.size(), 1U);
        ASSERT_EQ(update.entities[0].texts.size(), 1U);
        const foxglove_msgs::TextPrimitive& text = update.entities[0].texts[0];
        EXPECT_DOUBLE_EQ(text.font_size, 0.24);
        EXPECT_DOUBLE_EQ(text.color.r, 16.0 / 255.0);
        EXPECT_DOUBLE_EQ(text.color.g, 32.0 / 255.0);
        EXPECT_DOUBLE_EQ(text.color.b, 48.0 / 255.0);
        EXPECT_DOUBLE_EQ(text.color.a, 1.0);
        EXPECT_TRUE(text.billboard);
        EXPECT_FALSE(text.scale_invariant);
    }
}

TEST(SceneContract, ImmutableListsSelectOnlyConfiguredModels) {
    const std::set<std::string> fs150s{"uav1", "uav2"};
    const std::set<std::string> scouts{"ugv1"};
    const std::set<std::string> mecanums{"ugv2"};

    EXPECT_EQ(selectRobotModelKind("uav1", fs150s, scouts, mecanums, false, true), RobotModelKind::kFs150);
    EXPECT_EQ(selectRobotModelKind("ugv1", fs150s, scouts, mecanums, false, true), RobotModelKind::kScout);
    EXPECT_EQ(selectRobotModelKind("ugv2", fs150s, scouts, mecanums, false, true), RobotModelKind::kMecanum);
    EXPECT_EQ(selectRobotModelKind("uav9", fs150s, scouts, mecanums, false, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("ugv9", fs150s, scouts, mecanums, false, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("ground_plane", fs150s, scouts, mecanums, false, true), RobotModelKind::kNone);
}

TEST(SceneContract, ModelListsMustBeDisjoint) {
    EXPECT_TRUE(modelListsAreDisjoint({}, {}, {"uav1"}, {"ugv1"}, {"ugv2"}));
    EXPECT_FALSE(modelListsAreDisjoint({"shared"}, {}, {"uav1"}, {"ugv1"}, {"shared"}));
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
    appendSceneEntity(RobotModelKind::kScout, "ugv1", markers, 0U, ros::Time(42, 0), "world", blackLabelStyle(),
                      &update);

    ASSERT_EQ(update.entities.size(), 1U);
    EXPECT_EQ(update.entities[0].id, "xgc2/scout/ugv1");
    EXPECT_EQ(update.entities[0].frame_id, "world");
    ASSERT_EQ(update.entities[0].models.size(), 1U);
    EXPECT_EQ(update.entities[0].models[0].url, marker.mesh_resource);
}

TEST(SceneContract, MecanumMarkersBecomeMecanumSceneEntity) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://mecanum_description/meshes/nexus_base_link.STL";
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);

    foxglove_msgs::SceneUpdate update;
    appendSceneEntity(RobotModelKind::kMecanum, "ugv1", markers, 0U, ros::Time(42, 0), "world", blackLabelStyle(),
                      &update);

    ASSERT_EQ(update.entities.size(), 1U);
    EXPECT_EQ(update.entities[0].id, "xgc2/mecanum/ugv1");
    ASSERT_EQ(update.entities[0].models.size(), 1U);
    EXPECT_EQ(update.entities[0].models[0].url, marker.mesh_resource);
}

TEST(SceneContract, RobotAndPathUsePersistentIndependentEntityIDs) {
    visualization_msgs::Marker mesh;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.mesh_resource = "package://fs150_description/meshes/iris.stl";
    mesh.pose.orientation.w = 1.0;
    mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0;

    visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.ns = "uav1_actual_path";
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.02;
    for (int index = 0; index < 151; ++index) {
        geometry_msgs::Point point;
        point.x = static_cast<double>(index) * 0.1;
        path.points.push_back(point);
    }

    visualization_msgs::Marker label;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.header.frame_id = "uav1/label";
    label.pose.orientation.w = 1.0;
    label.text = "UAV 1";
    label.scale.z = 0.32;

    visualization_msgs::MarkerArray markers;
    markers.markers = {mesh, path, label};

    foxglove_msgs::SceneUpdate robot_update;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kRobot, markers, 0U, ros::Time(42, 0), "world",
                          blackLabelStyle(), &robot_update);
    ASSERT_EQ(robot_update.entities.size(), 1U);
    EXPECT_EQ(robot_update.entities[0].id, "xgc2/px4/uav1");
    EXPECT_EQ(robot_update.entities[0].models.size(), 1U);
    // The label left this entity. Geometry is drawn where the message put it;
    // a label is drawn wherever its anchor currently is, and one entity cannot
    // hold both because an entity names exactly one frame.
    EXPECT_TRUE(robot_update.entities[0].texts.empty());
    EXPECT_FALSE(robot_update.entities[0].frame_locked);
    EXPECT_EQ(robot_update.entities[0].frame_id, "world");
    EXPECT_TRUE(robot_update.entities[0].lines.empty());
    EXPECT_TRUE(robot_update.entities[0].lifetime.isZero());

    foxglove_msgs::SceneUpdate path_update;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0), "world",
                          blackLabelStyle(), &path_update);
    ASSERT_EQ(path_update.entities.size(), 1U);
    EXPECT_EQ(path_update.entities[0].id, "xgc2/px4/uav1/path");
    EXPECT_EQ(path_update.entities[0].lines.size(), 1U);
    EXPECT_TRUE(path_update.entities[0].models.empty());
    EXPECT_TRUE(path_update.entities[0].texts.empty());
    EXPECT_TRUE(path_update.entities[0].lifetime.isZero());

    // A label follows its anchor instead of being redrawn: the entity names the
    // robot's own label frame and asks the viewer to re-resolve it. That is the
    // whole reason a label transmitted twice a second can move thirty times a
    // second, so both halves are asserted here.
    foxglove_msgs::SceneUpdate label_update;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kLabel, markers, 0U, ros::Time(42, 0),
                          "world", blackLabelStyle(), &label_update);
    ASSERT_EQ(label_update.entities.size(), 1U);
    EXPECT_EQ(label_update.entities[0].id, "xgc2/px4/uav1/label");
    EXPECT_EQ(label_update.entities[0].frame_id, "uav1/label");
    EXPECT_TRUE(label_update.entities[0].frame_locked);
    EXPECT_EQ(label_update.entities[0].texts.size(), 1U);
    EXPECT_TRUE(label_update.entities[0].models.empty());
    EXPECT_TRUE(label_update.entities[0].lines.empty());

    foxglove_msgs::SceneUpdate legacy_full_update;
    appendSceneEntity(RobotModelKind::kFs150, "uav1", markers, 0U, ros::Time(42, 0), "world", blackLabelStyle(),
                      &legacy_full_update);
    EXPECT_LT(ros::serialization::serializationLength(robot_update),
              ros::serialization::serializationLength(legacy_full_update));
}

TEST(SceneContract, SceneCadenceSeparatesRobotAndPathUpdatesWithoutDrift) {
    SceneUpdateCadence cadence(10.0, 2.0);

    SceneUpdateCadenceDecision decision = cadence.take(ros::Time(10, 0));
    EXPECT_TRUE(decision.publish_robot);
    EXPECT_TRUE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 50000000));
    EXPECT_FALSE(decision.publish_robot);
    EXPECT_FALSE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 100000000));
    EXPECT_TRUE(decision.publish_robot);
    EXPECT_FALSE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 500000000));
    EXPECT_TRUE(decision.publish_robot);
    EXPECT_TRUE(decision.publish_path);

    // A simulated-clock reset immediately republishes a complete pair so the
    // consumer cannot remain stuck with stale entities from the old epoch.
    decision = cadence.take(ros::Time(1, 0));
    EXPECT_TRUE(decision.publish_robot);
    EXPECT_TRUE(decision.publish_path);
}

TEST(SceneContract, MixedFleetSceneSerializationBudgetIsBounded) {
    foxglove_msgs::SceneUpdate legacy_updates;
    foxglove_msgs::SceneUpdate robot_updates;
    foxglove_msgs::SceneUpdate path_updates;

    for (int robot_index = 1; robot_index <= 10; ++robot_index) {
        const bool is_uav = robot_index <= 6;
        const RobotModelKind kind = is_uav ? RobotModelKind::kFs150 : RobotModelKind::kScout;
        const std::string name = is_uav ? "uav" + std::to_string(robot_index) : "ugv" + std::to_string(robot_index - 6);
        const int model_count = is_uav ? 5 : 6;
        visualization_msgs::MarkerArray markers;
        for (int model_index = 0; model_index < model_count; ++model_index) {
            visualization_msgs::Marker mesh;
            mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
            mesh.mesh_resource = is_uav ? "package://fs150_description/meshes/iris_prop_ccw.dae"
                                        : "package://scout_description/meshes/wheel.dae";
            mesh.pose.orientation.w = 1.0;
            mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0;
            markers.markers.push_back(mesh);
        }

        visualization_msgs::Marker path;
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.ns = name + "_actual_path";
        path.pose.orientation.w = 1.0;
        path.scale.x = 0.02;
        for (int point_index = 0; point_index < 151; ++point_index) {
            geometry_msgs::Point point;
            point.x = static_cast<double>(point_index) * 0.1;
            point.y = static_cast<double>(robot_index);
            path.points.push_back(point);
        }
        markers.markers.push_back(path);

        visualization_msgs::Marker label;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.pose.orientation.w = 1.0;
        label.text = name;
        label.scale.z = 0.32;
        markers.markers.push_back(label);

        appendSceneEntity(kind, name, markers, 0U, ros::Time(42, 0), "world", blackLabelStyle(), &legacy_updates);
        appendSceneEntityPart(kind, name, SceneEntityPart::kRobot, markers, 0U, ros::Time(42, 0), "world",
                              blackLabelStyle(), &robot_updates);
        appendSceneEntityPart(kind, name, SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0), "world",
                              blackLabelStyle(), &path_updates);
    }

    ASSERT_EQ(legacy_updates.entities.size(), 10U);
    ASSERT_EQ(robot_updates.entities.size(), 10U);
    ASSERT_EQ(path_updates.entities.size(), 10U);
    EXPECT_EQ(robot_updates.entities.front().id, "xgc2/px4/uav1");
    EXPECT_EQ(robot_updates.entities.back().id, "xgc2/scout/ugv4");
    EXPECT_EQ(path_updates.entities.front().id, "xgc2/px4/uav1/path");
    EXPECT_EQ(path_updates.entities.back().id, "xgc2/scout/ugv4/path");

    const std::size_t legacy_bytes_per_second =
        ros::serialization::serializationLength(legacy_updates) * static_cast<std::size_t>(30);
    const std::size_t bounded_bytes_per_second =
        ros::serialization::serializationLength(robot_updates) * static_cast<std::size_t>(10) +
        ros::serialization::serializationLength(path_updates) * static_cast<std::size_t>(2);
    RecordProperty("legacy_bytes_per_second", static_cast<int>(legacy_bytes_per_second));
    RecordProperty("bounded_bytes_per_second", static_cast<int>(bounded_bytes_per_second));
    EXPECT_LT(bounded_bytes_per_second * static_cast<std::size_t>(4), legacy_bytes_per_second);
}

TEST(SceneContract, EntityIDsRemainKindScoped) {
    EXPECT_EQ(sceneEntityID(RobotModelKind::kFs150, "uav1"), "xgc2/px4/uav1");
    EXPECT_EQ(sceneEntityID(RobotModelKind::kScout, "ugv1"), "xgc2/scout/ugv1");
    EXPECT_EQ(sceneEntityID(RobotModelKind::kMecanum, "ugv1"), "xgc2/mecanum/ugv1");
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kFs150, "uav1", SceneEntityPart::kPath), "xgc2/px4/uav1/path");
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kScout, "ugv1", SceneEntityPart::kPath), "xgc2/scout/ugv1/path");
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kMecanum, "ugv1", SceneEntityPart::kPath),
              "xgc2/mecanum/ugv1/path");
    EXPECT_THROW(sceneEntityID(RobotModelKind::kNone, "unknown"), std::invalid_argument);
}

} // namespace
} // namespace gazebo_sim_visualization

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
