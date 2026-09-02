#include "gazebo_sim_visualization/scene_contract.hpp"

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Pose.h>
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

    EXPECT_EQ(selectRobotModelKind("uav1", fs150s, scouts, mecanums, true), RobotModelKind::kFs150);
    EXPECT_EQ(selectRobotModelKind("ugv1", fs150s, scouts, mecanums, true), RobotModelKind::kScout);
    EXPECT_EQ(selectRobotModelKind("ugv2", fs150s, scouts, mecanums, true), RobotModelKind::kMecanum);
    EXPECT_EQ(selectRobotModelKind("uav9", fs150s, scouts, mecanums, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("ugv9", fs150s, scouts, mecanums, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("ground_plane", fs150s, scouts, mecanums, true), RobotModelKind::kNone);
    EXPECT_EQ(selectRobotModelKind("uav1", fs150s, scouts, mecanums, false), RobotModelKind::kFs150);
    EXPECT_EQ(selectRobotModelKind("ugv1", fs150s, scouts, mecanums, false), RobotModelKind::kNone);
}

TEST(SceneContract, ModelListsMustBeDisjoint) {
    EXPECT_TRUE(modelListsAreDisjoint({"uav1"}, {"ugv1"}, {"ugv2"}));
    EXPECT_FALSE(modelListsAreDisjoint({"shared"}, {"ugv1"}, {"shared"}));
}

TEST(SceneContract, RobotKindOwnsUppercaseMarkerClassAndCanonicalNamespaceOwnsNumber) {
    struct Case {
        RobotModelKind kind;
        const char* ros_namespace;
        const char* expected;
    };
    const Case cases[] = {
        {RobotModelKind::kFs150, "/uav1", "UAV 1"},
        {RobotModelKind::kScout, "/ugv2", "UGV 2"},
        {RobotModelKind::kMecanum, "/ugv3", "UGV 3"},
        // Mixed fleets may bind a Scout scene model to a /uavN slot. Robot
        // category, not the lowercase interface prefix, still owns UGV.
        {RobotModelKind::kScout, "/uav7", "UGV 7"},
    };

    for (const Case& test_case : cases) {
        visualization_msgs::Marker untouched;
        untouched.type = visualization_msgs::Marker::LINE_STRIP;
        untouched.text = "unchanged";
        visualization_msgs::Marker label;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.text = "lowercase-interface-name";
        visualization_msgs::MarkerArray markers;
        markers.markers = {untouched, label};

        applyRobotMarkerLabel(&markers, 1U, test_case.kind, test_case.ros_namespace);

        EXPECT_EQ(markers.markers[0].text, "unchanged");
        EXPECT_EQ(markers.markers[1].text, test_case.expected);
    }

    visualization_msgs::MarkerArray markers;
    EXPECT_THROW(applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kNone, "/uav1"), std::invalid_argument);
    EXPECT_THROW(applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kFs150, "uav1"), std::invalid_argument);
    EXPECT_THROW(applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kScout, "/scout1"), std::invalid_argument);
}

TEST(SceneContract, CanonicalPosePublishesOnlyBodyAndUprightLabelTransforms) {
    geometry_msgs::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = -2.0;
    pose.position.z = 3.0;
    pose.orientation.z = std::sqrt(0.5);
    pose.orientation.w = std::sqrt(0.5);
    const ros::Time stamp(12, 34);

    struct Case {
        RobotModelKind kind;
        const char* name;
        double label_height;
    };
    const Case cases[] = {
        {RobotModelKind::kFs150, "uav1", 0.55},
        {RobotModelKind::kScout, "ugv1", 0.65},
        {RobotModelKind::kMecanum, "ugv2", 0.32},
    };
    for (const Case& test_case : cases) {
        const auto transforms = canonicalRobotPoseTransforms(test_case.kind, test_case.name, pose, stamp, "world");
        ASSERT_EQ(transforms.size(), 2U);
        EXPECT_EQ(transforms[0].child_frame_id, std::string(test_case.name) + "/base_link");
        EXPECT_EQ(transforms[1].child_frame_id, std::string(test_case.name) + "/label");
        EXPECT_EQ(transforms[0].header.stamp, stamp);
        EXPECT_DOUBLE_EQ(transforms[0].transform.translation.x, pose.position.x);
        EXPECT_NEAR(transforms[0].transform.rotation.z, std::sqrt(0.5), 1.0e-12);
        EXPECT_DOUBLE_EQ(transforms[1].transform.translation.z, pose.position.z + test_case.label_height);
        EXPECT_DOUBLE_EQ(transforms[1].transform.rotation.w, 1.0);
    }
    EXPECT_THROW(canonicalRobotPoseTransforms(RobotModelKind::kNone, "uav1", pose, stamp, "world"),
                 std::invalid_argument);
    EXPECT_THROW(canonicalRobotPoseTransforms(RobotModelKind::kFs150, "uav1", pose, ros::Time(), "world"),
                 std::invalid_argument);
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

    // Geometry is not a scene entity any more. Every mesh here is a link of the
    // robot's own URDF, which the viewer loads once and places from transforms,
    // so a part that carried it would draw each vehicle a second time.
    foxglove_msgs::SceneUpdate path_only;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0),
                          "world", blackLabelStyle(), &path_only);
    for (const foxglove_msgs::SceneEntity& entity : path_only.entities) {
        EXPECT_TRUE(entity.models.empty()) << "a scene part shipped robot geometry the URDF already draws";
    }

    foxglove_msgs::SceneUpdate path_update;
    appendSceneEntityPart(RobotModelKind::kFs150, "uav1", SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0),
                          "world", blackLabelStyle(), &path_update);
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
    EXPECT_LT(ros::serialization::serializationLength(label_update),
              ros::serialization::serializationLength(legacy_full_update));
}

TEST(SceneContract, MixedSlotIdentityChangesPresentationButPreservesSceneModelAnchors) {
    visualization_msgs::Marker mesh;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.mesh_resource = "package://scout_description/meshes/base_link.dae";
    mesh.pose.orientation.w = 1.0;
    mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0;

    visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.ns = "ugv1_actual_path";
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.02;
    path.points.resize(2);
    path.points[1].x = 1.0;

    visualization_msgs::Marker label;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.header.frame_id = "ugv1/label";
    label.pose.orientation.w = 1.0;
    label.text = "ugv1";
    label.scale.z = 0.32;

    visualization_msgs::MarkerArray markers;
    markers.markers = {mesh, path, label};
    applyRobotMarkerLabel(&markers, 0U, RobotModelKind::kScout, "/uav7");

    // Re-labeling is presentation-only. The scene-model marker namespace and
    // anchor frame remain ugv1 so pose/TF lookup cannot drift to the slot name.
    EXPECT_EQ(markers.markers[0].mesh_resource, mesh.mesh_resource);
    EXPECT_EQ(markers.markers[1].ns, "ugv1_actual_path");
    EXPECT_EQ(markers.markers[2].header.frame_id, "ugv1/label");
    EXPECT_EQ(markers.markers[2].text, "UGV 7");

    foxglove_msgs::SceneUpdate label_update;
    appendSceneEntityPart(RobotModelKind::kScout, "uav7", SceneEntityPart::kLabel, markers, 0U,
                          ros::Time(42, 0), "world", blackLabelStyle(), &label_update);
    ASSERT_EQ(label_update.entities.size(), 1U);
    EXPECT_EQ(label_update.entities[0].id, "xgc2/scout/uav7/label");
    EXPECT_EQ(label_update.entities[0].frame_id, "ugv1/label");
    ASSERT_EQ(label_update.entities[0].texts.size(), 1U);
    EXPECT_EQ(label_update.entities[0].texts[0].text, "UGV 7");
    EXPECT_TRUE(label_update.entities[0].models.empty());

    foxglove_msgs::SceneUpdate path_update;
    appendSceneEntityPart(RobotModelKind::kScout, "uav7", SceneEntityPart::kPath, markers, 0U,
                          ros::Time(42, 0), "world", blackLabelStyle(), &path_update);
    ASSERT_EQ(path_update.entities.size(), 1U);
    EXPECT_EQ(path_update.entities[0].id, "xgc2/scout/uav7/path");
    EXPECT_EQ(path_update.entities[0].frame_id, "world");
    ASSERT_EQ(path_update.entities[0].lines.size(), 1U);
    EXPECT_TRUE(path_update.entities[0].models.empty());
}

TEST(SceneContract, SceneCadenceSeparatesRobotAndPathUpdatesWithoutDrift) {
    SceneUpdateCadence cadence(10.0, 2.0);

    SceneUpdateCadenceDecision decision = cadence.take(ros::Time(10, 0));
    EXPECT_TRUE(decision.publish_label);
    EXPECT_TRUE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 50000000));
    EXPECT_FALSE(decision.publish_label);
    EXPECT_FALSE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 100000000));
    EXPECT_TRUE(decision.publish_label);
    EXPECT_FALSE(decision.publish_path);

    decision = cadence.take(ros::Time(10, 500000000));
    EXPECT_TRUE(decision.publish_label);
    EXPECT_TRUE(decision.publish_path);

    // A simulated-clock reset immediately republishes a complete pair so the
    // consumer cannot remain stuck with stale entities from the old epoch.
    decision = cadence.take(ros::Time(1, 0));
    EXPECT_TRUE(decision.publish_label);
    EXPECT_TRUE(decision.publish_path);
}

TEST(SceneContract, MixedFleetSceneSerializationBudgetIsBounded) {
    foxglove_msgs::SceneUpdate legacy_updates;
    foxglove_msgs::SceneUpdate label_updates;
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
        appendSceneEntityPart(kind, name, SceneEntityPart::kLabel, markers, 0U, ros::Time(42, 0), "world",
                              blackLabelStyle(), &label_updates);
        appendSceneEntityPart(kind, name, SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0), "world",
                              blackLabelStyle(), &path_updates);
    }

    ASSERT_EQ(legacy_updates.entities.size(), 10U);
    ASSERT_EQ(label_updates.entities.size(), 10U);
    ASSERT_EQ(path_updates.entities.size(), 10U);
    EXPECT_EQ(label_updates.entities.front().id, "xgc2/px4/uav1/label");
    EXPECT_EQ(label_updates.entities.back().id, "xgc2/scout/ugv4/label");
    EXPECT_EQ(path_updates.entities.front().id, "xgc2/px4/uav1/path");
    EXPECT_EQ(path_updates.entities.back().id, "xgc2/scout/ugv4/path");

    const std::size_t legacy_bytes_per_second =
        ros::serialization::serializationLength(legacy_updates) * static_cast<std::size_t>(30);
    const std::size_t bounded_bytes_per_second =
        ros::serialization::serializationLength(label_updates) * static_cast<std::size_t>(10) +
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
    EXPECT_EQ(sceneEntityPartID(RobotModelKind::kMecanum, "ugv1", SceneEntityPart::kPath), "xgc2/mecanum/ugv1/path");
    EXPECT_THROW(sceneEntityID(RobotModelKind::kNone, "unknown"), std::invalid_argument);
}

TEST(SceneContract, HistoricalPathNamespaceIsSharedAcrossKinds) {
    const RobotModelKind kinds[] = {RobotModelKind::kFs150, RobotModelKind::kScout, RobotModelKind::kMecanum};
    const char* names[] = {"uav1", "ugv1", "ugv2"};
    const char* ids[] = {"xgc2/px4/uav1/path", "xgc2/scout/ugv1/path", "xgc2/mecanum/ugv2/path"};
    for (std::size_t index = 0U; index < 3U; ++index) {
        visualization_msgs::Marker path;
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.ns = std::string(names[index]) + "_actual_path";
        path.pose.position.z = 0.18;
        path.pose.orientation.w = 1.0;
        path.scale.x = 0.02;
        geometry_msgs::Point point;
        point.z = 0.18;
        path.points.push_back(point);
        visualization_msgs::MarkerArray markers;
        markers.markers.push_back(path);
        foxglove_msgs::SceneUpdate update;
        appendSceneEntityPart(kinds[index], names[index], SceneEntityPart::kPath, markers, 0U, ros::Time(42, 0),
                              "world", blackLabelStyle(), &update);
        ASSERT_EQ(update.entities.size(), 1U);
        EXPECT_EQ(update.entities[0].id, ids[index]);
        EXPECT_EQ(update.entities[0].frame_id, "world");
        ASSERT_EQ(update.entities[0].lines.size(), 1U);
        EXPECT_DOUBLE_EQ(update.entities[0].lines[0].pose.position.z, 0.18);
        EXPECT_DOUBLE_EQ(update.entities[0].lines[0].points[0].z, 0.18);
    }
}

TEST(CanonicalWorldPose, AcceptsOnlyTheKindSpecificSlotViewerSample) {
    const ros::Time now(10, 0);
    CanonicalPoseSample pose;
    pose.available = true;
    pose.pose.position.x = 1.25;
    pose.pose.position.z = 0.16;
    pose.pose.orientation.w = 1.0;
    pose.stamp = now;
    pose.frame_id = "world";

    const CanonicalWorldPose selected =
        selectSlotVisualizationWorldPose(RobotModelKind::kScout, pose, now, 0.5);
    ASSERT_TRUE(selected.found);
    EXPECT_EQ(selected.stamp, now);
    EXPECT_EQ(selected.frame_id, "world");
    EXPECT_DOUBLE_EQ(selected.pose.position.z, 0.16);

    CanonicalPoseSample map_pose = pose;
    map_pose.frame_id = "map";
    const CanonicalWorldPose fs150_map =
        selectSlotVisualizationWorldPose(RobotModelKind::kFs150, map_pose, now, 0.5);
    ASSERT_TRUE(fs150_map.found);
    EXPECT_EQ(fs150_map.frame_id, "world");
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kScout, map_pose, now, 0.5).found);
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kMecanum, map_pose, now, 0.5).found);

    CanonicalPoseSample namespaced_map = pose;
    namespaced_map.frame_id = "uav1/map";
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, namespaced_map, now, 0.5).found);

    CanonicalPoseSample odom_pose = pose;
    odom_pose.frame_id = "odom";
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, odom_pose, now, 0.5).found);

    CanonicalPoseSample mavros_local = pose;
    mavros_local.frame_id = "uav1/local_origin";
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, mavros_local, now, 0.5).found);

    CanonicalPoseSample missing;
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, missing, now, 0.5).found);

    CanonicalPoseSample zero_stamp = pose;
    zero_stamp.stamp = ros::Time();
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, zero_stamp, now, 0.5).found);

    CanonicalPoseSample stale = pose;
    stale.stamp = ros::Time(9, 0);
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kFs150, stale, now, 0.5).found);
    EXPECT_FALSE(selectSlotVisualizationWorldPose(RobotModelKind::kNone, pose, now, 0.5).found);

    EXPECT_FALSE(isWorldFixedFrame("map"));
    EXPECT_FALSE(isWorldFixedFrame("odom"));
    EXPECT_TRUE(isWorldFixedFrame("/world"));
    EXPECT_EQ(slotVisualizationPoseTopic(RobotModelKind::kFs150, "/uav7"),
              "/uav7/mavros/local_position/pose");
    EXPECT_EQ(slotVisualizationPoseTopic(RobotModelKind::kScout, "/ugv1"), "/ugv1/pose");
    EXPECT_EQ(slotVisualizationPoseTopic(RobotModelKind::kMecanum, "/ugv2"), "/ugv2/pose");
    EXPECT_THROW(slotVisualizationPoseTopic(RobotModelKind::kNone, "/uav7"), std::invalid_argument);
    EXPECT_THROW(slotVisualizationPoseTopic(RobotModelKind::kFs150, "uav7"), std::invalid_argument);
    EXPECT_THROW(slotVisualizationPoseTopic(RobotModelKind::kScout, "/uav7/extra"), std::invalid_argument);
}

TEST(WorldFixedFrameRoot, AdvertisesParentOnTfWithoutMovingDisplays) {
    const ros::Time now(10, 0);
    const geometry_msgs::TransformStamped root = worldFixedFrameRoot("world", now);
    EXPECT_EQ(root.header.frame_id, "world");
    EXPECT_EQ(root.child_frame_id, kWorldFixedFrameRootChild);
    EXPECT_EQ(root.header.stamp, now);
    EXPECT_DOUBLE_EQ(root.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(root.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(root.transform.translation.z, 0.0);
    EXPECT_DOUBLE_EQ(root.transform.rotation.w, 1.0);
}

TEST(FrozenVisualizationRosterReady, AllowsMissingSiblingPoses) {
    EXPECT_FALSE(frozenVisualizationRosterReady(0U, 0U));
    EXPECT_TRUE(frozenVisualizationRosterReady(4U, 0U));
    EXPECT_TRUE(frozenVisualizationRosterReady(4U, 1U));
    EXPECT_TRUE(frozenVisualizationRosterReady(4U, 3U));
    EXPECT_TRUE(frozenVisualizationRosterReady(4U, 4U));
    EXPECT_FALSE(frozenVisualizationRosterReady(4U, 5U));
}

} // namespace
} // namespace gazebo_sim_visualization

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
