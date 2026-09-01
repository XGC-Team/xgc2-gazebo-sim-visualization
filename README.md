# XGC2 Gazebo Sim Visualization

ROS Noetic adapter that places frozen Experiment robots into RViz and Lichtblick.

`gazebo_auto_visualizer_node` consumes one slot-owned viewer pose and drives
reusable visualizers from
`xgc2_robot_visualization`. The source is fixed by kind: FS150 uses the fused
MAVROS local pose, while ground robots use the slot pose after localization
projection. It does not fall back to raw VRPN, Gazebo truth, or TF, and it does
not apply `localizationOffset` or `hybridSource` itself.

The Mecanum renderer is a required build dependency. Standalone CI builds its
immutable capability source when production APT has not caught up; release
trains instead consume the exact staged APT candidate.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

## Launch

```bash
roslaunch gazebo_sim_visualization gazebo_auto_visualization_rviz.launch \
  tracked_fs150_models:=uav1 \
  tracked_scout_models:=ugv1 \
  tracked_mecanum_models:=ugv2 \
  marker_color:=#000000
```

Standalone launches must name every scene model. There is no Gazebo
auto-discovery and no `tracked_uav_models` / `tracked_ugv_models` alias.

## Canonical pose

Robot body placement, slot labels, and history Path all read one sample:

- FS150 topic: `/<namespace>/mavros/local_position/pose`
  (`geometry_msgs/PoseStamped`). This is the PX4/MAVROS fused estimate, so the
  3D model exposes a broken vision/EKF alignment before takeoff.
- Scout/Mecanum topic: `/<namespace>/pose` (`geometry_msgs/PoseStamped`), after
  the Experiment's one localization offset.
- Frame: ground-robot poses must be product Fixed Frame `world` ENU. FS150
  admits the exact MAVROS `map` label and interprets its fused ENU coordinates
  in Experiment `world`; this is intentional so a disagreement between the
  FCU estimate and the expected Experiment pose is visible before takeoff.
  `odom`, namespaced frame aliases, and every fallback remain refused.
- Freshness: `canonical_pose_timeout` (default `0.5` s) applies only to that
  sample. A missing or stale pose hides that robot; it does not fall back to
  another source.

A frozen roster (`XGC2_ROBOT_VISUALIZATION_ROSTER`) maps scene-model identity
used by URDF/TF to the Experiment slot namespace that owns the selected topic.

## UAV rotor state

Rotor animation is a visual cue from `/<scene_model>/mavros/state` and
extended state. It is not a pose source.

- `mavros_msgs/State.armed == true`: rotors advance at the landed-state tier.
- Missing or stale state, or `armed == false`: rotors stop at the current phase.

## Lichtblick SceneUpdate

The same node can publish labels and history trails as persistent
`foxglove_msgs/SceneEntity` on `/xgc/scene`. FS150, Scout, and Mecanum use
separate rendering paths and separate entity identities. Robot meshes stay in
the robot's own URDF, placed from `/xgc/tf`; this node does not republish those
meshes into the scene.

XGC starts this mode with typed model lists derived from the experiment run's
immutable Robot snapshot:

- `publish_scene_update`: enables SceneUpdate output; defaults to `false` so existing RViz launches are unchanged.
- `scene_update_topic`: defaults to `/xgc/scene`.
- `tracked_fs150_models` / `tracked_scout_models` / `tracked_mecanum_models`
- `marker_color`: required canonical lowercase `#rrggbb` color shared by every robot label.
- `publish_markers` and `publish_transforms`: may be disabled for the dedicated Lichtblick publisher.

Algorithm `nav_msgs/Path`, `geometry_msgs/PoseArray`, and
`visualization_msgs/Marker` / `MarkerArray` stay on their declared ROS topics.
Core Lichtblick layout subscribes to them directly. This node does not
republish or restyle those messages.

This package is not the camera-image overlay owner. Image AR URDF layering
(`runMode` / `hybridSource`) and native Lichtblick topic settings live in the
Core Lichtblick layout plugin.

## UGV wheel state

Scout UGV markers are rendered through
`xgc2_robot_visualization::ScoutUgvVisualizer`. Wheel animation is a visual cue,
not a real wheel-speed display:

- Fresh `/<ugv_name>/twist` is preferred as the actual motion hint.
- Fresh `/<ugv_name>/cmd_vel` is used when no twist is available.
- Without a fresh motion hint, wheel motion is estimated from pose changes.
- Static or near-static motion keeps the wheels stopped.

## Mecanum visual state

Mecanum UGV markers are rendered through
`xgc2_robot_visualization::MecanumUgvVisualizer`, using the same Nexus chassis,
left/right roller-wheel meshes, wheel shafts, and range-sensor meshes as the
Gazebo model. Its four wheel phases follow the simulator's Mecanum equations,
including `linear.y` sideways motion and yaw coupling; they are not Scout
differential-drive wheel phases.

`mecanum_mesh_scale` defaults to `0.001`, matching the millimetre-authored
`mecanum_description` STL assets. It is deliberately separate from
the Scout `ugv_mesh_scale` parameter.
