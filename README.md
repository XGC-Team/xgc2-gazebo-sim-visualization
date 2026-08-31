# XGC2 Gazebo Sim Visualization

ROS Noetic adapter that places frozen Experiment robots into RViz and Lichtblick.

`gazebo_auto_visualizer_node` consumes each slot's canonical world pose
`/<namespace>/pose` and drives reusable visualizers from
`xgc2_robot_visualization`. It does not select among raw VRPN, Gazebo truth,
MAVROS `local_position`, or TF, and it does not apply `localizationOffset` or
`hybridSource`. Those are already decided by the upstream projection.

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

- Topic: `/<namespace>/pose` (`geometry_msgs/PoseStamped`)
- Frame: product Fixed Frame `world` ENU. `map`, `odom`, and MAVROS local
  frames are refused.
- Freshness: `canonical_pose_timeout` (default `0.5` s) applies only to that
  sample. A missing or stale pose hides that robot; it does not fall back to
  another source.

A frozen roster (`XGC2_ROBOT_VISUALIZATION_ROSTER`) maps scene-model identity
used by URDF/TF to the Experiment slot namespace that owns `/<slot>/pose`.

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
