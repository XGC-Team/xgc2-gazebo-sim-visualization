# XGC2 Gazebo Sim Visualization

ROS Noetic RViz adapter for XGC2 Gazebo Classic simulations.

This repository contains the `gazebo_sim_visualization` package. It discovers
Gazebo models and adapts Gazebo/MAVROS topics into reusable robot visualizers
from `xgc2_robot_visualization`.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

## Launch

```bash
roslaunch gazebo_sim_visualization gazebo_auto_visualization_rviz.launch
```

## UAV rotor state

`gazebo_auto_visualizer_node` discovers UAV poses from `/gazebo/model_states`
and passes them to `xgc2_robot_visualization::Fs150UavVisualizer`. Rotor
animation is controlled only by `/<uav_name>/mavros/state`:

- `mavros_msgs/State.armed == true`: rotors advance at `rotor_speed_rad_s`.
- Missing or stale state, or `armed == false`: rotors stop at the current phase.

Relevant parameters:

- `mavros_state_topic_suffix`: defaults to `/mavros/state`.
- `mavros_state_timeout`: defaults to `2.0` seconds, long enough for typical MAVROS state heartbeat intervals while still stopping on stale state.
- `rotor_speed_rad_s`: visual fixed rotor speed while active.

## Lichtblick SceneUpdate

The same node can publish each selected vehicle as a persistent
`foxglove_msgs/SceneEntity` on `/xgc/scene`. FS150, Scout, and Mecanum use
separate rendering paths and separate entity identities; a Mecanum vehicle is
never rendered with the Scout mesh.

XGC starts this mode with typed model lists derived from the experiment run's
immutable Swarm Asset snapshot. The relevant node parameters are:

- `publish_scene_update`: enables SceneUpdate output; defaults to `false` so existing RViz launches are unchanged.
- `scene_update_topic`: defaults to `/xgc/scene`.
- `tracked_fs150_models`: comma-separated FS150 Gazebo/VRPN model names.
- `tracked_scout_models`: comma-separated Scout Gazebo/VRPN model names.
- `tracked_mecanum_models`: comma-separated Mecanum Gazebo/VRPN model names.
- `allow_auto_discovery`: may be disabled so only robots declared by the run snapshot appear.
- `publish_markers` and `publish_transforms`: may be disabled for the dedicated Lichtblick publisher.

`tracked_uav_models` and `tracked_ugv_models` remain compatibility aliases for
FS150 and Scout respectively. A model may appear in exactly one list; conflicting
or duplicated type assignments fail at startup. Automatic discovery only covers
the legacy FS150/Scout name patterns, because a name such as `ugv1` is not enough
to distinguish Scout from Mecanum. Pass `tracked_mecanum_models` explicitly for
direct RViz launches.

## UGV wheel state

Scout UGV markers are rendered through
`xgc2_robot_visualization::ScoutUgvVisualizer`. Wheel animation is a visual cue,
not a real wheel-speed display:

- Fresh `/<ugv_name>/twist` is preferred as the actual motion hint.
- Fresh `/<ugv_name>/cmd_vel` is used when no twist is available.
- Without a fresh motion hint, wheel motion is estimated from pose changes.
- Static or near-static motion keeps the wheels stopped.

Relevant parameters:

- `ugv_motion_timeout`: defaults to `0.5` seconds.
- `ugv_cmd_vel_topic_suffix`: defaults to `/cmd_vel`.
- `ugv_twist_topic_suffix`: defaults to `/twist`.
- `ugv_visual_wheel_radius`: defaults to `0.08`.
- `ugv_visual_track_width`: defaults to `0.416`.
- `ugv_wheel_motion_deadband`: defaults to `0.02`.
- `ugv_max_visual_wheel_speed_rad_s`: defaults to `35.0`.

## Mecanum visual state

Mecanum UGV markers are rendered through
`xgc2_robot_visualization::MecanumUgvVisualizer`, using the same Nexus chassis,
left/right roller-wheel meshes, wheel shafts, and range-sensor meshes as the
Gazebo model. Its four wheel phases follow the simulator's Mecanum equations,
including `linear.y` sideways motion and yaw coupling; they are not Scout
differential-drive wheel phases.

`mecanum_mesh_scale` defaults to `0.001`, matching the millimetre-authored
Nexus STL assets and their Gazebo SDF scale. It is deliberately separate from
the Scout `ugv_mesh_scale` parameter.

For a direct mixed RViz launch, declare each type explicitly:

```bash
roslaunch gazebo_sim_visualization gazebo_auto_visualization_rviz.launch \
  tracked_fs150_models:=uav1 \
  tracked_scout_models:=ugv1 \
  tracked_mecanum_models:=ugv2 \
  allow_auto_discovery:=false
```
