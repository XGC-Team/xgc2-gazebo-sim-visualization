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
