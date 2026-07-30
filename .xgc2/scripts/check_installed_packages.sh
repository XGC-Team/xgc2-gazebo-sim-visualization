#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-visualization >/dev/null
test "$(rospack find gazebo_sim_visualization)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_visualization"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_visualization/gazebo_auto_visualizer_node"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_visualization/rviz/gazebo_auto_visualization.rviz"

roslaunch --files gazebo_sim_visualization gazebo_auto_visualization_rviz.launch \
  rviz:=false marker_color:=#000000 \
  >/tmp/xgc2-gazebo-auto-visualization-files.txt

echo "Installed package check passed"
