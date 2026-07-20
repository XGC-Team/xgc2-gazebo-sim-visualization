#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-visualization >/dev/null
test "$(rospack find gazebo_sim_visualization)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_visualization"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_visualization/gazebo_auto_visualizer_node"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_visualization/rviz/gazebo_auto_visualization.rviz"
test -f "/usr/share/xgc2/process-definitions/xgc2-gazebo-sim-visualization.json"
python3 -m json.tool /usr/share/xgc2/process-definitions/xgc2-gazebo-sim-visualization.json >/dev/null

roslaunch --files gazebo_sim_visualization gazebo_auto_visualization_rviz.launch rviz:=false \
  >/tmp/xgc2-gazebo-auto-visualization-files.txt

echo "Installed package check passed"
