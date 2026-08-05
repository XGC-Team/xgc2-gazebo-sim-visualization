#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-visualization >/dev/null
test "$(rospack find gazebo_sim_visualization)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_visualization"
NODE="/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_visualization/gazebo_auto_visualizer_node"
SCENE_CONTRACT_LIBRARY="/opt/ros/${ROS_DISTRO}/lib/libgazebo_scene_contract.so"
test -x "${NODE}"
test -f "${SCENE_CONTRACT_LIBRARY}"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_visualization/rviz/gazebo_auto_visualization.rviz"

ldd_output="$(ldd "${NODE}")"
if grep -Fq 'not found' <<<"${ldd_output}"; then
  printf '%s\n' "${ldd_output}" >&2
  echo "gazebo_auto_visualizer_node has unresolved shared-library dependencies" >&2
  exit 1
fi
grep -Fq "libgazebo_scene_contract.so => ${SCENE_CONTRACT_LIBRARY} " <<<"${ldd_output}"

roslaunch --files gazebo_sim_visualization gazebo_auto_visualization_rviz.launch \
  rviz:=false marker_color:=#000000 \
  >/tmp/xgc2-gazebo-auto-visualization-files.txt

echo "Installed package check passed"
