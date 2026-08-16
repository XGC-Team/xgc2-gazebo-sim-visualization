#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dependency_lock="${script_dir}/../dependencies/xgc2-robot-visualization.env"
package_name="ros-noetic-xgc2-robot-visualization"
header_path="/opt/ros/noetic/include/xgc2_robot_visualization/mecanum_ugv_visualizer.hpp"
description_publisher_path="/opt/ros/noetic/lib/xgc2_robot_visualization/xgc2_robot_description_publisher_node"

if [[ ! -f "${dependency_lock}" ]]; then
  echo "missing robot visualization dependency lock: ${dependency_lock}" >&2
  exit 1
fi
# shellcheck source=../dependencies/xgc2-robot-visualization.env
source "${dependency_lock}"

if [[ ! "${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+-[0-9]+$ ]]; then
  echo "invalid robot visualization capability version: ${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}" >&2
  exit 1
fi
if [[ ! "${XGC2_ROBOT_VISUALIZATION_STANDALONE_SOURCE_REF}" =~ ^[0-9a-f]{40}$ ]]; then
  echo "robot visualization standalone source must be a full SHA" >&2
  exit 1
fi

selected_version="$(apt-cache policy "${package_name}" | awk '/Candidate:/ {print $2; exit}')"
if [[ -z "${selected_version}" || "${selected_version}" == "(none)" ]]; then
  echo "configured XGC2 APT sources have no ${package_name} candidate" >&2
  exit 1
fi
if ! dpkg --compare-versions "${selected_version}" ge "${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}"; then
  echo "selected ${package_name} ${selected_version}; need ${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION} or newer" >&2
  exit 1
fi
apt-get install -y --no-install-recommends "${package_name}=${selected_version}"

installed_version="$(dpkg-query -W -f='${Version}' "${package_name}")"
if [[ "${installed_version}" != "${selected_version}" ]]; then
  echo "installed ${package_name} ${installed_version} does not equal selected ${selected_version}" >&2
  exit 1
fi
if ! dpkg --compare-versions "${installed_version}" ge "${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}"; then
  echo "installed ${package_name} ${installed_version} lacks the required visualization runtime capability" >&2
  exit 1
fi
test -f "${header_path}"
test -x "${description_publisher_path}"
