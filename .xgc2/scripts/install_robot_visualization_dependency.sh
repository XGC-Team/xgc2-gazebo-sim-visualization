#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
product_metadata="${script_dir}/../product.yml"
package_name="ros-noetic-xgc2-robot-visualization"
header_path="/opt/ros/noetic/include/xgc2_robot_visualization/mecanum_ugv_visualizer.hpp"
path_history_header_path="/opt/ros/noetic/include/xgc2_robot_visualization/path_history.hpp"
description_publisher_path="/opt/ros/noetic/lib/xgc2_robot_visualization/xgc2_robot_description_publisher_node"

XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION="$(awk '
  $1 == "-" && $2 == "ros-noetic-xgc2-robot-visualization" && $3 == "(>=" {
    gsub(/\)/, "", $4); print $4; matches++
  }
  END { if (matches != 1) exit 1 }
' "${product_metadata}")"

if [[ ! "${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+-[0-9]+$ ]]; then
  echo "invalid robot visualization capability version: ${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}" >&2
  exit 1
fi
selected_version="$(apt-cache policy "${package_name}" | awk '/Candidate:/ {print $2; exit}')"
if [[ -z "${selected_version}" || "${selected_version}" == "(none)" ]]; then
  echo "configured APT indexes have no ${package_name} candidate" >&2
  exit 1
fi
if ! dpkg --compare-versions "${selected_version}" ge "${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}"; then
  echo "APT selected ${package_name} ${selected_version}; need ${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION} or newer" >&2
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
test -f "${path_history_header_path}"
test -x "${description_publisher_path}"
