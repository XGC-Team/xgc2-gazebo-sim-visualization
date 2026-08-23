#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dependency_lock="${script_dir}/../dependencies/xgc2-robot-visualization.env"
package_name="ros-noetic-xgc2-robot-visualization"
header_path="/opt/ros/noetic/include/xgc2_robot_visualization/mecanum_ugv_visualizer.hpp"
path_history_header_path="/opt/ros/noetic/include/xgc2_robot_visualization/path_history.hpp"
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

selected_version=""
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
  selected_version="$(apt-cache policy "${package_name}" | awk '/Candidate:/ {print $2; exit}')"
  if [[ -z "${selected_version}" || "${selected_version}" == "(none)" ]]; then
    echo "release overlay has no ${package_name} candidate" >&2
    exit 1
  fi
  if ! dpkg --compare-versions "${selected_version}" ge "${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}"; then
    echo "release overlay selected ${package_name} ${selected_version}; need ${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION} or newer" >&2
    exit 1
  fi
  apt-get install -y --no-install-recommends "${package_name}=${selected_version}"
else
  work_dir="$(mktemp -d /tmp/xgc2-robot-visualization-source.XXXXXX)"
  cleanup() {
    if [[ -d "${work_dir}" && "${work_dir}" == /tmp/xgc2-robot-visualization-source.?????? ]]; then
      rm -rf -- "${work_dir}"
    fi
  }
  trap cleanup EXIT

  source_dir="${work_dir}/source"
  build_workspace="${work_dir}/workspace"
  deb_dir="${work_dir}/debs"
  git init -q "${source_dir}"
  git -C "${source_dir}" remote add origin \
    https://github.com/XGC-Team/xgc2-robot-visualization.git
  git -C "${source_dir}" fetch --depth 1 origin \
    "${XGC2_ROBOT_VISUALIZATION_STANDALONE_SOURCE_REF}"
  git -C "${source_dir}" checkout -q --detach FETCH_HEAD
  test "$(git -C "${source_dir}" rev-parse HEAD)" = \
    "${XGC2_ROBOT_VISUALIZATION_STANDALONE_SOURCE_REF}"

  source_version="$(sed -n 's/^version:[[:space:]]*//p' \
    "${source_dir}/.xgc2/product.yml" | head -n 1)"
  if [[ "${source_version}" != "${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}" ]]; then
    echo "standalone robot visualization source ${source_version:-<empty>} does not provide ${XGC2_ROBOT_VISUALIZATION_MINIMUM_VERSION}" >&2
    exit 1
  fi

  mkdir -p "${build_workspace}/src"
  rsync -a --delete "${source_dir}/" \
    "${build_workspace}/src/xgc2_robot_visualization/"
  (
    cd "${build_workspace}"
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    DESTDIR="${build_workspace}/install-root" catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCATKIN_ENABLE_TESTING=OFF
  )
  PACKAGE_VERSION="${source_version}" \
    "${source_dir}/.xgc2/scripts/package_debs.sh" \
      --install-root "${build_workspace}/install-root" \
      --output-dir "${deb_dir}"
  apt-get install -y --no-install-recommends \
    "${deb_dir}/${package_name}_"*.deb
  selected_version="${source_version}"
fi

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
