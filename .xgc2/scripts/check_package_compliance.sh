#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

bash -n .xgc2/scripts/*.sh

if git ls-files | grep -E '(^|/)(build|devel|install|install-root|\.work|\.ci|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|install-root|\.work|\.ci|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .github/workflows/ci.yml
  .github/workflows/release.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/install_robot_visualization_dependency.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/package_debs.sh
  CMakeLists.txt
  README.md
  package.xml
  process-definitions/xgc2-gazebo-sim-visualization.json
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

grep -q "id: xgc2-gazebo-sim-visualization" .xgc2/product.yml
grep -Eq '^version: [0-9]+\.[0-9]+\.[0-9]+-[0-9]+$' .xgc2/product.yml
grep -q "ros-noetic-xgc2-gazebo-sim-visualization" .xgc2/product.yml
grep -q "PACKAGE=\"ros-noetic-xgc2-gazebo-sim-visualization\"" .xgc2/scripts/package_debs.sh
grep -q "gazebo_sim_visualization" package.xml
grep -q "gazebo_auto_visualizer_node" CMakeLists.txt
grep -q "gazebo_auto_visualization_rviz.launch" .xgc2/scripts/check_installed_packages.sh
grep -q 'ros-noetic-xgc2-robot-visualization (>= 0.1.0-9)' .xgc2/scripts/package_debs.sh
grep -Fq 'XGC2_ROBOT_VISUALIZATION_STANDALONE_SOURCE_REF' \
  .xgc2/scripts/install_robot_visualization_dependency.sh
grep -Fq 'apt-cache policy "${package_name}"' \
  .xgc2/scripts/install_robot_visualization_dependency.sh
grep -Fq '/workspace/repo/.xgc2/scripts/install_robot_visualization_dependency.sh' \
  .xgc2/scripts/build_debs_in_docker.sh
if grep -Fq 'ros-noetic-xgc2-robot-visualization' \
    .xgc2/scripts/build_debs_in_docker.sh; then
  echo "robot visualization must be resolved through its capability dependency script" >&2
  exit 1
fi
grep -q '"version": "1.4.0"' process-definitions/xgc2-gazebo-sim-visualization.json
grep -q '_tracked_mecanum_models:=${mecanumModels}' process-definitions/xgc2-gazebo-sim-visualization.json
grep -q 'package://mecanum_description/meshes/nexus_base_link.STL' test/scene_contract_test.cpp

echo "Package compliance checks passed."
