#!/usr/bin/env bash
# Configures and builds g1_backflip_mujoco with plain CMake (no ROS, no colcon).
# Checks out the wb_humanoid_mpc submodule (and its OCS2 fork) first if needed.
#
# Usage: build.sh [build-dir] [extra cmake args...]
#   default build dir: <repo>/build/g1_backflip
#
# Environment:
#   MUJOCO_DIR          MuJoCo release directory (include/, lib/) if MuJoCo has no CMake config
#   CMAKE_PREFIX_PATH   where pinocchio 3 is installed (e.g. /opt/openrobots, or `python3 -m cmeel cmake`)
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$(cd "$here/.." && pwd)"
repo="$(cd "$pkg/../.." && pwd)"
build="${1:-$repo/build/g1_backflip}"
shift || true

# wb_humanoid_mpc + its OCS2 fork (only the submodules the build needs)
if [ ! -f "$repo/src/wb_humanoid_mpc/lib/ocs2_ros2/ocs2_core/CMakeLists.txt" ]; then
  git -C "$repo" submodule update --init src/wb_humanoid_mpc
  git -C "$repo/src/wb_humanoid_mpc" submodule update --init lib/ocs2_ros2
fi

# pinocchio installed with pip ("pip install pin"): add its cmeel prefix automatically
if python3 -c "import cmeel" 2>/dev/null; then
  cmeel_prefix="$(python3 -m cmeel cmake 2>/dev/null | tail -n 1)"
  if [ -n "$cmeel_prefix" ]; then
    export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+$CMAKE_PREFIX_PATH:}$cmeel_prefix"
  fi
fi

cmake -S "$pkg" -B "$build" -DCMAKE_BUILD_TYPE=Release ${MUJOCO_DIR:+-DMUJOCO_DIR="$MUJOCO_DIR"} "$@"
cmake --build "$build" -j"$(nproc)"

echo
echo "Built: $build/g1_backflip_mujoco"
echo "  GUI      : $build/g1_backflip_mujoco"
echo "  headless : $build/g1_backflip_mujoco --headless"
echo "  plan only: $build/g1_backflip_plan --csv plan.csv"
