#!/usr/bin/env bash
# Configures and builds anymal_perceptive_mujoco with plain CMake (no ROS, no colcon).
#
# Usage: build.sh [build-dir] [extra cmake args...]
#   default build dir: <repo>/build/anymal_perceptive
#
# Environment:
#   MUJOCO_DIR          MuJoCo release directory (include/, lib/) if MuJoCo has no CMake config
#   CMAKE_PREFIX_PATH   where pinocchio 3 is installed (e.g. /opt/openrobots, or `python3 -m cmeel cmake`)
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$(cd "$here/.." && pwd)"
repo="$(cd "$pkg/../.." && pwd)"
build="${1:-$repo/build/anymal_perceptive}"
shift || true

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
echo "Built: $build/anymal_perceptive_mujoco"
echo "  GUI      : $build/anymal_perceptive_mujoco"
echo "  headless : $build/anymal_perceptive_mujoco --headless --vx 0.3 --goal-x 8.3"
echo "  tests    : (cd $build && ctest --output-on-failure)"
