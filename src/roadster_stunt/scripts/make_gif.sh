#!/usr/bin/env bash
# Records the roadster stunt GIF shown in the top-level README (needs ffmpeg; uses xvfb-run when there is no display).
#
# Usage: make_gif.sh [path/to/roadster_stunt_mujoco] [output.gif]
#   default executable: ./install/roadster_stunt/lib/roadster_stunt/roadster_stunt_mujoco (run from the workspace root)
#   default output:     <repo>/doc/gifs/roadster.gif
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exe="${1:-./install/roadster_stunt/lib/roadster_stunt/roadster_stunt_mujoco}"
out="${2:-$here/../../../doc/gifs/roadster.gif}"

# 960x540 render, 640 px wide GIF at 15 fps, the roll in half speed
args=(--record "$out" --width 960 --height 540 --scale 640 --fps 15 --slowmo 0.5)

if [ -z "${DISPLAY:-}" ]; then
  xvfb-run -a -s "-screen 0 1280x720x24" "$exe" "${args[@]}"
else
  "$exe" "${args[@]}"
fi
