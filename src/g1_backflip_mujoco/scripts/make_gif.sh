#!/usr/bin/env bash
# Records doc/gifs/g1_backflip.gif: a lock-step run (stand, crouch, backflip, landing), rendered offscreen at half
# speed (needs ffmpeg; uses xvfb-run when there is no display).
#
# Usage: make_gif.sh [path/to/g1_backflip_mujoco] [output.gif]
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../.." && pwd)"
exe="${1:-$repo/build/g1_backflip/g1_backflip_mujoco}"
out="${2:-$repo/doc/gifs/g1_backflip.gif}"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# 960x540 MP4 at 0.5x playback (30 fps video = 15 simulated frames per second), then a 480 px GIF
args=(--headless --flip-time 1.0 --duration 6.0 --record "$tmp/run.mp4" --width 960 --height 540 --fps 30 --scale 960 --playback 0.5)
status=0
if [ -z "${DISPLAY:-}" ]; then
  xvfb-run -a -s "-screen 0 1280x720x24" "$exe" "${args[@]}" || status=$?
else
  "$exe" "${args[@]}" || status=$?
fi
if [ "$status" -ne 0 ]; then
  echo "WARNING: the recorded run was not a successful backflip (exit code $status); the GIF shows it anyway" >&2
fi

ffmpeg -loglevel error -y -i "$tmp/run.mp4" \
  -vf "fps=15,scale=480:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle" \
  -loop 0 "$out"
echo "wrote $out ($(du -h "$out" | cut -f1))"
