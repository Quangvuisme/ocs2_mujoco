#!/usr/bin/env bash
# Records doc/gifs/g1.gif: a lock-step run (stand up, walk 2.5 m at 0.4 m/s, stop), rendered
# offscreen (needs ffmpeg; uses xvfb-run when there is no display).
#
# Usage: make_gif.sh [path/to/g1_walking_mujoco] [output.gif]
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../.." && pwd)"
exe="${1:-$repo/build/g1_walking/g1_walking_mujoco}"
out="${2:-$repo/doc/gifs/g1.gif}"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# 960x540 MP4 at 1.5x playback, then a 480 px / 10 fps GIF without dithering (keeps the file small)
args=(--headless --vx 0.4 --goal-x 2.5 --duration 16 --record "$tmp/run.mp4"
      --width 960 --height 540 --fps 10 --scale 960 --playback 1.5)
status=0
if [ -z "${DISPLAY:-}" ]; then
  xvfb-run -a -s "-screen 0 1280x720x24" "$exe" "${args[@]}" || status=$?
else
  "$exe" "${args[@]}" || status=$?
fi
if [ "$status" -ne 0 ]; then
  echo "WARNING: the recorded run did not reach the goal (exit code $status); the GIF shows it anyway" >&2
fi

ffmpeg -loglevel error -y -i "$tmp/run.mp4" \
  -vf "fps=10,scale=480:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle" \
  -loop 0 "$out"
echo "wrote $out ($(du -h "$out" | cut -f1))"
