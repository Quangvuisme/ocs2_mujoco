#!/usr/bin/env bash
# Records doc/gifs/anymal_perceptive.gif: a lock-step run over the whole terrain course, rendered
# offscreen (needs ffmpeg; uses xvfb-run when there is no display).
#
# Usage: make_gif.sh [path/to/anymal_perceptive_mujoco] [output.gif]
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../.." && pwd)"
exe="${1:-$repo/build/anymal_perceptive/anymal_perceptive_mujoco}"
out="${2:-$repo/doc/gifs/anymal_perceptive.gif}"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# 960x540 MP4 at 2.5x playback, then a 480 px / 8 fps GIF without dithering (keeps the file small)
args=(--headless --vx 0.3 --goal-x 8.3 --duration 60 --record "$tmp/run.mp4"
      --width 960 --height 540 --fps 10 --scale 960 --playback 2.5 --map-stride 3)
if [ -z "${DISPLAY:-}" ]; then
  xvfb-run -a -s "-screen 0 1280x720x24" "$exe" "${args[@]}"
else
  "$exe" "${args[@]}"
fi

ffmpeg -loglevel error -y -i "$tmp/run.mp4" \
  -vf "fps=8,scale=480:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle" \
  -loop 0 "$out"
echo "wrote $out ($(du -h "$out" | cut -f1))"
