#!/usr/bin/env bash
# stream_ffmpeg.sh - reference ffmpeg pipeline used by watchpass_streamer.
#
# The ffmpeg_streamer node builds an equivalent invocation programmatically, but
# this script is handy for testing the encoder/transport stage in isolation:
# it reads raw video from a FIFO (or stdin) and pushes it to a server.
#
# Usage:
#   # 1) create a FIFO the node (or any producer) can write raw frames into
#   mkfifo /tmp/watchpass.raw
#   # 2) start the encoder, pointing at that FIFO
#   ./stream_ffmpeg.sh --input /tmp/watchpass.raw --size 640x480 --fps 30 \
#       --target rtsp://127.0.0.1:8554/watchpass
#
# Targets:
#   rtsp://HOST:8554/NAME   push to mediamtx -> exposed as WebRTC at :8889
#   rtp://HOST:PORT         raw RTP/H.264
#   whip://.../whip         native WebRTC (ffmpeg >= 7.1 with the whip muxer)
set -euo pipefail

INPUT="pipe:0"
SIZE="640x480"
FPS="30"
PIX_FMT="rgb24"
TARGET="rtsp://127.0.0.1:8554/watchpass"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input)   INPUT="$2"; shift 2 ;;
    --size)    SIZE="$2"; shift 2 ;;
    --fps)     FPS="$2"; shift 2 ;;
    --pix-fmt) PIX_FMT="$2"; shift 2 ;;
    --target)  TARGET="$2"; shift 2 ;;
    -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

# Choose muxer flags based on the target scheme.
case "$TARGET" in
  rtsp://*) OUT=(-f rtsp -rtsp_transport tcp "$TARGET") ;;
  rtp://*)  OUT=(-f rtp "$TARGET") ;;
  whip://*) OUT=(-f whip "$TARGET") ;;
  *)        OUT=(-f rtsp "$TARGET") ;;
esac

exec ffmpeg -hide_banner -loglevel warning \
  -f rawvideo -pix_fmt "$PIX_FMT" -s "$SIZE" -r "$FPS" -i "$INPUT" \
  -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p \
  "${OUT[@]}"
