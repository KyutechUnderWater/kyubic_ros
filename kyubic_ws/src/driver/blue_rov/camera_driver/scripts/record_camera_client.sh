#!/usr/bin/env bash
# 地上PC (既定: 192.168.2.98) で BlueROV カメラ映像を録画する。
#
# camera_driver が forward_host へ転送する RTP/H264 (UDP) を受信し MP4 に保存する。
# ロボット側で camera_driver を起動してから、このスクリプトを地上PCで実行する。
#
# Windows PC の場合は record_camera_client.ps1 または record_camera_client.bat を使う。
#
# Usage:
#   ./record_camera_client.sh [output_dir] [udp_port]
#
# Example:
#   ./record_camera_client.sh ~/bluerov_videos 5601

set -euo pipefail

OUTPUT_DIR="${1:-${HOME}/bluerov_videos}"
UDP_PORT="${2:-5601}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_FILE="${OUTPUT_DIR}/bluerov_${TIMESTAMP}.mp4"

mkdir -p "${OUTPUT_DIR}"

echo "UDP ${UDP_PORT} で RTP/H264 を受信し、${OUTPUT_FILE} に録画します。"
echo "停止するには Ctrl+C を押してください。"

cleanup() {
    echo
    echo "録画を停止しました: ${OUTPUT_FILE}"
}
trap cleanup EXIT INT TERM

exec gst-launch-1.0 -e \
    udpsrc port="${UDP_PORT}" caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
    rtpjitterbuffer latency=200 ! \
    rtph264depay ! \
    h264parse ! \
    mp4mux ! \
    filesink location="${OUTPUT_FILE}"
