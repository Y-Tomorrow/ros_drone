#!/usr/bin/env bash
# 从 config/target_escape_teleop.yaml 读取 mission.target_initial_distance_m，
# 并传给 two_uav_depth_escape.launch 的 target_offset_east_m（与 YAML 保持单一来源）。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
D="$(python3 "${ROOT}/scripts/read_mission_distance.py" "${ROOT}/config/target_escape_teleop.yaml")"
exec roslaunch ros_drone two_uav_depth_escape.launch target_offset_east_m:="${D}" "$@"
