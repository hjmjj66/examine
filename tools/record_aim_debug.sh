#!/usr/bin/env bash

set -u

# Record the control/targeting chain needed to diagnose /ly/aim/result jumps.
# This intentionally excludes camera images, compressed images, and point clouds.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_BASE="${1:-${ROOT_DIR}/bags}"
OUT_DIR="${OUT_BASE}/aim_debug_${STAMP}"
META_DIR="${OUT_DIR}_meta"

# Allow running this script from a fresh terminal. Source every workspace that
# exists; later workspaces overlay the message/package environment correctly.
for setup in \
  /opt/ros/humble/setup.bash \
  /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash \
  "${ROOT_DIR}/install/setup.bash"; do
  if [[ -f "${setup}" ]]; then
    # shellcheck disable=SC1090
    # Generated ROS setup files use optional variables without nounset-safe
    # expansions, so relax nounset only while sourcing them.
    set +u
    source "${setup}"
    set -u
  fi
done

mkdir -p "${OUT_BASE}" "${META_DIR}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：找不到 ros2，请确认 ROS2 已安装。" >&2
  exit 1
fi

TOPIC_LIST=(
  /ly/aim/result
  /ly/gimbal/angles
  /ly/game/bullet
  /ly/bullet/speed
  /ly/control/angles
  /ly/control/firecode
  /ly/aim/armor_targets
  /decider/selected_target_id
  /aim_predictor/fused/target_states
  /aim_predictor/front_0/target_states
  /aim_predictor/front_1/target_states
  /aim_predictor/back/target_states
  /aim_outpost_predictor/outpost_state
  /aim_solver/front_0/armor_pose_sets
  /aim_solver/front_1/armor_pose_sets
  /aim_solver/back/armor_pose_sets
  /aim_detector/front_1/armor_sets
  /gx_camera_1/camera_info
  /aim_solver/front_1/visualization
  /aim_outpost_predictor/visualization
  /armor_controller/debug/target_point_barrel
  /armor_controller/debug/selected_armor_index
  /controller/selected_armor
  /rosout
  /tf
  /tf_static
)

ros2 node list > "${META_DIR}/nodes.txt" 2>&1 || true
ros2 topic list -t > "${META_DIR}/topics.txt" 2>&1 || true
ros2 topic info /ly/aim/result -v > "${META_DIR}/aim_result_topic_info.txt" 2>&1 || true
ros2 topic info /aim_predictor/fused/target_states -v \
  > "${META_DIR}/fused_target_states_topic_info.txt" 2>&1 || true
ros2 interface show sentry_msgs/msg/AimResult \
  > "${META_DIR}/aim_result_interface.txt" 2>&1 || true
ros2 interface show aim_msgs/msg/TargetStateArray \
  > "${META_DIR}/target_state_array_interface.txt" 2>&1 || true

mapfile -t AVAILABLE_TOPICS < <(ros2 topic list 2>/dev/null || true)

has_topic() {
  local wanted="$1"
  local topic
  for topic in "${AVAILABLE_TOPICS[@]}"; do
    if [[ "${topic}" == "${wanted}" ]]; then
      return 0
    fi
  done
  return 1
}

RECORD_TOPICS=()
for topic in "${TOPIC_LIST[@]}"; do
  if has_topic "${topic}"; then
    RECORD_TOPICS+=("${topic}")
  else
    echo "未发现，跳过：${topic}" | tee -a "${META_DIR}/skipped_topics.txt"
  fi
done

if [[ "${#RECORD_TOPICS[@]}" -eq 0 ]]; then
  echo "错误：没有发现可录制话题。请确认所有节点已经启动。" >&2
  exit 1
fi

printf '%s\n' "${RECORD_TOPICS[@]}" > "${META_DIR}/recorded_topics.txt"
{
  echo "开始时间：$(date --iso-8601=seconds)"
  echo "录制目录：${OUT_DIR}"
  echo "元数据目录：${META_DIR}"
  echo "录制话题："
  printf '  %s\n' "${RECORD_TOPICS[@]}"
  echo
  echo "操作流程：启动后完成 1~2 次‘下电调角度→上电让自瞄转云台’循环，完成后按 Ctrl-C。"
} | tee "${META_DIR}/README.txt"

cleanup() {
  echo
  echo "录制已停止，正在写入 bag..."
  ros2 bag info "${OUT_DIR}" > "${META_DIR}/bag_info.txt" 2>&1 || true
  echo "完成。请打包以下两个目录后发我："
  echo "  ${OUT_DIR}"
  echo "  ${META_DIR}"
}
trap cleanup EXIT INT TERM

echo
echo "开始录制，完成你的操作后按 Ctrl-C。"
ros2 bag record --storage sqlite3 -o "${OUT_DIR}" "${RECORD_TOPICS[@]}"
