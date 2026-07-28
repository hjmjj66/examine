#!/usr/bin/env bash

set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_BASE="${1:-${ROOT_DIR}/bags}"
OUT_DIR="${OUT_BASE}/fire_control_debug_${STAMP}"
META_DIR="${OUT_DIR}_meta"

for setup in \
  /opt/ros/humble/setup.bash \
  /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash \
  "${ROOT_DIR}/install/setup.bash"; do
  if [[ -f "${setup}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${setup}"
    set -u
  fi
done

mkdir -p "${OUT_BASE}" "${META_DIR}"

TOPIC_LIST=(
  /ly/aim/result
  /ly/aim/armor_targets
  /ly/aim/select_target
  /ly/gimbal/angles
  /ly/gimbal/state
  /ly/gimbal/firecode
  /ly/control/angles
  /ly/control/trajectory
  /ly/control/firecode
  /ly/game/bullet
  /decider/selected_target_id
  /aim_predictor/fused/target_states
  /aim_predictor/front_0/target_states
  /aim_predictor/front_1/target_states
  /aim_outpost_predictor/outpost_state
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
ros2 topic info /ly/control/firecode -v > "${META_DIR}/control_firecode_topic_info.txt" 2>&1 || true
ros2 topic info /ly/control/trajectory -v > "${META_DIR}/control_trajectory_topic_info.txt" 2>&1 || true
ros2 param dump /armor_controller_node > "${META_DIR}/armor_controller_params.yaml" 2>&1 || true
ros2 param dump /debug_control_bridge > "${META_DIR}/debug_control_bridge_params.yaml" 2>&1 || true
ros2 param dump /gimbal_driver > "${META_DIR}/gimbal_driver_params.yaml" 2>&1 || true

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
  echo "错误：没有发现可录制话题。请先启动 gimbal_driver/debug_node 和 start.bash。" >&2
  exit 1
fi

printf '%s\n' "${RECORD_TOPICS[@]}" > "${META_DIR}/recorded_topics.txt"
{
  echo "开始时间：$(date --iso-8601=seconds)"
  echo "录制目录：${OUT_DIR}"
  echo "元数据目录：${META_DIR}"
  echo "请录到现象出现：自瞄固定朝一个点、firecode 翻转/不翻转、或实际打弹异常。"
  echo "完成后按 Ctrl-C。"
} | tee "${META_DIR}/README.txt"

cleanup() {
  echo
  echo "录制已停止，正在写入 bag info..."
  ros2 bag info "${OUT_DIR}" > "${META_DIR}/bag_info.txt" 2>&1 || true
  echo "完成："
  echo "  ${OUT_DIR}"
  echo "  ${META_DIR}"
}
trap cleanup EXIT INT TERM

ros2 bag record --storage sqlite3 -o "${OUT_DIR}" "${RECORD_TOPICS[@]}"
