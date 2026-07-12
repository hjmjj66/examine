#!/usr/bin/env bash
# =============================================================================
# sentry.aim_new 一键启动脚本
#
# 用法:
#   在 sentry.aim_new 目录下执行:
#   source install/setup.bash && bash start.bash
#   或者直接:
#   bash start.bash
#
# 停止: 按 Ctrl+C
# =============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"
GIMBAL_SETUP_FILE="/home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash"
PIDS=()

echo "======================================================================"
echo "  sentry.aim_new 系统启动"
echo "  工作空间: ${WORKSPACE_ROOT}"
echo "======================================================================"

if [ ! -f "${SETUP_FILE}" ]; then
    echo "[错误] 找不到 ${SETUP_FILE}"
    echo "       请先在 ROS 2 环境中完成 sentry.aim_new 的构建"
    exit 1
fi

if [ ! -f "${GIMBAL_SETUP_FILE}" ]; then
    echo "[ERROR] Missing ${GIMBAL_SETUP_FILE}"
    echo "       sentry_tf depends on gimbal_driver custom messages; build ros2_ly_ws_sentry first"
    exit 1
fi

source "${GIMBAL_SETUP_FILE}"
source "${SETUP_FILE}"

cleanup() {
    echo ""
    echo "======================================================================"
    echo "  正在停止所有已启动节点..."
    if ((${#PIDS[@]} > 0)); then
        kill "${PIDS[@]}" 2>/dev/null || true
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
    echo "  所有节点已停止"
    echo "======================================================================"
}

trap cleanup EXIT INT TERM

launch_bg() {
    local desc="$1"
    shift
    echo "  [启动] ${desc}"
    "$@" &
    PIDS+=("$!")
}

echo ""
echo "--- ① TF 基础设施（优先启动）---"
launch_bg "sentry_tf" ros2 launch sentry_tf sentry_tf.launch.py
sleep 2

echo ""
echo "--- ② 三相机检测链路 ---"
launch_bg "front camera #0 detector" ros2 launch aim_armor_detector front_camera_0_detector.launch.py
launch_bg "front camera #1 detector" ros2 launch aim_armor_detector front_camera_1_detector.launch.py
launch_bg "back camera detector" ros2 launch aim_armor_detector back_camera_detector.launch.py
sleep 3

echo ""
echo "--- ③ PnP 解算 + 目标预测 + 前哨站预测 ---"
launch_bg "aim_solver" ros2 launch aim_solver aim_solver.launch.py
launch_bg "aim_predictor" ros2 launch aim_predictor aim_predictor.launch.py
launch_bg "aim_outpost_predictor" ros2 launch aim_outpost_predictor aim_outpost_predictor.launch.py
sleep 1

echo ""
echo "--- ④ 决策 + 弹道控制 ---"
launch_bg "aim_armor_decider" ros2 launch aim_armor_decider aim_decider.launch.py
launch_bg "aim_armor_controller" ros2 launch aim_armor_controller armor_controller.launch.py

echo ""
echo "======================================================================"
echo "  全部配置节点已启动"
echo "  Ctrl+C 可停止所有节点"
echo ""

wait
