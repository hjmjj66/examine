#!/usr/bin/env bash
# =============================================================================
# sentry.aim 一键启动脚本
#
# 用法:
#   在任意目录下直接执行:
#   bash start.bash
#
# 停止: 按 Ctrl+C
# =============================================================================
source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source install/setup.bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"
GIMBAL_SETUP_FILE="/home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash"
PIDS=()

if [ ! -f "${GIMBAL_SETUP_FILE}" ]; then
    echo "[ERROR] Missing ${GIMBAL_SETUP_FILE}"
    echo "       sentry_tf depends on gimbal_driver custom messages; build ros2_ly_ws_sentry first"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "${GIMBAL_SETUP_FILE}"
cd "${WORKSPACE_ROOT}"
#colcon build --symlink-install --base-paths src

if [ ! -f "${SETUP_FILE}" ]; then
    echo "[错误] 找不到 ${SETUP_FILE}"
    exit 1
fi

source "${SETUP_FILE}"

echo "======================================================================"
echo "  sentry.aim 自瞄系统启动"
echo "  工作空间: ${WORKSPACE_ROOT}"
echo "======================================================================"

SOLVER_LAUNCH=(ros2 launch aim_solver aim_solver.launch.py)
if [[ -n "${AIM_SOLVER_CONFIG:-}" ]]; then
    SOLVER_LAUNCH+=("config_file:=${AIM_SOLVER_CONFIG}")
    echo "  Solver 配置覆盖: ${AIM_SOLVER_CONFIG}"
fi

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

check_processes() {
    local pid
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            echo "[错误] 自瞄节点启动失败或已提前退出 (pid=${pid})"
            exit 1
        fi
    done
}

echo ""
echo "--- ① TF 基础设施（优先启动）---"
# launch_bg "gimbal_driver" ros2 launch gimbal_driver gimbal_driver.launch.py
launch_bg "sentry_tf" ros2 launch sentry_tf sentry_tf.launch.py
sleep 1
check_processes

echo ""
echo "--- ② 三相机检测链路 ---"
launch_bg "front camera #0 detector" ros2 launch aim_armor_detector front_camera_0_detector.launch.py
launch_bg "front camera #1 detector" ros2 launch aim_armor_detector front_camera_1_detector.launch.py
# launch_bg "back camera detector" ros2 launch aim_armor_detector back_camera_detector.launch.py
sleep 1
check_processes

echo ""
echo "--- ③ PnP 解算 + 目标预测 + 前哨站预测 ---"
launch_bg "aim_solver" "${SOLVER_LAUNCH[@]}"
launch_bg "aim_predictor" ros2 launch aim_predictor aim_predictor.launch.py
launch_bg "aim_outpost_predictor" ros2 launch aim_outpost_predictor aim_outpost_predictor.launch.py
sleep 1
check_processes

echo ""
echo "--- ④ 决策 + 弹道控制 ---"
launch_bg "aim_armor_decider" ros2 launch aim_armor_decider aim_decider.launch.py
launch_bg "aim_armor_controller" ros2 launch aim_armor_controller armor_controller.launch.py
check_processes

echo ""
echo "======================================================================"
echo "  所有节点进程存活，等待相机、TF 与外部话题数据"
echo "  Ctrl+C 可停止所有节点"
echo ""

wait
