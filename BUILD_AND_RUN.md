# 构建与运行指南

`src` 是唯一构建与运行的自瞄源码；根目录的 `ly_aim` 仅是迁移参考，可在确认不再需要对照后删除。启动依赖云台工作区 `/home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash`。

## 1. 构建自瞄工作区

```bash
cd /home/hustlyrm/sentry.aim && source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && colcon build --symlink-install && source install/setup.bash
```

## 2. 启动完整自瞄

终端 1 启动真实云台驱动：

```bash
source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && ros2 launch gimbal_driver gimbal_driver.launch.py
```

终端 2 启动三相机检测、三路 PnP、融合预测、前哨站预测、决策和控制：

```bash
cd /home/hustlyrm/sentry.aim && bash start.bash
```

默认 `manual_fire_mode: true`，会输出瞄准角度但不自动发送开火码。

## 3. 指定攻击目标

```bash
source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source /home/hustlyrm/sentry.aim/install/setup.bash && ros2 topic pub -r 5 /ly/aim/select_target sentry_msgs/msg/AimTarget '{id: 6}'
```

## 4. 验证

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && ros2 topic echo /ly/gimbal/angles
```

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source /home/hustlyrm/sentry.aim/install/setup.bash && ros2 topic echo /ly/control/angles
```

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source /home/hustlyrm/sentry.aim/install/setup.bash && ros2 topic echo /ly/aim/result
```

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source /home/hustlyrm/sentry.aim/install/setup.bash && ros2 topic echo /aim_outpost_predictor/outpost_state
```

## 5. 录制自瞄诊断数据

新开一个终端，直接执行下面的命令即可，不需要手动 `source`：

```bash
cd /home/hustlyrm/sentry.aim && ./tools/record_aim_debug.sh
```

脚本会自动尝试加载 ROS2、云台工作区和当前自瞄工作区。必须在云台驱动、自瞄节点都启动后再执行录制命令。

录制开始后，按下面流程操作：

1. 启动云台与自瞄，确认遥控器处于下电状态；
2. 手动调整一次云台角度；
3. 遥控器上电，让自瞄接管并转动云台；
4. 重复“下电 → 调整云台角度 → 上电让自瞄转云台”流程 1～2 次；
5. 操作完成后，在录制脚本所在终端按 `Ctrl+C`。

脚本会在 `bags/` 下生成一个 bag 目录和一个同名的 `_meta` 元数据目录。录制结束后打包：

```bash
tar -czf aim_debug.tar.gz bags/aim_debug_* bags/aim_debug_*_meta
```

把生成的 `aim_debug.tar.gz` 发回即可。脚本只录控制、预测、TF 和调试话题，不录相机图像。

## 6. 启用自动开火

编辑 `src/aim_armor_controller/config/armor_controller.yaml`：

```yaml
manual_fire_mode: False
```

重新构建并重启终端 2：

```bash
cd /home/hustlyrm/sentry.aim && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-select aim_armor_controller && source install/setup.bash
```

```bash
cd /home/hustlyrm/sentry.aim && bash start.bash
```

## 7. 手动开火

当 `manual_fire_mode: true` 时，controller 不会发布开火码；由云台工作区使用的实际 FireCode 消息接口手动控制开火。

请以 `gimbal_driver` 当前提供的 FireCode 消息定义为准，避免向 `/ly/control/firecode` 发布错误的 `std_msgs/UInt8` 类型。

## 8. 停止

先用 Ctrl+C 停止终端 2，等待几秒让节点退出，再停止终端 1。Foxglove 单独用 Ctrl+C 停止。
或：
sudo pkill -9 -f "ros2|rclcpp|rclpy|launch|livox|dlio|foxglove|gimbal|camera|aim"
