# 构建与运行快速指南

当前启动使用 `/home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash`，而非 `ros2_ly_ws_sentry-Behavion`。
`/home/hustlyrm/sentry.aim/start.bash` 现在会先 source 云台工作空间，再 source aim 工作空间。

## 1. 构建

```bash
cd /home/hustlyrm/sentry.aim && source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && colcon build --symlink-install && source install/setup.bash
```

## 2. 枪管跟随，不自动开火

### 终端 1：云台驱动

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && ros2 launch gimbal_driver gimbal_driver.launch.py use_virtual_device:=false
```

### 终端 2：自瞄管线

```bash
cd /home/hustlyrm/sentry.aim && bash start.bash
```

### 终端 3：Foxglove

```bash
foxglove
```

ros2 topic pub -r 5 /ly/aim/select_target \
    sentry_msgs/msg/AimTarget '{id: 6}'

sudo pkill -9 -f "ros2|rclcpp|rclpy|launch|livox|dlio|foxglove|gimbal|camera|aim"

## 3. 验证

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

## 4. 启用自动开火

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

## 5. 手动开火

当 `manual_fire_mode: True` 时，可从任意终端发布。

单发射击：

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && ros2 topic pub -1 /ly/control/firecode std_msgs/msg/UInt8 "data: 1"
```

以 2 Hz 连续射击：

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && ros2 topic pub /ly/control/firecode std_msgs/msg/UInt8 "data: 1" -r 2
```

## 6. 停止

先用 Ctrl+C 停止终端 2，等待几秒让节点退出，再停止终端 1。Foxglove 单独用 Ctrl+C 停止。
