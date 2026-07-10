# 自瞄测试快速指南

## 1. 编译

```bash
cd /home/hustlyrm/sentry.aim
source /opt/ros/humble/setup.bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. 枪管跟随（不发射）

### 终端 1：云台驱动

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash
ros2 launch gimbal_driver gimbal_driver.launch.py use_virtual_device:=false
```

### 终端 2：自瞄链路

```bash
source /home/hustlyrm/sentry.aim/install/setup.bash
cd /home/hustlyrm/sentry.aim && bash start.bash
```

### 验证

```bash
ros2 topic echo /ly/control/angles        # 枪管控制角度
ros2 topic echo /ly/aim/result            # fire 应为 false
ros2 topic echo /aim_outpost_predictor/outpost_state   # 前哨站跟踪
```

## 3. 自动发射

修改 `src/aim_armor_controller/config/armor_controller.yaml` L18：

```yaml
manual_fire_mode: False
```

重编重启：

```bash
cd /home/hustlyrm/sentry.aim
colcon build --symlink-install --packages-select aim_armor_controller
source install/setup.bash
# 停终端 2，重新 bash start.bash
```

## 4. 手动发射

`manual_fire_mode: True` 时，在任意终端：

```bash
# 单发
ros2 topic pub -1 /ly/control/firecode std_msgs/msg/UInt8 "data: 1"

# 连发 2Hz
ros2 topic pub /ly/control/firecode std_msgs/msg/UInt8 "data: 1" -r 2
```

## 5. 停车

先 Ctrl+C 终端 2，等节点退完（几秒），再 Ctrl+C 终端 1。
