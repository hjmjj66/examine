# sentry.aim_new 启动说明


## 1. 准备

进入 `sentry.aim_new` 目录：

```bash
cd /path/to/sentry.aim_new
```

确认你已经具备这些环境：

- Ubuntu + ROS 2
- `colcon`
- 相机驱动依赖
- detector 依赖的推理环境
- 本工程依赖的外部工作空间已经能正常 `source`

如果你的环境还没配好，不要继续往下做。

## 2. 首次构建

在 ROS 2 环境里执行：

```bash
cd /path/to/sentry.aim_new
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

如果你的机器上还有额外依赖工作空间，先 `source` 它，再执行上面的构建命令。

## 3. 日常重新构建

代码改完后：

```bash
cd /path/to/sentry.aim_new
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果只想重编某个包：

```bash
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```


## 4. 一键启动

构建完成后，在 `sentry.aim_new` 目录执行：

```bash
cd /path/to/sentry.aim_new
bash start.bash
```

脚本会依次启动：

1. `sentry_tf`
2. 三路 detector
3. `aim_solver`
4. `aim_predictor`
5. `aim_outpost_predictor`
6. `aim_armor_decider`
7. `aim_armor_controller`

如果你已经手动 `source install/setup.bash`，也可以这样启动：

```bash
source install/setup.bash
bash start.bash
```

## 5. 启动后检查

启动完成后，先检查节点和话题：

```bash
ros2 node list
ros2 topic list
```

检查最终输出：

```bash
ros2 topic hz /ly/aim/result
```

如果要看三路相机 `camera_info`：

```bash
ros2 topic echo /gx_camera_0/camera_info
ros2 topic echo /gx_camera_1/camera_info
ros2 topic echo /usb_camera/camera_info
```

如果要检查三路 TF：

```bash
ros2 run tf2_ros tf2_echo gimbal_world gx_camera_0
ros2 run tf2_ros tf2_echo gimbal_world gx_camera_1
ros2 run tf2_ros tf2_echo gimbal_world usb_camera
```

## 6. 停止

在运行 `start.bash` 的终端里按：

```bash
Ctrl+C
```

脚本会自动停止它启动的所有节点。

## 7. 常见情况

### 7.1 `start.bash` 提示找不到 `install/setup.bash`

先构建：

```bash
cd /path/to/sentry.aim_new
colcon build --symlink-install
source install/setup.bash
```

### 7.2 某个节点能启动，但没有数据

先查：

```bash
ros2 node list
ros2 topic list
```

再分别看：

```bash
ros2 topic echo /gx_camera_0/image_raw
ros2 topic echo /gx_camera_1/image_raw
ros2 topic echo /usb_camera/image_raw
```

### 7.3 改了配置但效果没变

重新执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

然后重新运行：

```bash
bash start.bash
```
