# 构建与运行指南

`src` 是唯一构建与运行的自瞄源码；根目录的 `ly_aim` 仅是迁移参考，可在确认不再需要对照后删除。启动依赖云台工作区 `/home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash`。

## 1. 构建自瞄工作区

```bash
cd /home/hustlyrm/sentry.aim && source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && colcon build --symlink-install --base-paths src && source install/setup.bash
```
一键source:
source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source /home/hustlyrm/sentry.aim/install/setup.bash
## 2. 启动自瞄节点

`start.bash` 不启动云台驱动，也不启动行为树，只启动 TF、相机、预测、决策和控制节点。执行本节后，必须按第 10 节选择行为树、控制器直连或 debug bridge 模式；仅执行本节命令不会产生 `/ly/control/*` 控制输出。

终端 1：启动正式云台驱动并保持运行：

```bash
source /opt/ros/humble/setup.bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash
ros2 launch gimbal_driver gimbal_driver.launch.py
```

终端 2：启动自瞄链路：

```bash
cd /home/hustlyrm/sentry.aim && bash start.bash
```

正式链路不要使用 `debug_node.launch.py`；它还会启动 `debug.py` 控制桥，适合单独调试。

## 3. 手动指定攻击目标（关闭自动选目标后）

当前 `aim_armor_decider.yaml` 使用 `auto_select_nearest: true`，外部 `/ly/aim/select_target` 指令会被忽略并自动选择最近目标。若要手动指定目标，先将该参数改为 `false`，重新启动决策节点；否则无需执行下面的命令。

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

查看图像/TF 时间偏移估计器。`reason` 为
`insufficient_gimbal_motion`、`waiting_for_samples` 或
`score_not_identifiable` 时会冻结偏移，不会把噪声写入目标 EKF：

```bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source /home/hustlyrm/sentry.aim/install/setup.bash && ros2 topic echo /aim_solver/time_alignment_status
```

GX 相机的 Image 和 CameraInfo 使用同一个“设备 tick → ROS 时间”映射。
`timestamp_offset_sec` 只保留为设备侧残差补偿，默认是 `0.0`；Solver 的
`tf_timestamp_offset_sec` 是自适应估计器关闭时的后备值。
诊断录包还会记录 `/gx_camera_0/timestamp_status` 和
`/gx_camera_0/image_raw`；后者数据量较大，但能验证设备 tick 映射是否实际生效。

如需做六组对比，不要改正式配置文件：

```bash
cd /home/hustlyrm/sentry.aim
./tools/make_time_alignment_variants.sh /tmp/sentry_time_alignment_variants
```

分别用以下环境变量启动完整系统，每组都录制一次相同的“下电调角度→上电接管”动作：

```bash
AIM_SOLVER_CONFIG=/tmp/sentry_time_alignment_variants/fixed_zero.yaml bash start.bash
AIM_SOLVER_CONFIG=/tmp/sentry_time_alignment_variants/fixed_minus_20ms.yaml bash start.bash
AIM_SOLVER_CONFIG=/tmp/sentry_time_alignment_variants/fixed_minus_25ms.yaml bash start.bash
AIM_SOLVER_CONFIG=/tmp/sentry_time_alignment_variants/fixed_minus_30ms.yaml bash start.bash
AIM_SOLVER_CONFIG=/tmp/sentry_time_alignment_variants/fixed_minus_35ms.yaml bash start.bash
AIM_SOLVER_CONFIG=/tmp/sentry_time_alignment_variants/adaptive_minus_25ms_seed.yaml bash start.bash
```

每次启动后另开终端执行 `./tools/record_aim_debug.sh`，结束后先停录包，再停系统。

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
tar -czf aim_debug.tar.gz bags/aim_debug_[0-9]*
```

把生成的 `aim_debug.tar.gz` 发回即可。脚本还会录制 GX0 原始图像，因此 bag 可能很大。

## 6. 自动开火与控制输出参数

推荐的直连轨迹控制配置：

```yaml
publish_legacy_control_topics: true
publish_control_trajectory: true
publish_control_angles: false
manual_fire_mode: false
```

| 参数 | 推荐状态 | 作用 |
| --- | --- | --- |
| `publish_legacy_control_topics` | `true` | 发布兼容控制话题，其中 `/ly/control/firecode` 是自动开火通道 |
| `publish_control_trajectory` | `true` | 发布 `/ly/control/trajectory`，使用 MPC 六量轨迹控制云台 |
| `publish_control_angles` | `false` | 关闭旧的角度控制输出，避免与轨迹控制同时输出 |
| `manual_fire_mode` | `false` | 满足全部开火门控时允许控制器发布开火码 |

修改 YAML 后，重新构建 `aim_armor_controller` 并重启终端 2。

## 7. 手动开火

当 `manual_fire_mode: true` 时，controller 不会发布开火码；由云台工作区使用的实际 FireCode 消息接口手动控制开火。

请以 `gimbal_driver` 当前提供的 FireCode 消息定义为准，避免向 `/ly/control/firecode` 发布错误的 `std_msgs/UInt8` 类型。

## 8. 停止

先用 Ctrl+C 停止终端 2，等待几秒让节点退出，再停止终端 1。Foxglove 单独用 Ctrl+C 停止。`start.bash` 会在 Ctrl+C 时清理它启动的节点，不要使用无范围的进程匹配命令，以免误杀同机其他 ROS、相机或 Foxglove 进程。

## 9. 单独运行云台驱动

运行 `bash start.bash` 前，必须在终端 1 中单独启动并保持云台驱动运行：

```bash
source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && ros2 launch gimbal_driver gimbal_driver.launch.py
```

## 10. 云台控制链路与启动模式

云台控制有三种模式，必须选择其中一种控制发布者，不能让行为树、debug_control_bridge 和 armor_controller_node 同时发布 /ly/control/*。

### 10.1 正式行为树模式（推荐）

完整链路如下：

~~~text
sentry.aim -> /ly/aim/armor_targets + /ly/aim/result
           -> behavior_tree
           -> /ly/control/angles 或 /ly/control/trajectory
           -> gimbal_driver -> 下位机
~~~

终端 1：启动正式云台驱动和行为树。不要启动 gimbal_driver 的 debug_node.launch.py。

~~~bash
cd /home/hustlyrm/ros2_ly_ws_sentry
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch behavior_tree competition_autoaim.launch.py
~~~

终端 2：启动 sentry.aim 自瞄链路。

~~~bash
cd /home/hustlyrm/sentry.aim
source /opt/ros/humble/setup.bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash
source install/setup.bash
bash start.bash
~~~

该模式下，armor_controller.yaml 中应保持：

~~~yaml
publish_control_angles: false
publish_control_trajectory: false
~~~

armor_controller_node 只发布 /ly/aim/result；behavior_tree 负责校验有效结果并发布正式云台控制话题。behavior_tree 会接管 /ly/control/*，不能再并行启动 debug_node.launch.py。

### 10.2 不经过行为树的控制器直连模式

该模式由 sentry.aim 直接发布云台控制。配置：

~~~yaml
publish_control_angles: false
publish_control_trajectory: true
~~~

启动云台驱动时使用正式入口，不要启动 debug bridge：

~~~bash
# 终端 1
source /opt/ros/humble/setup.bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash
ros2 launch gimbal_driver gimbal_driver.launch.py

# 终端 2
cd /home/hustlyrm/sentry.aim
bash start.bash
~~~

`publish_control_angles` 发布直接角度控制；`publish_control_trajectory` 发布 MPC 六量轨迹控制。直连模式只开启其中一个，避免同一控制链同时接收两种控制消息。

### 10.3 使用 debug_control_bridge 的调试模式

该模式不经过行为树，由 debug_control_bridge 把 /ly/aim/result 转换为 /ly/control/*。

armor_controller.yaml 中保持：

~~~yaml
publish_control_angles: false
publish_control_trajectory: false
~~~

debug_mode.yaml 必须启用：

~~~yaml
aim_mode: true
~~~

然后启动：

~~~bash
# 终端 1
source /opt/ros/humble/setup.bash
source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash
ros2 launch gimbal_driver debug_node.launch.py

# 终端 2
cd /home/hustlyrm/sentry.aim
bash start.bash
~~~

`debug_node.launch.py` 默认加载 `debug_mode.yaml`，当前默认 `aim_mode: true`，会订阅 `/ly/aim/result`；只有将 `aim_mode` 设为 `false` 时才不会订阅。同时启动行为树或打开 armor_controller_node 的两个直接控制参数，都会造成多个节点争抢 `/ly/control/*`。

### 10.4 控制链路验证

查看 result 是否为有效瞄准结果：

~~~bash
ros2 topic echo /ly/aim/result
~~~

只有 follow=true 的结果才会被行为树或 debug_control_bridge 接受。检查正式云台控制话题：

~~~bash
ros2 topic info -v /ly/control/angles
ros2 topic info -v /ly/control/trajectory
~~~

确认发布者只有一个，并且类型分别为：

~~~text
gimbal_driver/msg/GimbalAngles
gimbal_driver/msg/GimbalTrajectory
~~~

如果只能看到 /ly/aim/result，而 /ly/control/angles 和 /ly/control/trajectory 没有发布者，说明自瞄结果还没有进入云台控制链。
