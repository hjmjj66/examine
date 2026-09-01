# 构建与运行指南

`src` 是唯一构建与运行的自瞄源码；根目录的 `ly_aim` 仅是迁移参考，可在确认不再需要对照后删除。启动依赖云台工作区 `/home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash`。

## 1. 构建自瞄工作区

```bash
cd /home/hustlyrm/sentry.aim && source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && colcon build --symlink-install --base-paths src && source install/setup.bash
```
一键source:
source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source /home/hustlyrm/sentry.aim/install/setup.bash
## 2. 完整自瞄启动

云台驱动和自矄链路必须分开启动，并保持在两个终端中运行。`start.bash` 不启动云台驱动，只启动 TF、相机、solver、tracker、决策和控制节点。

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

## 3. 指定攻击目标（以id=6前哨站为例）

```bash
source /opt/ros/humble/setup.bash && source /home/hustlyrm/ros2_ly_ws_sentry/install/setup.bash && source /home/hustlyrm/sentry.aim/install/setup.bash && ros2 topic pub -r 5 /ly/aim/select_target sentry_msgs/msg/AimTarget '{id: 6}'
```

## 4. 自动开火与控制输出参数

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

## 5. 手动开火

当 `manual_fire_mode: true` 时，controller 不会发布开火码；由云台工作区使用的实际 FireCode 消息接口手动控制开火。

## 6. 停止

先用 Ctrl+C 停止终端 2，等待几秒让节点退出，再停止终端 1。Foxglove 单独用 Ctrl+C 停止。
或：
sudo pkill -9 -f "ros2|rclcpp|rclpy|launch|livox|dlio|foxglove|gimbal|camera|aim"

## 7. 云台控制链路与启动模式

云台控制有三种模式，必须选择其中一种控制发布者，不能让行为树、debug_control_bridge 和 armor_controller_node 同时发布 /ly/control/*。

### 7.1 正式行为树模式（推荐）

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

### 7.2 不经过行为树的控制器直连模式

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

publish_control_angles 是直接角度控制的关键；publish_control_trajectory 用于额外发布 MPC 六量轨迹。两个参数都打开时，只允许 armor_controller_node 作为 /ly/control/* 的控制发布者。

### 7.3 使用 debug_control_bridge 的调试模式

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

debug_node.launch.py 默认 aim_mode=false，不会订阅 /ly/aim/result；同时启动行为树或打开 armor_controller_node 的两个直接控制参数，都会造成多个节点争抢 /ly/control/*。
