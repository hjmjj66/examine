# sentry.aim_new Topic、包处理流程与消息说明

这份文档说明 3 件事：

1. 整体 topic 之间的关系
2. 信息进入每个包后，这个包做了什么处理
3. `aim_msgs` / `sentry_msgs` 里各个消息的内容是什么

文档默认以 `sentry.aim_new/src1` 当前代码为准。

---

## 1. 整体链路总览

主链路可以拆成 8 段：

1. `sentry_tf`
2. `aim_camera_driver`
3. `aim_armor_detector`
4. `aim_solver`
5. `aim_predictor`
6. `aim_outpost_predictor`
7. `aim_armor_decider`
8. `aim_armor_controller`

整体数据流是：

```text
/ly/gimbal/angles
/ly/gimbal/big_yaw_angles
        |
        v
    sentry_tf
        |
        v
相机驱动(image_raw + camera_info)
        |
        v
armor_detector(2D 装甲板)
        |
        v
aim_solver(3D 位姿 / ArmorPoseSet)
        |
        +-------------------------------> aim_outpost_predictor(front_0 outpost 支路)
        |
        v
aim_predictor(普通车辆 TargetState)
        |
        +--------------------+
        |                    |
        v                    v
aim_armor_decider       aim_armor_controller
        |                    ^
        v                    |
/decider/selected_target_id -+
/ly/aim/armor_targets

外部还会输入：
/ly/aim/select_target
/ly/bullet/speed

最终输出：
/ly/aim/result
/ly/control/angles
/ly/control/firecode
```

---

## 2. Topic 总表

下面只列当前主流程里真正用到的 topic。

### 2.1 TF 与外部输入

| Topic | 来源 | 去向 | 作用 |
|---|---|---|---|
| `/ly/gimbal/angles` | 外部云台/底盘系统 | `sentry_tf`、`aim_armor_controller` | 提供当前 yaw/pitch；TF 构树；控制器读当前云台姿态 |
| `/ly/gimbal/big_yaw_angles` | 外部大 yaw 角输入 | `sentry_tf` | 构建大 yaw 对应 TF |
| `/ly/bullet/speed` | 外部测速/射击系统 | `aim_armor_controller` | 控制器弹道解算用弹速 |
| `/ly/aim/select_target` | 外部决策输入 | `aim_armor_decider` | 外部直接指定优先攻击哪辆车 |

### 2.2 相机输出

| Topic | 来源 | 去向 | 作用 |
|---|---|---|---|
| `/gx_camera_0/image_raw` | `GxCameraComponent` | `aim_armor_detector(front_0)` | 前相机 0 原图 |
| `/gx_camera_0/camera_info` | `GxCameraComponent` | `aim_solver(front_0)` | 前相机 0 内参 |
| `/gx_camera_1/image_raw` | `GxCameraComponent` | `aim_armor_detector(front_1)` | 前相机 1 原图 |
| `/gx_camera_1/camera_info` | `GxCameraComponent` | `aim_solver(front_1)` | 前相机 1 内参 |
| `/usb_camera/image_raw` | `UsbCameraComponent` | `aim_armor_detector(back)` | 后相机原图 |
| `/usb_camera/camera_info` | `UsbCameraComponent` | `aim_solver(back)` | 后相机内参 |

### 2.3 detector 输出

| Topic | 来源 | 去向 | 作用 |
|---|---|---|---|
| `/aim_detector/front_0/armor_sets` | `aim_armor_detector` | `aim_solver(front_0)` | 前相机 0 的 2D 装甲板检测结果 |
| `/aim_detector/front_1/armor_sets` | `aim_armor_detector` | `aim_solver(front_1)` | 前相机 1 的 2D 装甲板检测结果 |
| `/aim_detector/back/armor_sets` | `aim_armor_detector` | `aim_solver(back)` | 后相机的 2D 装甲板检测结果 |

### 2.4 solver 输出

| Topic | 来源 | 去向 | 作用 |
|---|---|---|---|
| `/aim_solver/front_0/armor_pose_sets` | `aim_solver` | `aim_predictor(front_0)`、`aim_outpost_predictor` | 前相机 0 的 3D 装甲板位姿 |
| `/aim_solver/front_1/armor_pose_sets` | `aim_solver` | `aim_predictor(front_1)` | 前相机 1 的 3D 装甲板位姿 |
| `/aim_solver/back/armor_pose_sets` | `aim_solver` | `aim_predictor(back)` | 后相机的 3D 装甲板位姿 |

### 2.5 predictor 输出

| Topic | 来源 | 去向 | 作用 |
|---|---|---|---|
| `/aim_predictor/front_0/target_states` | `aim_predictor` | `aim_armor_decider`、`aim_armor_controller` | 前相机 0 的整车状态估计 |
| `/aim_predictor/front_1/target_states` | `aim_predictor` | `aim_armor_decider`、`aim_armor_controller` | 前相机 1 的整车状态估计 |
| `/aim_predictor/back/target_states` | `aim_predictor` | `aim_armor_decider`、`aim_armor_controller` | 后相机的整车状态估计 |

### 2.6 outpost predictor 输出

| Topic | 来源 | 去向 | 作用 |
|---|---|---|---|
| `/aim_outpost_predictor/outpost_state` | `aim_outpost_predictor` | `aim_armor_decider`、`aim_armor_controller` | 前哨站目标状态 |

### 2.7 decider 输出

| Topic | 来源 | 去向 | 作用 |
|---|---|---|---|
| `/ly/aim/armor_targets` | `aim_armor_decider` | 外部系统 | 发布当前可打目标列表 |
| `/decider/selected_target_id` | `aim_armor_decider` | `aim_armor_controller` | 发布当前最终选中的目标 id |

### 2.8 controller 输出

| Topic | 来源 | 去向 | 作用 |
|---|---|---|---|
| `/ly/aim/result` | `aim_armor_controller` | 外部系统 | 最终瞄准结果与开火结果 |
| `/ly/control/angles` | `aim_armor_controller` | 外部控制接口 | 输出控制角度 |
| `/ly/control/firecode` | `aim_armor_controller` | 外部控制接口 | 输出开火码 |
| `/controller/selected_armor` | `aim_armor_controller` | 调试/可视化 | 当前实际选中的装甲板 |
| `/armor_controller/debug/target_point_barrel` | `aim_armor_controller` | 调试 | 枪管系目标点 |
| `/armor_controller/debug/selected_armor_index` | `aim_armor_controller` | 调试 | 选中了哪块板 |

---

## 3. 各包分别做了什么

## 3.1 `sentry_tf`

路径：
[tf_node.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/sentry_tf/src/tf_node.cpp)

### 输入

- `/ly/gimbal/angles`
- `/ly/gimbal/big_yaw_angles`
- `sentry_tf.yaml` 中的静态外参参数

### 处理

它不处理图像，不处理目标。

它只做一件事：**把外部云台角度和静态外参转换成 TF 树**。

主要发布两类 TF：

1. 动态 TF  
   由小 yaw、pitch、大 yaw 实时更新

2. 静态 TF  
   例如 `yaw_frame -> barrel_joint_frame` 的固定偏移  
   以及 launch 里额外发布的相机外参静态 TF

### 输出

- TF 树本身

### 它对后续的意义

`aim_solver`
`aim_predictor`
`aim_outpost_predictor`
`aim_armor_controller`

都依赖这棵 TF 树做坐标变换。

---

## 3.2 `aim_camera_driver`

路径：
[gx_camera_component.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_camera_driver/src/gx_camera_component.cpp)
[usb_camera_component.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_camera_driver/src/usb_camera_component.cpp)

### 输入

- 相机设备本身
- yaml 中的成像参数
- yaml 中的内参参数

### 处理

这个包负责：

1. 打开相机
2. 读取图像
3. 组装 ROS `Image`
4. 组装 ROS `CameraInfo`

前相机使用大恒工业相机组件。  
后相机使用 USB/V4L2 相机组件。

### 输出

- `image_raw`
- `camera_info`

### 处理结果的意义

`Image` 给 detector。  
`CameraInfo` 或内参给 solver。

---

## 3.3 `aim_armor_detector`

路径：
[aim_armor_detector_node.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_armor_detector/src/aim_armor_detector_node.cpp)

### 输入

- 相机原图 `sensor_msgs/Image`
- 模型路径
- 敌方颜色
- PCA 纠正参数

### 处理

每路 detector 的流程是：

1. 接收图像
2. 放入内部最新帧缓存
3. 工作线程取最新帧
4. OpenVINO 异步推理
5. 过滤并整理出装甲板结果

它的输出仍然是 **二维检测结果**，也就是：

- 每块装甲板的四个角点
- 该板的类别

此时还没有三维位姿。

### 输出

- `aim_msgs/ArmorSetArray`

### 对后续的意义

solver 会把这里的 2D 角点，结合内参和 TF，解成 3D 位姿。

---

## 3.4 `aim_solver`

路径：
[aim_solver_node.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_solver/src/aim_solver_node.cpp)

### 输入

- 三路 `ArmorSetArray`
- 三路内参
- TF

### 处理

这个包做的是 **PnP 位姿解算**。

具体流程：

1. 读取 detector 输出的每块装甲板四角点
2. 根据装甲板类型确定 3D 模型尺寸
3. 用相机内参做 `solvePnP`
4. 得到该装甲板在当前坐标系下的 3D `Pose`
5. 需要时再把姿态变换到 `target_frame`
6. 可选做 yaw 优化

普通装甲板输出到三路 `ArmorPoseSetArray`。

前哨站装甲板还会额外从 `Pose` 反算四个 3D 角点，组装成 `ArmorArray` 供前哨站预测器使用。

### 输出

- `/aim_solver/front_0/armor_pose_sets`
- `/aim_solver/front_1/armor_pose_sets`
- `/aim_solver/back/armor_pose_sets`

### 对后续的意义

predictor 不再看 2D 图像角点，而是看 solver 产出的 3D 装甲板位姿。

---

## 3.5 `aim_predictor`

路径：
[aim_predictor_node.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_predictor/src/aim_predictor_node.cpp)

### 输入

- 三路 `ArmorPoseSetArray`
- TF
- 各类 EKF 参数

### 处理

这个包做的是 **普通车辆目标跟踪与状态估计**。

它不是逐板输出，而是把同一辆车的装甲板观测融合成一个整车状态：

- 车中心位置
- 车中心速度
- yaw
- 角速度
- 半径
- 偏移量
- 预测出的各块装甲板位姿

它内部对不同角速度段使用不同过程噪声配置。

### 输出

- 三路 `TargetStateArray`

### 对后续的意义

decider 和 controller 都是基于这里的“整车状态”做后续处理，而不是再直接使用 detector 或 solver 原始输出。

---

## 3.6 `aim_outpost_predictor`

路径：
[aim_outpost_predictor_node.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_outpost_predictor/src/aim_outpost_predictor_node.cpp)

### 输入

- `/aim_solver/front_0/armor_pose_sets`
- TF
- 前哨站匹配与 EKF 参数

### 处理

这个包专门处理 **前哨站目标**，和普通车辆 predictor 分开。

它会：

1. 从 solver 输出里筛出 `id == outpost_id` 的装甲板
2. 把装甲板位姿转成内部观测量
3. 预测前哨站状态
4. 做三槽位匹配
5. 更新 EKF
6. 输出前哨站中心、角速度、预测装甲板、主打板等状态

### 输出

- `aim_msgs/OutpostState`

### 对后续的意义

decider 可以把前哨站作为一个特殊目标处理。  
controller 可以按前哨站专用逻辑选板、开火。

---

## 3.7 `aim_armor_decider`

路径：
[armor_decider_node.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_armor_decider/src/armor_decider_node.cpp)

### 输入

- 三路 `TargetStateArray`
- `OutpostState`
- 外部 `/ly/aim/select_target`

### 处理

这个包不算弹道，也不算姿态。  
它只做 **目标层面的决策汇总**。

它的处理逻辑大致是：

1. 接收三路普通目标状态
2. 接收前哨站状态
3. 接收外部指定目标 id
4. 前双相机候选优先合并
5. 没有前向候选时再退到后相机
6. 若外部指定 id 有效，则优先跟该目标
7. 把当前候选目标列表发布出去
8. 把当前最终目标 id 发布给 controller

### 输出

- `sentry_msgs/AimTargetArray`
- `aim_msgs/SelectedTargetId`

### 对后续的意义

controller 通过它知道“最终该打哪辆车/前哨站”。

---

## 3.8 `aim_armor_controller`

路径：
[armor_controller_node.cpp](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_armor_controller/src/armor_controller_node.cpp)

### 输入

- 三路 `TargetStateArray`
- `OutpostState`
- `SelectedTargetId`
- `/ly/gimbal/angles`
- `/ly/bullet/speed`
- TF

### 处理

这个包做的是 **最终控制解算**。

它需要完成：

1. 从三路 `TargetState` 和 `OutpostState` 里找到当前活动目标
2. 根据 `SelectedTargetId` 确认当前优先打谁
3. 对普通车辆做智能选板
4. 对前哨站做前哨站专用选板
5. 把目标点变换到枪管系
6. 结合弹速解弹道
7. 判断是否满足开火条件
8. 发布最终控制量和开火结果

### 输出

- `sentry_msgs/AimResult`
- `/ly/control/angles`
- `/ly/control/firecode`
- 调试 topic

### 它是整条链路的最后一层

前面所有包都在“整理信息”；  
controller 才真正把这些信息变成“怎么转、能不能打、什么时候开火”。

---

## 4. 三条主链路分别怎么走

## 4.1 普通车辆主链路

```text
image_raw
-> ArmorSetArray
-> ArmorPoseSetArray
-> TargetStateArray
-> decider 选目标
-> controller 选板 + 弹道
-> AimResult / control angles / firecode
```

## 4.2 前哨站主链路

```text
front_0 armor_pose_sets
-> aim_outpost_predictor
-> OutpostState
-> decider / controller
-> 最终控制输出
```

## 4.3 外部指定目标链路

```text
/ly/aim/select_target
-> aim_armor_decider
-> /decider/selected_target_id
-> aim_armor_controller
```

---

## 5. 消息说明

下面按消息逐个说明字段含义。

## 5.1 `aim_msgs`

### `ArmorClass.msg`

路径：
[ArmorClass.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ArmorClass.msg)

字段：

- `class_id`：装甲板类别 id
- `team`：队伍颜色/队伍标记

---

### `Armor.msg`

路径：
[Armor.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/Armor.msg)

字段：

- `header`：时间戳、坐标系
- `corners[4]`：装甲板四个角点
- `armor_class`：装甲板类别信息

含义：

这是最基础的一块装甲板数据结构。  
在 detector 阶段，`corners` 一般表示图像平面角点；  
在某些 3D 支路里，也可能表示世界系/目标系下的角点。

---

### `ArmorArray.msg`

路径：
[ArmorArray.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ArmorArray.msg)

字段：

- `header`
- `armors[]`

含义：

一组 `Armor` 的集合。

---

### `ArmorSet.msg`

路径：
[ArmorSet.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ArmorSet.msg)

字段：

- `header`
- `armors[]`
- `id`

含义：

表示同一个目标 id 下的一组装甲板。

---

### `ArmorSetArray.msg`

路径：
[ArmorSetArray.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ArmorSetArray.msg)

字段：

- `header`
- `armor_sets[]`

含义：

detector 的主输出。  
表示当前一帧里检测到的所有目标，每个目标下面挂一组装甲板。

---

### `ArmorPoseSet.msg`

路径：
[ArmorPoseSet.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ArmorPoseSet.msg)

字段：

- `header`
- `armor_poses[]`
- `id`

含义：

同一个目标 id 下，一组装甲板的三维位姿。

---

### `ArmorPoseSetArray.msg`

路径：
[ArmorPoseSetArray.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ArmorPoseSetArray.msg)

字段：

- `header`
- `armor_pose_sets[]`

含义：

solver 的主输出。  
它把 detector 的 2D 角点，变成了一帧中所有目标的 3D 装甲板位姿集合。

---

### `ArmorSetWithState.msg`

路径：
[ArmorSetWithState.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ArmorSetWithState.msg)

字段：

- `header`
- `armors[]`
- `id`
- `tracking`
- `converged`
- `jumped`
- `yaw`
- `angular_velocity`
- `radius`
- `radius_offset`
- `height_offset`
- `velocity_x/y/z`
- `center_x/y/z`

含义：

这是“某一目标的一组装甲板 + 该目标的预测状态”。

当前 `new` 主链路里普通 controller 不再直接用它作为输入，但这个消息本身保留了完整的“板集合 + 车体状态”结构。

---

### `ArmorSetArrayWithState.msg`

路径：
[ArmorSetArrayWithState.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ArmorSetArrayWithState.msg)

字段：

- `header`
- `armor_sets[]`

含义：

多个 `ArmorSetWithState` 的集合。

---

### `TargetState.msg`

路径：
[TargetState.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/TargetState.msg)

字段：

- `header`
- `id`
- `tracking`
- `converged`
- `jumped`
- `center`
- `velocity`
- `yaw`
- `angular_velocity`
- `radius`
- `radius_offset`
- `height_offset`
- `predicted_armors[]`

含义：

这是普通车辆 predictor 的主状态消息。  
它描述的是“这辆车的整体状态”，而不是单块板。

---

### `TargetStateArray.msg`

路径：
[TargetStateArray.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/TargetStateArray.msg)

字段：

- `header`
- `targets[]`

含义：

predictor 的主输出。  
一帧里可能有多辆车，每辆车一个 `TargetState`。

---

### `OutpostState.msg`

路径：
[OutpostState.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/OutpostState.msg)

字段：

- `header`
- `id`
- `tracking`
- `converged`
- `jumped`
- `center`
- `velocity`
- `yaw`
- `angular_velocity`
- `radius`
- `low_height_offset`
- `high_height_offset`
- `has_primary_armor`
- `primary_slot`
- `primary_armor`
- `predicted_armors[]`

含义：

这是前哨站专用状态消息。  
它和普通 `TargetState` 的区别在于：

- 前哨站是 3 板模型
- 包含高低板偏移
- 包含当前主打板 `primary_armor`

---

### `OutpostPredictorState.msg`

路径：
[OutpostPredictorState.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/OutpostPredictorState.msg)

字段：

- `header`
- `initialized`
- `x`
- `y`
- `z1`
- `z2`
- `z3`
- `theta`
- `omega`

含义：

这是旧式/中间态的前哨站预测状态表达。  
当前 `new` 的主外部接口主要使用 `OutpostState`，不是这个。

---

### `SelectedTargetId.msg`

路径：
[SelectedTargetId.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/SelectedTargetId.msg)

字段：

- `header`
- `id`
- `valid`

含义：

decider 输出的“当前最终锁定目标 id”。

---

### `ControlAngles.msg`

路径：
[ControlAngles.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/ControlAngles.msg)

字段：

- `header`
- `yaw`
- `pitch`

含义：

控制角度消息。当前 `new` 主外部控制更多走 `/ly/control/angles`，但这个消息类型仍然保留。

---

### `GimbalState.msg`

路径：
[GimbalState.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_msgs/msg/GimbalState.msg)

字段：

- `header`
- `yaw`
- `pitch`
- `bullet_speed`
- `aim_request`

含义：

云台状态消息类型。当前 `new` 的主外部输入更多直接使用 `gimbal_driver/msg/GimbalAngles` 和 `/ly/bullet/speed`。

---

## 5.2 `sentry_msgs`

### `AimTarget.msg`

路径：
[AimTarget.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/sentry_msgs/msg/AimTarget.msg)

字段：

- `header`
- `position`
- `id`

含义：

一个可攻击目标的简化表达：位置 + id。  
decider 收外部指定目标时用这个格式；  
发布候选目标列表时也会用这个格式。

---

### `AimTargetArray.msg`

路径：
[AimTargetArray.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/sentry_msgs/msg/AimTargetArray.msg)

字段：

- `header`
- `aim_targets[]`

含义：

decider 输出的“当前候选目标列表”。

---

### `AimResult.msg`

路径：
[AimResult.msg](E:/桌面/sentry.aim/sentry.aim_new/src1/sentry_msgs/msg/AimResult.msg)

字段：

- `header`
- `fire`
- `pitch`
- `yaw`

含义：

controller 最终输出的瞄准结果。  
它表示：

- 这一拍是否允许开火
- 应该输出的 pitch / yaw 是多少

---

## 6. 你在调试时最该盯的几条 topic

如果要快速排链路，优先看下面这些：

### 相机层

- `/gx_camera_0/image_raw`
- `/gx_camera_1/image_raw`
- `/usb_camera/image_raw`

### detector 层

- `/aim_detector/front_0/armor_sets`
- `/aim_detector/front_1/armor_sets`
- `/aim_detector/back/armor_sets`

### solver 层

- `/aim_solver/front_0/armor_pose_sets`
- `/aim_solver/front_1/armor_pose_sets`
- `/aim_solver/back/armor_pose_sets`

### predictor 层

- `/aim_predictor/front_0/target_states`
- `/aim_predictor/front_1/target_states`
- `/aim_predictor/back/target_states`

### 前哨站

- `/aim_outpost_predictor/outpost_state`

### 决策与控制

- `/ly/aim/select_target`
- `/decider/selected_target_id`
- `/ly/aim/armor_targets`
- `/ly/aim/result`

---

## 7. 最后一句话概括

这套系统本质上是：

- `camera_driver` 负责把图像取出来
- `armor_detector` 负责找到 2D 装甲板
- `solver` 负责把 2D 解成 3D
- `predictor` / `outpost_predictor` 负责把单帧观测变成连续目标状态
- `decider` 负责决定打谁
- `controller` 负责决定怎么打，并给出最终控制输出
