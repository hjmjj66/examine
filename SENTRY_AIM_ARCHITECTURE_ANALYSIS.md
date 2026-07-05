# RoboMaster 哨兵机器人自瞄框架 — 完整架构分析

---

## 1. 整体架构概览

### 1.1 包的依赖关系图

```
                         aim_msgs (消息基础设施)
                         /    |    |      \
            aim_armor_     aim_  |    |     aim_armor_
            detector       solver|    |     decider + controller
             (2D检测)    (PnP解算)|    |       (决策+控制)
                  \          /   |    |
                   \        /    |    |
                aim_predictor   aim_outpost_predictor
                (普通车跟踪)     (前哨站跟踪)
```

### 1.2 完整处理管线（8 级流水线）

```
/ly/gimbal/angles  +  /ly/gimbal/big_yaw_angles
            |
            v
    [1] sentry_tf — 构建动态+静态 TF 树
            |
            v
    [2] aim_camera_driver — 三路相机采集 (GX_0, GX_1, USB)
            |  image_raw + camera_info
            v
    [3] aim_armor_detector — 三路 OpenVINO 推理 → 2D 装甲板角点
            |  ArmorSetArray (含 corners[4], class_id, team)
            v
    [4] aim_solver — 三路 PnP 解算 → 3D 位姿 + Yaw 优化
            |  ArmorPoseSetArray (含 Pose position+orientation, id)
            |
            +------------------------------> [5b] aim_outpost_predictor
            |                                 前哨站专用 EKF (id=6)
            |                                   → OutpostState
            v
    [5a] aim_predictor — 三路普通车 EKF 跟踪
            |  TargetStateArray (center, velocity, yaw, angular_vel, radius, predicted_armors)
            |
            +--------------------+
            |                    |
            v                    v
    [6] aim_armor_decider     [7] aim_armor_controller
        → SelectedTargetId     |← 弹道解算 + 选板 + 开火判定
        → AimTargetArray        |
                                v
                         /ly/aim/result        (fire, pitch_deg, yaw_deg)
                         /ly/control/angles    (GimbalAngles)
                         /ly/control/firecode  (UInt8)
```

### 1.3 三条独立子链路

| 链路 | 路径 | 特点 |
|------|------|------|
| **普通车辆** | image → ArmorSetArray → ArmorPoseSetArray → TargetStateArray → decider → controller | 4 板模型，EKF 11 维状态 |
| **前哨站** | front_0 ArmorPoseSet → OutpostState → decider → controller | 3 板模型，独立 EKF，有主打板概念 |
| **外部指定** | /ly/aim/select_target → decider → SelectedTargetId → controller | 通过 AimTarget 消息指定目标 ID，decider 优先跟随 |

---

## 2. 完整自瞄流程：从图像到控制指令

### 2.1 端到端数据流（以一发子弹为例）

```
时间 t0: 相机曝光
    ↓ (0~2ms 传输延迟)
时间 t1: detector 收到 image_raw (1280×1024 bgr8)
    ↓ (Pipeline 队列 + OpenVINO 异步推理 ~3-8ms)
时间 t2: detector 输出 ArmorSetArray
    └── 例: 检测到 id=1 两轮英雄车，含 2 块装甲板
    └── 每板: corners[4] 四角点 (像素坐标) + class_id + team
    ↓ (topic 传输)
时间 t3: solver 收到 ArmorSetArray
    ↓ (PnP + TF变换 + Yaw优化 ~0.5ms)
时间 t4: solver 输出 ArmorPoseSetArray
    └── 每板: Pose (x,y,z 世界坐标/m + orientation 四元数)
    ↓ (分流)
    ├→ outpost_predictor (仅 id=6)
    └→ predictor (id≠6)
    ↓ (EKF predict+update)
时间 t5: predictor 输出 TargetStateArray
    └── 每车: center(x,y,z), velocity(vx,vy,vz), yaw, angular_vel, radius, predicted_armors[4]
    ↓ (多路汇聚)
时间 t6: decider 发布 SelectedTargetId + AimTargetArray
    └── 选最近/外部指定的目标
    ↓
时间 t7: controller 定时器触发 (100Hz)
    ├── resolveActiveTarget(): 从三路 TargetState + OutpostState 中匹配 decider 选定的 id
    ├── selectArmorCandidate(): 选板
    │   ├── 运动预测: predict_time = fly_time + measurement_age + system_delay
    │   ├── predictLegacyTarget(): center += vel*dt, yaw += ang_vel*dt
    │   ├── chooseLegacyAimPoint(): 根据转速选择瞄准策略 (打脸/旋中/低切)
    │   ├── 迭代收敛: fly_time 收敛 (<= 1ms 阈值)
    │   └── 输出: 最终瞄准点的世界坐标 point_world
    ├── transformWorldToBarrel(): 世界系 → 枪管系
    ├── calcPitchYaw(): 牛顿迭代弹道解算
    │   └── 考虑空气阻力 (Cd=0.42, ρ=1.169) + 重力 (9.794)
    │   └── 输出: pitch(rad), yaw(rad), fly_time(sec)
    ├── evaluateLegacyFireControl(): 开火判定
    │   └── yaw_error < tolerance_deg && pitch_error < tolerance_deg
    │   └── 旋中模式额外检查: 装甲板面朝窗口
    └── 发布:
        ├── AimResult: {fire: bool, pitch: 15.3°, yaw: 42.7°}
        ├── GimbalAngles: {yaw: 42.7, pitch: 15.3}
        └── UInt8 firecode: 99 (开火)
```

---

## 3. 各包详细分析

### 3.1 sentry_tf — TF 树发布

**职责**：将云台角度和静态外参转换为 ROS 2 TF 树。

**输入**：
- `/ly/gimbal/angles` (GimbalAngles: yaw°, pitch°) — 小云台实时角度
- `/ly/gimbal/big_yaw_angles` (Float32) — 大云台实时角度
- `sentry_tf.yaml` 中的静态外参参数

**内部处理**：
- 将角度 (度) 转为 TF 变换，发布动态 TF：`base_link → gimbal_big_yaw → gimbal_small_yaw → gimbal_world / gimbal_barrel_joint`
- 发布静态 TF：`gimbal_barrel_joint → gimbal_barrel → camera_link`
- 使所有后续模块可以通过统一的 TF 树做坐标变换

**输出**：
- TF 树广播 (tf2_msgs/TFMessage)

---

### 3.2 aim_camera_driver — 相机驱动

**职责**：从物理相机采集图像并发布 ROS 消息。

**输入**：
- 大恒工业相机 (GX) 设备 (前两路，1280×1024 @ ~100Hz)
- USB/V4L2 相机 (后一路，1280×720 @ 30Hz)
- YAML 中的内参 (camera_matrix 3×3 + distortion_coefficients)

**内部处理**：
- 打开相机设备，配置曝光/增益/白平衡
- 循环读取图像帧 → 填充 sensor_msgs/Image (bgr8)
- 组装 sensor_msgs/CameraInfo (含内参矩阵 K 和畸变系数 D)

**输出**：
- `/gx_camera_0/image_raw`, `/gx_camera_0/camera_info`
- `/gx_camera_1/image_raw`, `/gx_camera_1/camera_info`
- `/usb_camera/image_raw`, `/usb_camera/camera_info`

**关键实现细节**：
- 前相机 detector 使用 `ComposableNodeContainer` 将 CameraComponent 和 Detector 放在同一进程
- `use_intra_process_comms` 实现零拷贝指针传递，避免 DDS 序列化

---

### 3.3 aim_armor_detector — 装甲板 2D 检测

**职责**：用 OpenVINO 推理模型从图像中检测装甲板 2D 角点。

**输入**：
- `sensor_msgs/Image` (bgr8, 1280×1024)
- ONNX 模型权重 (assets/0526.onnx)
- 敌方颜色参数 (enemy_color=-2)

**内部处理**：

```
1. onImage(): 接收图像，放入 latest_msg 缓存，通知工作线程
2. processLoop(): 主循环
   a. takeLatestFrame(): 取最新帧 (跳过积压的旧帧)
   b. 流水线恢复结果 (如果当前 slot 有 in-flight 推理)
   c. submitFrame(): cv_bridge 解码 → OpenVINO startAsync (异步推理)
3. 推理完成后 getResult(): 
   a. 解析网络输出 → 每个装甲板的 4 角点 (8 个 landmarks) + 类别标签 + 颜色
   b. PCA 灯条纠正 (LightbarPcaCorrector): 优化灯条顶点精度
      - 在灯条两侧 padding 区域内沿轴线搜索
      - 用像素亮度加权 PCA 找主轴方向
      - 纠正因倾斜/透视导致的角点偏移
   c. 按 label 分组 (同一 label = 同一车辆)
4. publishDetections(): 组装 ArmorSetArray 发布
```

**输出**：
- `aim_msgs/ArmorSetArray`
  - `header`: 原始图像时间戳 + frame_id
  - `armor_sets[]`: 按 class_id 分组
    - `id`: 车辆类别 (0=哨兵, 1=英雄, 2=工程, 3/4=步兵, 6=前哨站, 7=基地)
    - `armors[]`: 该车的所有装甲板
      - `corners[4]`: 四角点像素坐标 (x, y) — 排列: [左上, 左下, 右下, 右上]
      - `armor_class.class_id`: 车辆类别
      - `armor_class.team`: 0=蓝方, 1=红方

**关键参数**：
- model_path: OpenVINO 模型路径
- enemy_color: -2 (双方都打)
- PCA: padding_scale=0.07, search_start_ratio=0.4, search_end_ratio=0.6

---

### 3.4 aim_solver — PnP 3D 位姿解算

**职责**：将 detector 的 2D 角点解算为 3D 世界坐标位姿。

**输入**：
- `ArmorSetArray` (三路)
- 相机内参 (从 yaml 参数或 camera_info topic)
- TF 树 (camera_link → gimbal_world 变换)

**装甲板 3D 模型尺寸 (常量)**：

| 装甲板类型 | 宽度 (m) | 灯条长度 (m) | 3D 模型点 |
|-----------|---------|-------------|----------|
| 大板 (英雄) | 0.230 | 0.056 | (±0.115, ±0.028) Z轴 |
| 小板 (其他) | 0.135 | 0.056 | (±0.0675, ±0.028) Z轴 |

**内部处理详细流程**：

```
onArmorSets(pipeline, msg):

1. 相机内参检查
   └── 确保 camera_matrix(3×3) 和 distortion_coefficients 可用

2. TF 查询
   └── camera_frame → target_frame (gimbal_world)
   └── lookup_timeout = 0.05s

3. 遍历每块装甲板:
   a. buildObservation(): 
      - 从 corners[4] 提取左/右灯条的 top/bottom 端点
      - 计算每个灯条的单位轴向量 axis = (top-bottom)/norm
      - 灯条长度归一化验证 (norm > 1e-6)

   b. solvePnP():
      - 图像点 (4 点): [左下, 左上, 右上, 右下] (对应 3D 模型点)
      - 3D 模型点: [(-w/2, -l/2), (-w/2, l/2), (w/2, l/2), (w/2, -l/2)]
      - 算法: cv::SOLVEPNP_IPPE (单解时) 或 cv::solvePnPGeneric (双解时)
      - 输出: rvec (罗德里格斯向量) + tvec (平移向量)

   c. sortPnPResult() (通用模式):
      - IPPE 可能返回 2 个解 (装甲板正面/背面歧义)
      - 按重投影误差 ratio 排序 (threshold=3.0)
      - Roll 角检查 (threshold=15°)
      - 根据灯条角度判断正确解: 
        如果 armor_angle > 0° 且 solution0.yaw > 0 且 solution1.yaw < 0 → swap

   d. Rodrigues → 旋转矩阵 → Quaternion

   e. TF 变换: camera 系 Pose → gimbal_world 系 Pose

   f. optimizeYaw() (启用时):
      - 当前 yaw 附近 ±70° 范围内，以 1° 为步长搜索
      - 对每个 yaw 重新投影 4 个角点，计算与原始检测角点的距离
      - 选择重投影误差最小的 yaw
      - 固定 pitch: 普通板 15°, 前哨站 -15° (基于经验的前置角度)
      - 固定 roll: 0°
      - 重新组装旋转矩阵 → Quaternion
```

**输出**：
- `aim_msgs/ArmorPoseSetArray`
  - `header`: 时间戳 + frame_id (gimbal_world)
  - `armor_pose_sets[]`:
    - `id`: 车辆类别
    - `armor_poses[]`: 每块装甲板的 Pose
      - `position`: (x, y, z) 世界坐标 (米)
      - `orientation`: 四元数 (w, x, y, z) — 装甲板法向

**数值范围示例**：
- 典型前哨站距离: (3~8m, 0~2m, -0.5~0.5m)
- 典型车辆距离: (1~10m, -5~5m, -0.3~0.5m)
- 位置精度: ~1-5cm (近距离) / ~5-15cm (远距离)

---

### 3.5 aim_predictor — 普通车辆 EKF 目标跟踪

**职责**：将单帧装甲板观测融合为连续的整车状态估计。

**输入**：
- `ArmorPoseSetArray` (三路，过滤掉 id=6 的前哨站)
- TF (查云台 yaw 用)
- EKF 参数 (过程噪声三段配置)

**状态向量 (11维)**：
```
x = [cx, vx, cy, vy, cz, vz, yaw, ω, radius, radius_offset, height_offset]
     ──  c = 车辆中心位置 (世界坐标)
          v = 车辆中心速度
               yaw = 车辆朝向 (0~2π, 0 号板朝向)
                    ω = 角速度 (rad/s)
                         radius = 车辆中心到 0 号板距离
                                 radius_offset = 高低板半径差
                                         height_offset = 高低板高度差
```

**装甲板模型 (4板)**：
```
板 0 (正对): angle = yaw + 0*π/2, r = radius,          z = cz
板 1 (右侧): angle = yaw + 1*π/2, r = radius+radius_offset, z = cz+height_offset
板 2 (后侧): angle = yaw + 2*π/2, r = radius,          z = cz
板 3 (左侧): angle = yaw + 3*π/2, r = radius+radius_offset, z = cz+height_offset
```

**观测量 (4维/每板)**：
```
z = [yaw, pitch, distance, armor_yaw]   (球坐标 + 板朝向)
```

**内部处理详细流程**：

```
onArmorPoseSets(pipeline, msg):

1. 预处理
   a. 从 ArmorPoseSetArray 提取所有观测
      - 过滤 outpost_id
      - xyz = (position.x, position.y, position.z)
      - ypr = poseToYpr(orientation)    // 四元数 → yaw/pitch/roll
      - ypd = xyzToYpd(xyz)             // 笛卡尔 → 球坐标 (yaw, pitch, distance)
   b. 查云台当前 yaw

2. 速度自适应过程噪声选择
   └── |ω| < middle_threshold (2.0 rad/s):  低噪声 xy=1600, z=1600, yaw=400
   └── middle_threshold ≤ |ω| < high_threshold (6.0):  中噪声 (同值)
   └── |ω| ≥ high_threshold:  高噪声 (同值)

3. 所有 tracker 先 predict
   ├── f = I + dt 项: f(0,1)=dt, f(2,3)=dt, f(4,5)=dt, f(6,7)=dt
   ├── Q 矩阵: 离散白噪声模型 (a*σ², b*σ², b*σ², c*σ²)
   │   a = dt⁴/4, b = dt³/2, c = dt²
   └── 转移: x_new = F·x, yaw 归一化

4. 观测关联与选择
   a. 多板门限: 如果同一 ID 有 >1 板, 用 yaw_error < multi_armor_yaw_gate(60°)
   b. 按 yaw_error 排序, 选最近云台的那块做主更新

5. 初始化 / 更新
   a. 初始化 (needs_initialize):
      - 由 0 号板装甲位置反算车辆中心:
        cx = armor_x + radius * cos(armor_yaw)
        cy = armor_y + radius * sin(armor_yaw)
        cz = armor_z
      - P0 = diag[1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1]
      - 先用首板 initialize, 再用其他板 update
   b. 更新 (已跟踪):
      - matchArmorIndex(): 匹配当前观测对应哪个板号
      - observationJacobian(): 计算雅可比矩阵 (链式: dx_dstate × dypd_dx)
      - R 矩阵 (观测噪声协方差):
        r_yaw = 4e-3
        r_pitch = 4e-3
        r_dist = log(|delta_angle|+1) + 0.1
        r_armor_yaw = log(|armor_ypd.z|+1)/200 + 0.09
      - EKF update()

6. 发散检测
   └── radius ∈ (0.05, 0.5) m
   └── radius + radius_offset ∈ (0.05, 0.5) m

7. 输出 TargetStateArray
   └── 每辆车: center, velocity, yaw, angular_velocity, radius, predicted_armors[]
   └── predicted_armors: 4 块板的 3D Pose (由 EKF 状态反算)
```

**输出**：
- `aim_msgs/TargetStateArray`
  - `targets[]`:
    - `id`: uint8 车辆类别
    - `tracking`, `converged`, `jumped`
    - `center`: Point (x, y, z) 世界坐标
    - `velocity`: Vector3 (vx, vy, vz) m/s
    - `yaw`: float64 rad (0号板朝向)
    - `angular_velocity`: float64 rad/s
    - `radius`: float64 m
    - `radius_offset`, `height_offset`: float64 m
    - `predicted_armors[]`: Pose[] (4块板的预测3D位姿)

**数值范围示例**：
- 车辆速度: vx/vy 通常在 -3~3 m/s, vz 极小 (±0.2)
- 角速度: -10~10 rad/s (包含陀螺旋转)
- 半径: 约 0.2~0.35 m (英雄~0.3, 步兵~0.25)
- 收敛需要: update_count > 3

---

### 3.6 aim_outpost_predictor — 前哨站专用跟踪器

**职责**：专门处理前哨站三装甲板目标的 EKF 跟踪，含槽位匹配逻辑。

**输入**：
- `/aim_solver/front_0/armor_pose_sets` (仅 id=6)
- TF (查云台 yaw)

**状态向量 (11维)**：
```
x = [cx, vx, cy, vy, cz, vz, yaw, ω, radius, low_offset, high_offset]
     ──                                          ────    ─────    ──────
                                                  低板z偏移   高板z偏移
```

**装甲板模型 (3板, 120°间隔)**：
```
槽位 0 (低板):  angle = yaw + 0*2π/3, z = cz + low_offset   (low_offset < 0)
槽位 1 (中板):  angle = yaw + 1*2π/3, z = cz
槽位 2 (高板):  angle = yaw + 2*2π/3, z = cz + high_offset  (high_offset > 0)
```

**内部处理独特之处**：

```
1. 多槽位联合匹配
   a. 收集所有 id=6 的装甲板观测 (最多 3 个，按 distance 排序)
   b. predictedArmorStates() → 3 个槽位的预测状态 (x,y,z,yaw)
   c. findBestAssignment(): DFS 搜索最优观测-槽位分配
      - outpostMatchCost(): 5 维匹配代价
        cost = yaw_error/σ_yaw + pitch_error/σ_pitch + distance_error/σ_dist
               + angle_error/σ_angle + z_error/σ_z
      - 门限: match_yaw_gate, match_pitch_gate, match_distance_gate, match_angle_gate, match_z_gate
      - 旋转方向惩罚: 如果反向 π/3 跳转, cost += reverse_direction_penalty
      - DFS 枚举所有分配, 选总代价最小的

2. 主打板 (primary_armor) 选择
   └── choosePrimarySlot():
      - 首次 (无主打): 需要 ≥2 个观测才选, 选代价最低的槽位
      - 已有主打: 保守保持, 仅在主打丢失时按旋转方向跳到相邻槽位

3. 快重锚 (fast_reanchor)
   └── 若主打板连续丢失且匹配代价过高, 在第 fast_reanchor_frames 帧强制重锚到最低代价槽

4. 角速度钳位
   └── 收敛后钳位 |ω| ≤ angular_velocity_clamp (2.51 rad/s)

5. 联合 EKF update
   └── 多观测同时更新: z 维度 = obs_count × 4
   └── H 矩阵 = block_diag(H_slot0, H_slot1, ...)

6. NIS 检测
   └── 计算归一化创新平方 (Normalized Innovation Squared)
   └── NIS > χ²_{4,0.95} = 9.4877
   └── 滑动窗口失败率 > max_nis_failure_ratio → 重新初始化
```

**输出**：
- `aim_msgs/OutpostState`
  - `has_primary_armor`: 是否有主打板
  - `primary_slot`: 主打板槽位 (0/1/2)
  - `primary_armor`: Pose (主打板 3D 位姿)
  - `low_height_offset`, `high_height_offset`: 高低板偏移量
  - 其他字段同 TargetState

---

### 3.7 aim_armor_decider — 目标决策器

**职责**：从多路探测结果中选出最终攻击目标。

**输入**：
- 三路 `TargetStateArray`
- `OutpostState`
- 外部 `/ly/aim/select_target` (AimTarget)

**内部处理逻辑**：

```
1. 候选收集
   └── 前相机优先: 合并 front_0 + front_1 的 targets, 按 id 去重取最优
   └── 无前向候选时退到 back 相机

2. 外部指定优先
   └── 如果 select_target 发来的 id 在候选列表中 → 直接选该 id
   └── select_target_timeout = 0.5s, 超时后自动失效

3. 默认策略: 选最近的目标
   └── nearestCandidate(): distance² = x² + y² + z²

4. 前哨站特殊处理
   └── 如果 selected_target_id == outpost_id (6) → 不做普通候选收集, 直接跟踪前哨站

5. 发布
   └── AimTargetArray: 所有候选目标的位置+id
   └── SelectedTargetId: 最终选定的目标 id + valid 标志
```

**输出**：
- `sentry_msgs/AimTargetArray`: 候选目标列表
- `aim_msgs/SelectedTargetId`: {header, id: uint8, valid: bool}

---

### 3.8 aim_armor_controller — 弹道解算与控制输出

这是整个管线的最核心一环，详见第 4 节。

---

## 4. 核心处理链路：Solver → Controller 深度解析

### 4.1 数据流总体

```
solver (3D Pose)
  │  ArmorPoseSetArray: {id, Pose[]}  ← 世界坐标下的装甲板位姿
  │
  ├────────────────┬────────────────┐
  │                │                │
  v                v                v
predictor        outpost_         (分路处理)
  │  EKF 跟踪     predictor
  │  11维状态     │  3板EKF + 槽位匹配
  │               │
  v               v
TargetState      OutpostState
数组             单目标
  │   center(x,y,z)  │  center + low/high_offset
  │   velocity(vx,vy,vz)  │  + primary_slot
  │   yaw, ω, radius     │  + predicted_armors[3]
  │   predicted_armors[4] │
  │                      │
  └──────┬───────────────┘
         │
         v
     decider
         │ SelectedTargetId: {id=1, valid=true}
         v
     controller (100Hz 定时器)
         │
         ├── resolveActiveTarget(): 匹配 id, 构造 LegacyTargetModel
         │
         ├── selectArmorCandidate(): 选板 + 弹道 = 瞄准角度
         │   ├── 运动预测
         │   ├── 装甲板选择 (智能选板/旋中/前哨站专用)
         │   └── 牛顿迭代弹道解算
         │
         ├── evaluateLegacyFireControl(): 开火判定
         │
         └── 发布 AimResult + GimbalAngles + Firecode
```

### 4.2 第 1 环节 — Solver: PnP 解算 (2D→3D)

**输入数据**：
```
Armor: {
  corners[4]: [
    {x: 512.3, y: 401.2},  // 左上
    {x: 509.1, y: 445.8},  // 左下
    {x: 698.7, y: 443.5},  // 右下
    {x: 700.1, y: 399.3}   // 右上
  ],
  armor_class: {class_id: 1, team: 1}
}
```

**处理步骤**：

**(a) buildObservation — 提取观测结构**
```
左灯条: 
  top    = (512.3, 401.2)
  bottom = (509.1, 445.8)
  axis   = (3.2, -44.6) / 44.71 = (0.0716, -0.9974)

右灯条:
  top    = (700.1, 399.3)
  bottom = (698.7, 443.5)
  axis   = (1.4, -44.2) / 44.22 = (0.0317, -0.9995)
```

**(b) solvePnP — IPPE 算法**
```
3D 模型点 (大板, w=0.230, l=0.056):
  P0 = ( 0, -0.115,  0.028)  左下
  P1 = ( 0, -0.115, -0.028)  左上
  P2 = ( 0,  0.115, -0.028)  右上
  P3 = ( 0,  0.115,  0.028)  右下

2D 图像点 (重排):
  [左下, 左上, 右上, 右下]

solvePnP(3D_pts, 2D_pts, K, D, rvec, tvec, SOLVEPNP_IPPE)

结果:
  rvec = [-2.89, 0.15, 0.02]  (罗德里格斯, 近 -π 表示朝向相机)
  tvec = [2.35, 1.12, -0.08]  米 (相机坐标系)
```

**(c) Rodrigues → Rotation → Quaternion**
```
rvec [-2.89, 0.15, 0.02] 
  → Rodrigues → 3×3 旋转矩阵 R
  → RPY = rotationMatrixToRPY(R * R_camera_to_ros)
     roll  ≈ -0.05 rad (-2.9°)
     pitch ≈  0.08 rad (4.6°)
     yaw   ≈ -2.93 rad (-167.8°)，装甲板法向朝相机
  → rotationMatrixToQuaternion(R)
     q = (w=0.XXX, x=0.XXX, y=0.XXX, z=0.XXX)
```

**(d) TF 变换 — camera 系 → gimbal_world 系**
```
camera_pose = Pose(tvec, q)
  ↓ T_cam→world (从 TF 树查得)
world_pose = T_cam→world · camera_pose
  position: (2.80, 0.15, 0.42)  米 (gimbal_world 系)
```

**(e) optimizeYaw — 重投影优化 (核心!)**
```
当前 yaw ≈ -2.93 rad

以 ±70° 范围, 1° 步长遍历:
  for yaw in [-2.93-1.22, -2.93+1.22]:
    pitch = fixedPitchForArmorType(armor_type)
           = 15° * π/180  (普通板, 基于经验的前置俯仰)
           = 0.2618 rad
    roll = 0
    
    构造旋转矩阵 R(yaw, pitch, roll)
    用 R 和 tvec 将 4 个 3D 模型点反投影到图像
    ↓ cv::projectPoints()
    
    计算 4 个角点的重投影误差 (像素距离和)
    选最小误差的 yaw

最终:
  best_yaw = -2.90 rad (优化后)
  重投影误差 = 1.85 px (原来可能是 5.3 px)
  
  更新 Pose.orientation
```

**输出数据**：
```
ArmorPoseSet {
  id: 1,
  armor_poses: [
    Pose {
      position: {x: 2.80, y: 0.15, z: 0.42}  米
      orientation: {w: 0.120, x: 0.052, y: -0.003, z: 0.991}
      // 装甲板中心在 gimbal_world 系下的位置, 
      // 法向大致朝向哨兵相机
    }
  ]
}
```

### 4.3 第 2 环节 — Predictor: EKF 初始化与状态融合

**输入**: `ArmorPoseSet` → 提取 `ArmorMeasurement`

```
ArmorMeasurement {
  id: 1
  xyz: Eigen::Vector3d(2.80, 0.15, 0.42)   // position
  ypr: Eigen::Vector3d(-2.90, 0.08, -0.05)   // orientation → yaw/pitch/roll
  ypd: Eigen::Vector3d(0.0535, -0.148, 2.836) // xyz → (yaw, pitch, distance)
}
// ypd 计算: 
//   distance = sqrt(2.80² + 0.15² + 0.42²) = 2.836
//   yaw      = atan2(0.15, 2.80) = 0.0535 rad (3.07°)
//   pitch    = atan2(-0.42, sqrt(2.80²+0.15²)) = -0.148 rad (-8.5°)
```

**初始化 (首次看到这辆车)**：

```
从 0 号板观测反推车辆中心:
  armor_yaw = -2.90 rad  (板朝向)
  radius = 0.20 m        (初始估计)
  
  cx = 2.80 + 0.20 * cos(-2.90) = 2.80 + 0.20 * (-0.971) = 2.606
  cy = 0.15 + 0.20 * sin(-2.90) = 0.15 + 0.20 * (-0.239) = 0.102
  cz = 0.42

EKF 状态 x₀ = [2.606, 0, 0.102, 0, 0.42, 0, -2.90, 0, 0.20, 0, 0]
                                          ────         ──
                                          yaw         radius
初始协方差 P₀ = diag[1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1]
                   ──  ──     ──  ──     ──  ──   ──   ───
                   cm  高速     低速      高度 高   yaw  角速  radius
                   位置 不确定   不确定    不确定 不确定 不确定 不确定
```

**一次 predict-update 循环**：

```
t=0.01s 后 (100Hz 帧率):
  dt = 0.01s

predict():
  F = I + [dt in (0,1), (2,3), (4,5), (6,7)]
  → 位置 × 速度 × dt, yaw × ω × dt
  
  x_pred = [2.606, 0, 0.102, 0, 0.42, 0, -2.90, 0, 0.20, 0, 0]
  # 刚初始化, 速度为 0, 所以预测和之前一样
  
  Q = 离散白噪声, 低转速 (ω=0 < 2.0 rad/s):
    Q_xy = 1600 * [dt⁴/4 dt³/2; dt³/2 dt²]
    Q_z  = 1600 * [dt⁴/4 dt³/2; dt³/2 dt²]
    Q_yaw = 400 * [dt⁴/4 dt³/2; dt³/2 dt²]

下一帧观测到达:
  xyz = (2.605, 0.098, 0.418), ypr = (-2.91, 0.08, -0.04)
  → ypd = (0.0376, -0.149, 2.609)

matchArmorIndex():
  预测 4 板位置 (从当前 x 推算) → 找最匹配的
  → matched_index = 0 (没跳变)

update():
  z = [ypd.yaw, ypd.pitch, ypd.distance, ypr.yaw]
    = [0.0376, -0.149, 2.609, -2.91]
  
  h = observationJacobian(x, 0)  → 4×11 雅可比
  
  r_yaw = 4e-3, r_pitch = 4e-3
  r_dist = log(|-0.0163|+1) + 0.1 ≈ 0.116
  r_yaw_armor = log(2.609+1)/200 + 0.09 ≈ 0.096
  
  K = P·H^T·(H·P·H^T + R)^(-1)
  x_new = x + K·(z - h(x))
  
  更新后:
  x[0] = 2.605, x[1] = -0.12  ← 速度开始估计
  x[2] = 0.099, x[3] = -0.08
  x[6] = -2.909, x[7] = -0.15  ← 角速度开始估计
  x[8] = 0.22  ← radius 逐渐收敛
```

### 4.4 第 3 环节 — Controller: 从 TargetState 到控制指令

controller 是唯一主动运行的节点 (100Hz 定时器)。所有中间节点都是事件驱动 (收到消息就处理)。

#### 步骤 1: resolveActiveTarget — 匹配活动目标

```
输入:
  selected_target_id = {id: 1, valid: true}  (decider 选定打 id=1)
  
  查三路 TargetStateArray + OutpostState 中 id=1 的消息:
    front_0: 有 (最新, timestamp 更新)
    front_1: 有 (稍旧)
    back: 无
    
  selectBestTargetStateMatch(id=1, ...):
    优先 tracking=true 的, 然后 converged=true, 然后最新的
    
    → 选中 front_0 中的 id=1 TargetState

构造 LegacyTargetModel:
  model.center = (2.605, 0.099, 0.418)  ← EKF 估计的车辆中心
  model.velocity = (-0.12, -0.08, 0.005)
  model.yaw = -2.909
  model.angular_velocity = -0.15
  model.radius = 0.22
  model.radius_offset = -0.01
  model.height_offset = 0.02
  model.armor_count = 4
  model.is_outpost = false

  measurement_age_sec ≈ 0.015s  (从消息时间戳到现在)
```

#### 步骤 2: selectArmorCandidate — 选板 + 弹道

```
2a. 先对 center 做弹道解算 (获取初始 fly_time):
  transformWorldToBarrel(center.x, center.y, center.z, "gimbal_world", stamp):
    查 TF: gimbal_barrel_joint ← gimbal_world
    center_barrel = T · center_world
    = (2.62, 0.05, 0.48)  ← gun 管坐标系的中心点

  calcPitchYaw(center_barrel.x, center_barrel.y, center_barrel.z):
    牛顿迭代解 pitch:
    
    初始 θ = atan2(0.48, √(2.62²+0.05²)) ≈ 0.181 rad (10.4°)
    
    迭代 1:
      k1 = Cd × ρ × π × d²/(8m)
         = 0.42 × 1.169 × π × 0.0168²/(8×0.0032) ≈ 0.0173
      cth = cos(0.181) ≈ 0.984
      t = (exp(0.0173×2.620) - 1) / (0.0173 × 23.0 × 0.984)
        ≈ (1.0463 - 1) / 0.391 ≈ 0.1184 s
      δz = 0.48 - 23×sin(0.181)×0.1184/0.984 + 0.5×9.794×0.1184²/0.984²
         = 0.48 - 0.504 + 0.071 = 0.047 m
      
      分母 = -(23×0.1184)/0.984² + 9.794×0.1184²/23² × sin(0.181)/0.984³
          ≈ -2.814 - 0.0006 ≈ -2.815
      
      θ_new = 0.181 - 0.047/(-2.815) = 0.181 + 0.0167 = 0.1977 rad
    
    迭代 2:
      cth = cos(0.1977) ≈ 0.9805
      t = (exp(0.0173×2.620)-1)/(0.0173×23×0.9805) ≈ 0.0463/0.390 ≈ 0.1187
      δz = 0.48 - 23×sin(0.1977)×0.1187/0.9805 + 0.5×9.794×0.1187²/0.9805²
         = 0.48 - 0.549 + 0.072 = 0.003 m
      ...
    
    最终收敛: pitch ≈ 0.199 rad (11.41°), yaw ≈ 0.019 rad (1.1°), fly_time ≈ 0.119s

2b. 计算预测时间:
  timing = computeLegacyPredictTime({
    measurement_age_sec: 0.015,
    fly_time_sec: 0.119,
    include_processing_delay: true,
    system_response_time_sec: 0.0
  })
  
  processing_delay = 0.015
  predict_time = 0.119 + 0.015 = 0.134s

2c. 运动预测:
  predictLegacyTarget(target, dt=0.134):
    center.x += (-0.12) × 0.134 = 2.605 - 0.016 = 2.589
    center.y += (-0.08) × 0.134 = 0.099 - 0.011 = 0.088
    center.z += 0.005 × 0.134 = 0.419  (几乎不变)
    yaw     += (-0.15) × 0.134 = -2.909 - 0.020 = -2.929

2d. 装甲板选择:
  buildLegacyArmors(predicted_target):
    板 0: angle = -2.929+0 = -2.929, r=0.22, z=0.419
           position = (2.589 - 0.22×cos(-2.929), 
                       0.088 - 0.22×sin(-2.929),
                       0.419)
                     = (2.589 - (-0.215), 0.088 - (-0.047), 0.419)
                     = (2.804, 0.135, 0.419)
    
    板 1: angle = -2.929+π/2 = -1.358, r=0.21, z=0.439
           position = (2.589 - 0.21×cos(-1.358),
                       0.088 - 0.21×sin(-1.358),
                       0.439)
                     = (2.589 - 0.043, 0.088 - (-0.206), 0.439)
                     = (2.546, 0.294, 0.439)
    板 2/3: ...

  chooseLegacyAimPoint():
    转速 |ω| = 0.15 < 2.0 (低速)
    → 低速选板逻辑 (chooseLegacyLowSpeedArmorIndex)
    
    计算 4 块板的 delta_angle = limitRad(armor_yaw - center_yaw):
      板 0: limitRad(-2.929 - atan2(0.088, 2.589))
          = limitRad(-2.929 - 0.034) = -2.963
      板 1: limitRad(-1.358 - 0.034) = -1.392
      板 2, 板 3: ...
    
    门限过滤 (|delta_angle| < 60°):
      板 0: |-2.963| = 2.963 > π/3 ✗
      板 1: |-1.392| = 1.392... > π/3 ✗ (超过60°了)
      板 2: ... 
      板 3: ...
    
    → 如果全部超出门限, 返回 none → 本轮不选板

  (简化示例: 假设板 1 和 板 3 在门限内)
  
  jump 逻辑 + lock_index 防抖:
    lock_index 初始 -1 → 选 |delta_angle| 最小的板 → 板 X
    下一帧 lock_index = 上次选的板号, 只要还在门限内就保持

2e. 迭代收敛 (普通车辆):
  用 aim_candidate 对应板的世界坐标, 重新做弹道解算
  fly_time 是否收敛? |iter_fly_time - prev_fly_time| < 0.001s?
  最多迭代 max_prediction_iterations = 10 次

2f. 最终结果:
  selected_aim_point_world = (bx, by, bz)  ← 最终瞄准点 (世界坐标)
  
  变换到枪管系:
    barrel_point = T_world→barrel · aim_point_world
    = (2.789, -0.012, 0.478)

  再次弹道解算 (最终):
    pitch = 0.197 rad (11.29°)
    yaw   = -0.0044 rad (-0.25°)  ← 枪管系, 基本正前方
    fly_time = 0.121s

  → pitch_setpoint_deg = 11.29°
  → yaw_setpoint_deg = gimbal_yaw_deg + (-0.25°)
                      = 45.0° + (-0.25°) = 44.75°

  angle_error (与当前云台姿态):
    yaw_error   = |44.75° - 45.0°| = 0.25°
    pitch_error = |11.29° - 10.5°| = 0.79°
```

#### 步骤 3: evaluateLegacyFireControl — 开火判定

```
if yaw_error_deg < shoot_yaw_tolerance_deg (1.0°)
   AND pitch_error_deg < shoot_pitch_tolerance_deg (1.0°):
    0.25° < 1.0° ✓
    0.79° < 1.0° ✓
    → gimbal_ready = true
    → shoot_flag = true

旋中模式额外检查:
  if spin_center_aim:
    装甲板朝面检查: |legacyArmorFacingError| < shoot_face_tolerance_rad (2°)
    每面只能发射一次 (armor_fired_in_face_window 记录)

发射频率:
  elapsed > 1/fire_rate_hz (0.1s) ?
  last_fire_time 更新
  firecode: 96 ↔ 99 交替 (告知底盘发射机构)
```

#### 步骤 4: 发布最终控制指令

```
AimResult:
  fire: true
  pitch: 11.3°
  yaw: 44.8°

GimbalAngles:
  yaw: 44.8   (float32 度)
  pitch: 11.3  (float32 度)

UInt8 firecode:
  data: 99
```

---

## 5. 完整一帧数据流追踪

以下追踪一帧图像从相机曝光到最终控制输出的完整数据变化：

### 帧: host_timestamp = 123456789.000000000

```
┌─────────────────────────────────────────────────────────────────┐
│ [相机层] aim_camera_driver                                       │
├─────────────────────────────────────────────────────────────────┤
│ Input:  GX CMOS 曝光 → sensor_msgs/Image                        │
│         1280×1024, bgr8                                         │
│         stamp: sec=123456789, nanosec=0                         │
│         frame_id: "gx_camera_0"                                 │
│                                                                 │
│ Output: /gx_camera_0/image_raw                                  │
│         同上 (传输延迟 ~1-2ms)                                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              v
┌─────────────────────────────────────────────────────────────────┐
│ [检测层] aim_armor_detector (front_0)                            │
├─────────────────────────────────────────────────────────────────┤
│ Input:  sensor_msgs/Image (1280×1024 bgr8)                      │
│                                                                 │
│ Process: cv_bridge 解码 → OpenVINO 异步推理 → PCA 纠正           │
│                                                                 │
│ Output: /aim_detector/front_0/armor_sets                        │
│   ArmorSetArray {                                               │
│     header: stamp=123456789, frame_id="gx_camera_0"             │
│     armor_sets[2]:                                              │
│       [0] ArmorSet {                                            │
│             id: 1 (英雄车)                                       │
│             armors[2]:                                          │
│               [0] Armor { corners: [(512,401),(509,445),...] }  │
│               [1] Armor { corners: [(700,399),(698,443),...] }  │
│           }                                                     │
│       [1] ArmorSet { id: 3 (步兵), armors[1]: ... }             │
│   }                                                             │
│                                                                 │
│ (推理延迟 ~5ms, 加上解码/后处理 ~7ms 总延迟)                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              v
┌─────────────────────────────────────────────────────────────────┐
│ [解算层] aim_solver (front_0)                                    │
├─────────────────────────────────────────────────────────────────┤
│ Input:  ArmorSetArray (上一步输出)                                │
│         camera_matrix:                                         │
│           [2650,    0, 620]                                     │
│           [   0, 2650, 490]                                     │
│           [   0,    0,   1]                                     │
│         dist_coeffs: [k1,k2,p1,p2,k3]                          │
│                                                                 │
│ Process:                                                        │
│   id=1, 大板, w=0.230m, l=0.056m                                │
│   板0: corners → observation → solvePnP(IPPE)                   │
│     tvec = (2.35, 1.12, -0.08) 相机系                           │
│     rvec = (-2.89, 0.15, 0.02)                                 │
│     → Rodrigues → RPN → Quaternion                              │
│     → TF transform: camera→gimbal_world                         │
│     → position: (2.80, 0.15, 0.42) 世界坐标                     │
│     → optimizeYaw: 140°范围 1°步长搜索                           │
│   best_yaw = -2.90 rad, 重投影误差 = 1.85 px                    │
│                                                                 │
│ Output: /aim_solver/front_0/armor_pose_sets                     │
│   ArmorPoseSetArray {                                           │
│     header: frame_id="gimbal_world", stamp=123456789            │
│     armor_pose_sets[2]:                                         │
│       [0] ArmorPoseSet {                                        │
│             id: 1 (英雄车)                                       │
│             armor_poses[2]: 两块板的 Pose                        │
│           }                                                     │
│       [1] ArmorPoseSet { id: 3 (步兵) ... }                     │
│   }                                                             │
│                                                                 │
│ (PnP + Yaw优化 ~0.5ms)                                          │
└─────────────────────────────────────────────────────────────────┘
                              │
              ┌───────────────┴───────────────┐
              v                               v
┌─────────────────────────┐  ┌─────────────────────────────────────┐
│ [前哨站] outpost_predictor│  │ [预测层] aim_predictor (front_0)       │
├─────────────────────────┤  ├─────────────────────────────────────┤
│ Input: ArmorPoseSetArray │  │ Input: ArmorPoseSetArray (过滤 id≠6)│
│ Process:                 │  │                                     │
│   过滤 id=6 的板         │  │ Process:                            │
│   → 无 id=6, skip       │  │   id=1 hero: 2 板观测               │
│   (本例跳过)              │  │   → ArmorMeasurement[2]             │
│                          │  │   → EKF predict (dt=0.01s)          │
│ Output: (无)             │  │   → matchArmorIndex (未跳变)         │
│                          │  │   → EKF update                      │
│                          │  │                                     │
│                          │  │   状态更新 (11维):                   │
│                          │  │   x = [2.605, -0.12, 0.099, -0.08, │
│                          │  │         0.418, 0.005, -2.909, -0.15,│
│                          │  │         0.22, -0.01, 0.02]          │
│                          │  │                                     │
│                          │  │ Output: TargetStateArray              │
│                          │  │   targets[1]: id=1                   │
│                          │  │     center: (2.605, 0.099, 0.418)   │
│                          │  │     velocity: (-0.12,-0.08,0.005)   │
│                          │  │     yaw: -2.909 rad                 │
│                          │  │     ω: -0.15 rad/s                  │
│                          │  │     radius: 0.22m                   │
│                          │  │     predicted_armors[4]: Pose[]     │
│                          │  │     tracking: true, converged: true │
└─────────────────────────┘  └─────────────────────────────────────┘
                                              │
              ┌───────────────────────────────┤
              v                               v
┌────────────────────────┐    ┌────────────────────────────────────┐
│ [决策层] decider        │    │ [控制层] controller (100Hz timer)    │
├────────────────────────┤    ├────────────────────────────────────┤
│ Input:                 │    │ Input:                              │
│   front_0 TargetState  │    │   front_0/front_1/back TargetState  │
│   front_1 TargetState  │    │   OutpostState                     │
│   back TargetState     │    │   SelectedTargetId: id=1, valid     │
│   OutpostState         │    │   GimbalAngles: yaw=45°, pitch=10.5°│
│                        │    │   BulletSpeed: 23.0 m/s             │
│ Process:               │    │                                     │
│   候选: [id=1, id=3]   │    │ Process:                            │
│   外部未指定, 选最近:   │    │   1. resolveActiveTarget: id=1      │
│   id=1 (d²=2.605²+     │    │      → front_0 TargetState →       │
│         0.099²+0.418²  │    │        LegacyTargetModel           │
│         = 6.97)        │    │   2. selectArmorCandidate:         │
│                        │    │      a. center 弹道:                │
│ Output:                │    │         fly_time ≈ 0.119s          │
│   SelectedTargetId:    │    │         predict_time = 0.134s      │
│     id: 1, valid: true │    │      b. 运动预测 0.134s:            │
│   AimTargetArray:      │    │         center → (2.589,0.088,     │
│     [id=1,pos=(2.6,    │    │                    0.419)          │
│      0.1,0.42),        │    │         yaw → -2.929               │
│      id=3,pos=...]     │    │      c. buildLegacyArmors → 4板    │
│                        │    │      d. chooseLegacyAimPoint       │
│                        │    │         → 低速选板, 选板1           │
│                        │    │         → aim_point = (2.546,      │
│                        │    │            0.294, 0.439)           │
│                        │    │      e. 迭代收敛 fly_time           │
│                        │    │      f. transformWorldToBarrel     │
│                        │    │         → (2.789, -0.012, 0.478)  │
│                        │    │      g. calcPitchYaw               │
│                        │    │         → pitch: 0.197 rad (11.3°) │
│                        │    │         → yaw:  44.75°              │
│                        │    │                                     │
│                        │    │   3. 开火判定:                       │
│                        │    │      yaw_error:  0.25° < 1.0° ✓    │
│                        │    │      pitch_error: 0.79° < 1.0° ✓  │
│                        │    │      → shoot_flag = true            │
│                        │    │      elapsed > 0.1s? ✓             │
│                        │    │      → fire = true                  │
│                        │    │                                     │
│                        │    │ Output:                             │
│                        │    │   /ly/aim/result:                   │
│                        │    │     {fire:true, pitch:11.3°,        │
│                        │    │      yaw:44.8°}                     │
│                        │    │   /ly/control/angles:               │
│                        │    │     {yaw:44.8, pitch:11.3}         │
│                        │    │   /ly/control/firecode:             │
│                        │    │     {data:99}                      │
└────────────────────────┘    └────────────────────────────────────┘
```

### 数值变化总结

| 环节 | 数据形式 | 关键数值变化 |
|------|---------|------------|
| 相机 | Image像素 | 1280×1024, 每像素 3 字节 |
| detector | 角点像素坐标 | corners: (512,401), (509,445), (700,399), (698,443) |
| solver (PnP) | 3D位姿 | tvec: (2.35,1.12,-0.08) 相机系 → 变换后 position: (2.80,0.15,0.42) 世界系 |
| solver (Yaw优化) | 优化后位姿 | yaw 从 -2.93 优化到 -2.90, 重投影误差 1.85 px |
| predictor (转换) | 球坐标 | ypd: (yaw=0.0535rad, pitch=-0.148rad, distance=2.836m) |
| predictor (EKF) | 状态向量 | x = [2.605, -0.12, 0.099, -0.08, 0.418, 0.005, -2.909, -0.15, 0.22, -0.01, 0.02] |
| controller (预测) | 预测后 target | center: (2.589, 0.088, 0.419), yaw: -2.929 |
| controller (选板) | 板位姿 | 板1: (2.546, 0.294, 0.439), yaw: -1.358 |
| controller (TF变换) | 枪管坐标 | barrel: (2.789, -0.012, 0.478) |
| controller (弹道) | 控制角度 | pitch: 0.197 rad (11.3°), yaw最终: 44.75° (相对world yaw) |
| controller (输出) | 执行指令 | fire: true, GimbalAngles: {44.8°, 11.3°}, firecode: 99 |

---

## 6. 关键技术点总结

### 6.1 Solver 的关键创新
- **Yaw 重投影优化**: PnP 解的 yaw 角度最不可靠 (因为装甲板是平面), 通过 ±70° 暴力搜索 + 重投影验证, 大幅提高 yaw 精度
- **固定 pitch 假设**: 基于装甲板安装角度已知 (车辆约 15° 倾斜), 用固定 pitch 来约束优化
- **IPPE 双解消歧**: 利用灯条倾角判断装甲板的正/反面

### 6.2 Predictor 的 11 维 EKF
- **过程噪声自适应**: 根据角速度分三段 (低 < 2 rad/s, 中 < 6, 高 ≥ 6), 每段独立配置
- **高低板建模**: 4 板车辆中 0/2 号板和 1/3 号板半径和高度不同, 用 radius_offset 和 height_offset 建模
- **板号匹配**: matchArmorIndex() 通过 yaw 和距离找到当前观测对应的板号

### 6.3 Controller 的弹道解算
- **牛顿迭代**: 含空气阻力的倾斜弹道模型, Cd=0.42, ρ=1.169, 迭代约 3-5 次收敛
- **预测-弹道迭代收敛**: fly_time 本身依赖目标位置, 目标位置又依赖预测时间 (= fly_time + delay), 需要迭代直到 fly_time 收敛
- **智能选板**: 低速时选朝向自己的板; 高速旋转时按 coming_side/leaving_angle 策略选即将正对枪口的板; 旋中模式额外检查面朝角度

### 6.4 前哨站的特殊处理
- **3 槽位联合匹配**: DFS 搜索最优观测→槽位分配, 支持旋转方向惩罚
- **主打板 (primary_armor)**: 跟踪过程中识别并锁定"最可能是真装甲板"的槽位
- **快重锚**: 主打板丢失时快速切换到其他槽位
