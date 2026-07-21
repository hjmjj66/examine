# 双前相机共享 Predictor 改造与回退手册

## 1. 目标、边界与当前基线

### 1.1 改造目标

两路前相机的 `ArmorPoseSetArray` 已由 `aim_solver` 变换至同一
`gimbal_world` 坐标系。因此，普通装甲板目标不应分别维护两份（或更多份）
EKF 状态，而应维护**一份按目标 ID 索引的共享 tracker map**：

```text
front_0 pose sets ─┐
                   ├─ 时间排序 + 同板关联 ──> shared tracker map ──> fused target states
front_1 pose sets ─┘                                  (一个 EKF/目标)
```

每条观测保留来源相机信息；同一共享 tracker 根据来源使用对应的**测量噪声协方差
`R`**更新。过程噪声 `Q` 继续描述敌方运动模型，不按相机区分。

本方案只覆盖普通装甲板 predictor。前哨站继续由 `aim_outpost_predictor` 处理；
后相机先维持现有回退逻辑，待双前相机稳定后再决定是否纳入共享 tracker。

### 1.2 当前行为（必须作为回退基线）

当前代码已存在 `front_0`、`front_1`、`back` 和 `fused` 四条 `Pipeline`。每个
Pipeline 各有 `trackers`，即同一个敌方 ID 会拥有独立的 EKF 状态和协方差。两路
前相机消息会：

1. 各自更新 `front_0` / `front_1` tracker；
2. 在 30 ms 的 wall-time 窗口中按 ID、三维距离 8 cm 做简单合并；
3. 再更新独立的 `fused` tracker。

控制器和决策器目前均订阅 `/aim_predictor/fused/target_states`。因此，改造期间必须
保持该话题的消息类型和语义可用，避免触及 `aim_msgs`、controller 或 decider 的接口。

当前工作区已有未提交改动，且其中包括 predictor 融合逻辑。**不能把 `HEAD` 当作
“现在版本”的回退点。**本文第 8 节要求先把当前相关文件制成独立基线提交和 tag。

## 2. 目标架构与关键设计决策

### 2.1 一套状态，不是一套“噪声参数”

共享对象是每个目标 ID 的 EKF：状态 `x`、协方差 `P`、上次处理时间、连续检测状态和
丢失状态都只有一份。两相机差异只体现在每条观测的可信度和时间信息上。

`NormalTargetTracker::update()` 当前在内部固定构造四维观测
`z = [yaw, pitch, distance, armor_yaw]` 的 `R`。改造后应由调用方或一个独立的
`MeasurementNoiseModel` 根据 `camera_source` 构造：

```text
R_front_0 = diag(var_yaw_0, var_pitch_0, var_range_0, var_armor_yaw_0)
R_front_1 = diag(var_yaw_1, var_pitch_1, var_range_1, var_armor_yaw_1)
```

初版仍可采用对角阵，但参数必须是**方差**而非标准差，并明确单位：角度为 `rad²`、
距离为 `m²`。距离、斜视角和装甲板朝向会影响 PnP 精度，所以推荐把现有的距离/斜视角
修正保留，再乘以相机对应的标定系数；不要直接猜一组固定数字。

`Q` 保持全局共享：它表示目标平移、旋转和模型失配的变化率。某台相机丢帧或延迟大，
应由正确的时间戳、`R` 和丢失超时处理，而不应为该相机另设一套 `Q`。

### 2.2 时间排序与同板处理

两相机的曝光、推理和 ROS 调度延迟不同，消息可能乱序。共享 tracker 必须保证时间不
回退；否则后到的旧观测会破坏状态推进。

本次实现选择**零等待**路径：收到一条前相机观测就立即更新共享 tracker，不建立融合
timer 或重排队列。这样相对于原有的 30 ms 前相机合并窗口只会减少等待，不会增加延迟。
tracker 维护最后已处理 timestamp；晚于该时间的消息记录调试日志并丢弃，绝不使
`last_stamp_` 回退。后续若实测乱序丢弃率较高，才评估完整 OOSM（out-of-sequence
measurement）重放；不能用一个新的等待窗口作为未经验证的替代。

本次实现没有增加跨相机去重或等待窗口：两路到达的观测会立即、顺序地更新同一份状态。
这符合“分别以各自 `R` 更新同一 predictor”的目标，也没有额外时延。当前三台相机的
`R` 缩放均为 `1.0`，与旧的硬编码模型等价。完成残差标定后，若发现同一物理装甲板的
双观测存在强相关性，再评估马氏距离关联和择优更新；不得以增加固定等待时间为代价。

### 2.3 确认、丢失与输出

现有 `min_consecutive_detections_to_track` 和 `max_lost_count` 以“收到一次消息”为单位。
共享输入后，两相机帧率会让确认速度变为两倍，而某一路空帧也可能错误推进丢失计数。

改为按融合后的时间事件处理：

- 目标确认：按时间连续性或按去重后的观测 bundle 计数；
- 目标失效：维护 `last_successful_update_stamp`，通过
  `target_lost_timeout_sec` 判断；
- 输出：仍发布 `/aim_predictor/fused/target_states`，header 使用最后一个已按序处理的
  measurement timestamp。

这样下游 controller 和 decider 不需要修改。

## 3. 已实施改动

本次实现采用直接接管的低延迟版本，没有保留并行 legacy tracker。

| 文件 | 修改内容 |
|---|---|
| `include/aim_predictor/normal_target_tracker.hpp` | 为 `ArmorMeasurement` 增加 `CameraSource source`；将 `update()` 改为接收测量噪声模型或已构造的 `R`。 |
| `src/aim_predictor/normal_target_tracker.cpp` | 把当前硬编码 `R` 的计算抽成可注入函数；修正 timestamp 单调性，禁止 `last_stamp_` 倒退。 |
| `include/aim_predictor/aim_predictor_node.hpp` | 移除三条独立预测 Pipeline，只保留一个 fused/shared Pipeline 和三路输入订阅。 |
| `src/aim_predictor/aim_predictor_node.cpp` | 前 0、前 1 收到消息立即更新共享 tracker；仅在两路前相机最近均无普通装甲板观测时，后相机才更新该 tracker。 |
| `config/aim_predictor.yaml` | 移除独立 target-state 输出和 30 ms 融合窗口；增加按时间丢失、确认间隔及三台相机的 `R` 缩放参数。 |
| `test/` 与 `CMakeLists.txt` | 添加“缩放为 1.0 保持旧 R”和“单相机缩放只影响对应观测维度”的单测。 |

不修改 `aim_msgs`。来源只在 predictor 内部使用，避免扩大 ROS 接口变更面。

### 后续：标定测量噪声与时间偏移

1. 在静止目标、多个距离、多个偏航角下分别录两台前相机数据；
2. 将每相机输出与可信真值或高质量拟合轨迹比较，统计 yaw、pitch、distance、armor-yaw
   的残差均值和方差；
3. 均值显著非零时先做相机偏置/手眼标定修正，不能只增大 `R` 掩盖偏差；
4. 分析两路图像 timestamp 与 solver 输出 timestamp 的差，校验各自的链路时延；
5. 当前 solver 只有共用的 `tf_timestamp_offset_sec`。若两路时延不同，应新增**每相机**
   的时间偏移标定并在进入共享 tracker 前体现；
6. 将统计得到的方差写入 YAML，并保留采集 bag、脚本和结果表以便复现。

## 4. 具体处理流程（伪代码）

```cpp
onCameraPoseSets(CameraSource source, Msg msg) {
  if (source == Back && hasRecentFrontTarget(msg.header.stamp)) {
    return;
  }
  if (msg.header.stamp < shared_pipeline.last_processed_stamp) {
    return;  // 不回退 EKF 时间，也不等待重排
  }
  shared_pipeline.predict(msg.header.stamp);
  for (const auto& measurement : toMeasurements(source, msg)) {
    shared_pipeline.update(measurement, R_by_camera[source]);
  }
  expireTrackersByElapsedTime();
  publishFusedState();
}
```

`selectOrFuseMeasurements()` 的初版应只做“择优”，不做数值平均。直接平均 pose / yaw
会破坏角度环绕和非线性观测模型；若后续要融合，必须在 EKF 的观测空间中按协方差处理。

## 5. 验证与验收

### 5.1 单元测试

- 前 0、前 1 的消息到达顺序颠倒，但 timestamp 有序时，最终状态一致；
- 晚到消息不会使 tracker 时间回退；
- 对相同创新量，较小 `R` 的相机对状态修正更大；
- 同一物理装甲板的双观测只更新一次；不同装甲板不被错误去重；
- 任一路相机单独工作时能正常初始化、跟踪、超时删除；
- `legacy_parallel` 的输出与当前基线测试样本保持一致；
- outpost ID 仍被普通 predictor 排除。

### 5.2 集成测试

```bash
colcon build --packages-select aim_predictor --symlink-install
colcon test --packages-select aim_predictor
colcon test-result --verbose
source install/setup.bash
```

使用 rosbag 重放时，至少检查：

```bash
ros2 topic echo --once /aim_predictor/fused/target_states
ros2 topic echo --once /aim_predictor/shared_candidate/target_states
ros2 node info /aim_predictor_node
```

### 5.3 放行标准

- 无 timestamp 倒退、无负 `dt` 被静默处理；
- 两相机切换时不出现额外的目标重初始化、半径发散或角速度尖峰；
- 单相机遮挡/掉帧时，目标保持时间不差于 legacy；
- shared 的端到端延迟不超过 legacy 加上约 5 ms，或满足项目确定的实时预算；
- NIS、创新量和丢失次数在录包对比中不劣于 legacy；
- controller/decider 在切换前后始终从相同的 fused topic 收到兼容消息。

阈值中的“约 5 ms”和“不劣于”应在首次录包基准完成后替换为团队认可的数值。

## 6. 失败判据与即时运行回退

以下任一项出现即停止部署并源码级回退：持续 TF 时间错位告警、输出 timestamp 倒退、
相机交界处频繁重初始化、协方差异常收缩、控制输出振荡、目标丢失率明显高于基线。

当前实现为满足“只维护一套”与零等待要求，没有编译期保留 `legacy_parallel` 模式，
因此**不存在仅改 YAML 的运行期回退**。回退必须使用第 7、8 节的已验证 Git tag/worktree
重新构建并重启节点；controller/decider 的 fused 话题保持不变。

## 7. 源码级回退前的强制准备

### 7.1 创建精确的当前基线

在动 predictor 代码**之前**，创建一个只包含本改造相关路径的安全提交和不可变 tag。
由于当前工作区存在未提交内容，以下路径必须包含未跟踪的融合头文件和测试：

```bash
git switch -c safety/predictor-before-shared-ekf-YYYYMMDD
git add -- \
  src/aim_predictor \
  src/aim_armor_controller/config/armor_controller.yaml \
  src/aim_armor_decider/config/aim_armor_decider.yaml
git commit -m "safety: snapshot predictor before shared tracker refactor"
git tag -a predictor-before-shared-ekf-YYYYMMDD -m "Known-good predictor baseline"
git rev-parse predictor-before-shared-ekf-YYYYMMDD
```

把最后一条命令输出的 SHA、对应参数文件和已验证 rosbag 名称记录到测试记录中。只有确认
这些目标路径代表“现在能工作的版本”后，才可以继续改造。

上述提交只暂存列出的路径，不会包含工作区中其他无关改动。执行前仍应运行
`git status --short` 和 `git diff --cached` 做人工确认。

### 7.2 隔离开发

推荐从 tag 另建 worktree 开发，使当前可运行目录保持不动：

```bash
git worktree add ../sentry.aim-shared-predictor \
  -b feature/shared-predictor \
  predictor-before-shared-ekf-YYYYMMDD
```

在新 worktree 实现、构建和录包测试。当前目录可继续作为 legacy 对照环境。这样回退时
不需要对包含其他人改动的主工作区执行任何破坏性 git 命令。

## 8. 源码级回退步骤

### 8.1 推荐回退：切回已验证 worktree / tag

这是首选方式。停止 shared 版本进程，切回以
`predictor-before-shared-ekf-YYYYMMDD` 构建的 legacy worktree，重新构建 predictor 并启动。
该方法不修改原开发工作区，因此不会误删其他未提交工作。

```bash
source install/setup.bash
colcon build --packages-select aim_predictor --symlink-install
source install/setup.bash
```

然后按当前 bringup 启动，并确认 `/aim_predictor/fused/target_states` 恢复发布。

### 8.2 必须在同一部署目录原地回退时

原地回退会覆盖 target 路径内的后续改动，执行前必须先保存这些改动。仅在确认没有需要
保留的本地变更，或已另行备份后使用：

```bash
BASE_TAG=predictor-before-shared-ekf-YYYYMMDD
git diff --binary -- \
  src/aim_predictor \
  src/aim_armor_controller/config/armor_controller.yaml \
  src/aim_armor_decider/config/aim_armor_decider.yaml \
  > /tmp/predictor-before-rollback.patch

git restore --source "$BASE_TAG" --staged --worktree -- \
  src/aim_predictor \
  src/aim_armor_controller/config/armor_controller.yaml \
  src/aim_armor_decider/config/aim_armor_decider.yaml
```

若 shared 改造创建了基线不存在的新文件，只能**明确列出这些文件**后删除；不要执行全局
`git clean -fd`，也不要执行 `git reset --hard`。删除完成后重新构建并启动，再使用
`git diff -- "$BASE_TAG" -- <目标路径>` 验证为零差异。

`/tmp/predictor-before-rollback.patch` 仅保存已跟踪文件差异；如果原地回退前存在必须保留
的未跟踪文件，应先复制到单独目录或建立独立提交。因而 worktree 回退始终优于原地回退。

## 9. 提交与发布纪律

建议提交顺序如下，保证每一步都可独立回退：

1. `safety`：当前基线提交与 tag；
2. `refactor`：只引入 source、`R` 注入和时间单调性，不改变控制输出；
3. `feature`：增加 shared candidate / shadow 模式与测试；
4. `config`：录包标定后的噪声、时间窗口和超时参数；
5. `switch`：将 shared 接到 fused 生产话题；
6. `cleanup`：在完整验证周期后才删除 legacy 路径。

任何实机发布都应记录：Git SHA、参数文件 SHA、相机标定版本、TF 时间偏移、使用的 bag、
验收结果和操作者。这样才能在问题发生后准确回到“现在的版本”，而不是模糊地回到某个
旧 `HEAD`。
