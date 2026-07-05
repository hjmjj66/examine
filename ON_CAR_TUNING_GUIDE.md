# sentry.aim_new 上车调参指南

这份文档只讲一件事：**上车后先调哪些参数，按什么顺序调。**

不讲原理，不讲代码细节，直接照着做。

## 0. 上车前先确认

先不要急着调 EKF 和控制器，先确认下面 3 件事已经对：

1. 三个相机的内参已经填好
2. 三个相机的外参 / TF 已经通
3. 三路图像都能正常出图

如果这三件事没对，后面的参数越调越乱。

---

## 1. 推荐调参顺序

按这个顺序调：

1. 相机成像参数
2. detector 输入质量
3. solver 稳定性
4. predictor 跟踪参数
5. outpost predictor 参数
6. controller 开火与弹道参数
7. 最后再动经验补偿参数

---

## 2. 第一优先级：相机成像参数

### 前相机 `front_0`

文件：
[gx_camera_0.yaml](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_camera_driver/config/gx_camera_0.yaml)

先调这些：

- `exposure_time`
- `gain`
- `red_balance_ratio`
- `blue_balance_ratio`

### 前相机 `front_1`

文件：
[gx_camera_1.yaml](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_camera_driver/config/gx_camera_1.yaml)

先调这些：

- `exposure_time`
- `gain`
- `red_balance_ratio`
- `blue_balance_ratio`

### 后相机 `back`

文件：
[usb_camera.yaml](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_camera_driver/config/usb_camera.yaml)

先调这些：

- `exposure`
- `gain`
- `brightness`
- `saturation`
- `contrast`

### 上车时怎么判断

- 图像太暗：先加曝光，再少量加增益
- 图像拖影明显：减曝光
- 图像有噪点、发脏：减增益
- 灯条颜色失真：调白平衡/颜色比例

---

## 3. 第二优先级：detector 相关参数

文件：
[front_0_detector.yaml]
[front_1_detector.yaml]
[back_camera_detector.yaml]

优先关注：

- `enemy_color`
- `enable_pca_correction`
- `pca.lightbar_min_mean_brightness`
- `pca.normalize_max_brightness`
- `pca.padding_scale`
- `pca.min_sample_width`

### 什么时候调这些

- 明明看见灯条，但 detector 不稳定：先看 `lightbar_min_mean_brightness`
- 误检多：先看 `padding_scale`、`min_sample_width`
- 灯条提取形状不好：看 `enable_pca_correction`

如果图像质量本身就不好，不要先动 detector 参数，先回去调相机。

---

## 4. 第三优先级：solver 参数

文件：
[aim_solver.yaml](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_solver/config/aim_solver.yaml)

先调这些：

- `project_error_ratio_thres`
- `roll_thres_degree`
- `tf_lookup_timeout_sec`

如果需要再看：

- `enable_optimize_yaw`
- `optimize_yaw_pitch_deg`
- `optimize_yaw_outpost_pitch_deg`

### 上车时怎么判断

- 位姿偶尔大跳：先减小 `project_error_ratio_thres`
- 正常目标经常被过滤掉：适当增大 `project_error_ratio_thres`
- TF 偶发查不到：看 `tf_lookup_timeout_sec`

---

## 5. 第四优先级：普通目标 predictor 参数

文件：
[aim_predictor.yaml](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_predictor/config/aim_predictor.yaml)

最先调这些：

- `max_lost_count`
- `min_consecutive_detections_to_track`
- `multi_armor_yaw_gate_deg`

然后调过程噪声：

- `low_speed_process_noise_xy`
- `low_speed_process_noise_z`
- `low_speed_process_noise_yaw`
- `middle_speed_process_noise_xy`
- `middle_speed_process_noise_z`
- `middle_speed_process_noise_yaw`
- `high_speed_process_noise_xy`
- `high_speed_process_noise_z`
- `high_speed_process_noise_yaw`

速度分段阈值也要看：

- `middle_speed_angular_velocity_threshold`
- `high_speed_angular_velocity_threshold`

### 上车时怎么判断

- 目标一遮挡就掉：增大 `max_lost_count`
- 刚出现目标时很久都不开始跟：减小 `min_consecutive_detections_to_track`
- 旋转目标跟不上：增大对应速度段的 `process_noise_yaw`
- 平移目标跟不上：增大对应速度段的 `process_noise_xy`
- 预测点发飘、发抖：减小对应速度段的过程噪声

---

## 6. 第五优先级：前哨站 predictor 参数

文件：
[aim_outpost_predictor.yaml](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_outpost_predictor/config/aim_outpost_predictor.yaml)

先调这些：

- `armor_gimbal_yaw_gate_deg`
- `max_lost_count`
- `min_consecutive_detections_to_track`
- `max_nis_failure_ratio`

再调这些：

- `process_noise_xy`
- `process_noise_z`
- `process_noise_yaw`
- `angular_velocity_clamp`

必要时再看匹配参数：

- `match_yaw_sigma`
- `match_pitch_sigma`
- `match_distance_sigma`
- `match_angle_sigma`
- `match_z_sigma`
- `match_yaw_gate`
- `match_pitch_gate`
- `match_distance_gate`
- `match_angle_gate`
- `match_z_gate`
- `max_pair_cost`

### 上车时怎么判断

- 前哨站目标很容易掉：增大 `max_lost_count`
- 前哨站初始化太慢：减小 `min_consecutive_detections_to_track`
- 前哨站状态经常重置：先看 `max_nis_failure_ratio`
- 旋转跟不上：增大 `process_noise_yaw`
- 位置飘、抖：减小 `process_noise_xy` / `process_noise_z`

---

## 7. 第六优先级：controller 参数

文件：
[armor_controller.yaml](E:/桌面/sentry.aim/sentry.aim_new/src1/aim_armor_controller/config/armor_controller.yaml)

### 最先调

- `bullet_speed_alpha`
- `tol_deltax_m`
- `tol_deltay_m`
- `shoot_yaw_tolerance_deg`
- `shoot_pitch_tolerance_deg`
- `target_tf_timeout_sec`
- `target_msg_timeout_sec`

### 然后调

- `target_offset.x`
- `target_offset.y`
- `target_offset.z`

### 智能选板相关

- `enable_smart_selector`
- `comming_angle_deg`
- `leaving_angle_deg`
- `smart_selector_max_angular_velocity`
- `spin_center_aim_angular_velocity_threshold`
- `outpost_shoot_yaw_gate_deg`

### 最后再动

- `controller_config.shoot_table_adjust.enable`
- `controller_config.shoot_table_adjust.pitch.*`
- `controller_config.shoot_table_adjust.yaw.*`

### 上车时怎么判断

- 已经基本瞄上，但不肯开火：适当增大 `tol_deltax_m` / `tol_deltay_m`
- 开火太激进，容易空枪：减小 `tol_deltax_m` / `tol_deltay_m`
- 弹速变化时控制抖：减小 `bullet_speed_alpha`
- 弹速变化跟不上：增大 `bullet_speed_alpha`
- 始终稳定偏左/偏右/偏高/偏低：调 `target_offset`
- 近处准、远处偏，或者误差很有规律：最后再动 `shoot_table_adjust`

---

## 8. 上车时最值得先动的参数清单

如果时间很紧，优先只调这几组：

### 相机

- `exposure_time` / `exposure`
- `gain`

### solver

- `project_error_ratio_thres`

### predictor

- `max_lost_count`
- `process_noise_xy`
- `process_noise_yaw`

### outpost predictor

- `max_lost_count`
- `max_nis_failure_ratio`
- `process_noise_yaw`

### controller

- `tol_deltax_m`
- `tol_deltay_m`
- `bullet_speed_alpha`
- `target_offset.x/y/z`

---

## 9. 最后提醒

- 一次只改一类参数
- 每次只改一小步
- 先把图像调干净，再调 detector
- 先把 detector / solver 调稳，再调 predictor
- 先把 predictor 调稳，再调 controller
- `shoot_table_adjust` 放到最后
