# aim_handeye_calibrator

为哨兵自瞄提供基于棋盘格的相机内参标定和手眼标定。

默认按 `front_0` 相机适配：
- 图像话题：`/gx_camera_0/image_raw`
- 相机信息话题：`/gx_camera_0/camera_info`
- 手眼外参默认回填到 `src/sentry_tf/config/sentry_tf.yaml` 的 `barrel_to_camera`
- 内参默认回填到 `src/aim_solver/config/aim_solver.yaml` 的 `front_camera_matrix` / `front_distortion_coefficients`

## 节点

- 内参标定节点：`aim_camera_intrinsic_calibrator_node`
- 手眼标定节点：`aim_handeye_calibrator_node`

两者都提供服务：
- `~/capture_sample`
- `~/solve`
- `~/clear_samples`

## 内参标定前提

- 相机能稳定看到棋盘格
- 棋盘格参数已知
- 采样期间分辨率不要变化

## 内参使用流程

1. 修改 `config/aim_camera_intrinsic_calibrator.yaml`
   - `board_cols`：棋盘格横向内角点数
   - `board_rows`：棋盘格纵向内角点数
   - `square_size_m`：单格边长，单位米
2. 启动节点

```bash
ros2 launch aim_handeye_calibrator aim_camera_intrinsic_calibrator.launch.py
```

3. 移动棋盘格，覆盖画面中心、四角、不同距离和不同倾角
4. 每到一个新姿态，调用一次

```bash
ros2 service call /aim_camera_intrinsic_calibrator_node/capture_sample std_srvs/srv/Trigger {}
```

5. 采够样本后求解

```bash
ros2 service call /aim_camera_intrinsic_calibrator_node/solve std_srvs/srv/Trigger {}
```

6. 将输出里的 `front_camera_matrix` 和 `front_distortion_coefficients` 回填到 `src/aim_solver/config/aim_solver.yaml`

## 手眼标定前提

- 相机能看到固定不动的棋盘格标定板
- 云台能发布 `/gimbal/state`
- 已知当前相机内参
- 当前默认解的是 `gimbal_barrel -> gx_camera` 静态外参

节点内部先用棋盘格角点和 `solvePnP` 解出标定板在 `OpenCV optical frame` 下的位姿，再转换到当前工程相机坐标定义。

## 手眼使用流程

1. 先完成内参标定，或直接在 `config/aim_handeye_calibrator.yaml` 中填写
   - `camera_matrix`
   - `distortion_coefficients`
2. 确认棋盘格参数
   - `board_cols`
   - `board_rows`
   - `square_size_m`
3. 启动节点

```bash
ros2 launch aim_handeye_calibrator aim_handeye_calibrator.launch.py
```

4. 固定棋盘格不动，手动转动云台到多个姿态
5. 每到一个新姿态，调用一次

```bash
ros2 service call /aim_handeye_calibrator_node/capture_sample std_srvs/srv/Trigger {}
```

6. 采够样本后求解

```bash
ros2 service call /aim_handeye_calibrator_node/solve std_srvs/srv/Trigger {}
```

7. 将输出回填到 `src/sentry_tf/config/sentry_tf.yaml` 对应段

## front_1 怎么改

改两个配置文件即可：

- `config/aim_camera_intrinsic_calibrator.yaml`
  - `image_topic` 改成 `/gx_camera_1/image_raw`
  - `camera_name` 改成 `front_1`
  - `frame_id` 改成 `gx_camera_1`
  - `aim_solver_camera_matrix_key` 改成 `front_1_camera_matrix`
  - `aim_solver_distortion_key` 改成 `front_1_distortion_coefficients`
- `config/aim_handeye_calibrator.yaml`
  - `image_topic` 改成 `/gx_camera_1/image_raw`
  - `camera_info_topic` 改成 `/gx_camera_1/camera_info`
  - 如果你希望 `sentry_tf` 区分两个前相机，就把 `child_frame_id` 改成你实际使用的 frame 名
  - 如果外参回填段名也要区分，再把 `output_tf_section` 改成你准备写回的段名

## USB 后相机怎么改

同样改两个配置文件：

- `config/aim_camera_intrinsic_calibrator.yaml`
  - `image_topic` 改成 `/usb_camera/image_raw`
  - `camera_name` 改成 `back` 或你自己想用的名字
  - `frame_id` 改成 `usb_camera`
  - `aim_solver_camera_matrix_key` 改成 `back_camera_matrix`
  - `aim_solver_distortion_key` 改成 `back_distortion_coefficients`
- `config/aim_handeye_calibrator.yaml`
  - `image_topic` 改成 `/usb_camera/image_raw`
  - `camera_info_topic` 改成 `/usb_camera/camera_info`
  - `parent_frame_id` 改成 `gimbal_big_yaw`
  - `child_frame_id` 改成 `usb_camera`
  - `output_tf_section` 改成 `big_yaw_to_usb_camera`

## 样本建议

- 内参建议至少 `12` 组，手眼建议至少 `8` 组
- 更稳妥的范围是 `15~30` 组
- 不要只在一条线附近采样
- yaw 和 pitch 都要有明显变化
- 尽量覆盖实际使用角度范围
- 棋盘格角点要清晰，避免运动模糊、反光、过曝和严重遮挡

## 注意

- `board_cols` 和 `board_rows` 是内角点数，不是格子数
- `angles_in_degree` 要和 `/gimbal/state` 实际单位一致
- `yaw_sign`、`pitch_sign` 用来处理下位机角度正方向定义不一致的问题
- 若结果波动很大，优先检查：
  - 内参是否正确
  - 棋盘格边长是否正确
  - 棋盘格在手眼采样期间是否固定不动
  - 云台角度单位和符号是否正确
  - 回填的 `parent_frame_id` / `child_frame_id` 是否就是你要替换的那段 TF
