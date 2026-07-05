std_msgs/Header header
aim_msgs/Armor[] armors
uint8 id

# —— predictor 附加状态 (供 controller 智能选板) ——
bool tracking
bool converged
bool jumped
float64 yaw              # 车辆朝向角 (rad)
float64 angular_velocity  # 车辆角速度 (rad/s)
float64 radius           # 旋转半径 (m)
float64 radius_offset    # 侧板额外偏移 (m)
float64 height_offset    # 侧板高度偏移 (m)
float64 velocity_x       # 车辆中心 X 速度 (m/s)
float64 velocity_y
float64 velocity_z
float64 center_x         # 车辆中心 X 位置 (m)
float64 center_y
float64 center_z
