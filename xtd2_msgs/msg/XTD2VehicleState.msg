# XTD2VehicleState.msg

# 飞控状态
string offboard_state  # DISABLED, ENABLED, POSE_LOCAL_NED, POSE_LOCAL_FLU, VEL_NED, VEL_FLU, ACCEL_NED, ACCEL_FLU, ATTITUDE_FLU

# 当前位置信息
float64 x  # 当前位置 x
float64 y  # 当前位置 y
float64 z  # 当前位置 z
float64 heading  # 当前航向角

# 全局位置信息
float64 lat  # 纬度
float64 lon  # 经度
float64 alt  # 高度

# 初始位置信息
float64 init_x  # 初始位置 x
float64 init_y  # 初始位置 y
float64 init_z  # 初始位置 z
float64 init_heading  # 初始航向角

# 初始全局位置信息
float64 init_lat  # 初始纬度
float64 init_lon  # 初始经度
float64 init_alt  # 初始高度

# 时间戳
uint64 timestamp  # 消息时间戳（微秒） 