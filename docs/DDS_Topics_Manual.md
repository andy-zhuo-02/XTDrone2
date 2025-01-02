# 一、简介

本手册参考

* [PX4-Autopilot dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml)
* [uORB消息参考](https://docs.px4.io/main/zh/msg_docs/)

本手册选取了较常用的话题和XTDrone2中经常使用的话题进行分析和说明，旨在帮助读者快速入门

# 二、uXRCE-DDS client Publication


# 三、uXRCE-DDS client Subscription

### 飞机指令

#### vehicle_command

topic: /fmu/in/vehicle_command
type: px4_msgs::msg::VehicleCommand

[VehicleCommand定义](https://docs.px4.io/main/zh/msg_docs/VehicleCommand.html)

### 模式切换

#### offboard_control_mode

topic: /fmu/in/offboard_control_mode
type: px4_msgs::msg::OffboardControlMode

```
# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool position
bool velocity
bool acceleration
bool attitude
bool body_rate
bool thrust_and_torque
bool direct_actuator
```

### 机体运动

#### trajectory_setpoint

topic: /fmu/in/trajectory_setpoint
type: px4_msgs::msg::TrajectorySetpoint

```
# Trajectory setpoint in NED frame
# Input to PID position controller.
# Needs to be kinematically consistent and feasible for smooth flight.
# setting a value to NaN means the state should not be controlled

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# NED local world frame
float32[3] position # in meters
float32[3] velocity # in meters/second
float32[3] acceleration # in meters/second^2
float32[3] jerk # in meters/second^3 (for logging only)

float32 yaw # euler angle of desired attitude in radians -PI..+PI
float32 yawspeed # angular velocity around NED frame z-axis in radians/second
```
