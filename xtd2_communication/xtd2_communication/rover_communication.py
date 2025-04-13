"""
Rover Communication Module

This module provides communication functionalities for multirotor.

Author: Wang Lilel
Email: 15726686898@163.com

All rights reserved. This code is licensed under the MIT License.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleAttitudeSetpoint

from xtd2_msgs.srv import XTD2Cmd
from xtd2_msgs.msg import XTD2VehicleState

import sys
import math
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import argparse

class RoverCommNode(Node):
    def __init__(self):
        super().__init__('rover_comm_node')

        # PX4 DDS Publisher
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # 订阅 XTDrone2 的速度命令
        self.cmd_vel_sub = self.create_subscription(Twist, '/xtdrone2/command/velocity', self.cmd_vel_callback, 10)

        # 发布状态
        self.vehicle_state_pub = self.create_publisher(String, '/xtdrone2/state', 10)

        # 定时发送 setpoint（重要！维持 OFFBOARD 模式）
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 当前速度 setpoint（x, y, yaw_rate）
        self.current_setpoint = TrajectorySetpoint()
        self.current_setpoint.velocity = [0.0, 0.0, 0.0]
        self.current_setpoint.yaw = 0.0

        # 初始化状态
        self.offboard_started = False
        self.armed = False

        self.get_logger().info("RoverCommNode Initialized")

    def cmd_vel_callback(self, msg: Twist):
        # 只取平面内速度和角速度
        self.current_setpoint.velocity[0] = msg.linear.x
        self.current_setpoint.velocity[1] = msg.linear.y
        self.current_setpoint.velocity[2] = 0.0
        self.current_setpoint.yaw = msg.angular.z

    def timer_callback(self):
        if self.offboard_started:
            self.publish_trajectory_setpoint()
        self.publish_state()

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.velocity = self.current_setpoint.velocity
        msg.yaw = self.current_setpoint.yaw
        self.setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def publish_state(self):
        state_msg = String()
        status = f"ARMED: {self.armed}, OFFBOARD: {self.offboard_started}, vel: {self.current_setpoint.velocity}, yaw: {self.current_setpoint.yaw}"
        state_msg.data = status
        self.vehicle_state_pub.publish(state_msg)

    # === 用户控制 ===
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.armed = True
        self.get_logger().info('Sent ARM command')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.armed = False
        self.get_logger().info('Sent DISARM command')

    def start_offboard(self):
        self.publish_trajectory_setpoint()  # 先发送一次 setpoint
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)  # 1: custom, 6: offboard
        self.offboard_started = True
        self.get_logger().info('Switched to OFFBOARD mode')

    def stop_offboard(self):
        self.offboard_started = False
        self.get_logger().info('Stopped OFFBOARD mode')

def main(args=None):
    rclpy.init(args=args)
    node = RoverCommNode()

    # 自动起飞逻辑（可替换为 service）
    node.arm()
    node.start_offboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
        node.stop_offboard()
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()