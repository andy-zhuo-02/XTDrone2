"""
Multirotor Communication Module

This module provides communication functionalities for multirotor.

Author: Andy Zhuo
Email: zhuoan@stu.pku.edu.cn
Copyright (c) 2023 XTDrone2

All rights reserved. This code is licensed under the MIT License.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand
from xtd2_msgs.srv import XTD2Cmd

import sys
from tf_transformations import euler_from_quaternion

class MultirotorCommunication(Node):
    def __init__(self, vehicle_type, vehicle_id):
        self.vehicle_type = vehicle_type
        self.vehicle_id = int(vehicle_id)
        super().__init__(f'{vehicle_type}_{vehicle_id}_communication')

        self.OFFBOARD_STATE = "DISABLED"
        self.cmd = None

        # XTDrone2 Interface
        self.create_subscription(Pose, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd_pose_local_ned', self.cmd_pose_local_ned_callback, 10)
        self.create_subscription(Twist, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd_vel_ned', self.cmd_vel_ned_callback, 10)
        # self.cmd_accel_sub = self.create_subscription(Twist, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd_accel', self.cmd_accel_callback, 10)
        self.cmd_server = self.create_service(XTD2Cmd, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd', self.cmd_callback)

        # DDS Interface
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.dds_trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self.timer_ = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(f'{vehicle_type}_{vehicle_id} communication node started')
    
    def timer_callback(self):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        # Publish Offboard Control Mode
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if self.OFFBOARD_STATE == "POSE_LOCAL_NED":
            msg.position = True
            self.dds_trajectory_setpoint_pub.publish(self.cmd)
        elif self.OFFBOARD_STATE == "VEL_NED":
            msg.velocity = True
            self.dds_trajectory_setpoint_pub.publish(self.cmd)
        msg.timestamp = self.get_clock_microseconds()
        self.offboard_control_mode_pub.publish(msg)
    
    def get_clock_microseconds(self):
        t_ = self.get_clock().now().seconds_nanoseconds()
        return int(t_[0]*1e6 + t_[1]/1000)

    def cmd_pose_local_ned_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "POSE_LOCAL_NED"
        # Convert quaternion to euler angles
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [msg.position.x, msg.position.y, msg.position.z]
        cmd.yaw = yaw
        self.cmd = cmd
        
    def cmd_vel_ned_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "VEL_NED"
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        cmd.yawspeed = msg.angular.z
        self.cmd = cmd

    def cmd_accel_callback(self, msg):
        pass

    def cmd_callback(self, request, response):
        command = request.command
        if command == "ARM":
            self.arm()
            response.success = True
        elif command == "DISARM":
            self.disarm()
            response.success = True
        elif command == "HOVER":
            self.hover()
            self.OFFBOARD_STATE = "DISABLED"
            response.success = True
        elif command == "OFFBOARD":
            self.OFFBOARD_STATE = "ENABLED"
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            response.success = True
        else:
            self.get_logger().warn(f'Unknown command: {command}')       
            response.success = False 
        return response

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock_microseconds()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        # TODO：Check if the vehicle is already armed
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command send')
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command send')
    
    def hover(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_PAUSE_CONTINUE, 0.0)
        self.get_logger().info('Hover command send')

def main():
    rclpy.init(args=sys.argv)
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    multirotor_communication = MultirotorCommunication(vehicle_type, vehicle_id)
    rclpy.spin(multirotor_communication)
    multirotor_communication.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()