"""
Multirotor Communication Module

This module provides communication functionalities for multirotor.

Author: Andy Zhuo
Email: zhuoan@stu.pku.edu.cn

All rights reserved. This code is licensed under the MIT License.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition
from xtd2_msgs.srv import XTD2Cmd

import sys
import math
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class MultirotorCommunication(Node):
    def __init__(self, vehicle_type, vehicle_id):
        self.vehicle_type = vehicle_type
        self.vehicle_id = int(vehicle_id)
        super().__init__(f'{vehicle_type}_{vehicle_id}_communication')

        self.OFFBOARD_STATE = "DISABLED"
        self.cmd = None
        self.cur_vehicle_local_position = None
        self.cur_vehicle_global_position = None
        self.init_vehicle_local_position = None
        self.init_vehicle_global_position = None

        # XTDrone2 Interface
        self.create_subscription(Pose, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd_pose_local_ned', self.cmd_pose_local_ned_callback, 10)
        self.create_subscription(Pose, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd_pose_local_flu', self.cmd_pose_local_flu_callback, 10)
        self.create_subscription(Twist, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd_vel_ned', self.cmd_vel_ned_callback, 10)
        # self.cmd_accel_sub = self.create_subscription(Twist, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd_accel', self.cmd_accel_callback, 10)
        self.create_subscription(Twist, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd_vel_flu', self.cmd_vel_flu_callback, 10)
        self.cmd_server = self.create_service(XTD2Cmd, f'/xtdrone2/{vehicle_type}_{vehicle_id}/cmd', self.cmd_callback)

        # DDS Interface
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))
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
        
        # Publish Command
        self.cmd.timestamp = self.get_clock_microseconds()
        if self.OFFBOARD_STATE == "POSE_LOCAL_NED":
            msg.position = True
            self.dds_trajectory_setpoint_pub.publish(self.cmd)
        elif self.OFFBOARD_STATE == "POSE_LOCAL_FLU":
            msg.position = True
            self.dds_trajectory_setpoint_pub.publish(self.cmd)
        elif self.OFFBOARD_STATE == "VEL_NED":
            msg.position = False
            msg.velocity = True
            self.dds_trajectory_setpoint_pub.publish(self.cmd)
        elif self.OFFBOARD_STATE == "VEL_FLU":
            msg.position = False
            msg.velocity = True
            self.dds_trajectory_setpoint_pub.publish(self.cmd)
        
        msg.timestamp = self.get_clock_microseconds()  
        self.offboard_control_mode_pub.publish(msg)
    
    def vehicle_local_position_callback(self, msg):
        if self.init_vehicle_local_position is None:
            self.init_vehicle_local_position = msg
        self.cur_vehicle_local_position = msg

    def vehicle_global_position_callback(self, msg):
        if self.init_vehicle_global_position is None:
            self.init_vehicle_global_position = msg
        self.cur_vehicle_global_position = msg
    
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
        
    def cmd_pose_local_flu_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "POSE_LOCAL_FLU"
        # Convert quaternion to euler angles
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        theta = self.cur_vehicle_local_position.heading

        # Transfer position from FLU to NED, msg.position.xyz is flu, respectively
        p_n = msg.position.x * math.cos(theta) + msg.position.y * math.sin(theta)
        p_e = msg.position.x * math.sin(theta) - msg.position.y * math.cos(theta)
        p_d = -msg.position.z
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [p_n, p_e, p_d]
        cmd.yaw = self.init_vehicle_local_position.heading + yaw
        self.cmd = cmd

    def cmd_vel_ned_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "VEL_NED"
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [math.nan, math.nan, math.nan]
        cmd.velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        cmd.yaw = math.nan
        cmd.yawspeed = msg.angular.z
        self.cmd = cmd

    def cmd_vel_flu_callback(self, msg):
        """ Let a be heading angle
        [[cos a , sin a,  0],     [f]   [n]
         [-sin a, cos a,  0],  *  [l] = [e]
         [0     , 0    , -1]]     [u]   [d]
        """
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        self.OFFBOARD_STATE = "VEL_FLU"
        theta = self.cur_vehicle_local_position.heading 
        # Transform velocity from FLU to NED, msg.linear.xyz is flu, respectively
        v_n = msg.linear.x * math.cos(theta) + msg.linear.y * math.sin(theta)
        v_e = msg.linear.x * math.sin(theta) - msg.linear.y * math.cos(theta)
        v_d = -msg.linear.z
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position = [math.nan, math.nan, math.nan]
        cmd.velocity = [v_n, v_e, v_d]
        cmd.yaw = math.nan
        cmd.yawspeed = -msg.angular.z
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
            response.success = True
        elif command == "OFFBOARD":
            self.OFFBOARD_STATE = "ENABLED"
            self.get_logger().info('Offboard command send')
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            response.success = True
        elif command == "TAKEOFF":
            self.takeoff()
            response.success = True
        elif command == "LAND":
            self.land()
            response.success = True
        elif command == "RTL":
            self.rtl()
            response.success = True
        else:
            self.get_logger().warn(f'Unknown command: {command}')       
            response.success = False 
        return response

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock_microseconds()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
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
        # self.OFFBOARD_STATE = "DISABLED"
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_PAUSE_CONTINUE, 0.0)
        self.get_logger().info('Hover command send')

    def takeoff(self):
        # | Minimum pitch (if airspeed sensor present), desired pitch without sensor
        # | Empty
        # | Empty
        # | Yaw angle (if magnetometer present), ignored without magnetometer
        # | Latitude
        # | Longitude
        # | Altitude
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=0.0, param4=self.cur_vehicle_local_position.heading, param5=self.cur_vehicle_local_position.ref_lat, param6=self.cur_vehicle_local_position.ref_lon, param7=10.0)
        self.get_logger().info("Take off command send.")

    def land(self):
        # | Empty
        # | Empty
        # | Empty
        # | Desired yaw angle.
        # | Latitude
        # | Longitude
        # | Altitude|
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param4=self.cur_vehicle_local_position.heading, param5=self.cur_vehicle_global_position.lat, param6=self.cur_vehicle_global_position.lon, param7=0.0)
        self.get_logger().info("Land command send.")
    
    def rtl(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("RTL command send.")

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