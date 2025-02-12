"""
Multirotor Keyboard Control Module

This module provides keyboard control for multirotor.

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
import termios
import tty
from numpy import clip

import argparse


class MultirotorKeyboardControl(Node):
    MSG = """
Control Your XTDrone2!
To all drones  (press g to control the leader)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        
   a    s    d       g       j    k    l
        x       v    b   n        

w/x : increase/decrease forward  
a/d : increase/decrease leftward 
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s/k : hover 
CTRL-C to quit
"""
    def __init__(self, model, id, namespace=""):
        if model.startswith("gz_"):  # 删除gz_前缀
            model = model[3:]
        self.model = model
        self.id = int(id)
        self.namespace = namespace if namespace else f'{model}_{id}'

        super().__init__(f'{self.namespace}_keyboard_control')

        # Parameters
        self.declare_parameter("max_linear_velocity", 20.0)
        self.MAX_LINEAR = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self.declare_parameter("max_angular_velocity", 3.0)
        self.MAX_ANGULAR = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        self.declare_parameter("linear_velocity_step", 0.01)
        self.LINEAR_STEP = self.get_parameter("linear_velocity_step").get_parameter_value().double_value
        self.declare_parameter("angular_velocity_step", 0.01)
        self.ANGULAR_STEP = self.get_parameter("angular_velocity_step").get_parameter_value().double_value

        # XTDrone2 Interface
        self.cmd_pose_local_ned_publisher = self.create_publisher(Pose, f'/xtdrone2/{self.namespace}/cmd_pose_local_ned', 10)
        self.cmd_vel_ned_publisher = self.create_publisher(Twist, f'/xtdrone2/{self.namespace}/cmd_vel_flu', 10)
        self.cmd_client = self.create_client(XTD2Cmd, f'/xtdrone2/{self.namespace}/cmd')

        # Variables
        self.forward = 0.0
        self.leftward = 0.0
        self.upward = 0.0
        self.angular = 0.0

        self.loop()
    
    def loop(self):
        self.print_msg()
        while rclpy.ok():
            key = self.getkey()
            self.print_msg()

            # CTRL-C to quit
            if (key == '\x03'):  
                break

            # Linear velocity
            elif (key == 'w'):
                self.forward += self.LINEAR_STEP
                self.print_msg()
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (self.forward, self.leftward, self.upward, self.angular))
            elif (key == 'x'):
                self.forward -= self.LINEAR_STEP
                self.print_msg()
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (self.forward, self.leftward, self.upward, self.angular))
            elif (key == 'a'):
                self.leftward += self.LINEAR_STEP
                self.print_msg()
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (self.forward, self.leftward, self.upward, self.angular))
            elif (key == 'd'):
                self.leftward -= self.LINEAR_STEP
                self.print_msg()
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (self.forward, self.leftward, self.upward, self.angular))
            elif (key == 'i'):
                self.upward += self.LINEAR_STEP
                self.print_msg()
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (self.forward, self.leftward, self.upward, self.angular))
            elif (key == ','):
                self.upward -= self.LINEAR_STEP
                self.print_msg()
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (self.forward, self.leftward, self.upward, self.angular))
            # Angular velocity
            elif (key == 'j'):
                self.angular += self.ANGULAR_STEP
                self.print_msg()
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (self.forward, self.leftward, self.upward, self.angular))
            elif (key == 'l'):
                self.angular -= self.ANGULAR_STEP
                self.print_msg()
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (self.forward, self.leftward, self.upward, self.angular))
            
            # Actions
            else:
                req = XTD2Cmd.Request()
                if (key == 'r'):  # Return home
                    req.command = "RTL"
                    self.get_logger().info('Return home...')
                elif (key == 't'):  # Arm
                    req.command = "ARM"
                    self.get_logger().info('Arming...')
                elif (key == 'y'):  # Disarm
                    req.command = "DISARM"
                    self.get_logger().info('Disarming...')
                elif (key == 'v'):  # Takeoff
                    req.command = "TAKEOFF"
                    self.get_logger().info('Taking off...')
                elif (key == 'n'):  # Land
                    req.command = "LAND"
                    self.get_logger().info('Landing...')
                elif (key == 'b'):  # Offboard
                    req.command = "OFFBOARD"
                    self.get_logger().info('Switch to offboard mode')
                elif (key == 's' or 'k'):  # Hover
                    req.command = "HOVER"
                    self.get_logger().info('Hovering...')
                    self.forward = 0.0
                    self.leftward = 0.0
                    self.upward = 0.0
                    self.angular = 0.0
                else:
                    self.get_logger().warn('Invalid key')    
                    continue
                
                future = self.cmd_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3)
                if future.done():
                    self.get_logger().debug(f'Command response: {future.result().success}')
                else:
                    self.get_logger().warn('Service call failed %r' % (future.exception(),))


            # control signal clipping
            self.forward = clip(self.forward, -self.MAX_LINEAR, self.MAX_LINEAR)
            self.leftward = clip(self.leftward, -self.MAX_LINEAR, self.MAX_LINEAR)
            self.upward = clip(self.upward, -self.MAX_LINEAR, self.MAX_LINEAR)
            self.angular = clip(self.angular, -self.MAX_ANGULAR, self.MAX_ANGULAR)

            # Control
            msg = Twist()
            msg.linear.x = self.forward
            msg.linear.y = self.leftward
            msg.linear.z = self.upward
            msg.angular.z = self.angular
            self.cmd_vel_ned_publisher.publish(msg)


    
    def getkey(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def print_msg(self):
        print(self.MSG)
            

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='XTDrone2 Multirotor Communication Node')

    parser.add_argument('--model', type=str, help='Model name', required=True)
    parser.add_argument('--id', type=int, help='Vehicle id, should be unique in same model', required=True)
    parser.add_argument('--namespace', type=str, help='ROS namespace, {{model}}_{{id}} by default', required=False, default="")

    args, unknown = parser.parse_known_args()

    multirotor_keyboard_control = MultirotorKeyboardControl(args.model, args.id, args.namespace)
    multirotor_keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()