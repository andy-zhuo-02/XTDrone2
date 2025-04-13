"""
Rovor Keyboard Control Module

This module provides keyboard control for rover.

Author: Wang Lile
Email: 15726686898@163.com
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


class RoverKeyboardControl(Node):
    MSG = """
Control Your XTDrone2 Rover!
----------------------------
Movement:
    w/s : forward / backward
    a/d : left turn / right turn
    j/l : rotate left / rotate right
    space : stop

Commands:
    t : ARM
    y : DISARM
    r : Return Home
    b : Offboard Mode
    v : Start Mission / Takeoff
    n : Land / Stop Mission

CTRL-C to quit
"""

    def __init__(self, model, id, namespace=""):
        if model.startswith("gz_"):
            model = model[3:]
        self.model = model
        self.id = int(id)
        self.namespace = namespace if namespace else f'{model}_{id}'

        super().__init__(f'{self.namespace}_keyboard_control')

        # Parameters
        self.declare_parameter("max_linear_velocity", 3.0)
        self.MAX_LINEAR = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self.declare_parameter("max_angular_velocity", 2.0)
        self.MAX_ANGULAR = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        self.declare_parameter("linear_step", 0.1)
        self.LINEAR_STEP = self.get_parameter("linear_step").get_parameter_value().double_value
        self.declare_parameter("angular_step", 0.1)
        self.ANGULAR_STEP = self.get_parameter("angular_step").get_parameter_value().double_value

        # Publisher and service
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/xtdrone2/{self.namespace}/cmd_vel', 10)
        self.cmd_client = self.create_client(XTD2Cmd, f'/xtdrone2/{self.namespace}/cmd')

        # Control variables
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        self.loop()

    def loop(self):
        self.print_msg()
        while rclpy.ok():
            key = self.getkey()

            if key == '\x03':  # CTRL-C
                break

            elif key == 'w':
                self.linear_vel += self.LINEAR_STEP
            elif key == 's':
                self.linear_vel -= self.LINEAR_STEP
            elif key == 'a':
                self.angular_vel += self.ANGULAR_STEP
            elif key == 'd':
                self.angular_vel -= self.ANGULAR_STEP
            elif key == 'j':
                self.angular_vel += self.ANGULAR_STEP
            elif key == 'l':
                self.angular_vel -= self.ANGULAR_STEP
            elif key == ' ':
                self.linear_vel = 0.0
                self.angular_vel = 0.0
                self.get_logger().info("Stop movement.")

            # Commands
            elif key in ['r', 't', 'y', 'b', 'v', 'n']:
                req = XTD2Cmd.Request()
                cmd_map = {
                    'r': "RTL",
                    't': "ARM",
                    'y': "DISARM",
                    'b': "OFFBOARD",
                    'v': "TAKEOFF",
                    'n': "LAND"
                }
                req.command = cmd_map[key]
                self.get_logger().info(f"Sending command: {req.command}")
                future = self.cmd_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3)
                if future.done():
                    result = future.result()
                    self.get_logger().info(f"Command result: {result.success}")
                else:
                    self.get_logger().warn("Service call failed.")
                continue

            else:
                self.get_logger().warn("Invalid key.")
                continue

            # Clip values
            self.linear_vel = clip(self.linear_vel, -self.MAX_LINEAR, self.MAX_LINEAR)
            self.angular_vel = clip(self.angular_vel, -self.MAX_ANGULAR, self.MAX_ANGULAR)

            # Publish velocity
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_vel
            twist_msg.angular.z = self.angular_vel
            self.cmd_vel_publisher.publish(twist_msg)

            print(f"Current velocity => linear: {self.linear_vel:.2f}, angular: {self.angular_vel:.2f}")
            self.print_msg()

    def getkey(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def print_msg(self):
        print(self.MSG)


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='XTDrone2 Rover Keyboard Control Node')

    parser.add_argument('--model', type=str, required=True)
    parser.add_argument('--id', type=int, required=True)
    parser.add_argument('--namespace', type=str, default="")

    args, unknown = parser.parse_known_args()

    node = RoverKeyboardControl(args.model, args.id, args.namespace)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()