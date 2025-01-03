# offboard_control_demo.py -- adapted from https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp

import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        self.offboard_setpoint_counter_ = 0 

        self.timer_ = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        if self.offboard_setpoint_counter_ == 10:  
            # Change to offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)

            # Arm
            self.arm()
        
        # Offboard control mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Stop the counter after reaching 10 setpoints
        if self.offboard_setpoint_counter_ <= 10:
            self.offboard_setpoint_counter_ += 1

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        t_ = self.get_clock().now().seconds_nanoseconds()
        msg.timestamp = int(t_[0]*1e6 + t_[1]/1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        t_ = self.get_clock().now().seconds_nanoseconds()
        msg.timestamp = int(t_[0]*1e6 + t_[1]/1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        t_ = self.get_clock().now().seconds_nanoseconds()
        msg.timestamp = int(t_[0]*1e6 + t_[1]/1000)
        msg.position = [0.0, 0.0, -5.0]
        msg.yaw = -3.14
        self.trajectory_setpoint_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command send')
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command send')




def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()