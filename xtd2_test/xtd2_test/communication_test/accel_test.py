import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from xtd2_msgs.srv import XTD2Cmd
from px4_msgs.msg import VehicleLocalPosition
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class TestAccelerationPublisher(Node):
    def __init__(self, namespace):
        super().__init__('test_acceleration_publisher')
        self.publisher_ = self.create_publisher(Twist, f'/xtdrone2/{namespace}/cmd_accel_ned', 10)
        self.cmd_client = self.create_client(XTD2Cmd, f'/xtdrone2/{namespace}/cmd')
        
        # 订阅PX4的本地位置信息
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            f'/{namespace}/fmu/out/vehicle_local_position',
            self.position_callback,
            QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability)
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Test Acceleration Publisher Node started')

        # 状态变量
        self.is_takeoff_complete = False
        self.takeoff_start_time = None
        self.target_height = 10.0  # 目标起飞高度
        self.height_threshold = 0.5  # 高度阈值，认为到达目标高度
        self.cur_vehicle_local_position = None

        # Arm the drone
        self.arm_drone()

        # Takeoff the drone
        self.takeoff_drone()

    def position_callback(self, msg):
        if not self.is_takeoff_complete:
            self.cur_vehicle_local_position = msg
            if self.takeoff_start_time is None:
                self.takeoff_start_time = self.get_clock().now()
                self.get_logger().info('Starting takeoff...')
            
            # 检查是否达到目标高度
            if abs(-msg.z - self.target_height) < self.height_threshold:  # 因为msg.z是负值
                self.is_takeoff_complete = True
                self.get_logger().info('Takeoff completed, switching to offboard mode...')
                self.switch_to_offboard()
            else:
                # 如果起飞时间超过30秒，记录警告
                if (self.get_clock().now() - self.takeoff_start_time) > Duration(seconds=30):
                    self.get_logger().warn('Takeoff taking longer than expected...')

    def arm_drone(self):
        req = XTD2Cmd.Request()
        req.command = "ARM"
        self.get_logger().info('Arming...')
        
        future = self.cmd_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if future.done():
            self.get_logger().info(f'Arm command response: {future.result().success}')
        else:
            self.get_logger().warn('Arm service call failed %r' % (future.exception(),))

    def takeoff_drone(self):
        req = XTD2Cmd.Request()
        req.command = "TAKEOFF"
        self.get_logger().info('Taking off...')
        future = self.cmd_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if future.done():
            self.get_logger().info(f'Takeoff command response: {future.result().success}')
        else:
            self.get_logger().warn('Takeoff service call failed %r' % (future.exception(),))

    def switch_to_offboard(self):
        req = XTD2Cmd.Request()
        req.command = "OFFBOARD"
        self.get_logger().info('Switching to offboard mode...')
        future = self.cmd_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if future.done():
            self.get_logger().info(f'Offboard command response: {future.result().success}')
        else:
            self.get_logger().warn('Offboard service call failed %r' % (future.exception(),))   

    def timer_callback(self):
        if not self.is_takeoff_complete:
            return
            
        msg = Twist()
        msg.linear.x = 1.0  # 设置加速度的x分量
        msg.linear.y = 0.0  # 设置加速度的y分量
        msg.linear.z = 0.0  # 设置加速度的z分量
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing acceleration command: x=1.0, y=0.0, z=0.0')

def main(args=None):
    rclpy.init(args=args)
    namespace = 'x500_0'  # 替换为实际的命名空间
    test_publisher = TestAccelerationPublisher(namespace)
    rclpy.spin(test_publisher)
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()