import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestAccelerationPublisher(Node):
    def __init__(self, namespace):
        super().__init__('test_acceleration_publisher')
        self.publisher_ = self.create_publisher(Twist, f'/xtdrone2/{namespace}/cmd_accel_ned', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Test Acceleration Publisher Node started')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0  # 设置加速度的x分量
        msg.linear.y = 0.0  # 设置加速度的y分量
        msg.linear.z = 0.0  # 设置加速度的z分量
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing acceleration command: x=1.0, y=0.0, z=0.0')

def main(args=None):
    rclpy.init(args=args)
    namespace = 'x500_0'  # 替换为实际的命名空间
    test_publisher = TestAccelerationPublisher(namespace)
    rclpy.spin(test_publisher)
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()