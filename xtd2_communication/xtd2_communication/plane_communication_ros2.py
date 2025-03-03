import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
import sys

class Communication(Node):

    def __init__(self, vehicle_id):
        super().__init__('plane_'+vehicle_id+"_communication")
        
        # 基础属性设置
        self.vehicle_type = 'plane'
        self.vehicle_id = vehicle_id
        self.local_pose = None
        self.target_motion = PositionTarget()
        self.arm_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None

        # 设置QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS2订阅者
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose",
            self.local_pose_callback,
            qos_profile
        )

        self.cmd_pose_flu_sub = self.create_subscription(
            Pose,
            "/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_flu",
            self.cmd_pose_flu_callback,
            qos_profile
        )

        self.cmd_pose_enu_sub = self.create_subscription(
            Pose,
            "/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu",
            self.cmd_pose_enu_callback,
            qos_profile
        )

        self.cmd_sub = self.create_subscription(
            String,
            "/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd",
            self.cmd_callback,
            10
        )

        # ROS2发布者
        self.target_motion_pub = self.create_publisher(
            PositionTarget,
            self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local",
            qos_profile
        )

        # ROS2服务客户端
        self.arm_client = self.create_client(
            CommandBool,
            self.vehicle_type+'_'+self.vehicle_id+"/mavros/cmd/arming"
        )
        self.mode_client = self.create_client(
            SetMode,
            self.vehicle_type+'_'+self.vehicle_id+"/mavros/set_mode"
        )

        # 创建定时器，30Hz
        self.timer = self.create_timer(1/30, self.timer_callback)
        
        self.get_logger().info("Communication initialized")

    def timer_callback(self):
        """定时器回调函数，替代ROS1中的rate.sleep()"""
        self.target_motion_pub.publish(self.target_motion)

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def construct_target(self, x=0, y=0, z=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame

        if self.coordinate_frame == 1:
            target_raw_pose.position.x = x
            target_raw_pose.position.y = y
            target_raw_pose.position.z = z
        else:
            target_raw_pose.position.x = -y
            target_raw_pose.position.y = x
            target_raw_pose.position.z = z

        if self.mission == 'takeoff':
            target_raw_pose.type_mask = 4096
        elif self.mission == 'land':
            target_raw_pose.type_mask = 8192
        elif self.mission == 'loiter':
            target_raw_pose.type_mask = 12288
        else:
            target_raw_pose.type_mask = 16384

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = 9
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)
 
    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)

    def cmd_callback(self, msg):
        if msg.data == '':
            return

        elif msg.data == 'ARM':
            self.arm_state = self.arm()
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: Armed {self.arm_state}")

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: Armed {self.arm_state}")

        elif msg.data in ['takeoff', 'land', 'loiter', 'idle']:
            self.mission = msg.data
            self.get_logger().info(self.mission)
        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

    async def arm(self):
        """异步服务调用"""
        request = CommandBool.Request()
        request.value = True
        
        future = self.arm_client.call_async(request)
        await future
        
        if future.result().success:
            return True
        else:
            self.get_logger().error(f"{self.vehicle_type}_{self.vehicle_id}: arming failed!")
            return False

    async def disarm(self):
        request = CommandBool.Request()
        request.value = False
        
        future = self.arm_client.call_async(request)
        await future
        
        if future.result().success:
            return True
        else:
            self.get_logger().error(f"{self.vehicle_type}_{self.vehicle_id}: disarming failed!")
            return False

    async def flight_mode_switch(self):
        request = SetMode.Request()
        request.custom_mode = self.flight_mode
        
        future = self.mode_client.call_async(request)
        await future
        
        if future.result().mode_sent:
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode}")
            return True
        else:
            self.get_logger().error(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode} failed")
            return False

def main(args=None):
    rclpy.init(args=args)
    communication = Communication(sys.argv[1])
    
    try:
        rclpy.spin(communication)
    except KeyboardInterrupt:
        pass
    finally:
        communication.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 