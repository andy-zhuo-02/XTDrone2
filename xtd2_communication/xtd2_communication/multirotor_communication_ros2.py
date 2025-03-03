import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
from pyquaternion import Quaternion
import sys

class CommunicationNode(Node):
    def __init__(self, vehicle_type, vehicle_id):
        super().__init__(vehicle_type + '_' + vehicle_id + '_communication')
        
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.current_position = None
        self.current_yaw = 0
        self.hover_flag = 0
        self.coordinate_frame = 1
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = self.coordinate_frame
        self.arm_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.last_cmd = None

        # 设置QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # ROS2订阅者
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose',
            self.local_pose_callback,
            qos_profile
        )
        
        self.cmd_sub = self.create_subscription(
            String,
            f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd',
            self.cmd_callback,
            10
        )

        # 其他订阅者
        self.create_subscription(Pose, f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_pose_flu', 
                               self.cmd_pose_flu_callback, qos_profile)
        self.create_subscription(Pose, f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_pose_enu', 
                               self.cmd_pose_enu_callback, qos_profile)
        self.create_subscription(Twist, f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_vel_flu', 
                               self.cmd_vel_flu_callback, qos_profile)
        self.create_subscription(Twist, f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_vel_enu', 
                               self.cmd_vel_enu_callback, qos_profile)
        self.create_subscription(Twist, f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_accel_flu', 
                               self.cmd_accel_flu_callback, qos_profile)
        self.create_subscription(Twist, f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_accel_enu', 
                               self.cmd_accel_enu_callback, qos_profile)

        # ROS2发布者
        self.target_motion_pub = self.create_publisher(
            PositionTarget,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/setpoint_raw/local',
            qos_profile
        )

        # ROS2服务客户端
        self.arm_client = self.create_client(
            CommandBool,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/cmd/arming'
        )
        self.flight_mode_client = self.create_client(
            SetMode,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/set_mode'
        )
        self.set_param_client = self.create_client(
            ParamSet,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/param/set'
        )

        # 设置定时器
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)  # 30Hz

        # 设置参数
        while not self.set_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('参数服务不可用，等待中...')
        
        self.set_parameter()
        self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: communication initialized")

    def set_parameter(self):
        request = ParamSet.Request()
        request.param_id = "COM_RCL_EXCEPT"
        request.value = ParamValue(integer=4, real=0.0)
        self.set_param_client.call_async(request)

    def timer_callback(self):
        """主循环定时器回调"""
        self.target_motion_pub.publish(self.target_motion)

    def local_pose_callback(self, msg):
        self.current_position = msg.pose.position
        self.current_yaw = self.q2yaw(msg.pose.orientation)

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame
        
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz
        
        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        if(self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        if(self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        if(self.motion_type == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = 9
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
 
    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
        
    def cmd_vel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)  
 
    def cmd_vel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)    

    def cmd_accel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x,afy=msg.linear.y,afz=msg.linear.z,yaw_rate=msg.angular.z)    
            
    def cmd_accel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1 
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x,afy=msg.linear.y,afz=msg.linear.z,yaw_rate=msg.angular.z)    
            
    def hover_state_transition(self,x,y,z,w):
        if abs(x) > 0.02 or abs(y)  > 0.02 or abs(z)  > 0.02 or abs(w)  > 0.005:
            self.hover_flag = 0
            self.flight_mode = 'OFFBOARD'
        elif not self.flight_mode == "HOVER":
            self.hover_flag = 1
            self.flight_mode = 'HOVER'
            self.hover()

    def cmd_callback(self, msg):
        if msg.data == self.last_cmd or msg.data == '' or msg.data == 'stop controlling':
            return

        elif msg.data == 'ARM':
            self.arm_state = self.arm()
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: Armed {str(self.arm_state)}")

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: Armed {str(self.arm_state)}")

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: {msg.data}")

        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

        self.last_cmd = msg.data

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad
    
    def arm(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error(f"{self.vehicle_type}_{self.vehicle_id}: arming failed!")
            return False

    def disarm(self):
        request = CommandBool.Request()
        request.value = False
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error(f"{self.vehicle_type}_{self.vehicle_id}: disarming failed!")
            return False

    def hover(self):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(
            x=self.current_position.x,
            y=self.current_position.y,
            z=self.current_position.z,
            yaw=self.current_yaw
        )
        self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode}")

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
        else:
            request = SetMode.Request()
            request.custom_mode = self.flight_mode
            future = self.flight_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().mode_sent:
                self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode}")
                return True
            else:
                self.get_logger().error(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode} failed")
                return False

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print("Usage: ros2 run package_name node_name vehicle_type vehicle_id")
        return
        
    communication = CommunicationNode(sys.argv[1], sys.argv[2])
    
    try:
        rclpy.spin(communication)
    except KeyboardInterrupt:
        pass
    finally:
        communication.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
