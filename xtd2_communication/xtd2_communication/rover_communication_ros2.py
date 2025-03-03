import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
import sys

class Communication(Node):

    def __init__(self, vehicle_id):
        super().__init__('rover_' + vehicle_id + '_communication')
        
        self.vehicle_type = 'rover'
        self.vehicle_id = vehicle_id
        self.local_pose = None
        self.target_motion = PositionTarget()
        self.arm_state = False
        self.motion_type = 1
        self.flight_mode = None
        self.mission = None
        self.coordinate_frame = 1

        # ROS2订阅者
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose',
            self.local_pose_callback,
            1)
            
        self.mavros_sub = self.create_subscription(
            State,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/state',
            self.mavros_state_callback,
            1)
            
        self.cmd_pose_flu_sub = self.create_subscription(
            Pose,
            f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_pose_flu',
            self.cmd_pose_flu_callback,
            1)
            
        self.cmd_pose_enu_sub = self.create_subscription(
            Pose,
            f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_pose_enu',
            self.cmd_pose_enu_callback,
            1)
            
        self.cmd_vel_flu_sub = self.create_subscription(
            Twist,
            f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_vel_flu',
            self.cmd_vel_flu_callback,
            1)
            
        self.cmd_vel_enu_sub = self.create_subscription(
            Twist,
            f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_vel_enu',
            self.cmd_vel_enu_callback,
            1)
            
        self.cmd_sub = self.create_subscription(
            String,
            f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd',
            self.cmd_callback,
            3)

        # ROS2发布者
        self.target_motion_pub = self.create_publisher(
            PositionTarget,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/setpoint_raw/local',
            1)

        # ROS2服务客户端
        self.arm_client = self.create_client(
            CommandBool,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/cmd/arming')
            
        self.flight_mode_client = self.create_client(
            SetMode,
            f'{self.vehicle_type}_{self.vehicle_id}/mavros/set_mode')

        # 创建定时器，30Hz
        self.timer = self.create_timer(1/30, self.timer_callback)

        self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: communication initialized")

    def timer_callback(self):
        """主循环定时器回调"""
        self.target_motion_pub.publish(self.target_motion)

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, yaw=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame 

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz

        target_raw_pose.yaw = yaw

        if(self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
        if(self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = 9
        self.motion_type = 0
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=0)
 
    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=0)

    def cmd_vel_flu_callback(self, msg):
        self.coordinate_frame = 8
        self.motion_type = 1
        self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.angular.z,vz=msg.linear.z,yaw=0)
 
    def cmd_vel_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.motion_type = 1
        self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.angular.z,vz=msg.linear.z,yaw=0)

    async def arm(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        await future
        return future.result().success

    async def disarm(self):
        request = CommandBool.Request()
        request.value = False
        future = self.arm_client.call_async(request)
        await future
        return future.result().success

    async def flight_mode_switch(self):
        request = SetMode.Request()
        request.custom_mode = self.flight_mode
        future = self.flight_mode_client.call_async(request)
        await future
        if future.result().mode_sent:
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode}")
            return True
        else:
            self.get_logger().error(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode} failed")
            return False

    async def cmd_callback(self, msg):
        if msg.data == '':
            return

        elif msg.data == 'ARM':
            self.arm_state = await self.arm()
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: Armed {self.arm_state}")

        elif msg.data == 'DISARM':
            self.arm_state = not await self.disarm()
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: Armed {self.arm_state}")

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            self.get_logger().info(f"{self.vehicle_type}_{self.vehicle_id}: {msg.data}")

        else:
            self.flight_mode = msg.data
            await self.flight_mode_switch()

def main(args=None):
    rclpy.init(args=args)
    communication = Communication(sys.argv[1])
    rclpy.spin(communication)
    communication.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 