import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from px4_msgs.msg import UbicodersMsgAutoControlSetpoint, VehicleAttitude, UbicodersMsgAtt, UbicodersMsgIps, UbicodersMsgDebug
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from .rotation_utils import quaternion_to_euler
from .auto_control_utils import AutoPosControl
np.set_printoptions(precision=4, suppress=True)

class AutoControlNode(Node):
    def __init__(self):
        super().__init__('node_auto_control')

        self._ips_subs = self.create_subscription(UbicodersMsgIps, '/fmu/in/ubicoders_msg_ips', self.ips_callback, 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # or ReliabilityPolicy.BEST_EFFORT
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._att_subs = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_callback, qos_profile)
        self._px4_debug_subs =self.create_subscription(UbicodersMsgDebug, '/fmu/out/ubicoders_msg_debug', self.debug_callback, qos_profile)
        self.publisher_ = self.create_publisher(UbicodersMsgAutoControlSetpoint, '/fmu/in/ubicoders_auto_control_setpoint', 10)
        self.timer = self.create_timer(0.02, self.read_and_publish_data)

        self.ips = np.zeros((3))
        self.quat = np.zeros((4))
        self.eul = np.zeros((3))
        self.auto_control = AutoPosControl()

    def debug_callback(self, msg):
        self.auto_control.update_throttle(msg.throttle)
        self.get_logger().info(f"Throttle: {msg.throttle}")       
        
    
    def ips_callback(self, msg):    
        self.ips = np.array([msg.ips_x, msg.ips_y, msg.ips_z])
        #self.get_logger().info('Ips: "%s"' % self.ips)
    
    def att_callback(self, msg):
        self.quat = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        self.eul = quaternion_to_euler(self.quat)
        # self.get_logger().info(f"Quat: {self.quat}")
        #self.get_logger().info(f"Eul: {self.eul*180/np.pi} deg")

    def read_and_publish_data(self):
        auto_control_msg = UbicodersMsgAutoControlSetpoint()
        auto_control_msg.thrust = self.auto_control.pz_sp
        auto_control_msg.roll = 0.0
        auto_control_msg.pitch = 0.0
        auto_control_msg.yaw = 0.0
        self.publisher_.publish(auto_control_msg)
        self.get_logger().info(f"Thrust: {auto_control_msg.thrust}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
