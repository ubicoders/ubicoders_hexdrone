import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from px4_msgs.msg import UbicodersMsgSubs, UbicodersMsgIps
from geometry_msgs.msg import Vector3
import numpy as np

class HexdroneIPSPublisher(Node):

    def __init__(self):
        super().__init__('hexdrone_ips_node')
        qos_profile = QoSProfile(depth=10)
        
        self.hexdrone_ips_pub = self.create_publisher(UbicodersMsgIps, '/fmu/in/ubicoders_msg_ips', qos_profile)
        self.timer = self.create_timer(0.02, self.publish_ips_vdist)
        
        self.aruco_subs = self.create_subscription(
                Vector3,
                '/cam/obj_pose',
                self.subs_ips,
                qos_profile)
        
        self.ips = Vector3()
    
    def subs_ips(self, msg):
        self.ips = msg

    def publish_ips_vdist(self):
        msg = UbicodersMsgIps()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.ips_x = self.ips.x if not np.isnan(self.ips.x) else 0.0
        msg.ips_y = self.ips.y if not np.isnan(self.ips.y) else 0.0
        msg.ips_z = self.ips.z if not np.isnan(self.ips.z) else 0.0
        msg.vdist = 0.0
        self.hexdrone_ips_pub.publish(msg)    
        self.get_logger().info(f"Publishing: {msg.ips_x}, {msg.ips_y}, {msg.ips_z}, {msg.vdist}")

        



def main(args=None):
    rclpy.init(args=args)
    node = HexdroneIPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()