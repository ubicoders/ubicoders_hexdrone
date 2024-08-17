import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from px4_msgs.msg import UbicodersMsgSubs, UbicodersMsgIps


class HexdroneIPSPublisher(Node):

    def __init__(self):
        super().__init__('hexdrone_ips_node')
        qos_profile = QoSProfile(depth=10)
        self.hexdrone_test_pub = self.create_publisher(UbicodersMsgSubs, '/fmu/in/ubicoders_msg_subs', qos_profile)
        self.hexdrone_ips_pub = self.create_publisher(UbicodersMsgIps, '/fmu/in/ubicoders_msg_ips', qos_profile)
        #self.timer = self.create_timer(0.02, self.publish_msg)
        self.timer = self.create_timer(0.02, self.publish_ips)

    def publish_ips(self):
        msg = UbicodersMsgIps()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.ips_x = 1.0
        msg.ips_y = 2.0
        msg.ips_z = 3.0
        msg.vdist = 4.0
        self.hexdrone_ips_pub.publish(msg)    
        self.get_logger().info(f"Publishing: {msg.ips_x}, {msg.ips_y}, {msg.ips_z}, {msg.vdist}")

        

    def publish_msg(self):
        msg = UbicodersMsgSubs()
        # pack time in uint64
        msg.timestamp = self.get_clock().now().nanoseconds // 1000 
        msg.odd_number_input = 55
        self.hexdrone_test_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.odd_number_input)
        


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