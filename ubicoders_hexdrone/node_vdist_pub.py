import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('node_vdist_pub')
        self.publisher_ = self.create_publisher(Float32, '/vdist', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
        self.timer = self.create_timer(0.02, self.read_and_publish_data)
        

    def read_and_publish_data(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line is None:
                return
            try:
                float_value = float(line)
                msg = Float32(data=float_value)
                self.publisher_.publish(msg)
                #self.get_logger().info(f'Published: {float_value}')
            except:
                pass
            finally:
                pass
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
