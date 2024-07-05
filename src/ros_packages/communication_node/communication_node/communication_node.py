import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')
        self.subscription = self.create_subscription(String, 'processing_results', self.listener_callback, 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)  # Adjust port and baud rate as necessary

    def listener_callback(self, msg):
        self.send_data(msg.data)

    def send_data(self,data):
        self.ser.write((data + '\n').encode())  # Send data to Arduino with newline character

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()