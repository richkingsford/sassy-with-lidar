import os
import serial
import rclpy
from rclpy.node import Node
from sassy_msgs.msg import DriveCmd

class SerialBridge(Node):
    def __init__(self):
        super().__init__('sassy_serial_bridge')
        self.declare_parameter('serial_port', os.getenv('SASSY_SERIAL_PORT', '/dev/ttyACM0'))
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('serial_port').value
        self.ser = serial.Serial(port, int(self.get_parameter('baud_rate').value), timeout=0.1)
        self.create_subscription(DriveCmd, '/sassy/drive_cmd', self.on_cmd, 10)
        self.get_logger().info(f'Connected to Sassy Uno on {port}')
    def send(self, line): self.ser.write((line + '\n').encode())
    def on_cmd(self, msg):
        if msg.stop: self.send('S'); return
        t = max(-255, min(255, round(msg.throttle * 255)))
        s = max(-255, min(255, round(msg.steering * 255)))
        self.send(f'C,{t},{s}')
    def destroy_node(self):
        try: self.send('S'); self.ser.close()
        finally: super().destroy_node()
def main(args=None):
    rclpy.init(args=args); node = SerialBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
