"""Convert ROS 2 Twist commands to the Sassy Uno C,throttle,steering protocol."""
import os, time, serial, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('sassy_cmd_vel_bridge')
        self.declare_parameter('serial_port',os.getenv('SASSY_SERIAL_PORT','/dev/leia-uno'))
        self.declare_parameter('max_linear_mps',.20); self.declare_parameter('max_angular_rps',1.2)
        self.ser=serial.Serial(self.get_parameter('serial_port').value,115200,timeout=.1); time.sleep(1.5)
        self.last=time.monotonic(); self.sub=self.create_subscription(Twist,'/cmd_vel',self.on_cmd,10); self.timer=self.create_timer(.1,self.watchdog)
    def on_cmd(self,msg):
        linear=max(-1.,min(1.,msg.linear.x/self.get_parameter('max_linear_mps').value))
        angular=max(-1.,min(1.,msg.angular.z/self.get_parameter('max_angular_rps').value))
        self.ser.write(f'C,{round(linear*255)},{round(angular*255)}\n'.encode()); self.last=time.monotonic()
    def watchdog(self):
        if time.monotonic()-self.last>.25:self.ser.write(b'S\n')
    def destroy_node(self):
        self.ser.write(b'S\n'); self.ser.close(); super().destroy_node()
def main(args=None):
    rclpy.init(args=args); node=CmdVelBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node(); rclpy.shutdown()
