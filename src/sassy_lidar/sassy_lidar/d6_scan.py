"""Publish COIN-D6 packets as sensor_msgs/LaserScan."""
import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class D6Scan(Node):
    def __init__(self):
        super().__init__('sassy_d6_scan')
        self.declare_parameter('port','/dev/ttyTHS1'); self.declare_parameter('baud',230400)
        self.declare_parameter('frame_id','laser'); self.declare_parameter('topic','/scan')
        self.pub=self.create_publisher(LaserScan,self.get_parameter('topic').value,10)
        self.ser=serial.Serial(self.get_parameter('port').value,int(self.get_parameter('baud').value),timeout=.05)
        self.buf=bytearray(); self.scan=[]; self.previous=None
        self.timer=self.create_timer(.005,self.poll)
    def poll(self):
        self.buf.extend(self.ser.read(4096))
        while True:
            i=self.buf.find(b'\xaa\x55\x00\x19')
            if i<0: self.buf[:]=self.buf[-3:]; return
            if len(self.buf)<i+85: del self.buf[:i]; return
            p=bytes(self.buf[i:i+85]); del self.buf[:i+85]
            start=(int.from_bytes(p[4:6],'little')>>1)/64.; end=(int.from_bytes(p[6:8],'little')>>1)/64.
            if end<start: end+=360.
            if self.previous is not None and start+5<self.previous and self.scan:
                self.publish(); self.scan=[]
            self.previous=start
            for n in range(25):
                o=10+3*n; d=int.from_bytes(p[o+1:o+3],'little')/1000.
                a=(start+(end-start)*n/24.)%360.
                if .05<=d<=12: self.scan.append((a,d))
    def publish(self):
        msg=LaserScan(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id=self.get_parameter('frame_id').value
        msg.angle_min=0.; msg.angle_max=2*math.pi; msg.angle_increment=math.radians(.5); msg.range_min=.05; msg.range_max=12.
        msg.scan_time=.1; msg.time_increment=msg.scan_time/720.; msg.ranges=[float('inf')]*720
        for degrees,distance in self.scan:
            index=int(degrees*2)%720; msg.ranges[index]=min(msg.ranges[index],distance)
        self.pub.publish(msg)
    def destroy_node(self):
        self.ser.close(); super().destroy_node()
def main(args=None):
    rclpy.init(args=args); node=D6Scan()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node(); rclpy.shutdown()
