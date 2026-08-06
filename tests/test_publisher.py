#!/usr/bin/env python3
"""Simulates the Simulink vision node publishing to /erc/vision_targets."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math, time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_vision_publisher')
        self.pub = self.create_publisher(Point, '/erc/vision_targets', 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.t = 0.0
        self.get_logger().info('Publishing test targets at 10 Hz on /erc/vision_targets')

    def tick(self):
        msg = Point()
        # Simulates a marker drifting in a circle
        msg.x = 0.3 * math.sin(self.t)   # X offset in meters
        msg.y = 0.3 * math.cos(self.t)   # Y offset in meters
        msg.z = 102.0                      # Marker ID
        self.pub.publish(msg)
        self.t += 0.1

def main():
    rclpy.init()
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()