#!/usr/bin/env python3
"""Simulates the state machine receiving vision data."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_vision_subscriber')
        self.sub = self.create_subscription(
            Point, '/erc/vision_targets', self.callback, 10)
        self.count = 0
        self.get_logger().info('Waiting for targets on /erc/vision_targets...')

    def callback(self, msg):
        self.count += 1
        if self.count % 10 == 0:  # Print every 10th message to avoid spam
            self.get_logger().info(
                f'[#{self.count}] Target: X={msg.x:+.3f}m  Y={msg.y:+.3f}m  ID={msg.z:.0f}')

def main():
    rclpy.init()
    node = TestSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()