#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("Wall follower node started (right-hand rule)")

    def scan_callback(self, msg):
        # Simple sectors
        front = min(min(msg.ranges[0:20] + msg.ranges[-20:]), 3.0)
        right = min(msg.ranges[260:300]) if len(msg.ranges) > 300 else 1.0
        left = min(msg.ranges[60:100]) if len(msg.ranges) > 100 else 1.0

        twist = Twist()

        if front < 0.4:
            # Obstacle ahead → turn left
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        elif right > 0.6:
            # No wall on the right → turn right slightly
            twist.linear.x = 0.1
            twist.angular.z = -0.3
        elif right < 0.3:
            # Too close to right wall → turn left slightly
            twist.linear.x = 0.1
            twist.angular.z = 0.3
        else:
            # Wall at ideal distance → go forward
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
