#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')  # ✅ Node name here

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("NavigationNode (Wall Follower) started — right-hand rule")

    def scan_callback(self, msg):
        # Simple ranges for front, right, left
        front = min(min(msg.ranges[0:20] + msg.ranges[-20:]), 3.0)
        right = min(msg.ranges[260:300]) if len(msg.ranges) > 300 else 1.0
        left = min(msg.ranges[60:100]) if len(msg.ranges) > 100 else 1.0

        twist = Twist()

        # 🛑 Emergency case — too close to wall
        if right < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.8
            self.get_logger().warn("Too close to wall! Emergency turn")
        # ⛔ Obstacle in front
        elif front < 0.4:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        # ↩️ No wall on the right
        elif right > 0.7:
            twist.linear.x = 0.15
            twist.angular.z = -0.3
        # ↪️ Too close to wall on right
        elif right < 0.5:
            twist.linear.x = 0.15
            twist.angular.z = 0.5
        # ✅ Ideal distance from wall
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        # Debug info
        self.get_logger().info(
            f"[Ranges] Right: {right:.2f} | Front: {front:.2f} → Cmd: Lin {twist.linear.x:.2f}, Ang {twist.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
