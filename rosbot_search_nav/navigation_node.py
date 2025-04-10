#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("NavigationNode (Wall Follower) started — right-hand rule")
    def scan_callback(self, msg):
        num_ranges = len(msg.ranges)
        degrees_per_index = 270 / num_ranges
        def angle_to_index(angle):
            return int(angle / degrees_per_index)
        # Filter valid (non-zero, non-infinite) distances
        def valid_ranges(range_list):
            return [r for r in range_list if r > 0.0 and r < float('inf')]
        # Front: approx -20° to +20° (wraps from 250°–270° + 0°–20°)
        front_indices = list(range(angle_to_index(0), angle_to_index(20))) + list(range(angle_to_index(250), num_ranges))
        front = min(valid_ranges([msg.ranges[i] for i in front_indices]), default=3.0)
        # Right: approx 250°–270°
        right_indices = range(angle_to_index(250), num_ranges)
        right = min(valid_ranges([msg.ranges[i] for i in right_indices]), default=1.0)
        # Left: approx 80°–100°
        left_indices = range(angle_to_index(80), angle_to_index(100))
        left = min(valid_ranges([msg.ranges[i] for i in left_indices]), default=1.0)
        twist = Twist()
        # :octagonal_sign: Emergency case — too close to wall
        if right < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.8
            self.get_logger().warn("Too close to wall! Emergency turn")
        # :no_entry: Obstacle in front
        elif front < 0.4:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        # :leftwards_arrow_with_hook: No wall on the right
        elif right > 0.7:
            twist.linear.x = 0.15
            twist.angular.z = -0.3
        # :arrow_right_hook: Too close to wall on right
        elif right < 0.5:
            twist.linear.x = 0.15
            twist.angular.z = 0.5
        # :white_tick: Ideal distance from wall
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        # Debug info
        self.get_logger().info(
            f"[Ranges] Right: {right:.2f} | Front: {front:.2f} | Left: {left:.2f} → Cmd: Lin {twist.linear.x:.2f}, Ang {twist.angular.z:.2f}"
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
if __name__ == '__main__':
    main()