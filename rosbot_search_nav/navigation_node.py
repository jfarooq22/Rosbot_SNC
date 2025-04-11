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
        
        front_indices = list(range(angle_to_index(0), angle_to_index(45))) + \
                        list(range(angle_to_index(225), num_ranges))
        # Front: approx -45° to +45°
        front = min(valid_ranges([msg.ranges[i] for i in front_indices]), default=3.0)

         # Right: ~90°
        right_indices = range(angle_to_index(85), angle_to_index(95))
        right = min(valid_ranges([msg.ranges[i] for i in right_indices]), default=3.0)

        # Diagonal Right: 30°–60°
        diag_right_indices = range(angle_to_index(30), angle_to_index(60))
        diag_right = min(valid_ranges([msg.ranges[i] for i in diag_right_indices]), default=3.0)

        # Left: approx 80°–100°
        left_indices = range(angle_to_index(-85), angle_to_index(-95))
        left = min(valid_ranges([msg.ranges[i] for i in left_indices]), default=1.0)
        
        twist = Twist()

        if front < 0.8:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().warn("Obstacle ahead! Turning left")

        # Diagonal wall blocks a right turn — skip it
        elif right > 1.0 and diag_right < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.6
            self.get_logger().warn("Diagonal wall blocks right turn — turning left")

        # Too close to right wall
        elif right < 1.0:
            twist.linear.x = 0.0
            twist.angular.z = 0.8
            self.get_logger().warn("Too close to right wall — turning left")

        # Wall too far — gently curve right
        elif right > 1.0:
            twist.linear.x = 0.15
            twist.angular.z = -0.1

        # ✅ Good distance from wall — go straight
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0


        self.cmd_pub.publish(twist)
        # Debug info
        self.get_logger().info(
            f"[Ranges] Right: {right:.2f} | Diag→: {diag_right:.2f} | Front: {front:.2f} → Cmd: Lin {twist.linear.x:.2f}, Ang {twist.angular.z:.2f}"
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