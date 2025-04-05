#!/usr/bin/env python  

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros
import tf2_geometry_msgs

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('drive_speed', 0.2),
                ('rotate_speed', 0.3),
                ('loop_interval', 1.0)
            ]
        )
        self.drive_speed = self.get_parameter('drive_speed').value
        self.rotate_speed = self.get_parameter('rotate_speed').value

        # Publisher: status and cmd_vel
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: trigger_start
        self.start_sub = self.create_subscription(Empty, '/trigger_start', self.start_callback, 10)

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.exploring = False
        self.timer = self.create_timer(self.get_parameter('loop_interval').value, self.loop)

        self.get_logger().info("NavigationNode initialized. Waiting for /trigger_start...")
        self.exploring = True
        self.publish_status("Exploring (Auto-Start)")
        self.loop

    def start_callback(self, msg):
        if not self.exploring:
            self.get_logger().info("Start trigger received! Beginning exploration.")
            self.exploring = True
            self.publish_status("Exploring")

    def loop(self):
        if not self.exploring:
            # return
            self.exploring = True
            self.publish_status("Exploring (Auto-Start)")

        # Step 1: Move the robot in a wiggle pattern (you can replace this later with Explore Lite)
        cmd = Twist()
        cmd.linear.x = self.drive_speed
        cmd.angular.z = self.rotate_speed
        self.cmd_vel_pub.publish(cmd)

        # Step 2: Lookup robot position in map frame
        try:
            time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.1)
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', time, timeout=rclpy.duration.Duration(seconds=0.05)
            )
            poseS = PoseStamped()
            poseS.header.frame_id = 'base_link'
            poseS.pose.position.x = 0.0
            poseS.pose.position.y = 0.0
            poseS.pose.orientation.w = 1.0

            transformed_pose = self.tf_buffer.transform(poseS, 'map')
            self.get_logger().info(f"[Robot Map Position] x: {transformed_pose.pose.position.x:.2f}, y: {transformed_pose.pose.position.y:.2f}")

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"TF Error: {ex}")

    def publish_status(self, state):
        msg = String()
        msg.data = state
        self.status_pub.publish(msg)
        self.get_logger().info(f"[Status] {state}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
