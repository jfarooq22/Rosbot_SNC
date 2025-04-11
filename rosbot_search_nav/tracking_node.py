#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for /path_explore
        self.path_pub = self.create_publisher(Path, '/path_explore', 10)

        # Path data
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.path.poses = []

        # Timer to track robot position every second
        self.timer = self.create_timer(1.0, self.track_position)

    def track_position(self):
        try:
            # Get latest transform from base_link to map
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Create PoseStamped from transform
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            # Append to path
            self.path.poses.append(pose)
            self.path.header.stamp = pose.header.stamp

            # Publish
            self.path_pub.publish(self.path)
            self.get_logger().info(f'Pose added to path: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')

def main():
    rclpy.init()
    node = PathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
