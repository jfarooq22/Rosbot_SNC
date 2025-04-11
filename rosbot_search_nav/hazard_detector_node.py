# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import PointStamped
# from visualization_msgs.msg import Marker
# import tf2_ros
# import math

# class ObjectLoggerFromArray(Node):
#     def __init__(self):
#         super().__init__('object_logger_from_array')

#         self.create_subscription(Float32MultiArray, '/objects', self.object_callback, 10)
#         self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

#         self.marker_pub = self.create_publisher(Marker, '/hazards', 10)

#         self.laser_data = None
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.id_to_label = {
#             0: "Unknown",
#             1: "Explosive",
#             2: "Flammable Gas",
#             3: "Non-Flammable Gas",
#             4: "Dangerous When Wet",
#             5: "Flammable Solid",
#             6: "Spontaneously Combustible",
#             7: "Oxidizer",
#             8: "Organic Peroxide",
#             9: "Inhalation Hazard",
#             10: "Poison",
#             11: "Radioactive",
#             12: "Corrosive"
#         }

#     def laser_callback(self, msg):
#         self.laser_data = msg

#     def object_callback(self, msg):
#         if len(msg.data) < 1:
#             self.get_logger().info("🛑 No object detected.")
#             return

#         object_id = int(msg.data[0])
#         label = self.id_to_label.get(object_id, "Unknown")
#         self.get_logger().info(f"📸 Detected object ID: {object_id} → Label: {label}")

#         if not self.laser_data:
#             self.get_logger().warn("⚠️ No laser scan data available.")
#             return

#         center_index = len(self.laser_data.ranges) // 2
#         distance = self.laser_data.ranges[center_index]

#         if math.isinf(distance) or math.isnan(distance):
#             self.get_logger().warn("❌ Invalid laser distance (inf or nan).")
#             return

#         self.get_logger().info(f"📏 Distance to object: {distance:.2f} meters")

#         # Point in robot frame
#         point = PointStamped()
#         point.header.stamp = self.get_clock().now().to_msg()
#         point.header.frame_id = self.laser_data.header.frame_id
#         point.point.x = distance
#         point.point.y = 0.0
#         point.point.z = 0.0

#         try:
#             point_in_map = self.tf_buffer.transform(point, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
#             x = point_in_map.point.x
#             y = point_in_map.point.y
#             self.get_logger().info(f"🗺️ Object in map frame: x={x:.2f}, y={y:.2f} (Label: {label})")

#             # 🔴 Create and publish marker
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "hazards"
#             marker.id = object_id  # Use object ID as marker ID
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.pose.position = point_in_map.point
#             marker.pose.orientation.w = 1.0
#             marker.scale.x = 0.2
#             marker.scale.y = 0.2
#             marker.scale.z = 0.2
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.color.a = 1.0

#             self.marker_pub.publish(marker)
#             self.get_logger().info(f"📍 Marker published for {label} (ID: {object_id})")

#         except Exception as e:
#             self.get_logger().warn(f"⚠️ TF transform failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectLoggerFromArray()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()










import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from find_object_2d.msg import ObjectsStamped
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import math

class ObjectLoggerFromStamped(Node):
    def __init__(self):
        super().__init__('object_logger_from_stamped')

        self.create_subscription(ObjectsStamped, '/objectsStamped', self.object_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/hazards', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.id_to_label = {
            0: "Unknown",
            1: "Explosive",
            2: "Flammable Gas",
            3: "Non-Flammable Gas",
            4: "Dangerous When Wet",
            5: "Flammable Solid",
            6: "Spontaneously Combustible",
            7: "Oxidizer",
            8: "Organic Peroxide",
            9: "Inhalation Hazard",
            10: "Poison",
            11: "Radioactive",
            12: "Corrosive"
        }

    def object_callback(self, msg):
        data = msg.objects.data
        if len(data) < 4:
            self.get_logger().info("🛑 No object detected.")
            return

        object_id = int(data[0])
        label = self.id_to_label.get(object_id, "Unknown")
        self.get_logger().info(f"📸 Detected object ID: {object_id} → Label: {label}")

        # Get 3D position from message (x, y, z) at index 3, 4, 5
        try:
            point = PointStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = msg.header.frame_id
            point.point.x = data[3]
            point.point.y = data[4]
            point.point.z = data[5]

            self.get_logger().info(f"📏 Position in camera frame: x={point.point.x:.2f}, y={point.point.y:.2f}, z={point.point.z:.2f}")

            # Transform point to map frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            point_in_map = do_transform_point(point, transform)

            x = point_in_map.point.x
            y = point_in_map.point.y
            self.get_logger().info(f"🗺️ Object in map frame: x={x:.2f}, y={y:.2f} (Label: {label})")

            # Create and publish marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = msg.header.stamp
            marker.ns = "hazards"
            marker.id = object_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point_in_map.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            self.get_logger().info(f"📍 Marker published for {label} (ID: {object_id})")

        except Exception as e:
            self.get_logger().warn(f"⚠️ TF transform failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLoggerFromStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
