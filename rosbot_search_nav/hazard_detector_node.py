import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import math

class ObjectLoggerFromArray(Node):
    def __init__(self):
        super().__init__('object_logger_from_array')

        self.create_subscription(
            Float32MultiArray,
            '/objects',
            self.object_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.laser_data = None  # stores latest laser scan

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

    def laser_callback(self, msg):
        self.laser_data = msg  # store latest scan

    def object_callback(self, msg):
        if len(msg.data) < 1:
            self.get_logger().info("🛑 No object detected.")
            return

        object_id = int(msg.data[0])
        label = self.id_to_label.get(object_id, "Unknown")
        self.get_logger().info(f"📸 Detected object ID: {object_id} → Label: {label}")

        if not self.laser_data:
            self.get_logger().warn("⚠️ No laser scan data available.")
            return

        center_index = len(self.laser_data.ranges) // 2
        distance = self.laser_data.ranges[center_index]

        if math.isinf(distance) or math.isnan(distance):
            self.get_logger().warn("❌ Invalid laser distance (inf or nan).")
            return

        self.get_logger().info(f"📏 Estimated distance to object: {distance:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLoggerFromArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
