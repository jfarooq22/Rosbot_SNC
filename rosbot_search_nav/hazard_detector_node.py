import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ObjectLoggerFromArray(Node):
    def __init__(self):
        super().__init__('object_logger_from_array')

        self.create_subscription(
            Float32MultiArray,
            '/objects',
            self.object_callback,
            10
        )

        self.id_to_label = {
            1: "Explosive",
            2: "Flammable Gas",
            3: "Radioactive",
            4: "Corrosive",
            100: "START"
        }

    def object_callback(self, msg):
        if len(msg.data) < 1:
            self.get_logger().info("🛑 No object detected.")
            return

        object_id = int(msg.data[0])
        label = self.id_to_label.get(object_id, "Unknown")

        self.get_logger().info(f"📸 Detected object ID: {object_id} → Label: {label}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLoggerFromArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
