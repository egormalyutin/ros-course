import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String


class FilePublisher(Node):
    content = None
    last_mtime = None

    def __init__(self):
        super().__init__("file_publisher")

        self.declare_parameter("path", Parameter.Type.STRING)
        self.path = self.get_parameter("path").get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(String, "content", qos)

        self.content_poller = self.create_timer(0.2, self.poll_content)

    def poll_content(self):
        try:
            mtime = os.path.getmtime(self.path)
        except FileNotFoundError:
            self.get_logger().warn(f"{self.path} does not exist")

        if mtime != self.last_mtime:
            with open(self.path, "r") as file:
                self.content = file.read()

            if self.last_mtime != None:
                self.get_logger().info(f"{self.path} modified")

            s = String()
            s.data = self.content
            self.pub.publish(s)

        self.last_mtime = mtime


def main():
    rclpy.init()
    node = FilePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
