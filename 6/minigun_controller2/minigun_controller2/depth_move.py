import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class DepthMoveSubscriber(Node):
    def __init__(self):
        super().__init__("depth_move")
        self.subscription = self.create_subscription(
            Image, "/depth", self.listener_callback, 10
        )
        self.br = CvBridge()

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def log(self, *args):
        self.get_logger().info(" ".join(map(str, args)))

    def cmd_vel(self, linear, angular=0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_vel_pub.publish(t)

    def listener_callback(self, msg):
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        depth_array = np.array(cv_image, dtype=np.float32).clip(min=0, max=10)

        h, w = depth_array.shape

        s = 0.3
        l, r = 0.5 - s / 2, 0.5 + s / 2

        region = depth_array[int(h * l) : int(h * r), int(w * l) : int(w * r)]

        if np.mean(region) > 0.2:
            self.cmd_vel(1)
        else:
            self.cmd_vel(0)


def main(args=None):
    rclpy.init(args=args)
    node = DepthMoveSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
