import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np


class MinigunScanMove(Node):
    def __init__(self):
        super().__init__("scan_move")

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def log(self, *args):
        self.get_logger().info(" ".join(map(str, args)))

    def cmd_vel(self, linear, angular=0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_vel_pub.publish(t)

    def on_scan(self, scan: LaserScan):
        data = np.array(scan.ranges)[::-1]
        region = [0.4, 0.6]
        region_px = [
            int(len(data) * region[0]),
            int(len(data) * region[1]),
        ]

        crop = data[region_px[0] : region_px[1]]
        crop = crop[np.isfinite(crop)]
        if len(crop) == 0:
            mean_dist = 1000
        else:
            mean_dist = np.mean(crop)

        if mean_dist > 0.2:
            self.cmd_vel(0.5)
        else:
            self.cmd_vel(0)

        # data_finite = data[np.isfinite(data)]
        # min = np.min(data_finite)
        # max = np.max(data_finite)
        # normalized = (data - min) / (max - min)
        # normalized = np.where(np.isfinite(data), normalized, 1)
        # normalized = (normalized * 255).astype(np.uint8)
        # img = normalized.reshape(1, -1).repeat(400, axis=0)
        # cv2.imshow("img", img)
        # cv2.waitKey(1)
        # self.log(img.shape)
        # print(scan)


def main(args=None):
    rclpy.init(args=args)
    node = MinigunScanMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
