from datetime import datetime

from scipy.spatial.transform import Rotation
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def angle_norm(angle):
    return (angle + np.pi) % (np.pi * 2) - np.pi


class MinigunShoot(Node):
    def __init__(self):
        super().__init__("circle")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.barrel_pub = self.create_publisher(Float64, "barrel_cmd_vel", 10)

        self.target = 0
        self.started_firing = None

        self.odom_sub = self.create_subscription(Odometry, "odom", self.on_odom, 10)

    def log(self, *args):
        self.get_logger().info(" ".join(map(str, args)))

    def roll_barrel(self, speed):
        f = Float64()
        f.data = float(speed)
        self.barrel_pub.publish(f)

    def cmd_vel(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.vel_pub.publish(t)

    def on_odom(self, odom):
        q = odom.pose.pose.orientation
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        angle = r.as_euler("xyz")[-1]

        diff = angle_norm(self.target - angle)
        if abs(diff) > 0.05:
            self.cmd_vel(0, min(diff * 3, 3))
        else:
            self.cmd_vel(0, 0)

            if self.started_firing is None:
                self.started_firing = datetime.now()

            if (datetime.now() - self.started_firing).total_seconds() > 3:
                self.started_firing = None
                self.roll_barrel(0)
                self.target = self.target + np.pi / 2
            else:
                self.cmd_vel(-0.2, 0)
                self.roll_barrel(40)


def main(args=None):
    rclpy.init(args=args)
    node = MinigunShoot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
