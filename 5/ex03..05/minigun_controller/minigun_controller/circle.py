import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class MinigunCircle(Node):
    def __init__(self):
        super().__init__("circle")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.barrel_pub = self.create_publisher(Float64, "barrel_cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        t = Twist()
        t.linear.x = 0.5
        t.angular.z = 1.0
        self.pub.publish(t)
        f = Float64()
        f.data = 10.0
        self.barrel_pub.publish(f)


def main(args=None):
    rclpy.init(args=args)
    node = MinigunCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
