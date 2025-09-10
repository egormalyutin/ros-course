import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TextToCmdVel(Node):
    def __init__(self):
        super().__init__("text_to_cmd_vel")

        self.sub = self.create_subscription(String, "cmd_text", self.handler, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

    def handler(self, msg):
        if msg.data == "turn_left":
            t = Twist()
            t.angular.z = 1.5
            self.pub.publish(t)

        elif msg.data == "turn_right":
            t = Twist()
            t.angular.z = -1.5
            self.pub.publish(t)

        elif msg.data == "move_forward":
            t = Twist()
            t.linear.x = 1.
            self.pub.publish(t)

        elif msg.data == "move_backward":
            t = Twist()
            t.linear.x = -1.
            self.pub.publish(t)

        else:
            self.get_logger().warn("Unknown command received: " + msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()