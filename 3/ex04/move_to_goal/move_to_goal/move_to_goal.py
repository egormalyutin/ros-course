from math import atan2, pi
import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose


def angle_diff(a, b):
    return (b - a + pi) % (pi * 2) - pi


MAX_LINEAR_SPEED = 2.0
MAX_ANGULAR_SPEED = pi

LINEAR_MUL = 2
ANGULAR_MUL = 2


def abs_clamp(x, abs_max):
    if x > abs_max:
        return abs_max
    elif x < -abs_max:
        return -abs_max
    else:
        return x


class MoveToGoalNode(Node):
    def __init__(self, x, y, theta):
        super().__init__("move_to_goal")

        self.target_x = x
        self.target_y = y
        self.target_theta = theta

        self.complete = False

        self.pose_sub = self.create_subscription(
            Pose, "/turtle1/pose", self.on_pose, 10
        )

        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.timer = self.create_timer(0.005, self.step)

        self.pose = None

    def on_pose(self, pose: Pose):
        self.pose = pose

    def step(self):
        if self.pose is None:
            return

        dx = self.pose.x - self.target_x
        dy = self.pose.y - self.target_y

        d_norm = (dx**2 + dy**2) ** 0.5

        angle_to_target = atan2(dy, dx) - pi
        d_angle_to_target = angle_diff(self.pose.theta, angle_to_target)
        d_pose_angle = angle_diff(self.pose.theta, self.target_theta)

        linear_vel = 0.0
        angular_vel = 0.0

        # rotate towards the goal
        if abs(d_angle_to_target) > 0.3 and abs(d_norm) > 0.1:
            angular_vel = d_angle_to_target

        # both rotate towards the goal and move
        elif abs(d_norm) > 0.1:
            angular_vel = d_angle_to_target
            linear_vel = d_norm

        # at point, adjust the pose
        elif abs(d_pose_angle) > 0.1:
            angular_vel = d_pose_angle

        # else stop moving
        else:
            self.complete = True

        t = Twist()
        t.angular.z = abs_clamp(ANGULAR_MUL * angular_vel, MAX_ANGULAR_SPEED)
        t.linear.x = abs_clamp(LINEAR_MUL * linear_vel, MAX_LINEAR_SPEED)
        self.vel_pub.publish(t)


def main():
    rclpy.init()

    assert len(sys.argv) == 4
    _, x, y, theta = sys.argv

    node = MoveToGoalNode(float(x), float(y), float(theta))

    while rclpy.ok() and not node.complete:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
