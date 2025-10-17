import asyncio
import time
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action import ActionServer

from action_cleaning_robot_msgs.action import CleaningTask

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


def angle_norm(angle):
    return (angle + np.pi) % (np.pi * 2) - np.pi


CELL_SIZE = 0.1
ROBOT_SIZE = 0.2
SPEED = 4


def follow_point(xy, theta, goal):
    xy_diff = goal - xy

    xy_dist = np.linalg.norm(xy_diff)

    abs_angle_to_goal = np.arctan2(xy_diff[1], xy_diff[0])
    rel_angle_to_goal = angle_norm(theta - abs_angle_to_goal)

    # rotate towards the goal
    if abs(rel_angle_to_goal) > 0.1 and abs(xy_dist) > 0.1:
        return 0, rel_angle_to_goal * SPEED * 4

    # both rotate towards the goal and move
    elif abs(xy_dist) > 0.1:
        return SPEED, rel_angle_to_goal * SPEED * 4

    else:
        return 0, 0


class VacuumController:
    def __init__(self):
        self.points = np.empty((0, 2))
        self.goal = None

    def wipe_and_pick_goal(self, pose: Pose):
        xy = np.array([pose.x, pose.y])
        rel = self.points - xy

        # calculate distances
        dists = np.linalg.norm(rel, axis=1)

        # wipe occupied points
        vacant = dists >= ROBOT_SIZE
        self.points = self.points[vacant]
        rel = rel[vacant]
        dists = dists[vacant]

        if len(self.points) == 0:
            return

        # find nearest + least rotation point
        abs_angles = np.arctan2(rel[:, 1], rel[:, 0])
        rel_angles = angle_norm(pose.theta - abs_angles)

        scores = dists + abs(rel_angles) * 4

        goal = self.points[np.argmin(scores)]

        return goal

    def step(self, pose: Pose):
        self.goal = self.wipe_and_pick_goal(pose)
        if self.goal is None:
            return 0, 0

        return self.control(pose, self.goal)

    def control(self, pose: Pose, goal: np.ndarray):
        return follow_point(np.array([pose.x, pose.y]), pose.theta, goal)


def draw_circle(cx, cy, r):
    points = []

    for y in np.arange(0, 11, CELL_SIZE):
        for x in np.arange(0, 11, CELL_SIZE):
            if (x - cx) ** 2 + (y - cy) ** 2 <= r**2:
                points.append((x, y))

    return np.array(points)


def draw_square(cx, cy, r):
    points = []

    x0, y0 = cx - r / 2, cy - r / 2
    x1, y1 = cx + r / 2, cy + r / 2

    for y in np.arange(0, 11, CELL_SIZE):
        for x in np.arange(0, 11, CELL_SIZE):
            if x >= x0 and x <= x1 and y >= y0 and y <= y1:
                points.append((x, y))

    return np.array(points)


class CleaningActionServerNode(Node):
    def __init__(self):
        super().__init__("cleaning_action_server")

        self.vacuum = VacuumController()

        self.pose = None

        self.pose_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.on_pose,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.timer = self.create_timer(0.01, self.step)

        self.cleaning_action_server = ActionServer(
            self,
            CleaningTask,
            "clean",
            self.execute_action,
        )

    def on_pose(self, pose):
        self.pose = pose

    def cmd_vel(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(-angular)
        self.cmd_vel_pub.publish(t)

    def step(self):
        if self.pose is None:
            return

        cmd = self.vacuum.step(self.pose)

        if cmd is not None:
            self.cmd_vel(*cmd)

    def execute_action(self, goal_handle):
        rate = self.create_rate(30)

        req = goal_handle.request

        while self.pose is None:
            rate.sleep()

        if req.task_type == "clean_square":
            points = draw_square(self.pose.x, self.pose.y, req.area_size)
        elif req.task_type == "clean_circle":
            points = draw_circle(self.pose.x, self.pose.y, req.area_size)
        elif req.task_type == "return_home":
            points = np.array([[req.target_x, req.target_y]])
        else:
            goal_handle.abort()
            return CleaningTask.Result(
                success=False,
                cleaned_points=0,
                total_distance=0.0,
            )

        orig_n = len(points)

        self.vacuum.points = points

        prev_pose = None
        total_dist = 0.0

        while len(self.vacuum.points) > 0:
            fb = CleaningTask.Feedback()

            cleared = orig_n - len(self.vacuum.points)

            fb.current_cleaned_points = cleared
            fb.progress_percent = int(cleared / orig_n * 100)
            fb.current_x = self.pose.x
            fb.current_y = self.pose.y
            goal_handle.publish_feedback(fb)

            if prev_pose is not None:
                dx = prev_pose.x - self.pose.x
                dy = prev_pose.y - self.pose.y
                dist = (dx**2 + dy**2) ** 0.5
                total_dist += dist

            prev_pose = self.pose

            rate.sleep()

        goal_handle.succeed()
        return CleaningTask.Result(
            success=True,
            cleaned_points=orig_n,
            total_distance=total_dist,
        )


def main():
    rclpy.init()
    node = CleaningActionServerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
