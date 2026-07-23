#!/usr/bin/env python3
import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PathFollower(Node):
    def __init__(self, args):
        super().__init__("hexapod_path_demo")
        self.args = args
        self.pose = None
        self.waypoints = []
        self.waypoint_index = 0
        self.finished = False
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_timer(0.05, self.control_tick)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (position.x, position.y, yaw)
        if not self.waypoints:
            candidate_waypoints = self.build_waypoints(*self.pose)
            if any(
                abs(x) > self.args.arena_limit or abs(y) > self.args.arena_limit
                for x, y in candidate_waypoints
            ):
                self.get_logger().error(
                    f"{self.args.shape} exceeds safe arena limit "
                    f"+/-{self.args.arena_limit:.2f} m; refusing to move"
                )
                self.publish_stop()
                self.finished = True
                return
            self.waypoints = candidate_waypoints
            self.get_logger().info(
                f"tracking {self.args.shape}: {len(self.waypoints)} waypoints "
                f"from ({position.x:.3f}, {position.y:.3f})"
            )

    def local_to_world(self, points, start_x, start_y, start_yaw):
        cosine = math.cos(start_yaw)
        sine = math.sin(start_yaw)
        return [
            (
                start_x + cosine * local_x - sine * local_y,
                start_y + sine * local_x + cosine * local_y,
            )
            for local_x, local_y in points
        ]

    def build_waypoints(self, start_x, start_y, start_yaw):
        radius = self.args.radius
        samples = self.args.samples

        if self.args.shape == "goto":
            return [(self.args.target_x, self.args.target_y)]
        elif self.args.shape == "circle":
            local = [
                (
                    radius * math.sin(2.0 * math.pi * index / samples),
                    radius * (1.0 - math.cos(2.0 * math.pi * index / samples)),
                )
                for index in range(1, samples + 1)
            ]
        elif self.args.shape == "figure8":
            local = [
                (
                    2.0 * radius * math.sin(2.0 * math.pi * index / samples),
                    radius
                    * math.sin(2.0 * math.pi * index / samples)
                    * math.cos(2.0 * math.pi * index / samples),
                )
                for index in range(1, samples + 1)
            ]
        elif self.args.shape == "square":
            side = 2.0 * radius
            local = [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
        else:
            local = [
                (
                    4.0 * radius * index / samples,
                    radius * math.sin(2.0 * math.pi * index / samples),
                )
                for index in range(1, samples + 1)
            ]

        return self.local_to_world(local, start_x, start_y, start_yaw)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def control_tick(self):
        if self.pose is None or self.finished:
            return

        x, y, yaw = self.pose
        while self.waypoint_index < len(self.waypoints):
            goal_x, goal_y = self.waypoints[self.waypoint_index]
            distance = math.hypot(goal_x - x, goal_y - y)
            threshold = (
                self.args.tolerance
                if self.waypoint_index == len(self.waypoints) - 1
                else self.args.lookahead
            )
            if distance >= threshold:
                break
            self.waypoint_index += 1

        if self.waypoint_index >= len(self.waypoints):
            self.publish_stop()
            self.finished = True
            start_x, start_y = self.waypoints[-1]
            closure_error = math.hypot(x - start_x, y - start_y)
            self.get_logger().info(
                f"path complete at ({x:.3f}, {y:.3f}), "
                f"final error={closure_error:.3f} m"
            )
            return

        goal_x, goal_y = self.waypoints[self.waypoint_index]
        distance = math.hypot(goal_x - x, goal_y - y)
        bearing = math.atan2(goal_y - y, goal_x - x)
        heading_error = wrap_angle(bearing - yaw)

        command = Twist()
        command.angular.z = clamp(2.2 * heading_error, -self.args.max_turn, self.args.max_turn)
        if abs(heading_error) < 1.0:
            speed_scale = max(0.20, math.cos(heading_error))
            command.linear.x = min(self.args.speed, max(0.035, 1.2 * distance)) * speed_scale
        self.cmd_pub.publish(command)


def parse_args():
    parser = argparse.ArgumentParser(description="Run a closed-loop hexapod path demo.")
    parser.add_argument(
        "shape",
        choices=("goto", "circle", "figure8", "square", "s_curve"),
        help="relative path shape, or absolute goto target",
    )
    parser.add_argument("--target-x", type=float, default=0.0, help="goto world X coordinate")
    parser.add_argument("--target-y", type=float, default=0.0, help="goto world Y coordinate")
    parser.add_argument("--radius", type=float, default=0.20, help="shape radius/scale in metres")
    parser.add_argument("--speed", type=float, default=0.08, help="maximum forward command")
    parser.add_argument("--max-turn", type=float, default=0.55, help="maximum yaw command")
    parser.add_argument("--samples", type=int, default=40, help="curve waypoint count")
    parser.add_argument("--lookahead", type=float, default=0.055, help="curve lookahead in metres")
    parser.add_argument("--tolerance", type=float, default=0.025, help="waypoint tolerance in metres")
    parser.add_argument(
        "--arena-limit",
        type=float,
        default=0.75,
        help="absolute safe X/Y centre limit in metres",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = PathFollower(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for _ in range(5):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
