#!/usr/bin/env python3
"""Drive the dynamic course once, and record what the robot knew and what it did.

    ros2 run vica_description obstacle_trial --out /tmp/trial.csv

[TEST ONLY] Sends one Nav2 goal and publishes /initialpose. It never touches
/cmd_vel -- the controller under test drives, which is the point.

What it is for
--------------
Three numbers a guide robot's safety case needs, none of which a pass/fail
result gives you:

    감지거리        how far away the first reading of the walker arrives
    감속 시작거리   how far away the commanded speed starts coming down
    정지거리        how far the robot still travels after that

They are recorded rather than computed here. This writes one row per sample of
everything that could matter, and obstacle_report.py works out the distances
from it together with the simulator's own log of where the walker actually was.
Splitting it that way means a trial can be re-analysed without being re-driven,
which matters when a pass costs five minutes of GPU.

Why the walker's position is not in this file
---------------------------------------------
Because ROS does not know it. The walker is moved inside Isaac by play_stage,
which writes VICA_WALK_LOG; publishing its pose would be inventing a sensor the
robot does not have, and the robot's own view of it is exactly what is being
measured. The two logs share a simulation clock and are joined on it.
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range

# The forward cone the "nearest thing ahead" column is taken over. Half of it
# either side of straight ahead. Wide enough to hold a person stepping in from
# the side at a couple of metres, narrow enough that the corridor walls do not
# fill it: at 45 degrees a 2.0 m corridor's wall is 1.4 m away, which is
# further than anything this trial is about.
FORWARD_CONE_DEG = 45.0
SAMPLE_HZ = 20.0


class Trial(Node):
    def __init__(self, spec, out_path, limit):
        super().__init__("obstacle_trial")
        self.spec = spec
        self.out_path = out_path
        self.limit = limit
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.odom = None
        self.cmd = (0.0, 0.0)
        self.scan_ahead = float("inf")
        self.us = {"front_left": float("inf"), "front_right": float("inf")}

        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        self.create_subscription(LaserScan, "/scan", self._scan, sensor_qos)
        for side in ("front_left", "front_right"):
            self.create_subscription(
                Range, f"/ultrasonic/{side}",
                lambda m, s=side: self._range(m, s), 10)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.rows = []

    # -- callbacks ---------------------------------------------------------
    def _odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                     m.twist.twist.linear.x, m.twist.twist.angular.z)

    def _cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def _scan(self, m):
        half = math.radians(FORWARD_CONE_DEG) / 2.0
        best = float("inf")
        for i, r in enumerate(m.ranges):
            if r != r or math.isinf(r) or r <= m.range_min:
                continue
            a = m.angle_min + i * m.angle_increment
            a = (a + math.pi) % (2 * math.pi) - math.pi
            if abs(a) <= half and r < best:
                best = r
        self.scan_ahead = best

    def _range(self, m, side):
        # A probe with nothing in front of it reports its maximum, which is a
        # reading and not a detection. Recorded as-is; the report decides.
        self.us[side] = m.range

    # -- driving -----------------------------------------------------------
    def set_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        for _ in range(6):
            self.pose_pub.publish(msg)
            self._spin(0.2)

    def _spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _sample(self):
        if self.odom is None:
            return
        t, x, y, yaw, vx, wz = self.odom
        self.rows.append({
            "t": round(t, 3), "x": round(x, 4), "y": round(y, 4),
            "yaw": round(yaw, 4), "odom_vx": round(vx, 4), "odom_wz": round(wz, 4),
            "cmd_vx": round(self.cmd[0], 4), "cmd_wz": round(self.cmd[1], 4),
            "scan_ahead": round(self.scan_ahead, 4),
            "us_left": round(self.us["front_left"], 4),
            "us_right": round(self.us["front_right"], 4),
        })

    def run(self):
        sx, sy = self.spec["start"]
        gx, gy = self.spec["goal"]
        if not self.ac.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("navigate_to_pose 서버가 없습니다")
            return "no-server"
        self.set_pose(sx, sy, 0.0)
        self._spin(2.0)

        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x = float(gx)
        g.pose.pose.position.y = float(gy)
        g.pose.pose.orientation.w = 1.0

        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        if not fut.done() or not fut.result().accepted:
            return "rejected"
        handle = fut.result()
        result_fut = handle.get_result_async()

        period = 1.0 / SAMPLE_HZ
        started = time.monotonic()
        nxt = 0.0
        while time.monotonic() - started < self.limit:
            rclpy.spin_once(self, timeout_sec=0.02)
            el = time.monotonic() - started
            if el >= nxt:
                self._sample()
                nxt += period
            if result_fut.done():
                break
        if not result_fut.done():
            return "timeout"
        status = result_fut.result().status
        return "reached" if status == GoalStatus.STATUS_SUCCEEDED else "failed"

    def write(self, outcome):
        os.makedirs(os.path.dirname(os.path.abspath(self.out_path)), exist_ok=True)
        if self.rows:
            with open(self.out_path, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)
        meta = {
            "outcome": outcome,
            "samples": len(self.rows),
            "goal": self.spec["goal"],
            "start": self.spec["start"],
            "walker": self.spec.get("walker"),
            "low_box": self.spec.get("low_box"),
            "trigger_m": float(os.environ.get("VICA_WALK_TRIGGER_M", "4.0")),
            "walk_speed": float(os.environ.get("VICA_WALK_SPEED", "1.2")),
        }
        with open(os.path.splitext(self.out_path)[0] + ".json", "w") as fh:
            json.dump(meta, fh, indent=2)
        print(f"  {outcome}, {len(self.rows)} 표본 -> {self.out_path}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--spec", required=True, help="course json from the builder")
    ap.add_argument("--out", required=True, help="csv to write")
    ap.add_argument("--limit", type=float, default=180.0, help="seconds")
    args = ap.parse_args()

    with open(args.spec) as fh:
        spec = json.load(fh)

    rclpy.init()
    node = Trial(spec, args.out, args.limit)
    try:
        outcome = node.run()
        node.write(outcome)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
