#!/usr/bin/env python3
"""Drive a fixed path with /cmd_vel and record it, for pictures.

    ros2 run vica_description drive_scripted --out run.json \\
        --leg 8 0 --leg 0 90 --leg 5 0

[TEST ONLY] Publishes /cmd_vel directly. This is not navigation and does not
pretend to be: no map, no plan, no obstacle avoidance. It exists so that an
environment can be filmed with the robot in it while nav2 is being sorted out
separately, and so that a stage can be checked by eye without a map.

Each --leg is "<metres> <degrees>": drive that far, then turn that much in
place. Distances are commanded, not measured -- the recorded track is what
actually happened and is what the render uses.

Writes the same JSON shape replay_render.py reads, so the run can be replayed
into the stage with the real robot afterwards.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

V = 0.35          # m/s, comfortably under the 0.5 limit
W = 0.4           # rad/s


class Scripted(Node):
    def __init__(self, legs, out, origin=(0.0, 0.0)):
        super().__init__("drive_scripted")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.legs, self.out, self.origin = legs, out, origin
        self.pose = None
        self.track = []

    def _odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.pose = (t, m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        self.track.append([t,
                           m.pose.pose.position.x + self.origin[0],
                           m.pose.pose.position.y + self.origin[1],
                           yaw])

    def _hold(self, vx, wz, seconds):
        """Command for a wall-clock duration. Only used for the pauses."""
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)

    def _until(self, vx, wz, want, angular=False):
        """Command until odometry says the leg is done.

        Not a wall clock. The simulator runs at about a quarter of real time
        here, so a leg timed with time.monotonic() delivers a quarter of what
        was asked -- a 20 m tour came out 4.67 m. Measuring the thing being
        asked for removes the dependence on how fast the machine is.

        Gives up after a generous wall-clock limit so a robot pinned against a
        wall does not hold the script forever.
        """
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        start = self.pose
        done = 0.0
        deadline = time.monotonic() + 30.0 + want / max(abs(vx or wz), 1e-3) * 6.0
        while done < want and time.monotonic() < deadline:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.pose is None:
                continue
            if angular:
                d = self.pose[3] - start[3]
                while d > math.pi:
                    d -= 2 * math.pi
                while d < -math.pi:
                    d += 2 * math.pi
                done = abs(d)
            else:
                done = math.hypot(self.pose[1] - start[1], self.pose[2] - start[2])

    def run(self):
        t0 = time.monotonic()
        while self.pose is None and time.monotonic() - t0 < 30:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.pose is None:
            print("  /odom 이 없습니다")
            return 1
        self._hold(0.0, 0.0, 1.0)
        for metres, degrees in self.legs:
            if metres:
                self._until(V if metres > 0 else -V, 0.0, abs(metres))
                self._hold(0.0, 0.0, 0.5)
            if degrees:
                rad = math.radians(degrees)
                self._until(0.0, W if rad > 0 else -W, abs(rad), angular=True)
                self._hold(0.0, 0.0, 0.5)
        self._hold(0.0, 0.0, 1.0)

        d = 0.0
        for a, b in zip(self.track, self.track[1:]):
            d += math.hypot(b[1] - a[1], b[2] - a[2])
        json.dump({"records": [{"label": "scripted", "result": "recorded",
                                "goal": None, "track": self.track}]},
                  open(self.out, "w"))
        print(f"  {len(self.track)} 샘플, 실제 주행 {d:.2f} m -> {self.out}")
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--origin", nargs=2, type=float, default=(0.0, 0.0),
                    metavar=("X", "Y"),
                    help="where the robot was spawned in the stage. /odom "
                         "starts at zero wherever that is, and replay_render "
                         "places the robot at track coordinates in the stage's "
                         "own frame, so the two agree only when the spawn was "
                         "the origin. It was for the hospital and the office "
                         "and not for the arm test room, where the render put "
                         "the robot back in the corridor it had been moved out "
                         "of.")
    ap.add_argument("--leg", nargs=2, type=float, action="append", metavar=("M", "DEG"),
                    required=True, help='"<metres> <degrees>", repeatable')
    args = ap.parse_args()
    rclpy.init()
    node = Scripted([(m, d) for m, d in args.leg], args.out, args.origin)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
