#!/usr/bin/env python3
"""Drive the width course and record what happened, repeatably.

    ros2 run vica_description width_trials --phase gaps  --repeats 3
    ros2 run vica_description width_trials --phase lane --width 0.90 --repeats 3

[TEST ONLY] Sends Nav2 goals and publishes /initialpose. It does not touch
/cmd_vel: the controller under test drives, which is the whole point. Reads
/odom and /scan.

The question is which corridor width driving stops working at, and answering it
takes more care than sending a goal and seeing what happens. Four things went
wrong repeatedly while measuring rotation, and each of them is guarded here:

    single trials rank nothing        near a threshold a run is close to a coin
                                      toss, so every width is driven --repeats
                                      times and the spread is reported
    a wedged robot measures the wedge  clearance is checked before each trial
                                      and a trial that starts too close to
                                      something is skipped and said to be
    a goal carries two demands        travel and final heading. Every goal here
                                      is given the heading the robot will
                                      already have, so a lane trial measures the
                                      lane and not a turn on the spot it cannot
                                      do anyway
    the wall clock lies               the simulator runs at 0.6 of real time, so
                                      durations come from message stamps

Failures are classified by what the robot did, not by parsing logs:

    reached      the action returned success
    stuck        the action failed after moving more than 0.1 m -- it got in and
                 could not get through
    no-start     it failed having moved less than that -- the planner refused,
                 or the controller never produced a command
    timeout      neither, within the limit

Lane trials alternate direction. The lane is a through-passage because below
1.60 m the robot cannot turn around, so a repeat means driving back the other
way rather than looping.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# The lidar sees the robot's own rear panel at 0.447 m. Excluding that sector is
# the difference between a clearance check and one that always fails.
SELF_RETURN_HALF_WIDTH_DEG = 20.0
MIN_START_CLEARANCE = 0.35
MOVED_THRESHOLD = 0.10
STATUS = {4: "reached", 6: "aborted", 5: "canceled"}


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class Trials(Node):
    def __init__(self, args):
        super().__init__("width_trials")
        self.args = args
        self.odom, self.scan = [], []
        self.create_subscription(Odometry, "/odom", self.odom.append, 50)
        self.create_subscription(
            LaserScan, "/scan", lambda m: self.scan.append(m), qos_profile_sensor_data)
        self.initial = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.spawn = (0.0, 0.0)   # set from the course spec before any trial
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if not self.ac.wait_for_server(timeout_sec=60):
            raise SystemExit("navigate_to_pose 액션 서버가 없습니다 -- nav2 가 떠 있나요?")

    def spin(self, seconds):
        t = time.time()
        while rclpy.ok() and time.time() - t < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self):
        """Latest odom pose. Does not clear the buffer.

        It used to, and the track was assembled from self.odom immediately
        after calling this, so every trial recorded an empty path and 0.0 s of
        sim time. The numbers that survived were the ones not taken from the
        track.
        """
        t = time.time()
        while rclpy.ok() and not self.odom and time.time() - t < 15:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not self.odom:
            raise SystemExit("/odom 이 없습니다 -- 시뮬레이터가 돌고 있나요?")
        m = self.odom[-1].pose.pose
        return m.position.x, m.position.y, yaw_of(self.odom[-1])

    def pose_map(self):
        """The same pose in the map frame, which is where the goals are.

        odom starts at zero wherever the robot spawned; the goals come from the
        course in world coordinates. Comparing them directly made every
        "distance to goal" wrong by the spawn offset -- a trial that arrived
        exactly reported 8.86 m remaining.
        """
        x, y, a = self.pose()
        return self.spawn[0] + x, self.spawn[1] + y, a

    def clearance(self):
        self.scan.clear()
        t = time.time()
        while rclpy.ok() and not self.scan and time.time() - t < 15:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not self.scan:
            return float("inf")
        s = self.scan[-1]
        best = float("inf")
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r <= s.range_min:
                continue
            deg = math.degrees(s.angle_min + i * s.angle_increment)
            if abs(abs(deg) - 180.0) <= SELF_RETURN_HALF_WIDTH_DEG:
                continue
            best = min(best, r)
        return best

    def seed_localisation(self, x, y, yaw):
        """Tell AMCL where the robot actually is.

        The params set an initial pose of (0, 0, 0) and re-apply it on every
        activation, and the course does not start the robot there. Without this
        the map is offset from the moment nav2 comes up.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        for _ in range(3):
            self.initial.publish(msg)
            self.spin(0.5)

    def goal(self, x, y, yaw, limit, label, guard=True):
        """One trial. Returns the record, or None if it was not run."""
        clear0 = self.clearance()
        x0, y0, a0 = self.pose_map()
        if guard and clear0 < MIN_START_CLEARANCE:
            print(f"    {label:26s} 건너뜀 (시작 여유 {clear0:.2f} m)", flush=True)
            return {"label": label, "result": "skipped", "clearance_start": clear0}

        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)

        self.odom.clear()
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print(f"    {label:26s} 접수 거부", flush=True)
            return {"label": label, "result": "rejected"}

        rf = handle.get_result_async()
        min_clear = clear0
        t_wall = time.time()
        while rclpy.ok() and not rf.done() and time.time() - t_wall < limit:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan:
                min_clear = min(min_clear, self.clearance_cached())

        # Track first, then the pose: the track is what the buffer holds.
        track = [[stamp(m), self.spawn[0] + m.pose.pose.position.x,
                  self.spawn[1] + m.pose.pose.position.y, yaw_of(m)]
                 for m in self.odom]
        x1, y1, a1 = self.pose_map()
        moved = math.hypot(x1 - x0, y1 - y0)
        sim_span = (track[-1][0] - track[0][0]) if len(track) > 1 else 0.0

        if rf.done():
            outcome = STATUS.get(rf.result().status, str(rf.result().status))
            if outcome != "reached":
                outcome = "stuck" if moved >= MOVED_THRESHOLD else "no-start"
        else:
            handle.cancel_goal_async()
            self.spin(1.0)
            outcome = "timeout"

        rec = {
            "label": label,
            "result": outcome,
            "goal": [x, y],
            "from": [x0, y0],
            "to": [x1, y1],
            "moved_m": round(moved, 3),
            "remaining_m": round(math.hypot(x - x1, y - y1), 3),
            "sim_seconds": round(sim_span, 2),
            "clearance_start": round(clear0, 3),
            "clearance_min": round(min_clear, 3),
            "track": track,
        }
        print(f"    {label:26s} {outcome:9s} 이동 {moved:5.2f} m  "
              f"남은거리 {rec['remaining_m']:5.2f} m  "
              f"최소여유 {min_clear:.2f} m  {sim_span:5.1f}s", flush=True)
        return rec

    def clearance_cached(self):
        if not self.scan:
            return float("inf")
        s = self.scan[-1]
        best = float("inf")
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r <= s.range_min:
                continue
            deg = math.degrees(s.angle_min + i * s.angle_increment)
            if abs(abs(deg) - 180.0) <= SELF_RETURN_HALF_WIDTH_DEG:
                continue
            best = min(best, r)
        return best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spec", default=None, help="the course's targets json")
    ap.add_argument("--phase", required=True, choices=("lane", "gaps"))
    ap.add_argument("--width", type=float, default=None, help="lane width to drive")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--limit", type=float, default=150.0,
                    help="wall seconds allowed per trial")
    ap.add_argument("--controller", default="unknown", help="recorded in the results")
    ap.add_argument("--spawn", default=None,
                    help="x,y the robot was actually spawned at, when the course "
                         "spec's own start was overridden")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    spec_path = args.spec
    if spec_path is None:
        from ament_index_python.packages import get_package_share_directory
        spec_path = os.path.join(
            get_package_share_directory("vica_description"),
            "isaac_vica_assets", "vica_widthcourse.json")
    spec = json.load(open(spec_path))

    # Refuse a stage that has not passed verify_stage.
    #
    # The chain is build -> prepare -> verify, and each step stamps the USD.
    # Without this check the harness will happily drive a stage whose sensors
    # and ROS graphs were thrown away by a rebuild, and produce a full table of
    # numbers about a robot that fell through the floor. That is worse than
    # failing, because the table looks like a result.
    stage_path = os.path.join(os.path.dirname(os.path.abspath(spec_path)),
                              spec.get("stage", ""))
    if os.path.isfile(stage_path):
        from pxr import Sdf

        layer = Sdf.Layer.FindOrOpen(stage_path)
        stamp = dict(layer.customLayerData).get("vica_verified") if layer else None
        if not stamp:
            print(f"  {os.path.basename(stage_path)} 가 verify_stage 를 통과하지 "
                  f"않았습니다.\n"
                  f"  isaac_vica_assets/make_stage.sh 로 다시 만든 뒤 실행하십시오.",
                  file=sys.stderr)
            return 2
        print(f"  스테이지 검증 통과: {stamp}", flush=True)

    rclpy.init()
    node = Trials(args)
    sx, sy = spec["start"]
    if args.spawn:
        sx, sy = (float(v) for v in args.spawn.split(","))
    node.spawn = (sx, sy)

    # The robot has to be where the course says it starts.
    #
    # Clearance alone does not establish that. A run once began with the robot
    # 2.2 m from its spawn and inside the 0.70 m lane, where clearance was a
    # plausible-looking 0.41 m, AMCL had been seeded at the spawn instead, and
    # every goal came back "Start occupied" in 0.1 s. The table said "no-start"
    # six times and none of it was about the lane.
    #
    # odom is zero at spawn, so its magnitude is the distance moved since.
    ox, oy, _ = node.pose()
    if math.hypot(ox, oy) > 0.5:
        print(f"  로봇이 출발 위치에 없습니다: odom ({ox:+.2f}, {oy:+.2f}) "
              f"= 스폰에서 {math.hypot(ox, oy):.2f} m.\n"
              f"  시뮬레이터를 다시 시작해 로봇을 스폰으로 되돌린 뒤 실행하십시오.",
              file=sys.stderr)
        rclpy.shutdown()
        return 3

    node.seed_localisation(sx, sy, 0.0)
    node.spin(3.0)

    records = []
    if args.phase == "gaps":
        west, east = spec["gap_run"]["west"], spec["gap_run"]["east"]
        print(f"\n  간격 구간 {west} -> {east}, {args.repeats}회", flush=True)
        # Entering the gauntlet is not itself a trial; it only has to happen.
        node.goal(west[0], west[1], 0.0, args.limit, "진입", guard=False)
        for k in range(args.repeats):
            target, heading = (east, 0.0) if k % 2 == 0 else (west, math.pi)
            rec = node.goal(target[0], target[1], heading, args.limit,
                            f"간격 통과 #{k + 1}")
            if rec:
                records.append(rec)
    else:
        lane = min(spec["lanes"], key=lambda l: abs(l["width"] - args.width))
        print(f"\n  레인 {lane['width']:.2f} m, {args.repeats}회", flush=True)
        node.goal(lane["entry"][0], lane["entry"][1], math.pi / 2, args.limit,
                  "레인 입구로", guard=False)
        for k in range(args.repeats):
            up = k % 2 == 0
            tgt = lane["exit"] if up else lane["entry"]
            heading = math.pi / 2 if up else -math.pi / 2
            rec = node.goal(tgt[0], tgt[1], heading, args.limit,
                            f"{'통과 위로' if up else '통과 아래로'} #{k + 1}")
            if rec:
                rec["width"] = lane["width"]
                records.append(rec)

    payload = {
        "controller": args.controller,
        "phase": args.phase,
        "width": args.width,
        "spec": os.path.basename(spec_path),
        "records": records,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(payload, fh, indent=1)
    print(f"  wrote {args.out}", flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
