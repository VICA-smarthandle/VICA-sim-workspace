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
import re
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
# The robot's own bodywork, dead astern. Measured 2026-08-07 off a stationary
# robot on open floor: returns under 0.6 m occupy 177.7 to 182.2 degrees and
# read 0.442-0.444 m. That is five beams of eight hundred.
#
# It was 20 degrees before, which is eight times wider than the thing it
# excludes and throws away real returns across a 40-degree arc behind the
# robot -- the arc a reversing or turning robot most needs. Clearance then
# reads more generous than it is, which is the wrong direction for a guard.
#
# 3 degrees covers the measured extent with margin. Anything closer than 0.6 m
# outside this sector is a real obstacle and should stop the trial.
SELF_RETURN_HALF_WIDTH_DEG = 3.0
MIN_START_CLEARANCE = 0.35
MOVED_THRESHOLD = 0.10
# How far AMCL may disagree with the intended spawn before the run is refused.
# Lanes are 3.4 m apart, so anything approaching a metre is already ambiguous
# about which lane is being driven; 0.5 m leaves room for AMCL settling
# without leaving room for being in the wrong place.
START_OFFSET_LIMIT = 0.5
STATUS = {4: "reached", 6: "aborted", 5: "canceled"}


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def inflation_of():
    """The inflation_radius the running nav2 was built with, or None.

    Read from the installed config rather than asked of the running node,
    because a parameter query answers for whichever costmap replies first and
    the two are supposed to agree. If they ever do not, this returns the pair
    and the report shows it rather than picking one.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("vica_description"),
                            "config", "vica_nav2_params.yaml")
        vals = re.findall(r"^\s*inflation_radius:\s*([0-9.]+)", open(path).read(),
                          re.M)
        uniq = sorted({float(v) for v in vals})
        return uniq[0] if len(uniq) == 1 else uniq
    except Exception:
        return None


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class Trials(Node):
    def __init__(self, args):
        super().__init__("width_trials")
        self.args = args
        self.odom, self.scan, self.amcl = [], [], []
        self.create_subscription(Odometry, "/odom", self.odom.append, 50)
        self.create_subscription(
            LaserScan, "/scan", lambda m: self.scan.append(m), qos_profile_sensor_data)
        # Keep only the latest -- this is a position, not a history, and an
        # unbounded buffer on a 10 Hz topic across a 500 s trial is 5000
        # messages nothing reads.
        #
        # Transient local, because AMCL publishes rarely: it updates on motion,
        # so a stationary robot produces nothing and a volatile subscriber that
        # arrives late sees an empty topic on a perfectly localised robot.
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose",
            lambda m: self.amcl.__setitem__(slice(None), [m]),
            QoSProfile(depth=5, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.initial = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.spawn = (0.0, 0.0)   # set from the course spec before any trial
        self.spawn_yaw = 0.0      # and the heading it spawned at
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
        """Where the robot is in the map frame, which is where the goals are.

        /amcl_pose when there is one, because that is the only source that is
        actually in the frame the goals are in. The fallback -- spawn plus
        odom -- is an assumption wearing a coordinate's clothes: it is right
        only while the spawn is what the caller thinks and odom has not been
        cleared since. Both of those failed.

        odom starts at zero wherever the robot spawned; the goals come from
        the course in world coordinates. Comparing them directly made every
        "distance to goal" wrong by the spawn offset -- a trial that arrived
        exactly reported 8.86 m remaining. Adding the spawn fixed that and
        introduced a quieter problem: the sum is only the map frame if AMCL
        agrees, and nothing checked that it did.
        """
        if self.amcl:
            p = self.amcl[-1].pose.pose
            return p.position.x, p.position.y, yaw_of(self.amcl[-1])
        x, y, a = self.pose()
        return self.odom_to_map(x, y, a)

    def odom_to_map(self, x, y, a):
        """Odom is aligned with the spawn heading, not with the map.

        Adding the spawn translation and stopping there is only right when the
        robot spawned facing along +x, which every course did until the corner
        one. Facing north, "forward" in odom is +y in the map, and the sum
        reported a robot driving east up an open strip while it was in fact
        driving north up a corridor. The track was 90 degrees wrong and the
        drift it appeared to show did not exist.
        """
        c, s = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
        return (self.spawn[0] + c * x - s * y,
                self.spawn[1] + s * x + c * y,
                a + self.spawn_yaw)

    def pose_source(self):
        return "amcl" if self.amcl else "odom+spawn"

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
        # Drop whatever AMCL last said before seeding, so the wait below cannot
        # be satisfied by a pose that predates the seed.
        self.amcl.clear()
        for _ in range(3):
            self.initial.publish(msg)
            self.spin(0.5)

    def wait_for_localisation(self, x, y, timeout=30.0, tol=0.30):
        """Wait until AMCL reports a pose that agrees with the seed.

        Publishing /initialpose does not mean AMCL has taken it. Its first
        published pose is (0, 0, 0) -- the parameter default, re-applied on
        every activation -- and the seeded one arrives some seconds later. In
        between, /amcl_pose answers with a position that is neither where the
        robot is nor where it was told the robot is.

        The harness used to read whatever was there after a three-second spin
        and record it as the trial's start. Measured against the simulator's
        own transform, the robot was at its spawn to within a millimetre and
        AMCL agreed to within 0.020 m once it had converged -- but the recorded
        starts were up to 3.2 m out, almost all of it in the corridor axis.
        Those were mid-convergence samples, and they went into the table as if
        the robot had been somewhere else.

        A start-offset guard was added before this and did not catch it: it ran
        immediately after seeding, when AMCL still agreed by construction. The
        check has to wait for the answer, not ask before it exists.
        """
        t = time.time()
        while rclpy.ok() and time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.amcl:
                p = self.amcl[-1].pose.pose.position
                if math.hypot(p.x - x, p.y - y) <= tol:
                    return math.hypot(p.x - x, p.y - y)
        return None

    def goal(self, x, y, yaw, limit, label, guard=True):
        """One trial. Returns the record, or None if it was not run."""
        clear0 = self.clearance()
        if guard and clear0 < MIN_START_CLEARANCE:
            print(f"    {label:26s} 건너뜀 (시작 여유 {clear0:.2f} m)", flush=True)
            return {"label": label, "result": "skipped", "clearance_start": clear0}

        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)

        # Sample the start after clearing, not before.
        #
        # x0 used to be read above this line, so it carried whatever odom had
        # accumulated since the simulator started playing while x1 carried
        # only what came after the clear. "moved" was then the difference
        # between two poses measured from different origins, and "from" was
        # not the start of anything. Three repeats of one lane reported
        # starts 12 m apart on a course where the spawn had not moved.
        self.odom.clear()
        self.spin(0.3)
        x0, y0, a0 = self.pose_map()
        src0 = self.pose_source()

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
        track = [[stamp(m), *self.odom_to_map(m.pose.pose.position.x,
                                              m.pose.pose.position.y,
                                              yaw_of(m))]
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
            "pose_source": src0,
            "start_offset_m": round(math.hypot(x0 - self.spawn[0],
                                               y0 - self.spawn[1]), 3),
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
    ap.add_argument("--spawn-yaw", type=float, default=None,
                    help="heading the robot was spawned at, radians; "
                         "defaults to the course spec's start_yaw")
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
    course_stamp = None
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
        course_stamp = stamp

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

    # The heading the robot was actually spawned at, not zero.
    #
    # Seeding AMCL at 0 while the stage put the robot elsewhere makes the
    # localiser start off by however much the two disagree, and every pose the
    # trial records is measured from that. The corner course is the first
    # course that spawns facing anywhere but east.
    spawn_yaw = float(spec.get("start_yaw", 0.0))
    if args.spawn_yaw is not None:
        spawn_yaw = args.spawn_yaw
    node.spawn_yaw = spawn_yaw
    node.seed_localisation(sx, sy, spawn_yaw)
    node.spin(3.0)

    # And where AMCL says it is, which is a different question.
    #
    # The odom check above only establishes that the robot has not moved since
    # the simulator placed it. It cannot tell whether the simulator placed it
    # where this run intends -- odom reads zero at the wrong spawn just as
    # happily as at the right one. A sweep once drove six lanes believing each
    # was the one named in the filename.
    #
    # AMCL is in the map frame, the same frame the goals are in, so this
    # compares like with like. What it has to establish is not that AMCL agrees
    # right now -- seeded at (sx, sy) it agrees by construction, and did so
    # while recording starts 3.2 m out -- but that AMCL has actually taken the
    # seed and settled on it.
    off = node.wait_for_localisation(sx, sy, timeout=30.0,
                                     tol=START_OFFSET_LIMIT)
    if off is None:
        here = ""
        if node.amcl:
            p = node.amcl[-1].pose.pose.position
            here = f" 마지막 보고 ({p.x:+.2f}, {p.y:+.2f})."
        print(f"  AMCL 이 30초 안에 스폰 ({sx:+.2f}, {sy:+.2f}) 으로 수렴하지 "
              f"않았습니다.{here}\n"
              f"  이 상태로 측정하면 파일 이름과 다른 레인을 주행합니다.",
              file=sys.stderr)
        rclpy.shutdown()
        return 4
    print(f"  출발 위치 확인: AMCL 오차 {off:.2f} m", flush=True)

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
        # Headings come from the lane when it states them.
        #
        # Both were pi/2 here, which is right for a straight lane running north
        # and wrong for anything that turns: a corner's exit faces along the
        # leg it ends on, and asking for pi/2 there tells the robot to arrive
        # sideways. It would have been read as the corner being impassable.
        # Straight courses say nothing and keep the old numbers.
        entry_yaw = lane.get("entry_yaw", math.pi / 2)
        exit_yaw = lane.get("exit_yaw", math.pi / 2)
        print(f"\n  레인 {lane['width']:.2f} m, {args.repeats}회   "
              f"진입 {math.degrees(entry_yaw):.0f}도 -> 탈출 "
              f"{math.degrees(exit_yaw):.0f}도", flush=True)
        node.goal(lane["entry"][0], lane["entry"][1], entry_yaw, args.limit,
                  "레인 입구로", guard=False)
        for k in range(args.repeats):
            up = k % 2 == 0
            tgt = lane["exit"] if up else lane["entry"]
            # Coming back, the robot arrives where it started facing the way
            # it came from, which is the entry heading reversed.
            heading = exit_yaw if up else entry_yaw - math.pi
            rec = node.goal(tgt[0], tgt[1], heading, args.limit,
                            f"{'통과 위로' if up else '통과 아래로'} #{k + 1}")
            if rec:
                rec["width"] = lane["width"]
                records.append(rec)

    # The course stamp travels with the numbers.
    #
    # Two sweeps were once compared that had driven different courses. Widths
    # were added between them, every lane moved, and the same "1.20 m" meant a
    # goal at x=6.10 in one run and x=11.20 in the other. Both tables looked
    # complete and neither said which course it was about, so the comparison
    # read as a controller result for several hours.
    #
    # Nothing here can detect that on its own -- a single run has nothing to
    # disagree with. So each run records what it drove, and width_report
    # refuses to put two stamps in one table.
    payload = {
        "controller": args.controller,
        "phase": args.phase,
        "width": args.width,
        "spec": os.path.basename(spec_path),
        "course_stamp": course_stamp,
        "spawn": [sx, sy],
        "inflation_radius": inflation_of(),
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
