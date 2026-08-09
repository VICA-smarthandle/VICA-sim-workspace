#!/usr/bin/env python3
"""What the handle does to the person holding it.

    ros2 run vica_description measure_handle --duration 60

Records while something else drives -- a Nav2 goal, teleop, measure_rotation --
and reports the motion at the handle rather than at the robot. Reads only:
nothing is published and nothing is commanded.

Every acceptance test in this workspace so far has been scored from the robot's
point of view: did it reach the goal, how many degrees did it turn, did the
control loop hold 20 Hz. None of that says whether a person walking behind with
their hand on the handle could follow. VICA is a smart handle, the person is
attached to the tail, and the tail is what swings.

The handle sits 0.565 m behind base_footprint by tape measure. The robot does
not pivot about base_footprint, though -- a differential drive turns about the
midpoint of its drive axle, 0.154 m forward of it -- so the handle is 0.719 m
from the real centre of rotation and every radian of yaw moves it that far:

    wz 0.4 rad/s  ->  handle sweeps sideways at 0.29 m/s
    wz 1.0 rad/s  ->  0.72 m/s, nearly three times the robot's own top speed

That is the arithmetic behind max_vel_theta being taken from 1.0 down to 0.4 on
the robot. It holds the handle's sideways speed at roughly the robot's forward
speed instead of well above it.

Nothing here assumes that geometry, though. The handle's position is computed
from the measured pose each sample and differentiated over message timestamps,
so if the robot pivots somewhere other than where the model says, the numbers
still describe what happened.

Reported at the median, the 95th percentile and the maximum. Not the maximum
alone: the robot's own devlog records a wheel-speed reading of 0.534 against a
commanded 0.26 that was chased as a conversion error before it turned out to be
a startup spike in a max, and the settled mean was correct all along.

First run, recorded while measure_rotation drove in-place turns at 0.2 to 0.6
rad/s -- the worst case for a handle, and not what ordinary navigation looks
like:

    handle speed          median 0.000   p95 0.279   max 0.308  m/s
    handle lateral accel  median 0.042   p95 0.306   max 1.005  m/s^2
    turning on the spot   13.0 s of 85.3 s
    guidance announcing   11.9 s, 4 on/off transitions

The geometry checks out against it: at a commanded 0.6 the robot achieved
0.394 rad/s, and 0.719 x 0.394 is 0.283 against a measured p95 of 0.279.

The number worth sitting with is the maximum. The robot's own top forward speed
is 0.26 m/s, and turning on the spot moved the handle at 0.308 -- faster than
this robot ever drives, from rotation alone, at a yaw rate inside its configured
limit. Forward speed is bounded by a parameter someone chose. The handle's speed
during a turn is bounded by that parameter times 0.719, and nobody chose it.

The turn-guidance columns mirror vica_user_guidance's turn_guide_node, which
tells the person a turn is coming when 20 degrees accumulate inside a 1.5 s
window, and stops at 10. Its config carries an explicit unverified risk: if the
path oscillates the announcement can switch on and off repeatedly. That shows up
here as the transition count, and a run with many transitions is one where the
person is being told to turn, then not, then turn again.
"""

import argparse
import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Tape measure, robot centre to the end of the handle (2026-07-29). The STL is
# 6 cm shorter and was not trusted for the rear.
HANDLE_X = -0.565

# Drive joints are at x = +0.154, and base_footprint is directly below
# base_link, so this is the pivot in the same frame the handle offset is in.
DRIVE_AXLE_X = 0.154

# vica_user_guidance/config/user_guidance.yaml, turn_guide_node.
GUIDE_WINDOW_SEC = 1.5
GUIDE_ENTER_DEG = 20.0
GUIDE_EXIT_DEG = 10.0

# Below this the robot is not really translating, so yaw is a turn on the spot:
# the case that walks the person sideways around a circle instead of forward.
IN_PLACE_SPEED = 0.03


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def summarise(name, values, unit):
    if not values:
        return f"  {name:34s} (표본 없음)"
    ordered = sorted(values)
    n = len(ordered)
    return (f"  {name:34s} 중앙값 {statistics.median(ordered):7.3f}   "
            f"p95 {ordered[int(0.95 * (n - 1))]:7.3f}   "
            f"최대 {ordered[-1]:7.3f}  {unit}")


class HandleProbe(Node):
    def __init__(self, args):
        super().__init__("measure_handle")
        self.args = args
        self.samples = []
        self.create_subscription(Odometry, args.odom, self.samples.append, 50)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=60.0,
                    help="wall seconds to record")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--handle-x", type=float, default=HANDLE_X,
                    help="handle offset from base_footprint, metres (negative "
                         "is behind)")
    args = ap.parse_args()

    rclpy.init()
    node = HandleProbe(args)
    print(f"\n  {args.odom} 기록 중, {args.duration:.0f}초. 그동안 로봇을 주행시키세요.",
          flush=True)
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    s = node.samples
    rclpy.shutdown()

    if len(s) < 30:
        print(f"  표본이 {len(s)}개뿐입니다 -- {args.odom} 가 발행되고 있나요?",
              file=sys.stderr)
        return 1

    # Handle position in the odom frame, from the measured pose. Not from a
    # kinematic model, so a pivot elsewhere than the model's still reads true.
    t, hx, hy, yaw, body_speed = [], [], [], [], []
    for m in s:
        th = yaw_of(m)
        p = m.pose.pose.position
        t.append(stamp(m))
        hx.append(p.x + args.handle_x * math.cos(th))
        hy.append(p.y + args.handle_x * math.sin(th))
        yaw.append(th)
        body_speed.append(abs(m.twist.twist.linear.x))

    speeds, lat_acc, in_place = [], [], 0.0
    vx_prev = vy_prev = None
    for i in range(1, len(t) - 1):
        dt = t[i + 1] - t[i - 1]
        if dt <= 0:
            continue
        vx = (hx[i + 1] - hx[i - 1]) / dt
        vy = (hy[i + 1] - hy[i - 1]) / dt
        speed = math.hypot(vx, vy)
        speeds.append(speed)

        # Two /odom messages can carry the same header stamp -- Isaac's graph
        # publishes on the render tick and the clock does not always advance
        # between two of them. The first version divided by max(dt, 1e-6),
        # which turns a duplicate stamp into a division by a microsecond and
        # reports an acceleration ~50000x too large. It only touches the tail,
        # so the median stayed believable while p95 and max were artefacts.
        #
        # A duplicate stamp carries no acceleration information, so skip it.
        # Clamping invents a number; skipping admits there isn't one.
        dt_a = t[i] - t[i - 1]
        if vx_prev is not None and speed > 1e-3 and dt_a > 0:
            ax = (vx - vx_prev) / dt_a
            ay = (vy - vy_prev) / dt_a
            # The component across the direction of travel. Forward
            # acceleration is felt as pace; this one is felt as being pulled
            # off balance, which is the failure that matters for a walking aid.
            lat_acc.append(abs((-vy * ax + vx * ay) / speed))
        vx_prev, vy_prev = vx, vy

        if body_speed[i] < IN_PLACE_SPEED and speed > 0.02:
            in_place += t[i + 1] - t[i]

    # The guidance node's own test, replayed on this run.
    enter = math.radians(GUIDE_ENTER_DEG)
    exit_ = math.radians(GUIDE_EXIT_DEG)
    unwrapped, prev = [0.0], yaw[0]
    for th in yaw[1:]:
        d = th - prev
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        unwrapped.append(unwrapped[-1] + d)
        prev = th
    announcing, transitions, announced_for = False, 0, 0.0
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > GUIDE_WINDOW_SEC:
            j += 1
        accum = abs(unwrapped[i] - unwrapped[j])
        was = announcing
        if not announcing and accum >= enter:
            announcing = True
        elif announcing and accum < exit_:
            announcing = False
        if announcing != was:
            transitions += 1
        if announcing and i + 1 < len(t):
            announced_for += t[i + 1] - t[i]

    span = t[-1] - t[0]
    pivot_dist = abs(args.handle_x - DRIVE_AXLE_X)
    print(f"\n{'=' * 74}")
    print(f" 손잡이 궤적   {len(s)}표본 / {span:.1f}초 (sim time)")
    print(f" 손잡이 위치 base_footprint 기준 x={args.handle_x:+.3f} m, "
          f"회전중심에서 {pivot_dist:.3f} m")
    print(f"{'=' * 74}")
    print(summarise("손잡이 속도", speeds, "m/s"))
    print(summarise("손잡이 횡가속도", lat_acc, "m/s^2"))
    print()
    print(f"  {'제자리 회전 시간':34s} {in_place:7.1f} 초   "
          f"({100 * in_place / span if span else 0:.0f} % of run)")
    print(f"  {'회전 안내 켜져 있던 시간':30s} {announced_for:7.1f} 초   "
          f"({100 * announced_for / span if span else 0:.0f} %)")
    print(f"  {'회전 안내 on/off 전환':32s} {transitions:7d} 회   "
          f"({transitions / span * 60 if span else 0:.1f} 회/분)")
    print()
    print("  제자리 회전은 사람을 옆걸음으로 원을 그리게 합니다. 전환이 잦다는 것은")
    print("  회전 안내가 켜졌다 꺼졌다 한다는 뜻이고, 그러면 안내를 신뢰할 수 없습니다.")
    print("  판정 기준값은 실사용자 시험에서 나와야 하며 여기서 정하지 않습니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
