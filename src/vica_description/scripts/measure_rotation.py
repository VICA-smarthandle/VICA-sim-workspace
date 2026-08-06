#!/usr/bin/env python3
"""How much of a commanded rotation does this robot actually deliver?

    ros2 run vica_description measure_rotation

Runs in either workspace and prints the same table, so simulation and the
physical robot can be put side by side. It drives: the robot must be on an open
floor with room to spin. Nothing else is required -- Nav2 does not need to be
running, and this publishes straight to /cmd_vel.

It checks that room before every trial, using /scan, and refuses to measure
without it. That guard is not precaution -- it is the fix for three separate
measurements this tool's predecessors got wrong. A wall inside the robot's
circumscribed radius becomes part of what is being measured, and the robot puts
itself there: an in-place turn here drifts a few millimetres per second, so a
run that starts clear walks into something after a handful of trials. Nothing
in the numbers says so. They just quietly stop meaning what they say.

Simulation cannot turn in place below about 0.35 rad/s on PhysX's default
solver counts, which is under its own configured wz_max of 0.4. The stage now
carries 64 position and 16 velocity iterations, set in fixup_vica_usd_joints.py,
which moves that floor to about 0.25. The figures below predate that change and
are the shape of what is left, not of what was worst.

Simulation, measured 2026-08-06 in open floor space, twice by different means
and in agreement:

    cmd wz    yaw got   loss
     0.30       0.074   0.226
     0.40       0.209   0.191
     0.50       0.296   0.204
     0.60       0.392   0.208
     0.70       0.473   0.227
     0.80       0.547   0.253

The loss is not a percentage. It is a near-constant 0.19-0.25 rad/s whatever is
asked for, which is what a constant resisting torque looks like -- Coulomb
friction somewhere in the turn, not a gain that needs scaling. At the configured
wz_max of 0.4 that constant eats about half the command.

Which half of the drivetrain gives it up is worth knowing too, so the wheel
rates are printed beside the yaw:

    commanded wheel rate is wz * wheel_separation / (2 * wheel_radius)
    wheels short of that        -> the joint drive is not delivering
    wheels there but yaw short  -> the wheels are turning and the robot is not,
                                   so the tyres are slipping or something is
                                   resisting the turn

The second reading is the one this stage gives, and it means what it says here
because Isaac derives odometry from the chassis prim's own transform rather
than from wheel encoders. Where odometry is integrated from encoders instead --
as it is on the physical robot -- it cannot see slip by construction, and the
yaw column has to come from an outside measurement of the real heading.

The wall-clock explanation was checked and does not hold, though its
arithmetic is convincing enough to be worth writing down. This stage runs at a
real-time factor of 0.610, so a rotation timed with a stopwatch would read
0.5 * 0.610 = 0.305 rad/s -- almost exactly the observed figure. It is the
right suspicion and the wrong answer here: nothing above divides by a wall
clock. /odom's twist.angular.z is computed by the simulator from physics state,
and differencing the pose against /odom header stamps is a second sim-time
reading of the same thing. At a commanded 0.5 they give 0.2931 and 0.2938. A
clock error cannot survive two clocks that do not disagree.

Two other candidates are also excluded and need not be re-examined. A 0.293
tread in the controller would put the wheels at 1.127 rad/s; they run at 1.636,
and the stage's DifferentialController is authored with wheelDistance 0.364.
The caster collision radius is already the ground-contact value of 0.042.

What matters is not the simulation number on its own but whether the robot
does the same thing. If it does, simulation is faithful and the real turn rate
is simply what it is -- plan around it. If the robot delivers its command and
simulation does not, the defect is in simulation's friction or joint drives and
belongs there.

What must not happen either way is compensating for it in the controller.
Raising wz_max to make simulation turn at the rate asked for would overshoot on
a robot that never had the deficit, which is the failure sim-to-real exists to
prevent.
"""

import argparse
import math
import statistics
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan

# Both workspaces' URDFs agree on these.
WHEEL_RADIUS = 0.065
WHEEL_SEPARATION = 0.364
LEFT_JOINT = "left_wheel_joint"
RIGHT_JOINT = "right_wheel_joint"

# Turning in place sweeps the footprint's far corner through this radius, so
# anything nearer is something the robot turns against rather than past.
CIRCUMSCRIBED_RADIUS = 0.65

# The lidar sees the robot's own rear panel at 0.447 m, dead astern. That is not
# an obstacle, and excluding the sector it occupies is the difference between a
# clearance check and a check that always fails.
SELF_RETURN_HALF_WIDTH_DEG = 20.0


def stamp(msg):
    """Message time, which is sim time when use_sim_time is on.

    Timing this against the wall clock is how a benchmark got thrown away here:
    a simulator running at 0.6x real time shows exactly the deficit being looked
    for, whether or not one exists.
    """
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Probe(Node):
    def __init__(self, args):
        super().__init__("measure_rotation")
        self.args = args
        self.pub = self.create_publisher(Twist, args.topic, 10)
        self.odom = []
        self.joints = []
        self.create_subscription(Odometry, args.odom, self.odom.append, 10)
        self.create_subscription(
            JointState, "/joint_states", self.joints.append, qos_profile_sensor_data)
        self.scan = []
        self.create_subscription(
            LaserScan, args.scan, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        self.scan = [msg]

    def clearance(self):
        """Nearest return that is not the robot's own bodywork, or None."""
        if not self.scan:
            return None
        msg = self.scan[0]
        nearest = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min:
                continue
            deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            if abs(abs(deg) - 180.0) <= SELF_RETURN_HALF_WIDTH_DEG:
                continue
            if nearest is None or r < nearest:
                nearest = r
        return nearest

    def wait_for_odom(self, timeout=20.0):
        waited = 0.0
        while rclpy.ok() and len(self.odom) < 5 and waited < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            waited += 0.5
        return len(self.odom) >= 5

    def hold(self, wz, seconds):
        """Publish a constant command for `seconds` of message time."""
        cmd = Twist()
        cmd.angular.z = wz
        t0 = stamp(self.odom[-1])
        while rclpy.ok() and stamp(self.odom[-1]) - t0 < seconds:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
        return t0, stamp(self.odom[-1])

    def trial(self, wz):
        # Room to turn, checked now rather than assumed from the last trial.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.scan:
                break
        near = self.clearance()
        if near is not None and near < self.args.clearance:
            return "blocked", near

        # Come to a full stop first, so every trial starts the same way and a
        # rate that only sustains an already-moving robot cannot be read as one
        # that starts it.
        self.odom.clear()
        while rclpy.ok() and len(self.odom) < 3:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.hold(0.0, self.args.settle)

        self.odom.clear()
        self.joints.clear()
        while rclpy.ok() and len(self.odom) < 3:
            rclpy.spin_once(self, timeout_sec=0.2)
        t0, t1 = self.hold(wz, self.args.hold)

        # Discard the first half: the smoother and the drives are still ramping.
        window = t0 + self.args.hold / 2
        od = [m for m in self.odom if window <= stamp(m) <= t1]
        js = [m for m in self.joints if window <= stamp(m) <= t1]
        if not od:
            return None

        yaw = statistics.mean(m.twist.twist.angular.z for m in od)
        wheels = None
        if js:
            names = list(js[-1].name)
            if LEFT_JOINT in names and RIGHT_JOINT in names:
                li, ri = names.index(LEFT_JOINT), names.index(RIGHT_JOINT)
                usable = [m for m in js if len(m.velocity) > max(li, ri)]
                if usable:
                    wheels = statistics.mean(
                        (abs(m.velocity[li]) + abs(m.velocity[ri])) / 2
                        for m in usable)
        return yaw, wheels


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/cmd_vel",
                    help="where the robot takes its velocity command")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--rates", default="0.2,0.3,0.4,0.5,0.6,0.7,0.8",
                    help="commanded yaw rates in rad/s")
    ap.add_argument("--hold", type=float, default=6.0,
                    help="seconds to hold each rate")
    ap.add_argument("--settle", type=float, default=2.5,
                    help="seconds stopped between trials")
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--clearance", type=float, default=0.85,
                    help="metres of free space required before each trial; "
                         "the circumscribed radius is 0.65")
    args = ap.parse_args()

    rates = [float(r) for r in args.rates.split(",")]

    rclpy.init()
    node = Probe(args)
    if not node.wait_for_odom():
        print(f"no messages on {args.odom} -- is the robot running?",
              file=sys.stderr)
        rclpy.shutdown()
        return 1

    print(f"\n  commanding {args.topic}, reading {args.odom}")
    print(f"  wheel radius {WHEEL_RADIUS}, separation {WHEEL_SEPARATION}\n")
    print(f"  {'cmd wz':>7}  {'wheel tgt':>9}  {'wheel got':>9}  "
          f"{'yaw got':>8}  {'loss':>7}  {'% of cmd':>8}")
    print("  " + "-" * 60)

    rows = []
    blocked = 0
    for wz in rates:
        result = node.trial(wz)
        if result is None:
            print(f"  {wz:7.2f}   no odometry in the measurement window")
            continue
        if result[0] == "blocked":
            print(f"  {wz:7.2f}   skipped: {result[1]:.2f} m of clearance, "
                  f"under the {args.clearance:.2f} m needed")
            blocked += 1
            continue
        yaw, wheels = result
        target = wz * WHEEL_SEPARATION / (2 * WHEEL_RADIUS)
        got = f"{wheels:9.3f}" if wheels is not None else "        -"
        print(f"  {wz:7.2f}  {target:9.3f}  {got}  {yaw:8.3f}  "
              f"{wz - yaw:7.3f}  {100 * yaw / wz:7.0f}%")
        rows.append((wz, yaw))

    node.pub.publish(Twist())
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    if blocked:
        print()
        print(f"  {blocked} of {len(rates)} trials skipped for want of room.")
        print("  Move the robot somewhere open and run it again. A partial")
        print("  table here is not a smaller result, it is a different one:")
        print("  the trials that did run are the ones the robot happened to")
        print("  have room for, which is not a property of the robot.")

    if len(rows) >= 3:
        losses = [wz - yaw for wz, yaw in rows]
        spread = max(losses) - min(losses)
        mean_loss = statistics.mean(losses)
        print()
        print(f"  loss averages {mean_loss:.3f} rad/s "
              f"({math.degrees(mean_loss):.1f} deg/s), spread {spread:.3f}")
        if spread < mean_loss / 2:
            print("  Nearly constant across the range, which is a resisting")
            print("  torque rather than a scale error. Look for what drags:")
            print("  caster contact friction, caster swivel, tyre scrub.")
        else:
            print("  Varies with the command, so it scales rather than")
            print("  subtracts. Look for a gain: wheel separation or radius")
            print("  in the controller, or a velocity limit being clipped.")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
