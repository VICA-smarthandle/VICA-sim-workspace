#!/usr/bin/env python3
"""How much of a commanded rotation does this robot actually deliver?

    ros2 run vica_description measure_rotation

Runs in either workspace and prints the same table, so simulation and the
physical robot can be put side by side. It drives: the robot must be on an open
floor with room to spin. Nothing else is required -- Nav2 does not need to be
running, and this publishes straight to /cmd_vel.

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
from sensor_msgs.msg import JointState

# Both workspaces' URDFs agree on these.
WHEEL_RADIUS = 0.065
WHEEL_SEPARATION = 0.364
LEFT_JOINT = "left_wheel_joint"
RIGHT_JOINT = "right_wheel_joint"


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
    for wz in rates:
        result = node.trial(wz)
        if result is None:
            print(f"  {wz:7.2f}   no odometry in the measurement window")
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
