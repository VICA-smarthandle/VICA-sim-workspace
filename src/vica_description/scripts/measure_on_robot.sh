#!/usr/bin/env bash
#
# Three measurements only the physical robot can give. Run it there, paste the
# output back, and simulation stops guessing.
#
#   ros2 run vica_description measure_on_robot
#
# Reads only. Nothing is published, nothing is changed, the robot does not move.
# Safe to run alongside anything, though item 3 needs the robot on the floor
# with room to turn.
#
# Each of these is a place where simulation currently carries an assumption.
# They are cheap to settle and expensive to leave: an assumption that turns out
# wrong invalidates whatever was measured against it, and the longer it stands
# the more results depend on it.
set -uo pipefail

echo "=============================================================="
echo " 1. Camera intrinsics and resolution"
echo "=============================================================="
echo
echo "Why: simulation renders depth at 848x480 with fx = fy = 433.0, giving"
echo "     88.8 x 58.0 degrees. That came from the D455 datasheet and from"
echo "     realsense_ros_local_changes.patch leaving depth_profile at '0,0,0',"
echo "     which means device default. Whether the device default really is"
echo "     848x480 here has not been checked."
echo
echo "     Vertical is the number that matters -- the robot's own nvblox notes"
echo "     reason from 'camera vertical FOV 58', and it is what decides whether"
echo "     an obstacle stays in view through a turn. A different resolution"
echo "     changes horizontal without touching it, so a mismatch here costs"
echo "     less than it looks, but it should still be known."
echo
timeout 15 ros2 topic echo /camera/camera/depth/camera_info --once 2>/dev/null \
    | grep -E "^height:|^width:|^ *- " | head -8 \
    || echo "  no camera_info -- is the RealSense container running?"

echo
echo "=============================================================="
echo " 2. Does /scan see the robot's own chassis?"
echo "=============================================================="
echo
echo "Why: in simulation the scan returns 0.447 m at +/-180 degrees, off the"
echo "     robot's own rear panel. base_link.stl puts a face at x -0.305 to"
echo "     -0.265 at the lidar's height, 0.450 m behind it, which matches to"
echo "     3 mm. So the geometry says the robot should see itself here too."
echo
echo "     If it does, the costmap carries a permanent phantom obstacle right"
echo "     behind the robot. Nothing acts on it today because there is no"
echo "     collision monitor on the robot, but it will matter for reversing"
echo "     and for any narrow-space turn."
echo
timeout 20 python3 - <<'PY' 2>/dev/null || echo "  probe failed -- is /scan publishing?"
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = Node("scan_probe")
got = []
node.create_subscription(LaserScan, "/scan", lambda m: got.append(m), qos_profile_sensor_data)
import time
t0 = time.time()
while rclpy.ok() and not got and time.time() - t0 < 15:
    rclpy.spin_once(node, timeout_sec=1.0)

if got:
    m = got[0]
    print(f"  ranges {len(m.ranges)}   "
          f"angle {math.degrees(m.angle_min):.0f}..{math.degrees(m.angle_max):.0f} deg   "
          f"increment {math.degrees(m.angle_increment):.3f} deg")
    print(f"  range_min {m.range_min:.3f}   range_max {m.range_max:.3f}")
    near = [(math.degrees(m.angle_min + i * m.angle_increment), r)
            for i, r in enumerate(m.ranges)
            if math.isfinite(r) and m.range_min < r < 0.9]
    print(f"\n  returns closer than 0.9 m: {len(near)}")
    for a, r in near[:10]:
        print(f"      {a:+8.1f} deg   {r:.3f} m")
    if not near:
        print("      none -- the robot does not see itself")
rclpy.shutdown()
PY

echo
echo "=============================================================="
echo " 3. Is wheel_base_m = 0.37 calibrated, or an estimate?"
echo "=============================================================="
echo
echo "Why: the URDF says the wheels are 0.364 m apart, measured with a tape."
echo "     encoder_feedback uses 0.370 for odometry. Those are different"
echo "     quantities and both can be right -- an effective tread absorbs tyre"
echo "     scrub that geometry does not describe -- but whether 0.370 was ever"
echo "     fitted or just written down has never been established."
echo
echo "     A full turn settles it. Command a rotation, count what the robot"
echo "     reports, and compare against where it actually ends up:"
echo
echo "       exactly 360 deg   -> 0.370 was calibrated, keep it"
echo "       around 354 deg    -> it is the geometric 0.364 under another name"
echo "                            and odometry has a 1.6 % scale error"
echo
echo "  Manual, on an open floor, robot facing a marked direction:"
echo "      ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\"
echo "          '{angular: {z: 0.4}}'"
echo "  Stop when /odom reports one full turn, then measure the real heading"
echo "  against the mark."
echo
echo
echo "=============================================================="
echo " 4. How much of a commanded rotation does the robot deliver?"
echo "=============================================================="
echo
echo "Why: simulation reaches about 61 % of what it is told -- 17.4 deg/s"
echo "     against a commanded 28.6. Whether that is a simulation defect or"
echo "     the robot behaving the same way decides what to do about it, and"
echo "     the two answers point in opposite directions:"
echo
echo "       the robot also delivers ~61 %  -> simulation is faithful. Leave"
echo "                                         the physics alone and plan"
echo "                                         around the real turn rate."
echo "       the robot delivers ~100 %      -> simulation has a defect in"
echo "                                         friction or the joint drives."
echo "                                         Fix it there. Do NOT compensate"
echo "                                         in MPPI -- parameters tuned to"
echo "                                         hide it would overshoot on the"
echo "                                         real robot, which is the exact"
echo "                                         failure sim-to-real is meant to"
echo "                                         prevent."
echo
echo "  Open floor, robot free to spin. Commands 0.4 rad/s (22.9 deg/s), the"
echo "  configured wz_max, and reports what odometry actually achieved:"
echo
timeout 30 python3 - <<'PY' 2>/dev/null || echo "  probe failed -- is /odom publishing?"
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

TARGET = 0.4  # rad/s, matches wz_max in both workspaces

rclpy.init()
node = Node("yaw_rate_probe")
pub = node.create_publisher(Twist, "/cmd_vel", 10)
seen = []
node.create_subscription(Odometry, "/odom", lambda m: seen.append(m), 10)

cmd = Twist()
cmd.angular.z = TARGET
t0 = time.time()
while rclpy.ok() and time.time() - t0 < 6.0:
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.05)
pub.publish(Twist())
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.05)

# Skip the first second: the smoother is still ramping up.
spun = [m for m in seen if m.header.stamp.sec or m.header.stamp.nanosec]
steady = spun[len(spun) // 4:] if len(spun) > 8 else spun
if steady:
    rates = [m.twist.twist.angular.z for m in steady]
    got = sum(rates) / len(rates)
    print(f"  commanded {TARGET:.3f} rad/s  ({math.degrees(TARGET):.1f} deg/s)")
    print(f"  achieved  {got:.3f} rad/s  ({math.degrees(got):.1f} deg/s)"
          f"   = {100 * got / TARGET:.0f} %")
    print(f"  samples {len(steady)}")
else:
    print("  no odometry received")
rclpy.shutdown()
PY

echo
echo "=============================================================="
echo " Paste all of the above back. Item 3 needs the physical reading too."
echo "=============================================================="
