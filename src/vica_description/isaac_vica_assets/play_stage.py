"""Open a stage, press Play, and let the app drive it.

    $ISAAC_SIM/python.sh play_stage.py <stage.usd> [seconds]

The loop is simulation_app.update() rather than SimulationContext.step(), which
is what the GUI does. The step() convenience produced every /odom timestamp
exactly twice, and that duplication was nearly chased into the graphs before
anyone questioned the harness.

Payloads are loaded explicitly before playing. Without it the robot's rigid
bodies exist with no articulation behind them and it free-falls, on a stage
whose own verification -- which does load them -- has just passed.

FRAME_RATE_LIMIT caps the main loop. Frames that render without a physics step
behind them are what duplicated the timestamps, so holding frames at or below
the physics rate removes them by construction and halves the GPU load.
"""

import sys

STAGE = sys.argv[1]
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
# Optional spawn override, so a trial can begin next to the thing it measures.
# Without it every lane is driven to from one corner of the course, and the
# approach -- 0 m for the nearest lane, 27 m for the furthest -- decides the
# result rather than the lane does. That is how a 1.20 m lane went from 3/3 to
# 0/3 by being moved to the far end when three widths were added.
SPAWN = None
SPAWN_YAW = None
if len(sys.argv) > 4:
    SPAWN = (float(sys.argv[3]), float(sys.argv[4]))
# Optional heading, degrees. The corner course needs it: its cells are entered
# northbound, and a robot dropped facing east has to turn 90 degrees on the
# spot before the corner it is there to measure. That turn is a separate
# difficulty with its own open question, and it failed first -- the trial moved
# 0.0 m with 1.383 m of clearance and never reached the corner at all.
if len(sys.argv) > 5:
    SPAWN_YAW = float(sys.argv[5])

from isaacsim import SimulationApp  # noqa: E402

import os  # noqa: E402

# Cap the main loop so frames never outrun physics. Duplicate timestamps happen
# only when a frame renders without a physics step behind it, so holding frames
# at or below the physics rate removes them by construction -- and halves the
# GPU load while doing it.
RATE = float(os.environ.get("FRAME_RATE_LIMIT", "0"))
_cfg = {"headless": True}
simulation_app = SimulationApp(_cfg)
if RATE > 0:
    import carb  # noqa: E402

    _s = carb.settings.get_settings()
    _s.set("/app/runLoops/main/rateLimitEnabled", True)
    _s.set("/app/runLoops/main/rateLimitFrequency", int(RATE))
    _s.set("/app/runLoops/main/rateLimitUsePrecisionSleep", True)
    print(f"=== main loop limited to {int(RATE)} Hz", flush=True)

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import isaacsim.core.utils.extensions as ext_utils  # noqa: E402

ext_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

print(f"=== opening {STAGE}", flush=True)
if not omni.usd.get_context().open_stage(STAGE):
    print("FAILED to open stage", flush=True)
    simulation_app.close()
    sys.exit(1)
for _ in range(30):
    simulation_app.update()

# Load payloads before playing. Without this the robot's rigid bodies exist
# with no articulation behind them and it free-falls: odom went to z -168 on a
# stage whose own verification, which does call Load(), had just passed. The
# default open does not always bring them in, and nothing says so.
_stage = omni.usd.get_context().get_stage()
_stage.Load()
simulation_app.update()

if SPAWN is not None:
    from pxr import Gf, UsdGeom  # noqa: E402

    _v = _stage.GetPrimAtPath("/World/VICA")
    _xf = UsdGeom.Xformable(_v)
    _has_rot = False
    for _op in _xf.GetOrderedXformOps():
        if _op.GetOpName() == "xformOp:translate":
            _op.Set(Gf.Vec3d(SPAWN[0], SPAWN[1], 0.0))
        if _op.GetOpName().startswith("xformOp:rotate"):
            _has_rot = True
            if SPAWN_YAW is not None:
                _op.Set(Gf.Vec3f(0.0, 0.0, SPAWN_YAW))
    if SPAWN_YAW is not None and not _has_rot:
        _xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, SPAWN_YAW))
    simulation_app.update()
    print(f"=== spawn moved to {SPAWN}"
          + (f" yaw {SPAWN_YAW:.1f} deg" if SPAWN_YAW is not None else ""),
          flush=True)

# --------------------------------------------------------------------------
# Ultrasonic probes -> sensor_msgs/Range
# --------------------------------------------------------------------------
# Published from here rather than from an OmniGraph, which is where every other
# ROS interface in this stage comes from. Two reasons, both practical:
#
#   - The bridge has no ROS2PublishRange node. It publishes LaserScan,
#     PointCloud, Image, Odometry, JointState, TF and a few more, and Range is
#     not among them.
#   - The generic ROS2Publisher can name any message type, but it resolves that
#     type into dynamic input attributes only after the node has been
#     evaluated, so the fields cannot be set in the same edit that creates it.
#
# nav2's RangeSensorLayer wants sensor_msgs/Range on /ultrasonic/front_left and
# /ultrasonic/front_right, which is what the robot's driver publishes. Matching
# that interface is the whole reason for modelling the probes, so the simplest
# thing that produces it exactly is the right one.
#
# The cost is that the probes only exist while a stage is driven through this
# script. Every trial goes through it, and replay_render never plays the
# timeline at all, so nothing that needs them loses them.
USONIC = [
    ("usonic_front_left", "/ultrasonic/front_left"),
    ("usonic_front_right", "/ultrasonic/front_right"),
]
USONIC_FOV = 0.5236        # 30 deg, the cone the raycast fan covers
USONIC_MIN = 0.02
USONIC_MAX = 4.0
USONIC_HZ = 10.0           # DYP-A22 is specified around 10 Hz

_usonic = None
try:
    import rclpy  # noqa: E402
    from rclpy.node import Node as _RclNode  # noqa: E402
    from sensor_msgs.msg import Range  # noqa: E402
    from isaacsim.sensors.physics import _sensor as _isaac_sensor  # noqa: E402

    stage_now = omni.usd.get_context().get_stage()
    found = []
    for prim in stage_now.Traverse():
        n = prim.GetName()
        for link, topic in USONIC:
            if n == f"{link}_ray":
                found.append((str(prim.GetPath()), link, topic))
    if found:
        if not rclpy.ok():
            rclpy.init(args=None)
        _node = _RclNode("vica_usonic")
        _iface = _isaac_sensor.acquire_lidar_sensor_interface()
        pubs = [(path, link, _node.create_publisher(Range, topic, 1))
                for path, link, topic in found]
        _usonic = (_node, _iface, pubs)
        print(f"=== ultrasonic: {len(pubs)} probe(s) -> "
              f"{', '.join(t for _, _, t in found)}", flush=True)
    else:
        print("=== ultrasonic: no *_ray prims in this stage, not publishing",
              flush=True)
except Exception as exc:                      # noqa: BLE001
    # Never fatal. A stage without probes, or a build without the sensor
    # extension, still has to drive: the probes are one costmap layer, and
    # taking the whole run down for them would trade a measurement for nothing.
    print(f"=== ultrasonic: disabled ({exc})", flush=True)
    _usonic = None


def _publish_usonic():
    """Nearest hit in each probe's fan, as a Range message."""
    node, iface, pubs = _usonic
    now = node.get_clock().now().to_msg()
    for path, link, pub in pubs:
        try:
            depths = iface.get_linear_depth_data(path)
        except Exception:                     # noqa: BLE001
            continue
        vals = [float(d) for d in (depths if depths is not None else [])
                if d == d and d > 0.0]
        m = Range()
        m.header.stamp = now
        m.header.frame_id = link
        m.radiation_type = Range.ULTRASOUND
        m.field_of_view = USONIC_FOV
        m.min_range = USONIC_MIN
        m.max_range = USONIC_MAX
        # No hit reads as max range, which is what an ultrasonic reports and
        # what clear_on_max_reading in the costmap layer is waiting for.
        m.range = min(vals) if vals else USONIC_MAX
        pub.publish(m)


timeline = omni.timeline.get_timeline_interface()
timeline.play()
print(f"=== playing for ~{SECONDS:.0f}s via app.update()", flush=True)

import time  # noqa: E402

started = time.time()
frames = 0
_next_usonic = 0.0
while time.time() - started < SECONDS:
    simulation_app.update()
    frames += 1
    if _usonic is not None:
        now = time.time()
        if now >= _next_usonic:
            _publish_usonic()
            _next_usonic = now + 1.0 / USONIC_HZ
    if frames % 600 == 0:
        print(f"    {frames} frames, {time.time() - started:.0f}s wall", flush=True)

timeline.stop()
if _usonic is not None:
    try:
        _usonic[0].destroy_node()
    except Exception:                         # noqa: BLE001
        pass
print(f"=== done, {frames} frames", flush=True)
simulation_app.close()
