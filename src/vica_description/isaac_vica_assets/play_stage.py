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
# Headless by default: rendering a window costs GPU that the physics needs,
# and every measurement in this repository was taken without one. Set
# VICA_HEADLESS=0 to watch instead -- worth it when the question is "what is
# the robot actually doing", which numbers answer slowly.
_HEADLESS = os.environ.get("VICA_HEADLESS", "1") not in ("0", "false", "")
_cfg = {"headless": _HEADLESS}
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

# Move the robot BEFORE anything else touches the stage.
#
# This used to sit after Load() and thirty update()s, and by then the physics
# parse had already placed the articulation at the pose the USD carried.
# Rewriting the parent Xform afterwards moved the colliders and left the root
# body: the robot then settled 48 mm lower, which is exactly where the chassis
# collider reaches the floor, and drove nowhere with its wheels turning.
#
# Measured, on this robot and on the August one alike:
#
#     spawn left alone      base_link z 0.190, held
#     spawn overridden      base_link z 0.142, held  <- chassis on the floor
#
# 0.142 is not a settling depth. The drive wheels are 0.065 radius on an axle
# 0.125 below base_link, so at 0.190 they touch the ground and at 0.142 they
# are 48 mm inside it. Nothing was holding the robot up but its belly.
#
# Authoring the pose here, before the first update, means the parse sees the
# robot where it is meant to be and there is nothing to disagree with.
if SPAWN is not None:
    from pxr import Gf, UsdGeom  # noqa: E402

    _s0 = omni.usd.get_context().get_stage()
    _v = _s0.GetPrimAtPath("/World/VICA")
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
    print(f"=== spawn moved to {SPAWN}"
          + (f" yaw {SPAWN_YAW:.1f} deg" if SPAWN_YAW is not None else ""),
          flush=True)

for _ in range(30):
    simulation_app.update()

# Load payloads before playing. Without this the robot's rigid bodies exist
# with no articulation behind them and it free-falls: odom went to z -168 on a
# stage whose own verification, which does call Load(), had just passed. The
# default open does not always bring them in, and nothing says so.
_stage = omni.usd.get_context().get_stage()
_stage.Load()
simulation_app.update()

timeline = omni.timeline.get_timeline_interface()
timeline.play()
print(f"=== playing for ~{SECONDS:.0f}s via app.update()", flush=True)

import math  # noqa: E402
import time  # noqa: E402

# Where the robot actually is, read off the stage.
#
# /odom is the only other answer and it is not independent: it comes from a
# graph in this same stage, so "the robot did not move" and "the odometry is
# broken" look identical from outside. This separates them.
from pxr import Gf as _Gf, Usd as _Usd, UsdGeom as _UsdGeom, UsdPhysics as _UsdPhysics  # noqa: E402

_root = None
for _p in _stage.Traverse():
    if _p.HasAPI(_UsdPhysics.ArticulationRootAPI):
        _root = _p
        break


def _world_xyz():
    """Position, and how far off level the robot is.

    The tilt is not decoration. A base_link that has dropped 48 mm reads the
    same whether the wheels sank through the floor or the robot pitched onto
    its nose, and those are different faults with different fixes. Roll and
    pitch separate them: level means the wheels are not holding it up, tilted
    means they are and something else is.
    """
    if _root is None:
        return None
    m = _UsdGeom.Xformable(_root).ComputeLocalToWorldTransform(_Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    r = m.ExtractRotationMatrix()
    # Roll and pitch off the rotated z axis, in degrees.
    up = (r[0][2], r[1][2], r[2][2])
    roll = math.degrees(math.atan2(up[1], up[2]))
    pitch = math.degrees(math.atan2(-up[0], math.hypot(up[1], up[2])))
    return (t[0], t[1], t[2], roll, pitch)


# Also written to a file, not only stdout.
#
# The pose trace is the one output worth keeping when a run is driven from a
# harness that pipes stdout somewhere and then dies: the shell goes, the
# simulator carries on, and the only record of where the robot went goes with
# the shell. VICA_POSE_LOG overrides the path.
_pose_log = os.environ.get("VICA_POSE_LOG", "/tmp/vica_play_pose.log")
try:
    _pose_fh = open(_pose_log, "w")
except OSError:
    _pose_fh = None


def _say(line):
    print(line, flush=True)
    if _pose_fh:
        _pose_fh.write(line + "\n")
        _pose_fh.flush()


# --------------------------------------------------------------------------
# The walker, on stages that have one
# --------------------------------------------------------------------------
# Inert on every other course: if there is no /World/Course/Walker prim, none
# of this runs and play_stage behaves exactly as it did.
#
# The walker is moved from here rather than by physics or by USD time samples.
# Time samples need the timeline to advance and this loop deliberately keeps it
# stopped, driving physics through update() instead -- the same reason the
# module docstring gives for not using SimulationContext.step(). So the runner
# owns the motion, which also makes the trigger exact: the walk starts when the
# robot is a set distance short of the crossing, not at a set time, so every
# repeat presents the robot with the same geometry however fast it got there.
_WALKER_PATH = "/World/Course/Walker"
_walker = _stage.GetPrimAtPath(_WALKER_PATH) if "_stage" in dir() else None
if _walker is not None and not _walker.IsValid():
    _walker = None

_walk_trigger = float(os.environ.get("VICA_WALK_TRIGGER_M", "4.0"))
_walk_speed = float(os.environ.get("VICA_WALK_SPEED", "1.2"))
_walk_log_path = os.environ.get("VICA_WALK_LOG", "/tmp/vica_walk.csv")
_walk_started = False
_walk_y = None
_walk_op = None
_walk_cross_x = _walk_park_y = _walk_end_y = None
_walk_fh = None

if _walker is not None:
    for _op in _UsdGeom.Xformable(_walker).GetOrderedXformOps():
        if _op.GetOpName() == "xformOp:translate":
            _walk_op = _op
    if _walk_op is not None:
        _t = _walk_op.Get()
        _walk_cross_x, _walk_park_y = float(_t[0]), float(_t[1])
        _walk_y = _walk_park_y
        # Where the walker stops. The corridor centre by default, which is a
        # person stepping out and staying there.
        #
        # Walking straight through was the first version and it measures the
        # wrong thing. Crossing a 2 m corridor at 1.2 m/s takes about three
        # seconds, in which a robot at 0.5 m/s covers 1.6 m, so unless the walk
        # begins within about a metre of the crossing the walker is gone before
        # the robot arrives and every trial reports that nothing was owed. The
        # question being asked is how late an obstacle can appear and still be
        # got around, and that needs one that is still there.
        #
        # A 0.35 m walker in the middle of a 2.0 m corridor leaves about 0.8 m
        # either side against a 0.505 m padded footprint, so going round is
        # possible and the trial is about whether it is possible in the
        # distance available. VICA_WALK_STOP_Y crosses fully again when that is
        # what is wanted.
        _walk_end_y = float(os.environ.get("VICA_WALK_STOP_Y", "0.0"))
        try:
            _walk_fh = open(_walk_log_path, "w")
            _walk_fh.write("t,robot_x,robot_y,robot_speed,walker_y,gap,walking\n")
        except OSError:
            _walk_fh = None
        print(f"=== walker at x {_walk_cross_x:.2f}, waits at y {_walk_park_y:+.2f}, "
              f"crosses to {_walk_end_y:+.2f} at {_walk_speed} m/s, "
              f"starts {_walk_trigger:.2f} m before the robot arrives", flush=True)
        print(f"=== walk log {_walk_log_path}", flush=True)
    else:
        print("=== walker prim has no translate op; not moving it", flush=True)

_SIM_DT = 1.0 / 60.0
_prev_xy = None


def _step_walker(frame):
    """Advance the walker and record one row. Returns nothing."""
    global _walk_started, _walk_y, _prev_xy
    if _walk_op is None:
        return
    p = _world_xyz()
    if p is None:
        return
    if _prev_xy is None:
        speed = 0.0
    else:
        speed = math.hypot(p[0] - _prev_xy[0], p[1] - _prev_xy[1]) / _SIM_DT
    _prev_xy = (p[0], p[1])

    if not _walk_started and p[0] >= _walk_cross_x - _walk_trigger:
        _walk_started = True
        print(f"=== walker steps out, robot at x {p[0]:+.3f}, "
              f"{_walk_cross_x - p[0]:.3f} m short of the crossing", flush=True)
    if _walk_started and _walk_y > _walk_end_y:
        _walk_y = max(_walk_end_y, _walk_y - _walk_speed * _SIM_DT)
        t = _walk_op.Get()
        _walk_op.Set(_Gf.Vec3d(float(t[0]), _walk_y, float(t[2])))

    if _walk_fh:
        gap = math.hypot(_walk_cross_x - p[0], _walk_y - p[1])
        _walk_fh.write(f"{frame * _SIM_DT:.3f},{p[0]:.4f},{p[1]:.4f},"
                       f"{speed:.4f},{_walk_y:.4f},{gap:.4f},"
                       f"{1 if _walk_started else 0}\n")
        if frame % 60 == 0:
            _walk_fh.flush()


started = time.time()
frames = 0
_p0 = _world_xyz()
if _p0:
    _say(f"=== robot at ({_p0[0]:+.3f},{_p0[1]:+.3f},{_p0[2]:+.3f}) "
         f"roll {_p0[3]:+.1f} pitch {_p0[4]:+.1f} deg")
while time.time() - started < SECONDS:
    simulation_app.update()
    frames += 1
    _step_walker(frames)
    if frames % 300 == 0:
        _p = _world_xyz()
        _d = math.hypot(_p[0] - _p0[0], _p[1] - _p0[1]) if _p else float("nan")
        _say(f"    {frames} frames, {time.time() - started:.0f}s, "
             f"robot ({_p[0]:+.3f},{_p[1]:+.3f},{_p[2]:+.3f}) "
             f"roll {_p[3]:+.1f} pitch {_p[4]:+.1f}  moved {_d:.3f} m")

timeline.stop()
if _walk_fh:
    _walk_fh.close()
print(f"=== done, {frames} frames", flush=True)
simulation_app.close()
