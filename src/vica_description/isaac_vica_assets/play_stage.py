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
from pxr import Usd as _Usd, UsdGeom as _UsdGeom, UsdPhysics as _UsdPhysics  # noqa: E402

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


started = time.time()
frames = 0
_p0 = _world_xyz()
if _p0:
    _say(f"=== robot at ({_p0[0]:+.3f},{_p0[1]:+.3f},{_p0[2]:+.3f}) "
         f"roll {_p0[3]:+.1f} pitch {_p0[4]:+.1f} deg")
while time.time() - started < SECONDS:
    simulation_app.update()
    frames += 1
    if frames % 300 == 0:
        _p = _world_xyz()
        _d = math.hypot(_p[0] - _p0[0], _p[1] - _p0[1]) if _p else float("nan")
        _say(f"    {frames} frames, {time.time() - started:.0f}s, "
             f"robot ({_p[0]:+.3f},{_p[1]:+.3f},{_p[2]:+.3f}) "
             f"roll {_p[3]:+.1f} pitch {_p[4]:+.1f}  moved {_d:.3f} m")

timeline.stop()
print(f"=== done, {frames} frames", flush=True)
simulation_app.close()
