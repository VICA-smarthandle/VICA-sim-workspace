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

timeline = omni.timeline.get_timeline_interface()
timeline.play()
print(f"=== playing for ~{SECONDS:.0f}s via app.update()", flush=True)

import time  # noqa: E402

started = time.time()
frames = 0
while time.time() - started < SECONDS:
    simulation_app.update()
    frames += 1
    if frames % 600 == 0:
        print(f"    {frames} frames, {time.time() - started:.0f}s wall", flush=True)

timeline.stop()
print(f"=== done, {frames} frames", flush=True)
simulation_app.close()
