"""Bring a stage up to the same state as the hospital one, in a single pass.

Three scripts have to run against a freshly built stage and the order matters:
sensors are attached first because the graphs reference the prims they create,
the joint fixup wants the timeline stopped, and the graphs are what publish. The
hospital stage got each of them by hand over several sessions, which is exactly
how a second stage ends up subtly different from the first.

    prepare_stage.py <stage.usd>

It stamps the stage when it finishes. verify_stage refuses a stage without the
stamp, and the trial harness refuses one without verify's. That chain exists
because a stage builder deletes and recreates its USD, so re-running it after
this throws away the sensors, the joint drives and every ROS graph -- and
nothing in the file said so. A width course was rebuilt twice for a wall
thickness fix, and the second run was measured with a robot that fell 126 m.
"""
import os
import sys
import time

STAGE = sys.argv[1]
HERE = "/home/sim/vica_ws/src/vica_description/isaac_vica_assets"

from isaacsim import SimulationApp  # noqa: E402
simulation_app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
import isaacsim.core.utils.extensions as ext_utils  # noqa: E402

ext_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

print(f"\n=== opening {STAGE}", flush=True)
if not omni.usd.get_context().open_stage(STAGE):
    print("FAILED to open stage", flush=True)
    simulation_app.close()
    sys.exit(1)
for _ in range(30):
    simulation_app.update()

for name in ("attach_vica_sensors.py",
             "fixup_vica_usd_joints.py",
             "build_vica_ros_graphs.py"):
    path = f"{HERE}/{name}"
    print(f"\n=== {name}", flush=True)
    try:
        # __file__ as well as __name__: a step that wants a file next to
        # itself has nothing to resolve against without it, and the failure is
        # a NameError at import time that takes the whole step with it.
        exec(compile(open(path).read(), path, "exec"),
             {"__name__": "vica_prep", "__file__": path})
    except Exception:
        import traceback
        print(f"!!! {name} RAISED", flush=True)
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        # os._exit rather than simulation_app.close() then sys.exit(1).
        # close() ends the process itself and it ends it with status 0, so the
        # sys.exit below it never ran and make_stage.sh read a failed prepare
        # as a successful one. It then verified a stage that the previous run
        # had prepared and printed "전부 통과", which is the exact shape of
        # failure this pipeline exists to prevent.
        os._exit(1)
    for _ in range(10):
        simulation_app.update()

# The stamp. Carries the source file's modification time, so rebuilding the
# stage invalidates it: verify_stage compares them and says which step is
# missing rather than leaving it to be discovered by a falling robot.
layer = omni.usd.get_context().get_stage().GetRootLayer()
data = dict(layer.customLayerData)
# A plain string. A nested dict with a list in it round-tripped through the
# crate format as "Attempted to unpack unsupported type enum value 0", and the
# reader crashed rather than reporting a bad stamp -- a silent-failure fix that
# introduced a silent failure.
data["vica_prepared"] = (
    time.strftime("%Y-%m-%d %H:%M:%S")
    + " | attach_vica_sensors,fixup_vica_usd_joints,build_vica_ros_graphs"
)
data.pop("vica_verified", None)
layer.customLayerData = data

omni.usd.get_context().save_stage()
print("\n=== stage saved and stamped", flush=True)
print("    다음: verify_stage.py 를 통과해야 측정에 쓸 수 있습니다.", flush=True)
simulation_app.close()
