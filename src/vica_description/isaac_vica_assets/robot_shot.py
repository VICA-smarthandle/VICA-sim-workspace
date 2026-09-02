"""Photograph the robot on its own, framed from its bounding box.

    $ISAAC_SIM/python.sh robot_shot.py [stage.usd] [--out DIR] [--res WxH]

Defaults to robot/vica/vica.usda, which is the asset the stages reference and
therefore the thing worth checking after a re-import.

Why not pose_shot.py
--------------------
That one exists to answer whether the arm holds its stowed pose, and its
docstring is explicit that the framing is fixed on purpose: an auto-framed
shot changes when the pose changes, which is exactly when two shots need to
be comparable. Right, for that question.

Wrong for this one. The robot grew a 1.075 m mast and shrank 8 cm at the
tail, and a camera aimed at a fixed z 0.42 from a fixed 4.2 m put the result
in the bottom third of the frame with the bumper cut off -- the bumper being
where the ultrasonic probes just went. So this script measures the robot and
backs the camera off until it fits, which is the right trade when the
question is "what shape is it now" rather than "did it move".

The views are orthogonal-ish rather than orthographic. A true side elevation
hides the two probes behind each other and hides the mast's offset entirely;
three-quarters shows depth. `front` looks along -x at the bumper, which is
the one that shows both probes and the gap between them.
"""

import os
import sys

STAGE = None
args = sys.argv[1:]
if args and not args[0].startswith("-"):
    STAGE = args[0]


def _opt(name, default=None):
    return args[args.index(name) + 1] if name in args else default


OUT = _opt("--out", "/tmp/vica_robot_shot")
RES = _opt("--res", "1600x1200")
W, H = (int(v) for v in RES.split("x"))

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.path.join(os.environ.get("VICA_WS", os.getcwd()),
                        "src/vica_description/isaac_vica_assets")

if STAGE is None:
    STAGE = os.path.join(HERE, "robot", "vica", "vica.usda")
STAGE = os.path.abspath(STAGE)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True, "width": W, "height": H})

import math  # noqa: E402

import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402

omni.usd.get_context().open_stage(STAGE)
for _ in range(120):
    simulation_app.update()

stage = omni.usd.get_context().get_stage()

# The robot asset has no light of its own -- it is meant to be referenced into
# a stage that brings one. Without this the render is black and the failure
# looks like a broken camera.
if not any(p.IsA(UsdLux.DomeLight) or p.IsA(UsdLux.DistantLight)
           for p in stage.Traverse()):
    dome = UsdLux.DomeLight.Define(stage, "/ShotDome")
    dome.CreateIntensityAttr(1500.0)
    key = UsdLux.DistantLight.Define(stage, "/ShotKey")
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(2.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 0.0, 35.0))

# Bounds of the robot only. A stage may carry a floor slab tens of metres
# across, and framing that frames nothing.
root = stage.GetPrimAtPath("/World/VICA") or stage.GetDefaultPrim()
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
rng = cache.ComputeWorldBound(root).ComputeAlignedRange()
lo, hi = rng.GetMin(), rng.GetMax()
centre = Gf.Vec3d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)
size = max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
print(f"robot bounds  x {lo[0]:.3f}..{hi[0]:.3f}  "
      f"y {lo[1]:.3f}..{hi[1]:.3f}  z {lo[2]:.3f}..{hi[2]:.3f}")

# A USD camera is 20.955 x 15.29 mm; the vertical is the binding one on a
# landscape frame. Half a metre of margin so nothing touches the edge.
FOCAL = 35.0
half_fov = math.atan(15.29 / 2 / FOCAL)
dist = (size / 2 + 0.5) / math.tan(half_fov)

VIEWS = {
    "front": Gf.Vec3d(1.0, 0.0, 0.25),
    "quarter": Gf.Vec3d(0.85, -0.85, 0.45),
    "side": Gf.Vec3d(0.0, -1.0, 0.20),
    "bumper": Gf.Vec3d(1.0, -0.25, -0.10),   # low and close, for the probes
}


def look_at(eye, target):
    z = (eye - target).GetNormalized()
    up = Gf.Vec3d(0, 0, 1)
    x = Gf.Cross(up, z).GetNormalized()
    y = Gf.Cross(z, x)
    m = Gf.Matrix4d(1.0)
    m.SetRow3(0, x)
    m.SetRow3(1, y)
    m.SetRow3(2, z)
    m.SetTranslateOnly(eye)
    return m


os.makedirs(OUT, exist_ok=True)
cam = UsdGeom.Camera.Define(stage, "/ShotCam")
cam.CreateFocalLengthAttr(FOCAL)
cam_xform = UsdGeom.Xformable(cam).AddTransformOp()

rp = rep.create.render_product("/ShotCam", (W, H))
writer = rep.WriterRegistry.get("BasicWriter")

for name, direction in VIEWS.items():
    d = Gf.Vec3d(direction).GetNormalized()
    # The bumper view is deliberately close: it is a detail shot, not a
    # portrait, and framing it like a portrait puts the probes at four pixels.
    scale = 0.34 if name == "bumper" else 1.0
    eye = centre + d * dist * scale
    if name == "bumper":
        eye = Gf.Vec3d(eye[0], eye[1], max(0.35, lo[2] + 0.30))
        target = Gf.Vec3d(hi[0], centre[1], lo[2] + 0.12)
    else:
        target = centre
    cam_xform.Set(look_at(eye, target))

    sub = os.path.join(OUT, name)
    writer.initialize(output_dir=sub, rgb=True)
    writer.attach([rp])
    # delta_time=0 so the timeline never advances.
    #
    # Without it the first shot came back as an empty grey frame. The robot
    # asset on its own carries rigid bodies and no floor -- it is meant to be
    # referenced into a stage that brings one -- so stepping physics drops it
    # out of frame before the writer gets a picture. replay_render.py makes
    # the same choice for the same reason: nothing here is being simulated,
    # so nothing should be allowed to move.
    for _ in range(12):
        rep.orchestrator.step(rt_subframes=16, delta_time=0.0)
    writer.detach()
    print(f"  {name:8s} eye ({eye[0]:+.2f}, {eye[1]:+.2f}, {eye[2]:+.2f}) -> {sub}")

print(f"\nwrote {OUT}")
simulation_app.close()
