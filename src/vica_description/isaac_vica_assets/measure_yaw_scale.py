"""Rotation scale, measured inside Isaac against the articulation's own pose.

No ROS. The wheel targets are computed exactly as the DifferentialController
computes them, so this measures the same thing nav2 would experience, without a
publisher, a subscriber or a clock in the way -- the ROS version of this test
returned nan and a rotation in the wrong direction.

    python.sh yaw_isaac.py <stage.usd> [wheel_distance]
"""
import math
import sys
import time

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

STAGE = sys.argv[1]
L = float(sys.argv[2]) if len(sys.argv) > 2 else 0.387
R = 0.065
SIM_DT = 1.0 / 60.0
TESTS = [0.2, 0.3, 0.4, 0.5]
SETTLE_S, MEASURE_S = 3.0, 8.0
SPAWN = (-20.0, -2.5)

omni.usd.get_context().open_stage(STAGE)
for _ in range(60):
    app.update()
stage = omni.usd.get_context().get_stage()
stage.Load()
app.update()
root = next(p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI))
vica = stage.GetPrimAtPath("/World/VICA")
J = {p.GetName(): p for p in stage.Traverse()
     if p.GetName() in ("left_wheel_joint", "right_wheel_joint")}
tl = omni.timeline.get_timeline_interface()
TC = Usd.TimeCode.Default()


def yaw():
    m = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(TC)
    v = m.TransformDir(Gf.Vec3d(1, 0, 0))
    return math.atan2(v[1], v[0])


def target(name, rad_s):
    a = J[name].GetAttribute("drive:angular:physics:targetVelocity")
    if not a:
        a = J[name].CreateAttribute("drive:angular:physics:targetVelocity",
                                    Sdf.ValueTypeNames.Float)
    a.Set(math.degrees(rad_s))


# The first play in a process does not apply drive targets. Burn one.
tl.play()
for _ in range(30):
    app.update()
tl.stop()
app.update()

print(f"  wheel_distance {L:.4f} m,  wheel_radius {R} m", flush=True)
print(f"  {'명령 w':>8} {'실측 w':>9} {'비율':>8} {'회전량':>10}", flush=True)
ratios = {}
for w in TESTS:
    tl.stop()
    app.update()
    for op in UsdGeom.Xformable(vica).GetOrderedXformOps():
        n = op.GetOpName()
        if n == "xformOp:translate":
            op.Set(Gf.Vec3d(SPAWN[0], SPAWN[1], 0.0))
        elif n == "xformOp:orient":
            op.Set(Gf.Quatd(1.0, 0, 0, 0))
        elif n.startswith("xformOp:rotate"):
            op.Set(Gf.Vec3f(0, 0, 0))
    for _ in range(5):
        app.update()
    target("left_wheel_joint", -w * L / 2.0 / R)
    target("right_wheel_joint", w * L / 2.0 / R)
    tl.play()
    for _ in range(int(SETTLE_S / SIM_DT)):
        app.update()
    prev, turned = yaw(), 0.0
    frames = int(MEASURE_S / SIM_DT)
    for _ in range(frames):
        app.update()
        cur = yaw()
        d = cur - prev
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        turned += d
        prev = cur
    got = turned / (frames * SIM_DT)
    ratios[w] = got / w
    print(f"  {w:8.2f} {got:9.3f} {got / w:8.3f} {turned:9.2f} rad", flush=True)
    target("left_wheel_joint", 0.0)
    target("right_wheel_joint", 0.0)
    tl.stop()

band = [ratios[w] for w in (0.3, 0.4, 0.5)]
mean = sum(band) / len(band)
print(f"\n  0.3~0.5 평균 {mean:.4f}   ->  보정값 {L / mean:.4f} m", flush=True)
app.close()
