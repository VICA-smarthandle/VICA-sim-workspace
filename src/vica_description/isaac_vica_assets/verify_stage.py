"""Refuse to hand over a stage that will not work.

    /home/sim/isaacsim/python.sh verify_stage.py <stage.usd> [seconds]

Exits non-zero on any failure, so it can gate a pipeline.

Every silent failure this project has spent a day on is checked here, because
each of them looked like success at the time:

    the stage builder deletes and recreates the USD, so re-running it after
    attaching sensors, fixing joints and building graphs throws all three away.
    Nothing records which steps a file has had.

    a missing payload, an unselected physics variant and an absent articulation
    root all raise nothing. The robot comes apart or falls through the floor,
    and the log is clean.

    gravity left at the UsdPhysics defaults is direction (0,0,0) and magnitude
    -inf. Their product is NaN and the robot leaves through the floor on the
    first step.

    a rigid body with no joint -- the D455 was one -- destabilises the whole
    articulation. The robot drives straight and will not turn, which reads as a
    controller problem for as long as you let it.

The static checks are the cheap half. The half that matters is playing the
stage and watching: everything above can pass and the robot can still fall,
which is what happened on the width course and is why this file exists.

Reading it also has to be done right. Opening a stage without loading payloads
reports zero articulation roots and zero rigid bodies on a stage that is
perfectly fine -- a diagnostic wrote exactly that here, and it was believed for
several minutes.
"""

import os
import sys
import time

STAGE = sys.argv[1]
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
# Bounds on how far the robot may move while nothing is commanding it. Both
# directions, because the first version measured only z0 - min(z) and passed a
# stage that launched the robot 12 m into the air: a robot going up has a
# perfectly good minimum. Explosions are the more common PhysX failure and they
# go outward, not down.
DRIFT_LIMIT_Z = 0.10
DRIFT_LIMIT_XY = 0.20
ROBOT_PATH = "/World/VICA"
EXPECTED_GRAPHS = [
    "ROS_Clock", "ROS_DifferentialDrive", "ROS_Odometry",
    "ROS_JointStates", "ROS_Lidar", "ROS_Camera",
]

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import math  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import isaacsim.core.utils.extensions as ext_utils  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

ext_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

failures = []


def check(label, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {label:44s} {detail}", flush=True)
    if not ok:
        failures.append(label)
    return ok


print(f"\n=== {STAGE}", flush=True)
if not omni.usd.get_context().open_stage(STAGE):
    print("  FAIL  스테이지를 열 수 없습니다")
    simulation_app.close()
    sys.exit(1)
for _ in range(60):
    simulation_app.update()
stage = omni.usd.get_context().get_stage()

# Payloads are loaded explicitly. Without this the counts below are all zero on
# a stage that is entirely correct.
stage.Load()
simulation_app.update()

print("\n--- 준비 스탬프", flush=True)
_layer = stage.GetRootLayer()
_stamp = dict(_layer.customLayerData).get("vica_prepared")
if not _stamp:
    check("prepare_stage 를 거쳤음", False,
          "스탬프 없음 -- prepare_stage.py 를 먼저 실행하십시오")
else:
    # Presence is the whole proof. The builder deletes and recreates the file,
    # which takes customLayerData with it, so a stamp that is here at all was
    # written after the last build. An mtime comparison was tried first and
    # invalidated itself: verify saves the layer, which changes the mtime.
    check("prepare_stage 를 거쳤음", True, str(_stamp))

print("\n--- 정적 검사", flush=True)

scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
check("physics scene 1개", len(scenes) == 1, f"{len(scenes)}개")
if scenes:
    sc = UsdPhysics.Scene(scenes[0])
    g = sc.GetGravityDirectionAttr().Get()
    mag = sc.GetGravityMagnitudeAttr().Get()
    good = g is not None and tuple(g) != (0.0, 0.0, 0.0) and mag not in (None,) \
        and mag == mag and abs(mag) != float("inf")
    check("중력이 sentinel 이 아님", good, f"{tuple(g) if g else g}  {mag}")

robot = stage.GetPrimAtPath(ROBOT_PATH)
check(f"{ROBOT_PATH} 존재", bool(robot))
if robot:
    vs = robot.GetVariantSets()
    sel = vs.GetVariantSet("Physics").GetVariantSelection() if "Physics" in vs.GetNames() else None
    check("physics variant = physx", sel == "physx", str(sel))

arts = [str(p.GetPath()) for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
check("articulation root 정확히 1개", len(arts) == 1, f"{len(arts)}개 {arts[:2]}")

bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
check("rigid body 존재", len(bodies) >= 9, f"{len(bodies)}개")

joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
jointed = set()
for j in joints:
    rel = UsdPhysics.Joint(j)
    for targets in (rel.GetBody0Rel().GetTargets(), rel.GetBody1Rel().GetTargets()):
        for t in targets:
            jointed.add(str(t))
loose = [str(b.GetPath()) for b in bodies if str(b.GetPath()) not in jointed]
check("조인트 없는 rigid body 0개", not loose, f"{loose[:3]}")

graphs = {p.GetName() for p in stage.Traverse()}
missing = [g for g in EXPECTED_GRAPHS if g not in graphs]
check("ROS 그래프 6개", not missing, f"없음: {missing}" if missing else "")

colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
check("충돌체 존재", len(colliders) >= 2, f"{len(colliders)}개")


def world_xyz(path):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    return (t[0], t[1], t[2])


print(f"\n--- 재생 검사 ({SECONDS:.0f}초)", flush=True)
timeline = omni.timeline.get_timeline_interface()
p0 = world_xyz(arts[0]) if arts else None
timeline.play()

import time  # noqa: E402

t0 = time.monotonic()
samples = []
while time.monotonic() - t0 < SECONDS:
    simulation_app.update()
    p = world_xyz(arts[0]) if arts else None
    if p is not None:
        samples.append(p)
timeline.stop()

if not samples:
    check("로봇 높이를 읽을 수 있음", False, "articulation root 없음")
else:
    dz = max(abs(p[2] - p0[2]) for p in samples)
    dxy = max(math.hypot(p[0] - p0[0], p[1] - p0[1]) for p in samples)
    end = samples[-1]
    check("높이가 변하지 않음", dz < DRIFT_LIMIT_Z,
          f"시작 z {p0[2]:.3f}, 최대 변화 {dz:.3f} m, 끝 {end[2]:.3f}")
    check("제자리에 머묾", dxy < DRIFT_LIMIT_XY,
          f"최대 이동 {dxy:.3f} m, 끝 ({end[0]:.2f}, {end[1]:.2f})")
    flat = [v for p in samples for v in p]
    check("좌표가 유한함", all(v == v and abs(v) != float("inf") for v in flat),
          "NaN/inf 없음")

print()
if failures:
    print(f"  실패 {len(failures)}건: {', '.join(failures)}", flush=True)
    print("  이 스테이지로는 측정하지 마십시오.", flush=True)
else:
    # Nothing is written here. This process has played the stage, and saving a
    # layer after playing bakes PhysX's writeback into the file -- that is how
    # the width course ended up with base_link 10 m from its spawn, jammed in
    # the narrowest lane, after passing its own verification. make_stage.sh
    # runs stamp_verified.py instead, in a process that only opened the file.
    print("  전부 통과. 측정에 써도 됩니다.", flush=True)
simulation_app.close()
sys.exit(1 if failures else 0)
