"""Refuse to hand over a stage that will not work.

    $ISAAC_SIM/python.sh verify_stage.py <stage.usd> [seconds]

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
    "ROS_UltrasonicLeft", "ROS_UltrasonicRight",
]

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import math  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import isaacsim.core.utils.extensions as ext_utils  # noqa: E402
from pxr import Sdf, Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

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
# Scoped to the robot. The fault this catches is a body inside the
# articulation with nothing holding it -- the D455 was one, and it destabilised
# the whole chain. A rigid body elsewhere in the stage is scenery: the dynamic
# course's walker is a kinematic cylinder with no joint on purpose, moved by
# the runner rather than by physics, and it is not a fault.
loose = [str(b.GetPath()) for b in bodies
         if str(b.GetPath()).startswith("/World/VICA/")
         and str(b.GetPath()) not in jointed]
check("로봇 안에 조인트 없는 rigid body 0개", not loose, f"{loose[:3]}")

graphs = {p.GetName() for p in stage.Traverse()}
missing = [g for g in EXPECTED_GRAPHS if g not in graphs]
check(f"ROS 그래프 {len(EXPECTED_GRAPHS)}개", not missing, f"없음: {missing}" if missing else "")

colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
check("충돌체 존재", len(colliders) >= 2, f"{len(colliders)}개")

# Every rigid body has a mass.
#
# camera_link had visual geometry, a fixed joint and no <inertial>. With
# merge_fixed_joints off that is a rigid body with nothing to weigh, and PhysX
# logged this on every single play:
#
#   The rigid body at .../base_link/camera_link has a possibly invalid inertia
#   tensor of {1.0, 1.0, 1.0} and a negative mass
#
# A negative mass is not confined to the link that has it. The articulation
# solves as one system, so the whole mass matrix goes, and the robot settled
# 6 mm low and 2.4 degrees nose-up at places where it should have been level.
# Nothing above catches it: the body count is right, it has a joint, the stage
# opens, and every static check passes. It went a month.
#
# The warning is in the log and the log is not read. This reads the masses.
massless = []
for b in bodies:
    if not b.HasAPI(UsdPhysics.MassAPI):
        massless.append(f"{b.GetName()}(질량 속성 없음)")
        continue
    m = UsdPhysics.MassAPI(b).GetMassAttr().Get()
    if m is None or m <= 0.0:
        massless.append(f"{b.GetName()}({m})")
check("rigid body 전부 질량 있음", not massless, ", ".join(massless[:4]))

# The floor collides with a small collider, not just a large one.
#
# The courses used to floor themselves with one 50 m box. The chassis box
# collided with it and the 65 mm wheel cylinders did not, at some places along
# it and not others, so the robot sat 48 mm low on its belly with its wheels
# underground and spun them 5050 degrees without moving. Settling height by x
# on the corner course, box floor: -22 0.183, -8.65 0.190, -4.3 0.171,
# 0 0.142, +8.65 0.142, +13 0.142, +22 0.170. With a ground plane instead,
# all seven read 0.190.
#
# The play check below only ever stands the robot on one spot, so it cannot
# see this. Requiring the plane is cheaper than sweeping for it.
planes = [p for p in stage.Traverse()
          if p.IsA(UsdGeom.Plane) and p.HasAPI(UsdPhysics.CollisionAPI)]
floors = [p for p in stage.Traverse()
          if p.GetName() == "Floor" and p.HasAPI(UsdPhysics.CollisionAPI)
          and p.GetAttribute("physics:collisionEnabled").Get() is not False]
check("바닥이 ground plane", bool(planes) and not floors,
      ("plane 없음" if not planes else "")
      + (f"  Floor 상자가 아직 충돌함: {[f.GetName() for f in floors]}"
         if floors else ""))


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

# --------------------------------------------------------------------------
# Drive check
# --------------------------------------------------------------------------
# Everything above asks whether the robot stays put. Nothing asked whether it
# can move, and a robot that cannot move passes all of it with full marks.
#
# That is not hypothetical. Two ultrasonic probes were added to the URDF with
# visual geometry and no <inertial>. The importer runs with merge_fixed_joints
# off, so each became its own rigid body with no mass, and the articulation's
# mass matrix went with them: the wheels turned, the body did not, and it sank
# 21 mm into the floor. Every static check passed, the drift checks passed
# because it drifted nowhere, and the stage was stamped. The failure surfaced
# an hour later as a screening run where every cell scored zero.
#
# So: command the wheels directly and require the body to translate. No ROS,
# no controller, no map -- if this fails, nothing downstream is worth running.
print(f"\n--- 구동 검사", flush=True)
DRIVE_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
DRIVE_RAD_S = 4.0        # about 0.26 m/s at the 0.065 m wheel radius
DRIVE_SECONDS = 3.0
# 3 s at 0.26 m/s is 0.78 m. 0.30 is well under that and well over the 0.19 m
# the wall-clock version returned on stages that were working, so a regression
# to that behaviour is caught rather than passed.
MIN_TRAVEL = 0.30

drive_prims = []
for prim in stage.Traverse():
    if prim.GetName() in DRIVE_JOINTS:
        drive_prims.append(prim)

if len(drive_prims) != len(DRIVE_JOINTS):
    check("구동 조인트 2개", False,
          f"{len(drive_prims)}개 찾음 {[p.GetName() for p in drive_prims]}")
elif not arts:
    check("구동 검사 가능", False, "articulation root 없음")
else:
    # Isaac's angular drive targets are degrees per second.
    # Settle, then measure, and allow one retry.
    #
    # This is the second play in the process: the drift check above already
    # played and stopped, and a stop resets the articulation. The first frames
    # after the second play are the solver picking the robot back up, and
    # measuring across them read 0.002 m on a stage that does 0.596 m
    # standalone. A gate that fails at random blocks builds for no reason and
    # teaches everyone to rerun it until it passes, which is the same as not
    # having one.
    SETTLE_FRAMES = 60
    # Counted in frames, not seconds off a wall clock.
    #
    # The clock version failed a stage the robot drives perfectly on. That
    # stage runs at 24 per cent of real time, so three wall seconds are 0.72
    # simulated ones and 4 rad/s covers 0.19 m, which is exactly what every
    # passing stage measured: 0.178, 0.182, 0.195. It was never measuring three
    # seconds of driving. On a heavier stage the same window held almost no
    # physics steps and the check read 0.000 m twice running, while driving the
    # same stage directly gave 100 per cent of commanded speed at every speed
    # from 0.13 to 0.52 m/s.
    #
    # A gate that fails on how busy the machine is teaches everyone to re-run
    # it until it passes, which is the same as not having one.
    SIM_DT = 1.0 / 60.0
    DRIVE_FRAMES = int(DRIVE_SECONDS / SIM_DT)

    def _attempt():
        for prim in drive_prims:
            a = prim.GetAttribute("drive:angular:physics:targetVelocity")
            if not a:
                a = prim.CreateAttribute("drive:angular:physics:targetVelocity",
                                         Sdf.ValueTypeNames.Float)
            a.Set(math.degrees(DRIVE_RAD_S))
        timeline.play()
        for _ in range(SETTLE_FRAMES):
            simulation_app.update()
        start = world_xyz(arts[0])
        travelled = 0.0
        for _ in range(DRIVE_FRAMES):
            simulation_app.update()
            q = world_xyz(arts[0])
            if q is not None and start is not None:
                travelled = max(travelled,
                                math.hypot(q[0] - start[0], q[1] - start[1]))
        timeline.stop()
        for prim in drive_prims:
            prim.GetAttribute("drive:angular:physics:targetVelocity").Set(0.0)
        return travelled

    moved = _attempt()
    retried = False
    if moved < MIN_TRAVEL:
        retried = True
        moved = _attempt()

    check("바퀴를 돌리면 실제로 나아감", moved >= MIN_TRAVEL,
          f"시뮬 {DRIVE_SECONDS:.0f}초 {DRIVE_RAD_S:.1f} rad/s 로 {moved:.3f} m "
          f"(최소 {MIN_TRAVEL} m)" + ("  [재시도함]" if retried else ""))

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
