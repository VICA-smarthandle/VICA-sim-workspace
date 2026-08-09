"""Play a stage, check the arm holds its stowed pose, and photograph it.

    $ISAAC_SIM/python.sh pose_shot.py <stage.usd> [--seconds N] [--out DIR]

Two questions in one launch, because each launch costs a minute of GPU and
they are asked about the same run: does the arm stay where the drives were
told to hold it, and what does that look like.

The pose was derived geometrically by scripts/stow_pose.py and checked against
mesh bounding boxes. Neither of those is a simulation: the arm variant is
imported with allow_self_collision False, so PhysX does not test arm against
chassis and will not object if the derivation was wrong. The picture is how a
breach gets noticed, which is the same argument replay_render.py makes about
tracks and courses.

Camera framing is fixed rather than automatic. An auto-framed shot changes
when the pose changes, which is exactly when the two shots need to be
comparable.
"""

import os
import sys

STAGE = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else None
if not STAGE:
    raise SystemExit(__doc__.strip().splitlines()[2].strip())


def _opt(name, default=None):
    if name in sys.argv:
        return sys.argv[sys.argv.index(name) + 1]
    for a in sys.argv:
        if a.startswith(name + "="):
            return a.split("=", 1)[1]
    return default


SECONDS = float(_opt("--seconds", 8.0))
OUT = _opt("--out", "/tmp/vica_pose_shot")
WIDTH, HEIGHT = 1280, 720

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import math  # noqa: E402

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

if not omni.usd.get_context().open_stage(os.path.abspath(STAGE)):
    raise SystemExit(f"스테이지를 열 수 없습니다: {STAGE}")
stage = omni.usd.get_context().get_stage()
for _ in range(60):
    app.update()

arm_joints = {}
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName().startswith("gen3_joint_"):
        a = prim.GetAttribute("drive:angular:physics:targetPosition")
        arm_joints[prim.GetName()] = a.Get() if a and a.HasAuthoredValue() else 0.0

# Frame from the robot's actual bounding box, not from its prim translation.
# The translation is the authored one and the camera placed against it looked
# at a wall: after play the articulation sits wherever physics put it, and in a
# stage where the robot was spawned somewhere other than the origin the two
# are not the same point at all.
robot = stage.GetPrimAtPath("/World/VICA")
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
bbox = cache.ComputeWorldBound(robot).ComputeAlignedRange() if robot else None
if bbox and not bbox.IsEmpty():
    lo, hi = bbox.GetMin(), bbox.GetMax()
    base = Gf.Vec3d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0)
    span = max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
else:
    base, span = Gf.Vec3d(0, 0, 0), 1.2
    lo = hi = None
print(f"  로봇 경계상자 min {tuple(round(v, 3) for v in lo) if lo else '?'} "
      f"max {tuple(round(v, 3) for v in hi) if hi else '?'}")
print(f"  프레이밍 중심 {tuple(round(v, 3) for v in base)}  최대 변 {span:.3f} m")

cam_path = "/World/PoseCam"
camera = UsdGeom.Camera.Define(stage, cam_path)
camera.CreateFocalLengthAttr(28.0)
# One transform op, set per view. Clearing and re-adding the op each time left
# the second and third cameras without a usable transform and both frames came
# back a uniform 2.7 kB of nothing.
cam_xform = UsdGeom.Xformable(camera).AddTransformOp()

# The test room is lit for driving, not for photographing a robot: from ground
# level the chassis is a silhouette. A dome fills the shadows and a distant
# light gives the arm an edge, both added here rather than in the stage so the
# stage stays the thing that was measured.
from pxr import UsdLux  # noqa: E402

dome = UsdLux.DomeLight.Define(stage, "/World/PoseDome")
dome.CreateIntensityAttr(900.0)
key = UsdLux.DistantLight.Define(stage, "/World/PoseKey")
key.CreateIntensityAttr(2200.0)
key.CreateAngleAttr(1.2)
UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 0.0, 35.0))


def look_at(eye, target, up=None):
    """Camera-to-world matrix, same convention as replay_render.py.

    Rows are the basis vectors and there is no transpose: writing them as
    columns instead pointed the camera at a wall and rendered a corner of the
    test room with no robot in it.
    """
    eye, target = Gf.Vec3d(*eye), Gf.Vec3d(*target)
    z = (eye - target).GetNormalized()
    up = Gf.Vec3d(*up) if up is not None else Gf.Vec3d(0, 0, 1)
    if abs(Gf.Dot(z, up)) > 0.999:
        up = Gf.Vec3d(0, 1, 0)
    x = Gf.Cross(up, z).GetNormalized()
    y = Gf.Cross(z, x)
    m = Gf.Matrix4d(1.0)
    m.SetRow3(0, x)
    m.SetRow3(1, y)
    m.SetRow3(2, z)
    m.SetTranslateOnly(eye)
    return m


# Framing is fixed, not derived from the bounding box. BBoxCache reads the
# authored pose, and the authored pose is the arm standing straight up at
# 1.605 m; the played pose is 0.95 m. Framing on the taller one put the robot
# in the bottom eighth of the frame.
#
# The lens and the distance are a pair, and the vertical field is what binds.
# A USD camera's vertical aperture is 15.29 mm, so 45 mm at 2.4 m sees
# 2 * 2.4 * tan(atan(15.29 / 90)) = 0.82 m of height -- and the stowed robot is
# 0.95 m tall, which is why the chassis kept falling off the bottom of the
# frame while the arm looked fine. 28 mm at 3.2 m sees 1.75 m.
D = 4.2
# Aimed at the middle of the robot's own height, and level for the two
# orthogonal views: a camera looking down puts the ground plane high in the
# frame and pushes the chassis off the bottom edge, which is what kept
# cropping the very part these shots exist to show.
TARGET_Z = 0.42
VIEWS = {
    "side": Gf.Vec3d(0.0, -D, TARGET_Z),
    "front": Gf.Vec3d(D, 0.0, TARGET_Z),
    "iso": Gf.Vec3d(D * 0.72, -D * 0.72, 0.90),
}
ORBIT = int(_opt("--orbit", 0))
# A pose to move to and back from, so the GIF shows the arm working rather
# than the camera walking. Without it the animation proves the pose is held
# and nothing else, and holding still is the one thing a still already shows.
SWEEP = _opt("--sweep", None)
SWEEP_FRAMES = int(_opt("--sweep-frames", 48))

timeline = omni.timeline.get_timeline_interface()
timeline.play()
steps = int(SECONDS * 60)
for _ in range(steps):
    app.update()

# What the joints actually did, against what they were told.
held = {}
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName().startswith("gen3_joint_"):
        a = prim.GetAttribute("state:angular:physics:position")
        held[prim.GetName()] = a.Get() if a and a.HasAuthoredValue() else None

print(f"\n  {SECONDS:.0f} 초 재생 후")
print(f"  {'조인트':<16}{'목표(도)':>10}{'실제(도)':>10}{'오차':>9}")
worst = 0.0
for n in sorted(arm_joints):
    tgt = arm_joints[n]
    got = held.get(n)
    if got is None:
        print(f"  {n:<16}{tgt:10.1f}{'읽을 수 없음':>10}")
        continue
    err = got - tgt
    worst = max(worst, abs(err))
    print(f"  {n:<16}{tgt:10.1f}{got:10.1f}{err:+9.2f}")
if held:
    print(f"\n  최대 오차 {worst:.2f} 도 ->",
          "자세 유지 ✅" if worst < 2.0 else "유지 실패 ⚠")

# Hide everything that is not the robot before photographing it. The test room
# is a 10 x 10 m box with interior walls, and a camera 3.5 m from the robot
# stands inside one of them: the first attempts framed a wall correctly and the
# robot not at all. The stage is never saved here, so this affects the picture
# and nothing else.
KEEP = {"VICA", "PoseCam", "PoseDome", "PoseKey"}
hidden = 0
world = stage.GetPrimAtPath("/World")
if world:
    for child in world.GetChildren():
        if child.GetName() in KEEP:
            continue
        child.SetActive(False)
        hidden += 1
print(f"  로봇만 남기고 {hidden}개 prim 을 껐습니다 (스테이지는 저장하지 않습니다)")
for _ in range(10):
    app.update()

os.makedirs(OUT, exist_ok=True)
rp = rep.create.render_product(cam_path, (WIDTH, HEIGHT))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])
rep.orchestrator.set_capture_on_play(False)

from PIL import Image  # noqa: E402

# The stills and the sweep do not survive being captured in one launch: after
# the three views are grabbed the annotator keeps handing back a uniform frame,
# and 31 of 32 sweep frames came out blank. --no-stills runs the sweep on a
# fresh render product instead of arguing with that.
STILLS = "--no-stills" not in sys.argv

print()
for name, offset in (VIEWS.items() if STILLS else ()):
    eye = Gf.Vec3d(base[0], base[1], 0.0) + offset
    target = Gf.Vec3d(base[0], base[1], TARGET_Z)
    cam_xform.Set(look_at(eye, target))
    for _ in range(4):
        app.update()
        rep.orchestrator.step(rt_subframes=8)
    arr = rgb.get_data()
    for _ in range(30):
        if getattr(arr, "size", 0):
            break
        app.update()
        rep.orchestrator.step(rt_subframes=8)
        arr = rgb.get_data()
    a = np.asarray(arr)
    if a.size == 0:
        print(f"    {name}: 렌더 결과가 비었습니다")
        continue
    if a.ndim == 3 and a.shape[2] == 4:
        a = a[:, :, :3]
    path = os.path.join(OUT, f"{name}.png")
    Image.fromarray(a.astype("uint8")).save(path)
    print(f"    wrote {path}")

if SWEEP:
    import json as _json
    target = {k: float(v) for k, v in _json.load(open(SWEEP)).items()}
    drives = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName().startswith("gen3_joint_"):
            drives[prim.GetName()] = prim.GetAttribute(
                "drive:angular:physics:targetPosition")
    start = {n: arm_joints.get(n, 0.0) for n in drives}
    eye = Gf.Vec3d(base[0], base[1] - D, 0.55)
    cam_xform.Set(look_at(eye, Gf.Vec3d(base[0], base[1], TARGET_Z)))
    frames = []
    half = SWEEP_FRAMES // 2
    for i in range(SWEEP_FRAMES):
        # Out and back, eased, so the ends are held rather than snapped past.
        u = i / max(1, half - 1) if i < half else (SWEEP_FRAMES - 1 - i) / max(1, half - 1)
        u = min(1.0, max(0.0, u))
        s = u * u * (3 - 2 * u)
        for n, attr in drives.items():
            attr.Set(math.degrees(start[n] + (target.get(n, 0.0) - start[n]) * s))
        # Let physics settle and the renderer catch up before grabbing. One
        # step was not enough: the annotator handed back frames that were a
        # uniform background colour, and a GIF of those is a GIF of nothing.
        for _ in range(4):
            app.update()
            rep.orchestrator.step(rt_subframes=6)
        a = np.asarray(rgb.get_data())
        for _ in range(20):
            if a.size and a.reshape(-1, a.shape[-1]).std(axis=0).max() > 1.0:
                break
            app.update()
            rep.orchestrator.step(rt_subframes=6)
            a = np.asarray(rgb.get_data())
        if a.size == 0:
            continue
        if a.ndim == 3 and a.shape[2] == 4:
            a = a[:, :, :3]
        f = os.path.join(OUT, f"sweep_{i:03d}.png")
        Image.fromarray(a.astype("uint8")).save(f)
        frames.append(f)
    print(f"    sweep {len(frames)} 프레임")
    for n, attr in drives.items():          # 원래 자세로 되돌린다
        attr.Set(math.degrees(start[n]))

# An orbit, when asked for. A still says where the arm is; going round says
# whether it stays inside the body from every side, which is the claim the
# footprint constraint makes and the one a single viewpoint cannot show.
if ORBIT:
    frames = []
    for i in range(ORBIT):
        ang = 2.0 * math.pi * i / ORBIT
        eye = Gf.Vec3d(base[0] + D * 0.78 * math.cos(ang),
                       base[1] + D * 0.78 * math.sin(ang),
                       0.72)
        cam_xform.Set(look_at(eye, Gf.Vec3d(base[0], base[1], TARGET_Z)))
        for _ in range(3):
            app.update()
            rep.orchestrator.step(rt_subframes=6)
        a = np.asarray(rgb.get_data())
        if a.size == 0:
            continue
        if a.ndim == 3 and a.shape[2] == 4:
            a = a[:, :, :3]
        f = os.path.join(OUT, f"orbit_{i:03d}.png")
        Image.fromarray(a.astype("uint8")).save(f)
        frames.append(f)
    print(f"    orbit {len(frames)} 프레임")
    if frames:
        import subprocess
        gif = os.path.join(OUT, "orbit.gif")
        pal = os.path.join(OUT, "palette.png")
        common = ["-y", "-framerate", "12", "-i",
                  os.path.join(OUT, "orbit_%03d.png")]
        subprocess.run(["ffmpeg", *common, "-vf",
                        "scale=720:-1:flags=lanczos,palettegen", pal],
                       capture_output=True)
        rc = subprocess.run(
            ["ffmpeg", *common, "-i", pal, "-lavfi",
             "scale=720:-1:flags=lanczos[x];[x][1:v]paletteuse", gif],
            capture_output=True)
        print(f"    wrote {gif}" if rc.returncode == 0
              else f"    gif 인코딩 실패: {rc.stderr.decode()[-200:]}")

timeline.stop()
app.close()
