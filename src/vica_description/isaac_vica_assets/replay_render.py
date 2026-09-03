"""Re-render a recorded run with the real robot, from the results JSON.

    $ISAAC_SIM/python.sh replay_render.py <result.json> [options]

      --stage PATH     course USD (default: read from the run's spec)
      --view top|iso|chase|follow camera framing (default iso)
      --focal MM     lens, default 24. 14 is a wide angle and is how to see
                     the room around the robot indoors; distance cannot do it.
      --follow-margin F  how far back the follow view sits, as a multiple of
                     the robot's own radius. 1.55 fills the frame with the
                     robot; 4 gives the room around it. iso and top frame the
                     course and are wrong indoors -- they put the camera
                     through a ceiling.
                                  follow frames the robot itself, and is the
                                  one to use when the robot has to be seen
      --out DIR        where frames and video land (default media/replay/<name>)
      --fps N          output frame rate (default 20)
      --stride N       use every Nth track sample (default 1)
      --res WxH        render resolution (default 1280x720)
      --no-trail       do not draw the path behind the robot
      --seconds A:B    only this slice of the run

Why this exists
---------------
Every picture in the report so far has been a rectangle standing in for the
robot, drawn by matplotlib from the same numbers the table was made of. That
is honest about the path and silent about everything else: it cannot show that
the robot is where the numbers say, nor that the course is the course.

This replays the recorded pose into the actual stage and renders the actual
robot. It is a check as much as a picture. If the track and the stage disagree
-- a lane measured against a rebuilt course, a spawn that landed in a wall --
the robot drives through geometry on screen and the disagreement stops being
invisible.

Rendering afterwards rather than during
---------------------------------------
Recording the viewport while a trial runs would spend GPU on pixels that the
measurement needs for physics, and would change the thing being measured. It
also cannot be applied to a run that has already happened, and there are
twenty-five of those.

So the timeline never plays here. Physics stays stopped and the robot is
posed frame by frame from the track. Nothing is simulated, so nothing can
drift from what was recorded: the pose on screen is the pose in the file, by
construction. The cost is that the wheels do not turn -- they are drawn in
their rest pose, because no articulation is being solved.
"""

import json
import math
import os
import subprocess
import sys

ARGV = sys.argv[1:]
if not ARGV or ARGV[0].startswith("-"):
    raise SystemExit(__doc__.strip().splitlines()[2].strip())


def _opt(name, default=None):
    if name in ARGV:
        return ARGV[ARGV.index(name) + 1]
    for a in ARGV:
        if a.startswith(name + "="):
            return a.split("=", 1)[1]
    return default


RESULT = ARGV[0]
VIEW = _opt("--view", "iso")
FPS = int(_opt("--fps", "20"))
STRIDE = max(1, int(_opt("--stride", "1")))
TRAIL = "--no-trail" not in ARGV
RES = _opt("--res", "1280x720")
WIDTH, HEIGHT = (int(v) for v in RES.lower().split("x"))
SLICE = _opt("--seconds")

HERE = os.path.dirname(os.path.abspath(__file__))
run = json.load(open(RESULT))
name = os.path.splitext(os.path.basename(RESULT))[0]
tag = f"{run.get('controller', 'run')}_{name}"
OUT = _opt("--out", os.path.join(HERE, "..", "..", "..", "media", "replay", tag))
OUT = os.path.abspath(OUT)

# The stage the run was actually driven on, not whichever course is newest.
# The spec name travels in the results file for exactly this reason.
STAGE = _opt("--stage")
if STAGE is None:
    spec = run.get("spec", "")
    if not spec:
        raise SystemExit("결과에 spec 이 없습니다. --stage 로 지정하십시오.")
    STAGE = os.path.join(HERE, json.load(
        open(os.path.join(HERE, spec)))["stage"])
STAGE = os.path.abspath(STAGE)
if not os.path.exists(STAGE):
    raise SystemExit(f"스테이지가 없습니다: {STAGE}")

# The longest track in the file. A lane run holds one record; a repeat run
# holds several and the interesting one is the pass that went furthest.
tracks = [r["track"] for r in run.get("records", []) if r.get("track")]
if not tracks:
    raise SystemExit(f"{RESULT} 에 track 이 없습니다 (실패 기록만 있는 파일).")
track = max(tracks, key=len)
record = next(r for r in run["records"] if r.get("track") is track)

t0 = track[0][0]
track = [[t - t0, x, y, a] for t, x, y, a in track]
if SLICE:
    lo, hi = (float(v) for v in SLICE.split(":"))
    track = [s for s in track if lo <= s[0] <= hi]
track = track[::STRIDE]
if len(track) < 2:
    raise SystemExit("남은 프레임이 2개 미만입니다.")

print(f"=== {RESULT}")
print(f"    스테이지 {STAGE}")
print(f"    결과 {record.get('result')}  이동 {record.get('moved_m')} m  "
      f"track {len(track)} 프레임  view {VIEW}")

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom  # noqa: E402

if not omni.usd.get_context().open_stage(STAGE):
    simulation_app.close()
    raise SystemExit(f"스테이지를 열지 못했습니다: {STAGE}")
for _ in range(30):
    simulation_app.update()

# Same reason as play_stage.py: the default open does not always bring
# payloads in, and a robot whose payload is missing renders as an empty Xform
# -- an empty frame that looks like a camera problem.
stage = omni.usd.get_context().get_stage()
stage.Load()
simulation_app.update()

ROBOT = "/World/VICA"
robot = stage.GetPrimAtPath(ROBOT)
if not robot.IsValid():
    simulation_app.close()
    raise SystemExit(f"{ROBOT} 이 스테이지에 없습니다.")


def pose_ops(prim):
    """Translate and Z-rotate ops on `prim`, created if missing, in order.

    The course builders author a translate op and nothing else, so the yaw op
    usually has to be added. Appending it is not enough: an op absent from
    xformOpOrder is authored and ignored, which renders as a robot that slides
    along the path without ever turning. Set the order explicitly.
    """
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    t = ops.get("xformOp:translate") or xf.AddTranslateOp()
    r = ops.get("xformOp:rotateZ") or xf.AddRotateZOp()
    xf.SetXformOpOrder([t, r])
    return t, r


translate, rotate = pose_ops(robot)

xs = [s[1] for s in track]
ys = [s[2] for s in track]
cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
# Frame the run, not the course. A 32 m course around a 9 m lane renders the
# robot four pixels tall.
x_span = max(max(xs) - min(xs), 3.0) + 2.0
y_span = max(max(ys) - min(ys), 3.0) + 2.0
span = max(x_span, y_span)

# Lens. 24 mm on a 20.955 mm aperture is about 47 degrees across, which is a
# normal lens and right for following the robot outdoors or down a course.
#
# Indoors, widening the shot means a wider lens and not more distance. Backing
# off does not work in a building: at a margin of 4 the follow camera wants 14 m
# of standoff and the office render came back as a picture of the sky through a
# curtain wall; putting that distance into height instead put it through the
# ceiling. 14 mm sees about 74 degrees from the same place, which is the room
# around the robot.
FOCAL_MM = float(_opt("--focal", "24.0"))

# Exposure, in stops. Indoors the stage is lit by one distant key that the roof
# blocks and a dome that a closed ceiling mostly blocks too, so a room that
# looks right in the viewport renders two stops down and the office came back
# looking like a power cut. This is the camera's own setting and changes
# nothing about the run; brightening the stage instead would mean rebuilding
# it, and would light the hospital wrong to fix the office.
EXPOSURE = float(_opt("--exposure", "0.0"))

camera = UsdGeom.Camera.Define(stage, "/World/ReplayCam")
camera.CreateFocalLengthAttr(FOCAL_MM)
camera.CreateExposureAttr(EXPOSURE)
camera.CreateHorizontalApertureAttr(20.955)
camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 500.0))
cam_xf = UsdGeom.Xformable(camera.GetPrim())
cam_xf.SetXformOpOrder([])
cam_op = cam_xf.AddTransformOp()

# A sphere around base_link that holds the whole robot, mast included, and the
# height of its centre. The mast tops out at 1.045 m and the body is 0.83 m
# long, so a centre at 0.52 m and a radius of 0.86 m contains both.
ROBOT_RADIUS = 0.86
ROBOT_CENTRE_Z = 0.52
# Room around it. 1.55 keeps the robot about two thirds of the frame height,
# which leaves the obstacle it is reacting to in shot without the robot itself
# touching an edge.
FOLLOW_MARGIN = float(_opt("--follow-margin", "1.55"))
# How far behind the robot the follow camera may stand before the rest of the
# distance is taken as height. A corridor is about 2 m wide and a room a few
# metres deep, so 3 m back is about as far as anything indoors allows.
FOLLOW_MAX_BACK = float(_opt("--follow-max-back", "3.0"))

HFOV = 2 * math.atan(20.955 / 2 / FOCAL_MM)
VFOV = 2 * math.atan(20.955 * HEIGHT / WIDTH / 2 / FOCAL_MM)


def look_at(eye, target, up=None):
    """Camera-to-world matrix for a camera at `eye` pointing at `target`.

    USD cameras look down their own -Z with +Y up. Straight-down views make
    the world up vector parallel to the view direction, where the cross
    product collapses and the matrix comes out singular -- the frame renders
    black with no error. Swap the up vector before that happens rather than
    after.
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


def camera_for(sample):
    _, x, y, a = sample
    if VIEW == "top":
        # Lay the long axis of the run along the long axis of the frame.
        #
        # A lane is 2.6 m wide and 9 m long, and a 16:9 frame is wider than it
        # is tall. Keeping map-north up meant fitting 9 m into the short axis,
        # which pulled the camera back far enough to show three neighbouring
        # lanes and shrink the robot to a smear. Turning the view a quarter
        # turn costs nothing and roughly halves the altitude needed.
        sideways = y_span > x_span
        long_span, short_span = ((y_span, x_span) if sideways
                                 else (x_span, y_span))
        h = max((long_span / 2) / math.tan(HFOV / 2),
                (short_span / 2) / math.tan(VFOV / 2))
        return look_at((cx, cy, h), (cx, cy, 0.0),
                       up=(1, 0, 0) if sideways else (0, 1, 0))
    if VIEW == "chase":
        # Behind and above the robot. 3.5 m back is enough to hold the robot
        # and the gap it is aiming at in one frame.
        back, up = 3.5, 2.2
        eye = (x - back * math.cos(a), y - back * math.sin(a), up)
        return look_at(eye, (x + 1.5 * math.cos(a), y + 1.5 * math.sin(a), 0.3))
    if VIEW == "follow":
        # The robot filling the frame, kept there for the whole run.
        #
        # top and iso both frame the course, which is the right choice for
        # seeing a path and the wrong one for seeing the robot: over a 27 m
        # approach it shrinks to a smear, and at the ends of the run it sits
        # against the edge of the frame with part of it outside. This view
        # ignores the course and frames the machine.
        #
        # Distance is solved rather than picked, from the vertical field of
        # view because it is the narrower one, so the margin holds at any
        # output size. ROBOT_RADIUS is a sphere around base_link that contains
        # the whole robot including the mast: half of the 0.83 m body length,
        # and 1.045 m to the top of the mast from the ground, is 0.86 m from a
        # centre at half that height.
        r = ROBOT_RADIUS * FOLLOW_MARGIN
        d = r / math.tan(VFOV / 2)
        # Behind and to one side, low enough that the mast reads as a mast
        # rather than as a line pointing at the camera.
        #
        # The extra distance a wide margin asks for goes upward once the
        # horizontal part reaches FOLLOW_MAX_BACK. Indoors, backing straight
        # off does not widen the shot, it leaves the building: at margin 4 the
        # camera wants 14 m of standoff and the office render came back as a
        # picture of the sky through a curtain wall. Height has no wall in the
        # way, and looking down is what actually shows the room around the
        # robot.
        back = min(d, FOLLOW_MAX_BACK)
        up = math.sqrt(max(d * d - back * back, 0.0)) + back * 0.38
        eye = (x - back * 0.91 * math.cos(a) - back * 0.42 * math.sin(a),
               y - back * 0.91 * math.sin(a) + back * 0.42 * math.cos(a),
               ROBOT_CENTRE_Z + up)
        return look_at(eye, (x, y, ROBOT_CENTRE_Z))
    d = (span / 2) / math.tan(HFOV / 2)
    return look_at((cx + 0.55 * d, cy - 0.75 * d, 0.75 * d), (cx, cy, 0.0))


trail_pts = None
if TRAIL:
    # A tube along the path already driven. Rebuilt each frame rather than
    # animated, because a curve with time samples needs the timeline to move
    # and the timeline is deliberately stopped here.
    curve = UsdGeom.BasisCurves.Define(stage, "/World/ReplayTrail")
    curve.CreateTypeAttr(UsdGeom.Tokens.linear)
    curve.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curve.CreateWidthsAttr([0.05, 0.05])
    curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    curve.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.35, 0.0)])
    curve.CreateCurveVertexCountsAttr([2])
    curve.CreatePointsAttr([Gf.Vec3f(0, 0, -50), Gf.Vec3f(0, 0, -50)])
    trail_pts = curve

rp = rep.create.render_product("/World/ReplayCam", (WIDTH, HEIGHT))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])

# Replicator captures on timeline play by default, and the timeline here never
# plays -- so app.update() alone renders the viewport and leaves the annotator
# empty forever. Turn that off and drive the graph explicitly instead.
rep.orchestrator.set_capture_on_play(False)


def render_once(subframes=4):
    """Advance the replicator graph without advancing time.

    delta_time=0.0 keeps the timeline where it is: this is a pose replay, and
    a graph step that moved time would let anything time-sampled in the stage
    drift away from the pose being drawn. The keyword set has changed between
    Isaac releases, so fall back rather than pin a signature.
    """
    try:
        rep.orchestrator.step(delta_time=0.0, rt_subframes=subframes,
                              pause_timeline=False)
    except TypeError:
        rep.orchestrator.step(rt_subframes=subframes)

os.makedirs(OUT, exist_ok=True)
for stale in os.listdir(OUT):
    if stale.endswith((".png", ".ppm")):
        os.remove(os.path.join(OUT, stale))

try:
    from PIL import Image
except ImportError:
    Image = None


def grab():
    """One RGB frame from the annotator, waiting for it to have one.

    The annotator hands back a zero-length array until the render product has
    produced its first frame, and a flat buffer rather than an image on some
    paths. Neither raises; both arrive as an array that is simply the wrong
    shape, which surfaces far away as "too many indices". Resolve it here.
    """
    for _ in range(20):
        a = np.asarray(rgb.get_data())
        if a.size >= WIDTH * HEIGHT * 3:
            if a.ndim == 1:
                a = a.reshape(HEIGHT, WIDTH, a.size // (WIDTH * HEIGHT))
            return a
        render_once()
    raise SystemExit("렌더 결과가 비어 있습니다 (annotator 가 프레임을 내지 않음)")


def write_frame(arr, path_png):
    a = np.asarray(arr)[:, :, :3]
    if Image is not None:
        Image.fromarray(a).save(path_png)
        return path_png
    # ffmpeg reads binary PPM natively, so a missing Pillow costs a little
    # disk and nothing else.
    ppm = path_png[:-4] + ".ppm"
    with open(ppm, "wb") as fh:
        fh.write(b"P6\n%d %d\n255\n" % (a.shape[1], a.shape[0]))
        fh.write(a.tobytes())
    return ppm


ext = "png"
for i, sample in enumerate(track):
    _, x, y, a = sample
    translate.Set(Gf.Vec3d(x, y, 0.0))
    rotate.Set(math.degrees(a))
    cam_op.Set(camera_for(sample))
    if trail_pts is not None and i >= 1:
        pts = [Gf.Vec3f(s[1], s[2], 0.02) for s in track[:i + 1]]
        trail_pts.GetPointsAttr().Set(pts)
        trail_pts.GetCurveVertexCountsAttr().Set([len(pts)])

    # One update to push the new transforms through, then a graph step to
    # render against them. Capturing without the update took the previous
    # frame's pose and the whole video lagged by one sample -- invisible
    # except at the moment the robot stops, which is the moment these videos
    # exist to show.
    simulation_app.update()
    render_once()
    out = write_frame(grab(), os.path.join(OUT, f"frame_{i:05d}.png"))
    ext = out.rsplit(".", 1)[1]
    if i % 50 == 0:
        print(f"    {i}/{len(track)}", flush=True)

print(f"=== {len(track)} 프레임 -> {OUT}", flush=True)

# Encode before closing the app. SimulationApp.close() ends the process, so
# everything written after it simply never ran: the first version of this
# script reported the frames it had rendered and produced no video at all,
# with no error, because the encoder was three lines below the close.
# Named after the output directory, not after the results file.
#
# The video used to take its name from the JSON, so two runs whose result files
# happened to share a name overwrote each other's video while keeping both sets
# of frames. A hospital tour and an office tour were both tour.json, and the
# second silently replaced the first.
_stem = os.path.basename(OUT.rstrip(os.sep)) or tag
mp4 = os.path.join(os.path.dirname(OUT), f"{_stem}.mp4")
gif = os.path.join(os.path.dirname(OUT), f"{_stem}.gif")
pattern = os.path.join(OUT, f"frame_%05d.{ext}")
enc = ["ffmpeg", "-y", "-framerate", str(FPS), "-i", pattern,
       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", mp4]
if subprocess.run(enc, capture_output=True).returncode == 0:
    print(f"=== {mp4}")
    # Palette pass: the default 256-colour quantiser turns a grey corridor
    # into visible banding, and the corridor walls are the thing being looked
    # at in these clips.
    pal = os.path.join(OUT, "palette.png")
    subprocess.run(["ffmpeg", "-y", "-i", mp4, "-vf",
                    "fps=12,scale=720:-1:flags=lanczos,palettegen", pal],
                   capture_output=True)
    if subprocess.run(
            ["ffmpeg", "-y", "-i", mp4, "-i", pal, "-lavfi",
             "fps=12,scale=720:-1:flags=lanczos[v];[v][1:v]paletteuse", gif],
            capture_output=True).returncode == 0:
        print(f"=== {gif}")
else:
    print("ffmpeg 인코딩 실패 -- 프레임은 남아 있습니다", file=sys.stderr)

simulation_app.close()
