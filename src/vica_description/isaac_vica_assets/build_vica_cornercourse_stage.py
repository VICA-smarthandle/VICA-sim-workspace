"""Build a course of L-corners, one per width, and put VICA at the mouth.

    /home/sim/isaacsim/python.sh build_vica_cornercourse_stage.py

Then: make_stage.sh --prepare-only vica_cornercourse.usd

Why a third course
------------------
The width course measures a straight corridor and the avoid course measures a
short bottleneck beside a block. Between them they answered how narrow a
straight place can be, and the answer split in two: 1.20 m when the narrow
stretch runs 5.4 m, 1.00 m when it runs 0.6 m.

Neither asks the robot to turn while it is in there. That matters here more
than it would on most robots, because turning is where this one's footprint
stops being 0.555 m wide and starts being 1.3012 m across:

    padded width, travelling straight      0.555 m
    padded circumscribed diameter          1.3012 m

The 1.3012 comes from the handle, 0.645 m behind base_link once padded. Going
straight the handle is 0.085 m off axis and irrelevant; pivoting, it sweeps the
whole circle. A corridor the robot fits through can therefore be a corridor it
cannot turn in, and nothing measured so far would have shown that.

The widths bracket that diameter deliberately: 1.80 and 1.60 above it, 1.30 on
it, 1.20 and 1.10 below. If the corner limit sits near 1.30 the sweep is about
the circumscribed circle. If it sits near 1.20, the same place the straight
corridor failed, then cornering costs nothing extra and the circle is not the
binding constraint. Either answer is worth having and the sweep cannot give
both.

Geometry
--------
Each cell is one right-angled corner, entered northbound and left along the
turn. Cells alternate which way they turn so a robot that is better at one
hand does not read as a robot that is better at wide corners.

        exit leg
    +-----------------
    |
    |   inner corner
    |   +------------
    |   |
    |   |  entry leg
    |   |

The entry leg is 3.0 m so the approach is a real approach rather than a pose
the robot starts in, and the exit leg is 3.0 m so arriving is distinguishable
from nosing into the opening and stopping.
"""

import json
import math
import os

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:                      # exec'd from the Script Editor
    HERE = "/home/sim/vica_ws/src/vica_description/isaac_vica_assets"

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

ROBOT = os.path.join(HERE, "robot", "vica", "vica.usda")
OUTPUT = os.path.join(HERE, "vica_cornercourse.usd")
TARGETS = os.path.join(HERE, "vica_cornercourse.json")

PHYSICS_VARIANT_SET = "Physics"
PHYSICS_VARIANT = "physx"

WALL_THICKNESS = 0.15
WALL_HEIGHT = 1.5
FLOOR_MARGIN = 2.0
FLOOR_THICKNESS = 0.5

# Bracketing the padded circumscribed diameter, 1.3012 m.
#
# Overridable so a single width can be built and probed without editing the
# sweep. That is not a convenience: the first build of this course failed its
# widest cell, and a course whose widest cell fails is telling you about
# itself rather than about the robot. Probing one very wide corner separates
# the two in five minutes instead of ninety.
#
#     VICA_CORNER_WIDTHS=3.00 build_vica_cornercourse_stage.py
CORNER_WIDTHS = [float(v) for v in
                 os.environ.get("VICA_CORNER_WIDTHS",
                                "1.80,1.60,1.40,1.30,1.20,1.10").split(",")]

ENTRY_LEN = 3.0          # northbound run before the corner
EXIT_LEN = 3.0           # run along the turn after it
# Centre to centre, derived rather than chosen. Cells alternate their hand, so
# a right-turning cell's exit leg reaches towards the next cell's left-turning
# one and the two meet in the middle. A pitch that clears the widest cell on
# its own is not a pitch that clears two facing each other: 6.4 looked ample
# and three of the six pairs overlapped.
LANE_PITCH = 0.0         # set below, from the widths
APPROACH_DEPTH = 5.0     # open strip the cell mouths face onto
GOAL_INSET = 0.8         # how far short of the exit leg's end the goal sits


def _required_pitch():
    """The widest pair of facing exit legs, plus a metre of daylight."""
    worst = 0.0
    for a, b in zip(CORNER_WIDTHS, CORNER_WIDTHS[1:]):
        worst = max(worst, (a / 2.0 + EXIT_LEN + WALL_THICKNESS)
                    + (b / 2.0 + EXIT_LEN + WALL_THICKNESS))
    return worst + 1.0


LANE_PITCH = _required_pitch()


def _cell_origin(index):
    span = LANE_PITCH * (len(CORNER_WIDTHS) - 1)
    return -span / 2.0 + index * LANE_PITCH


def _turn_sign(index):
    """+1 turns towards +x, -1 towards -x."""
    return 1 if index % 2 == 0 else -1


def _cell_top(width):
    """y of the outer wall: the entry run plus the width of the exit leg."""
    return ENTRY_LEN + width


def _course_extent():
    top = max(_cell_top(w) for w in CORNER_WIDTHS)
    reach = max(w / 2.0 for w in CORNER_WIDTHS) + EXIT_LEN + WALL_THICKNESS
    x_min = _cell_origin(0) - reach - LANE_PITCH / 4.0
    x_max = _cell_origin(len(CORNER_WIDTHS) - 1) + reach + LANE_PITCH / 4.0
    return x_min, x_max, -APPROACH_DEPTH, top + 2.0


def _wall(stage, path, x0, y0, x1, y1):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    prim = cube.GetPrim()
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(
        Gf.Vec3d((x0 + x1) / 2.0, (y0 + y1) / 2.0, WALL_HEIGHT / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3f(abs(x1 - x0), abs(y1 - y0), WALL_HEIGHT))
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def _cell(stage, index, width):
    """One L, built from four walls.

    Written for a right turn and mirrored in x for a left one, so the two
    hands are the same geometry rather than two pieces of arithmetic that
    have to be kept in agreement.
    """
    xc = _cell_origin(index)
    s = _turn_sign(index)
    w, t = width, WALL_THICKNESS
    top = _cell_top(w)
    inner_y = top - w                      # the exit leg's near edge
    far = w / 2.0 + EXIT_LEN               # how far the exit leg reaches
    root = f"/World/Course/Corner_{int(round(width * 100)):03d}"
    UsdGeom.Xform.Define(stage, root)

    def X(v):
        return xc + s * v

    # Outer side of the entry leg, running the whole way up to the outer
    # corner: this is the wall the robot's handle sweeps towards when it
    # pivots, and the reason a corner is not just a corridor.
    _wall(stage, f"{root}/EntryOuter", X(-w / 2 - t), 0.0, X(-w / 2), top)
    # Inner side, stopping where the corner opens.
    _wall(stage, f"{root}/EntryInner", X(w / 2), 0.0, X(w / 2 + t), inner_y)
    # Outer wall of the exit leg, continuing the entry leg's outer side.
    _wall(stage, f"{root}/ExitOuter", X(-w / 2 - t), top, X(far + t), top + t)
    # Inner wall of the exit leg, forming the inside of the corner.
    _wall(stage, f"{root}/ExitInner", X(w / 2), inner_y - t, X(far + t), inner_y)

    goal = (X(far - GOAL_INSET), inner_y + w / 2.0)
    return top, goal


def main():
    if os.path.exists(OUTPUT):
        os.remove(OUTPUT)
    if not os.path.exists(ROBOT):
        raise RuntimeError(f"robot not found: {ROBOT}")

    stage = Usd.Stage.CreateNew(OUTPUT)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    x_min, x_max, y_min, y_max = _course_extent()
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    fx = UsdGeom.Xformable(floor.GetPrim())
    fx.AddTranslateOp().Set(Gf.Vec3d((x_min + x_max) / 2.0,
                                     (y_min + y_max) / 2.0,
                                     -FLOOR_THICKNESS / 2.0))
    fx.AddScaleOp().Set(Gf.Vec3f(x_max - x_min + 2 * FLOOR_MARGIN,
                                 y_max - y_min + 2 * FLOOR_MARGIN,
                                 FLOOR_THICKNESS))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    light = UsdLux.DomeLight.Define(stage, "/World/Dome")
    light.CreateIntensityAttr(1200.0)

    UsdGeom.Xform.Define(stage, "/World/Course")

    # Enclose the approach strip. make_map_from_stage builds the grid from the
    # walls, so a strip with no walls in it is not in the map -- and the robot
    # spawns at y -1.8, which the first build left outside the grid entirely.
    # It is also the more honest course: the robot reaches a corner along a
    # corridor rather than across an open field.
    _wall(stage, "/World/Course/ApproachBack",
          x_min, -APPROACH_DEPTH - WALL_THICKNESS, x_max, -APPROACH_DEPTH)
    _wall(stage, "/World/Course/ApproachWest",
          x_min - WALL_THICKNESS, -APPROACH_DEPTH, x_min, 0.0)
    _wall(stage, "/World/Course/ApproachEast",
          x_max, -APPROACH_DEPTH, x_max + WALL_THICKNESS, 0.0)

    goals = []
    for i, width in enumerate(CORNER_WIDTHS):
        _, goal = _cell(stage, i, width)
        goals.append(goal)

    robot = UsdGeom.Xform.Define(stage, "/World/VICA")
    robot.GetPrim().GetPayloads().AddPayload(
        os.path.relpath(ROBOT, os.path.dirname(OUTPUT)))
    # Facing north, into the cell, rather than east.
    #
    # Every other course spawns the robot facing east and lets it turn into
    # the lane, and that turn is free there. It is not free here: the first
    # smoke test moved 0.0 m with 1.383 m of clearance, because the robot has
    # to pivot 90 degrees before it reaches the corner and this robot delivers
    # 73 to 85 % of a commanded rotation. The trial would have measured the
    # turn-in-place deficit and reported it as a corner result.
    start_x, start_y = _cell_origin(0), -1.8
    start_yaw = math.pi / 2
    UsdGeom.Xformable(robot).AddTranslateOp().Set(
        Gf.Vec3d(start_x, start_y, 0.0))
    UsdGeom.Xformable(robot).AddRotateXYZOp().Set(
        Gf.Vec3f(0.0, 0.0, math.degrees(start_yaw)))

    vset = stage.GetRootLayer()
    prim = world.GetPrim()
    vs = prim.GetVariantSets().AddVariantSet(PHYSICS_VARIANT_SET)
    vs.AddVariant(PHYSICS_VARIANT)
    vs.SetVariantSelection(PHYSICS_VARIANT)
    stage.GetRootLayer().Save()

    # Same schema as the other two courses, so width_trials reads any of them
    # without knowing which it has. entry_yaw and exit_yaw are what a corner
    # adds: the exit faces along the leg it ends on, not north.
    spec = {
        "stage": os.path.basename(OUTPUT),
        "start": [start_x, start_y],
        "start_yaw": start_yaw,
        "lanes": [
            {
                "width": w,
                "entry": [_cell_origin(i), -1.0],
                "entry_yaw": math.pi / 2,
                "exit": [goals[i][0], goals[i][1]],
                "exit_yaw": 0.0 if _turn_sign(i) > 0 else math.pi,
                "turn": "right" if _turn_sign(i) > 0 else "left",
            }
            for i, w in enumerate(CORNER_WIDTHS)
        ],
        "entry_len": ENTRY_LEN,
        "exit_len": EXIT_LEN,
        "circumscribed_diameter": 1.3012,
    }
    with open(TARGETS, "w") as fh:
        json.dump(spec, fh, indent=2)

    print(f"wrote {OUTPUT}")
    print(f"wrote {TARGETS}")
    print(f"  widths {CORNER_WIDTHS}  (circumscribed diameter 1.3012)")
    print(f"  entry {ENTRY_LEN} m, exit {EXIT_LEN} m, pitch {LANE_PITCH} m")
    print(f"  extent x {x_min:.2f}..{x_max:.2f}  y {y_min:.2f}..{y_max:.2f}")
    for i, w in enumerate(CORNER_WIDTHS):
        print(f"    {w:.2f} m  cell x {_cell_origin(i):+7.2f}  "
              f"turn {'right' if _turn_sign(i) > 0 else 'left ':5}  "
              f"goal ({goals[i][0]:+7.2f}, {goals[i][1]:+6.2f})")


main()
