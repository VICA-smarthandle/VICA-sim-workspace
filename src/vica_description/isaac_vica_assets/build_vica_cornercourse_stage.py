"""Build a course of L-corners, one per width, and put VICA at the mouth.

    $ISAAC_SIM/python.sh build_vica_cornercourse_stage.py

Then: make_stage.sh --prepare-only vica_cornercourse.usd

Why a third course
------------------
The width course measures a straight corridor and the avoid course measures a
short bottleneck beside a block. Between them they answered how narrow a
straight place can be, and the answer split in two: 1.20 m when the narrow
stretch runs 5.4 m, 1.00 m when it runs 0.6 m.

Neither asks the robot to turn while it is in there. That matters here more
than it would on most robots, because turning is where this one's footprint
stops being its width and starts being its circumscribed circle:

    padded width, travelling straight      0.505 m
    padded circumscribed diameter          0.945 m

2026-09-02: both numbers moved. The robot's body was shortened on 2026-08-29
(handle 19 -> 11 cm, body 80 -> 72 cm) and the footprint here followed, so the
circumscribed diameter went 1.301 -> 0.945 and the padded width 0.555 -> 0.505.
Every corner result recorded before that date was measured on the wider robot
and is not comparable to one recorded after.

The diameter still comes from the handle, now 0.465 m behind base_link once
padded. Going straight the handle is 0.085 m off axis and irrelevant;
pivoting, it sweeps the whole circle. A corridor the robot fits through can
therefore be a corridor it cannot turn in, and nothing else measured here
would have shown that.

The default widths bracket the new diameter: 1.50 and 1.20 above it, 1.00 just
above, 0.90 and 0.80 below. If the corner limit sits near 0.945 the sweep is
about the circumscribed circle. If it sits higher, cornering costs something
the circle does not explain. Either answer is worth having.

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
    # Falls back to VICA_WS when __file__ is undefined, which is what happens
    # when Isaac's Script Editor exec's this file. A hardcoded path worked on
    # one machine and made the repository unusable on any other.
    HERE = os.path.join(os.environ.get("VICA_WS", os.getcwd()),
                        "src/vica_description/isaac_vica_assets")

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

# Bracketing the padded circumscribed diameter, 0.9454 m.
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
                                "0.80,0.90,1.00,1.20,1.50").split(",")]

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


# --------------------------------------------------------------------------
# Low obstacle
# --------------------------------------------------------------------------
# A box too short for the lidar or the depth band to see. The lidar sweeps at
# 0.382 above the ground and the costmap's depth_scan band starts at 0.30, so
# at 0.25 this is invisible to both; the ultrasonic probes sit at 0.091 and it
# is the only thing that sees it.
#
# Without one of these the probes are decoration. Every wall in these courses
# is 1.5 m tall and the lidar reports all of it, so a run with the ultrasonic
# layers enabled and a run without them would produce identical numbers, and
# the layers would be untested rather than shown to work.
#
# Width is derived, not chosen. A fixed 0.30 m box in a 0.80 m lane leaves
# 0.50, and the padded footprint is 0.505 wide -- the lane would be sealed and
# the trial would measure a wall. This leaves at least LOW_MIN_GAP free.
LOW_OBSTACLE = os.environ.get("VICA_LOW_OBSTACLE", "1") not in ("0", "false", "")
LOW_HEIGHT = 0.25
LOW_DEPTH = 0.30           # along the direction of travel
LOW_MIN_GAP = 0.62         # padded width 0.505, plus a little
LOW_MAX_WIDTH = 0.30


def _low_width(lane_width):
    """How far the box may intrude, leaving LOW_MIN_GAP to pass through."""
    return max(0.0, min(LOW_MAX_WIDTH, lane_width - LOW_MIN_GAP))


def _low_box(stage, path, x0, y0, x1, y1):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    prim = cube.GetPrim()
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(
        Gf.Vec3d((x0 + x1) / 2.0, (y0 + y1) / 2.0, LOW_HEIGHT / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3f(abs(x1 - x0), abs(y1 - y0), LOW_HEIGHT))
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


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

    # In the entry leg, against its outer wall, before the corner. The robot
    # must clear it while already lining up for the turn, which is where the
    # padded footprint is widest across the direction of travel.
    lw = _low_width(w)
    if LOW_OBSTACLE and lw > 0:
        y = ENTRY_LEN / 2.0
        # X already carries the cell's hand. Multiplying by s again here
        # squared it, so the box was laid out along world +x whichever way the
        # cell faced, and in the left-handed cells -- 0.90 and 1.20 -- that put
        # it entirely inside the outer wall. Corner_120's corridor runs
        # x 8.05..9.25 and its box sat at 9.25..9.55. Two of the five cells had
        # no low obstacle in them at all, and 1.20 is the screening width, so
        # the screen was the one measuring a bare corner.
        a, b = X(-w / 2), X(-w / 2 + lw)
        _low_box(stage, f"{root}/LowBox",
                 min(a, b), y - LOW_DEPTH / 2, max(a, b), y + LOW_DEPTH / 2)

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
    # The floor box is drawn but does not collide. A ground plane collides
    # instead, because PhysX drops contacts between a box this large and a
    # collider as small as a 65 mm wheel, and drops them at some places and not
    # others. Settling height of the robot along the corner course, same stage,
    # one change between the rows:
    #
    #     floor box      -22 0.183  -8.65 0.190  -4.3 0.171  0 0.142
    #                    +8.65 0.142  +13 0.142  +22 0.170
    #     ground plane   all seven 0.190
    #
    # 0.190 is the wheels carrying the robot. 0.142 is the chassis box sitting
    # on the floor with the wheels 48 mm underneath it, which is the tell: the
    # large collider kept its contacts and the small one did not. At x=0 the
    # wheels spun 5050 degrees in 14 s and the robot moved 0.000 m.
    #
    # Keep the box. It is what the depth camera and the renders see, and the
    # plane is purpose=guide so it draws nothing.
    plane = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
    plane.CreateAxisAttr("Z")
    plane.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

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
        "circumscribed_diameter": 0.9454,
    }
    with open(TARGETS, "w") as fh:
        json.dump(spec, fh, indent=2)

    print(f"wrote {OUTPUT}")
    print(f"wrote {TARGETS}")
    print(f"  widths {CORNER_WIDTHS}  (circumscribed diameter 0.9454)")
    print(f"  entry {ENTRY_LEN} m, exit {EXIT_LEN} m, pitch {LANE_PITCH} m")
    print(f"  extent x {x_min:.2f}..{x_max:.2f}  y {y_min:.2f}..{y_max:.2f}")
    for i, w in enumerate(CORNER_WIDTHS):
        print(f"    {w:.2f} m  cell x {_cell_origin(i):+7.2f}  "
              f"turn {'right' if _turn_sign(i) > 0 else 'left ':5}  "
              f"goal ({goals[i][0]:+7.2f}, {goals[i][1]:+6.2f})")


main()
