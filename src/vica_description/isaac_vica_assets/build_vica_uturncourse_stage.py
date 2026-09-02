"""Build a course of dead-end corridors, one per width, for turning around in.

    $ISAAC_SIM/python.sh build_vica_uturncourse_stage.py

Then: make_stage.sh --prepare-only vica_uturncourse.usd

Why a fourth course
-------------------
The other three ask the robot to keep going. The width course measures a
straight passage, the avoid course a bottleneck beside a block, the corner
course a right-angle. In all three the robot leaves by a different opening
than it came in, and never has to reverse its heading.

The robot's own driving notes put a user requirement the other way round:
turn around inside a 3 m corridor, in 1.5 m of it. The real robot failed that
at 1.3 m on 2026-08-29, and the day spent on it closed two axes without
closing the requirement. Nothing in this repository could reproduce the
failure, because no course here asks for a U-turn.

This one asks for nothing else. Each cell is a corridor closed at the far end:
drive in, turn 180 degrees, drive out. There is no other way to satisfy it.

What the widths bracket
-----------------------
Turning on the spot needs the padded circumscribed circle to fit, and after
the 2026-08-29 body shortening that circle is 0.945 m across:

    padded width, travelling straight      0.505 m
    padded circumscribed diameter          0.945 m   (was 1.301)

So 1.00 m clears it by 55 mm and everything above is progressively easier.
Below 0.945 the answer is arithmetic rather than a measurement, which is why
this sweep starts at 1.00 while the corner and avoid courses go down to 0.80.

    1.00   55 mm of daylight. Turning on the spot is the only option that fits
    1.20   255 mm. A slight arc becomes possible
    1.50   the user's requirement, stated as a number
    1.80   an arc of radius 0.20 needs 1.186 m; this clears that comfortably
    2.00   wide enough that failure here is not about width at all

That last one earns its place. The first build of the corner course failed
its widest cell, and a course whose widest cell fails is describing itself
rather than the robot. A cell nobody can blame the walls for is how that gets
noticed in five minutes instead of ninety.

Geometry
--------
        +--------+      dead end
        |        |
        |        |      DEADEND_LEN of corridor, `width` across
        |        |
    ----+        +----  mouth, opening onto the approach strip
                        (robot spawns below, facing north)

The corridor is 4.0 m deep so the robot is fully inside before it turns, and
the goal it must reach on the way out sits on the approach strip rather than
in the mouth: a robot that pivots in the mouth with its nose still in the
corridor has not turned around in the corridor.
"""

import json
import math
import os

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:                      # exec'd from the Script Editor
    HERE = os.path.join(os.environ.get("VICA_WS", os.getcwd()),
                        "src/vica_description/isaac_vica_assets")

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

ROBOT = os.path.join(HERE, "robot", "vica", "vica.usda")
OUTPUT = os.path.join(HERE, "vica_uturncourse.usd")
TARGETS = os.path.join(HERE, "vica_uturncourse.json")

PHYSICS_VARIANT_SET = "Physics"
PHYSICS_VARIANT = "physx"

WALL_THICKNESS = 0.15
WALL_HEIGHT = 1.5
FLOOR_MARGIN = 2.0
FLOOR_THICKNESS = 0.5

# Overridable for the same reason the other two courses are: building one
# width and probing it separates a course problem from a robot problem.
#
#     VICA_UTURN_WIDTHS=3.00 build_vica_uturncourse_stage.py
UTURN_WIDTHS = [float(v) for v in
                os.environ.get("VICA_UTURN_WIDTHS",
                               "1.00,1.20,1.50,1.80,2.00").split(",")]

DEADEND_LEN = 4.0        # corridor depth, mouth to dead end
APPROACH_DEPTH = 5.0     # open strip the mouths face onto
# How far into the corridor the inbound goal sits, measured back from the dead
# end. 1.0 m keeps the goal clear of the end wall by more than the goal
# tolerance while still putting the whole robot inside.
INBOUND_INSET = 1.0
# How far out onto the approach strip the outbound goal sits. Beyond the
# robot's own length, so reaching it means the tail cleared the mouth too.
OUTBOUND_CLEAR = 1.2


def _required_pitch():
    """Widest pair of neighbouring corridors, plus a metre of daylight."""
    worst = 0.0
    for a, b in zip(UTURN_WIDTHS, UTURN_WIDTHS[1:]):
        worst = max(worst, a / 2.0 + b / 2.0 + 2 * WALL_THICKNESS)
    return worst + 1.0


LANE_PITCH = _required_pitch()


def _cell_origin(index):
    span = LANE_PITCH * (len(UTURN_WIDTHS) - 1)
    return -span / 2.0 + index * LANE_PITCH


def _course_extent():
    reach = max(w / 2.0 for w in UTURN_WIDTHS) + WALL_THICKNESS
    x_min = _cell_origin(0) - reach - LANE_PITCH / 4.0
    x_max = _cell_origin(len(UTURN_WIDTHS) - 1) + reach + LANE_PITCH / 4.0
    return x_min, x_max, -APPROACH_DEPTH, DEADEND_LEN + 2.0


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
    """One dead-end corridor: two side walls and an end wall."""
    xc = _cell_origin(index)
    w, t = width, WALL_THICKNESS
    root = f"/World/Course/UTurn_{int(round(width * 100)):03d}"
    UsdGeom.Xform.Define(stage, root)

    _wall(stage, f"{root}/West", xc - w / 2 - t, 0.0, xc - w / 2, DEADEND_LEN)
    _wall(stage, f"{root}/East", xc + w / 2, 0.0, xc + w / 2 + t, DEADEND_LEN)
    # The end wall runs the full width including both side walls, so the
    # corner where they meet is closed rather than leaving a slot the planner
    # can see through.
    _wall(stage, f"{root}/End",
          xc - w / 2 - t, DEADEND_LEN, xc + w / 2 + t, DEADEND_LEN + t)

    # Halfway in, against the west wall. The robot meets it on the way in, has
    # to turn around past it, and meets it again coming out -- so it is in the
    # way of the manoeuvre being measured rather than merely on the route.
    lw = _low_width(w)
    if LOW_OBSTACLE and lw > 0:
        y = DEADEND_LEN / 2.0
        _low_box(stage, f"{root}/LowBox",
                 xc - w / 2, y - LOW_DEPTH / 2, xc - w / 2 + lw, y + LOW_DEPTH / 2)

    inbound = (xc, DEADEND_LEN - INBOUND_INSET)
    outbound = (xc, -OUTBOUND_CLEAR)
    return inbound, outbound


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

    # Enclose the approach strip, for the reason the corner course records:
    # make_map_from_stage builds the grid from walls, so an unwalled strip is
    # not in the map at all and the robot spawns outside it.
    _wall(stage, "/World/Course/ApproachBack",
          x_min, -APPROACH_DEPTH - WALL_THICKNESS, x_max, -APPROACH_DEPTH)
    _wall(stage, "/World/Course/ApproachWest",
          x_min - WALL_THICKNESS, -APPROACH_DEPTH, x_min, 0.0)
    _wall(stage, "/World/Course/ApproachEast",
          x_max, -APPROACH_DEPTH, x_max + WALL_THICKNESS, 0.0)

    cells = [_cell(stage, i, w) for i, w in enumerate(UTURN_WIDTHS)]

    robot = UsdGeom.Xform.Define(stage, "/World/VICA")
    robot.GetPrim().GetPayloads().AddPayload(
        os.path.relpath(ROBOT, os.path.dirname(OUTPUT)))
    # Facing north, into the first corridor. Same reasoning as the corner
    # course: spawning east would make the trial measure a turn-in-place
    # before the cell was even entered.
    start_x, start_y = _cell_origin(0), -2.5
    start_yaw = math.pi / 2
    UsdGeom.Xformable(robot).AddTranslateOp().Set(
        Gf.Vec3d(start_x, start_y, 0.0))
    UsdGeom.Xformable(robot).AddRotateXYZOp().Set(
        Gf.Vec3f(0.0, 0.0, math.degrees(start_yaw)))

    prim = world.GetPrim()
    vs = prim.GetVariantSets().AddVariantSet(PHYSICS_VARIANT_SET)
    vs.AddVariant(PHYSICS_VARIANT)
    vs.SetVariantSelection(PHYSICS_VARIANT)
    stage.GetRootLayer().Save()

    # Same schema as the other three, so width_trials reads it without knowing
    # which course it has. The U-turn is expressed entirely in the yaws: the
    # entry goal faces into the dead end, the exit goal faces back out of it,
    # and nothing between them is reachable without reversing heading.
    spec = {
        "stage": os.path.basename(OUTPUT),
        "start": [start_x, start_y],
        "start_yaw": start_yaw,
        "lanes": [
            {
                "width": w,
                "entry": [cells[i][0][0], cells[i][0][1]],
                "entry_yaw": math.pi / 2,
                "exit": [cells[i][1][0], cells[i][1][1]],
                "exit_yaw": -math.pi / 2,
                "turn": "uturn",
            }
            for i, w in enumerate(UTURN_WIDTHS)
        ],
        "deadend_len": DEADEND_LEN,
        "circumscribed_diameter": 0.9454,
    }
    with open(TARGETS, "w") as fh:
        json.dump(spec, fh, indent=2)

    print(f"wrote {OUTPUT}")
    print(f"wrote {TARGETS}")
    print(f"  widths {UTURN_WIDTHS}  (circumscribed diameter 0.9454)")
    print(f"  dead end {DEADEND_LEN} m, pitch {LANE_PITCH:.2f} m")
    print(f"  extent x {x_min:.2f}..{x_max:.2f}  y {y_min:.2f}..{y_max:.2f}")
    for i, w in enumerate(UTURN_WIDTHS):
        slack = w - 0.9454
        note = "turn on the spot fits" if slack >= 0 else "DOES NOT FIT"
        print(f"    {w:.2f} m  cell x {_cell_origin(i):+7.2f}  "
              f"여유 {slack:+.3f} m  {note}")


main()
