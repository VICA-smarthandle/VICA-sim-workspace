"""A course for measuring what it costs to go round something.

    $ISAAC_SIM/python.sh build_vica_avoidcourse_stage.py

Writes vica_avoidcourse.usd and vica_avoidcourse.json beside this file.

The width course answered "how narrow a corridor". This one answers a
different question with the same units, so the two can be read against each
other: **a 1.20 m gap beside an obstacle -- is it the same as a 1.20 m
corridor?**

Each lane here is a wide corridor with a block in it, and the block is placed
so the free gap beside it is one of the widths the corridor course already
measured. If the robot passes a 1.20 m gap as readily as it passed a 1.20 m
lane, then obstacles cost nothing beyond their geometry. If it does not, the
difference is what avoidance costs, and it is measured rather than assumed.

Three things are deliberate.

The corridor is wide (2.6 m) and only the gap is narrow. In the width course
the robot had no choice of line; here it has to choose one, which is the part
that "avoidance" actually names.

The blocks alternate sides down the course. A robot that learned to hug one
wall would pass a one-sided course and fail a real corridor.

The gap edges line up with the corridor centre-line differently for each lane,
so the approach is never the same distance from the block. Approach distance
dominated the first width sweep -- a 1.20 m lane went from 3/3 to 0/3 purely
by being moved to the far end -- and per-lane spawning is why that stopped
mattering. The same spawn rule applies here: the harness starts the robot in
front of the lane under test.

The lane geometry is otherwise identical to the width course, so
width_trials, width_report and animate_run all work against this spec with no
changes.
"""

import json
import os

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else (
os.path.join(os.environ.get("VICA_WS", os.getcwd()),
                 "src/vica_description/isaac_vica_assets"))
ROBOT = os.path.join(HERE, "robot", "vica", "vica.usda")
OUTPUT = os.path.join(HERE, "vica_avoidcourse.usd")
TARGETS = os.path.join(HERE, "vica_avoidcourse.json")

PHYSICS_VARIANT_SET = "Physics"
PHYSICS_VARIANT = "physx"

WALL_THICKNESS = 0.15
WALL_HEIGHT = 1.5
# 0.1 m gave a 214:1 aspect ratio on the floor slab and PhysX threw the robot
# 12 m into the air. 0.5 m holds it: 32.4 m of drift became 0.002 m.
FLOOR_THICKNESS = 0.5

# The free gap beside the block. Same numbers as the width course's lanes, so
# the two tables line up column for column.
GAP_WIDTHS = [1.40, 1.20, 1.15, 1.10, 1.00, 0.90]

CORRIDOR_WIDTH = 2.6     # wide enough that the gap, not the corridor, decides
BLOCK_DEPTH = 0.6        # how far along the corridor the block extends
ENTRY_LEN = 3.0          # straight run before the block
EXIT_LEN = 3.0           # straight run after it
LANE_PITCH = 4.2         # centre to centre; corridor 2.6 plus walls and margin

APPROACH_DEPTH = 5.0     # open strip the lane mouths face onto
EXIT_DEPTH = 2.0


def _lane_origin(index):
    span = LANE_PITCH * (len(GAP_WIDTHS) - 1)
    return -span / 2.0 + index * LANE_PITCH


def _lane_height():
    return ENTRY_LEN + BLOCK_DEPTH + EXIT_LEN


def _course_extent():
    span = LANE_PITCH * (len(GAP_WIDTHS) - 1)
    x_min = -span / 2.0 - LANE_PITCH / 2.0
    x_max = span / 2.0 + LANE_PITCH / 2.0
    return x_min, x_max, -APPROACH_DEPTH, _lane_height() + EXIT_DEPTH


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


def _lane(stage, index, gap):
    """A straight corridor with one block in it.

    The block spans from one corridor wall inwards, leaving `gap` free on the
    other side. Sides alternate with the index.

        corridor   xc +/- W/2,  y 0 .. H
        block      one side,    y ENTRY .. ENTRY+DEPTH,  leaving `gap`
    """
    xc = _lane_origin(index)
    w, t = CORRIDOR_WIDTH, WALL_THICKNESS
    h = _lane_height()
    root = f"/World/Course/Lane_{int(round(gap * 100)):03d}"
    UsdGeom.Xform.Define(stage, root)

    _wall(stage, f"{root}/Left", xc - w / 2 - t, 0.0, xc - w / 2, h)
    _wall(stage, f"{root}/Right", xc + w / 2, 0.0, xc + w / 2 + t, h)

    # Block on the left for even lanes, right for odd. A line that works for
    # one side is not a line that works for a corridor.
    y0, y1 = ENTRY_LEN, ENTRY_LEN + BLOCK_DEPTH
    if index % 2 == 0:
        bx0, bx1 = xc - w / 2, xc + w / 2 - gap
        gap_centre = xc + w / 2 - gap / 2
    else:
        bx0, bx1 = xc - w / 2 + gap, xc + w / 2
        gap_centre = xc - w / 2 + gap / 2
    _wall(stage, f"{root}/Block", bx0, y0, bx1, y1)
    return h, gap_centre


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

    # Authored explicitly: the UsdPhysics defaults are direction (0,0,0) and
    # magnitude -inf, whose product is NaN, and the robot leaves through the
    # floor on the first step.
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    x_min, x_max, y_min, y_max = _course_extent()
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    fx = UsdGeom.Xformable(floor.GetPrim())
    fx.AddTranslateOp().Set(
        Gf.Vec3d((x_min + x_max) / 2, (y_min + y_max) / 2, -FLOOR_THICKNESS / 2))
    fx.AddScaleOp().Set(
        Gf.Vec3f(abs(x_max - x_min), abs(y_max - y_min), FLOOR_THICKNESS))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    UsdGeom.Xform.Define(stage, "/World/Course")

    lane_h = _lane_height()
    mouths, gap_centres = [], []
    for i, g in enumerate(GAP_WIDTHS):
        _, gc = _lane(stage, i, g)
        xc = _lane_origin(i)
        mouths.append((xc - CORRIDOR_WIDTH / 2, xc + CORRIDOR_WIDTH / 2))
        gap_centres.append(gc)

    # Close the approach and exit faces except at the corridor mouths, so the
    # robot cannot drive round the outside of a lane instead of through it.
    t = WALL_THICKNESS
    for label, y0, y1 in (("Entry", -t, 0.0), ("Exit", lane_h, lane_h + t)):
        edges = [x_min] + [v for pair in sorted(mouths) for v in pair] + [x_max]
        for k in range(0, len(edges) - 1, 2):
            a, b = edges[k], edges[k + 1]
            if b - a > 1e-6:
                _wall(stage, f"/World/Course/Face{label}_{k // 2}", a, y0, b, y1)

    _wall(stage, "/World/Course/Bound_S", x_min, y_min, x_max, y_min + t)
    _wall(stage, "/World/Course/Bound_N", x_min, y_max - t, x_max, y_max)
    _wall(stage, "/World/Course/Bound_W", x_min, y_min, x_min + t, y_max)
    _wall(stage, "/World/Course/Bound_E", x_max - t, y_min, x_max, y_max)

    # Without a light the stage renders black in the GUI and the RTX lidar has
    # nothing to trace against.
    light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    light.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-45.0, 0.0, 0.0))

    robot = UsdGeom.Xform.Define(stage, "/World/VICA")
    robot.GetPrim().GetReferences().AddReference(
        os.path.relpath(ROBOT, os.path.dirname(OUTPUT)))
    start_x = _lane_origin(0)
    start_y = -APPROACH_DEPTH / 2.0
    UsdGeom.Xformable(robot.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(start_x, start_y, 0.0))

    vset = robot.GetPrim().GetVariantSets().GetVariantSet(PHYSICS_VARIANT_SET)
    if vset.IsValid():
        vset.SetVariantSelection(PHYSICS_VARIANT)

    stage.GetRootLayer().Save()

    # Same schema as the width course, so the harness reads either without
    # knowing which it has. "width" is the free gap, which is the quantity the
    # two courses share.
    spec = {
        "stage": os.path.basename(OUTPUT),
        "start": [start_x, start_y],
        "lanes": [
            {
                "width": g,
                # A metre clear of the mouth: the approach is part of the
                # trial rather than a pose the robot is dropped into.
                "entry": [_lane_origin(i), -1.0],
                # Aimed at the corridor centre beyond the block, not at the
                # gap. Aiming at the gap would hand the robot the line it is
                # supposed to find.
                "exit": [_lane_origin(i), lane_h + 1.2],
                "gap_centre_x": gap_centres[i],
                "block_side": "left" if i % 2 == 0 else "right",
            }
            for i, g in enumerate(GAP_WIDTHS)
        ],
        "corridor_width": CORRIDOR_WIDTH,
        "block": {"depth": BLOCK_DEPTH, "y": [ENTRY_LEN, ENTRY_LEN + BLOCK_DEPTH]},
    }
    with open(TARGETS, "w") as fh:
        json.dump(spec, fh, indent=2)

    print(f"wrote {OUTPUT}")
    print(f"wrote {TARGETS}")
    print(f"  corridor {CORRIDOR_WIDTH} m, block depth {BLOCK_DEPTH} m")
    print(f"  gaps {GAP_WIDTHS}")
    print(f"  extent x {x_min:.2f}..{x_max:.2f}  y {y_min:.2f}..{y_max:.2f}")
    for i, g in enumerate(GAP_WIDTHS):
        print(f"    gap {g:.2f}  lane x {_lane_origin(i):+7.2f}  "
              f"block {'left ' if i % 2 == 0 else 'right'}  "
              f"gap centre x {gap_centres[i]:+7.2f}")


if __name__ == "__main__":
    main()
