"""A course whose only purpose is to find the width where driving stops working.

    /home/sim/isaacsim/python.sh build_vica_widthcourse_stage.py

The hospital stage and the test room both answer "does it drive", and neither
answers "how narrow". Their corridors are whatever the building happened to
have, so a failure there says the robot could not get through that place, not
that it cannot get through 0.9 m. This course is graded, so the answer comes out
as a number.

Two questions, two halves.

CORNERS. Six through-lanes, each a dog-leg with two 90 degree turns, at widths
from 0.70 to 1.40 m. Through-lanes rather than dead ends on purpose: below
1.60 m the robot cannot turn around, so a dead end is a trap and the run would
measure the trap instead of the corner.

GAPS. The wide approach corridor carries obstacles set against the far wall,
leaving passages from 0.60 to 1.20 m. One traverse tests all six.

The range is set by what the robot and the building are:

    robot width 0.455 + footprint padding 0.05  ->  0.555 m is the floor
    narrowest corridor on the real map                0.70 m
    median corridor on the real map                   1.40 m
    turning on the spot needs                         1.60 m

So the interesting band is 0.70 to 1.40, and it is sampled at 0.1 m through the
part where the answer is likely to sit. Nothing here is wider than 1.40: if
every lane passes, add wider ones rather than reading a clean sweep as proof
the robot handles everything.

Walls are 1.5 m tall so the lidar plane at 0.382 m sees wall rather than sky,
and static colliders with no rigid body so the solver never moves them.
"""

import json
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else (
    "/home/sim/vica_ws/src/vica_description/isaac_vica_assets")
ROBOT = os.path.join(HERE, "robot", "vica", "vica.usda")
OUTPUT = os.path.join(HERE, "vica_widthcourse.usd")
TARGETS = os.path.join(HERE, "vica_widthcourse.json")

PHYSICS_VARIANT_SET = "Physics"
PHYSICS_VARIANT = "physx"

WALL_THICKNESS = 0.15
WALL_HEIGHT = 1.5

# The floor slab's thickness, and it is not cosmetic.
#
# At 0.1 m this floor is 21.4 x 14.9, an aspect ratio of 214:1, and PhysX threw
# the robot off it: standing still and commanded nothing, it was launched 12 m
# into the air within three seconds and diverged from there. Measured against
# the same stage with only the floor changed:
#
#     21.4 x 14.9 x 0.1   drifted 32.4 m   exploded
#     21.4 x 14.9 x 0.5   drifted  0.002 m stable
#     10.0 x 10.0 x 0.1   drifted  0.002 m stable
#
# The test room survives at 0.1 only because it is 10 x 10, a ratio of 100:1.
# So this is a limit the courses were already close to rather than a property of
# this one. A thicker slab costs nothing -- it is below the floor line either
# way -- and 0.5 m leaves the ratio at 43:1.
FLOOR_THICKNESS = 0.5

# --- corner lanes ---------------------------------------------------------
# Width of each dog-leg lane, metres. Sampled at 0.1 m across the band the real
# building actually has.
# 0.05 m steps through 1.00-1.20, which is where the boundary sits: three
# passes each gave 3/3 at 1.20 and 0/3 at 1.00, and 0.20 m is too coarse an
# answer when the real building's corridors run from 0.70 to 1.40.
#
# Geometry says the gap should be larger still. The robot's own config computes
# that the hexagonal footprint needs 0.750 m to take a 90 degree corner on a
# 0.5 m arc, so anything above that is the tuning stopping it rather than the
# body.
LANE_WIDTHS = [0.70, 0.80, 0.90, 1.00, 1.05, 1.10, 1.15, 1.20, 1.40]

LANE_ENTRY_LEN = 2.0     # straight run before the first corner
LANE_JOG = 1.0           # sideways offset between the two corners
LANE_EXIT_LEN = 2.0      # straight run after the second corner
# Pitch has to clear the widest lane plus its jog plus both walls, with enough
# left over that a robot failing in one lane cannot be seen from the next.
LANE_PITCH = 3.4

# --- gap obstacles --------------------------------------------------------
# Passage left between the obstacle and the near wall of the approach corridor.
GAP_WIDTHS = [1.20, 1.00, 0.90, 0.80, 0.70, 0.60]
GAP_BOX_DEPTH = 0.5

# The approach is two corridors, not one, split by a divider.
#
# With the gap obstacles in the same corridor the lanes open onto, the eastern
# lanes sit behind the narrowest gaps: reaching the 1.40 m lane meant getting
# through the 0.60 m gap first, so a lane trial could fail for a reason that had
# nothing to do with the lane. The divider gives the lanes a clear access strip
# and puts the gauntlet below it, reachable from either end.
APPROACH_DEPTH = 6.0     # both corridors together; robot starts in the access one
ACCESS_DEPTH = 2.5       # clear strip the lane mouths open onto
EXIT_DEPTH = 3.5         # wide corridor after them, room to turn around


def _lane_origin(index):
    """Centre-line x of lane `index`'s entry segment."""
    span = LANE_PITCH * (len(LANE_WIDTHS) - 1)
    return -span / 2.0 + index * LANE_PITCH


def _course_extent():
    span = LANE_PITCH * (len(LANE_WIDTHS) - 1)
    x_min = -span / 2.0 - LANE_PITCH / 2.0
    x_max = span / 2.0 + LANE_JOG + LANE_PITCH / 2.0
    lane_h = LANE_ENTRY_LEN + max(LANE_WIDTHS) + LANE_EXIT_LEN
    return x_min, x_max, -APPROACH_DEPTH, lane_h + EXIT_DEPTH


def _wall(stage, path, x0, y0, x1, y1):
    """An axis-aligned wall slab, given its footprint corners."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    prim = cube.GetPrim()
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(
        Gf.Vec3d((x0 + x1) / 2.0, (y0 + y1) / 2.0, WALL_HEIGHT / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3f(abs(x1 - x0), abs(y1 - y0), WALL_HEIGHT))
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def _lane_height():
    """Every lane ends at the same y.

    Making the exit run a fixed length instead left each lane a different
    height, while the wall that closes the exit corridor sat at the tallest
    one. The band between them was open, so the lanes joined up above their
    exits and a robot could cross from one to another. Uniform height costs
    nothing but a slightly longer exit run in the narrow lanes.
    """
    return LANE_ENTRY_LEN + max(LANE_WIDTHS) + LANE_EXIT_LEN


def _lane(stage, index, width):
    """One dog-leg lane, as the four slabs that bound it.

    Local layout, with the entry centre-line at xc and the jog to +x:

        entry   xc +/- w/2,          y 0 .. L1
        bend    xc-w/2 .. xc+J+w/2,  y L1 .. L1+w
        exit    xc+J +/- w/2,        y L1+w .. H
    """
    xc = _lane_origin(index)
    w, t, j = width, WALL_THICKNESS, LANE_JOG
    l1 = LANE_ENTRY_LEN
    h = _lane_height()
    root = f"/World/Course/Lane_{int(round(width * 100)):03d}"
    UsdGeom.Xform.Define(stage, root)

    # Left side of the entry and the bend.
    _wall(stage, f"{root}/Left", xc - w / 2 - t, 0.0, xc - w / 2, l1 + w)
    # Everything to the right of the entry and below the bend.
    _wall(stage, f"{root}/RightLower", xc + w / 2, 0.0, xc + j + w / 2 + t, l1)
    # Everything to the left of the exit and above the bend.
    _wall(stage, f"{root}/LeftUpper", xc - w / 2 - t, l1 + w, xc + j - w / 2, h)
    # Right side of the bend and the exit.
    _wall(stage, f"{root}/Right", xc + j + w / 2, l1, xc + j + w / 2 + t, h)
    return h


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

    # Gravity authored explicitly. Left at the defaults UsdPhysics writes
    # direction (0,0,0) and magnitude -inf, whose product is NaN, and the robot
    # leaves through the floor on the first step.
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
    entries, exits = [], []
    for i, w in enumerate(LANE_WIDTHS):
        _lane(stage, i, w)
        xc = _lane_origin(i)
        entries.append((xc - w / 2, xc + w / 2))
        exits.append((xc + LANE_JOG - w / 2, xc + LANE_JOG + w / 2))

    # Close the approach and exit corridors except at the lane mouths.
    #
    # Without this the lane bank is open between lanes, and the gap obstacles
    # below stop being constrictions: the robot drives north into the dead space
    # between two lanes and around them. The measurement would then be of a
    # detour, not of a gap.
    t = WALL_THICKNESS
    for label, y0, y1, openings in (("Entry", -t, 0.0, entries),
                                    ("Exit", lane_h, lane_h + t, exits)):
        edges = [x_min] + [v for pair in sorted(openings) for v in pair] + [x_max]
        for k in range(0, len(edges) - 1, 2):
            a, b = edges[k], edges[k + 1]
            if b - a > 1e-6:
                _wall(stage, f"/World/Course/Face{label}_{k // 2}", a, y0, b, y1)

    # Outer boundary, so the lidar always has something to return and the robot
    # cannot wander off the floor.
    _wall(stage, "/World/Course/Bound_S", x_min, y_min, x_max, y_min + t)
    _wall(stage, "/World/Course/Bound_N", x_min, y_max - t, x_max, y_max)
    _wall(stage, "/World/Course/Bound_W", x_min, y_min, x_min + t, y_max)
    _wall(stage, "/World/Course/Bound_E", x_max - t, y_min, x_max, y_max)

    # The divider, with a way in at each end so the gauntlet can be driven
    # either direction.
    divider_top = -WALL_THICKNESS - ACCESS_DEPTH
    divider_bottom = divider_top - WALL_THICKNESS
    mouth = 1.6
    _wall(stage, "/World/Course/Divider_Mid",
          x_min + WALL_THICKNESS + mouth, divider_bottom,
          x_max - WALL_THICKNESS - mouth, divider_top)

    # Gap obstacles, in the gauntlet below the divider, rising from the south
    # boundary so the passage left is under the divider.
    span = x_max - x_min - 2 * t
    step = span / (len(GAP_WIDTHS) + 1)
    gaps = []
    for k, gap in enumerate(GAP_WIDTHS):
        cx = x_min + t + step * (k + 1)
        # The gauntlet runs from the south boundary up to the divider, so the
        # box has to stop `gap` short of the divider's underside.
        depth = (divider_bottom - (y_min + WALL_THICKNESS)) - gap
        y0 = y_min + t
        _wall(stage, f"/World/Course/Gap_{int(round(gap * 100)):03d}",
              cx - GAP_BOX_DEPTH / 2, y0, cx + GAP_BOX_DEPTH / 2, y0 + depth)
        gaps.append((gap, cx, y0 + depth, divider_bottom - (y0 + depth)))

    # Light. Hospital-stage experience: without one authored here the stage
    # renders black in the GUI and the RTX lidar has nothing to trace against.
    light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    light.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-45.0, 0.0, 0.0))

    # Robot, referenced rather than copied, at the mouth of the approach
    # corridor with the whole course ahead of it.
    robot = UsdGeom.Xform.Define(stage, "/World/VICA")
    robot.GetPrim().GetReferences().AddReference(
        os.path.relpath(ROBOT, os.path.dirname(OUTPUT)))
    # Every gap shares its northern edge at the corridor face, so a single
    # lateral line clears all of them; the robot starts on it. Half way down the
    # corridor sounds fairer and is not -- it is inside the boxes for every gap
    # but the widest, and the first run started 0.40 m from a wall.
    # Clear of everything, not on the tightest line. Starting on the narrowest
    # gap's centre line left 0.02 m beside the padded footprint, which is the
    # test rather than a place to begin it; the planner can route down to it.
    start_y = divider_top + ACCESS_DEPTH / 2.0
    start_x = x_min + 1.5
    UsdGeom.Xformable(robot.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(start_x, start_y, 0.0))

    vset = robot.GetPrim().GetVariantSets().GetVariantSet(PHYSICS_VARIANT_SET)
    if vset.IsValid():
        vset.SetVariantSelection(PHYSICS_VARIANT)

    stage.GetRootLayer().Save()

    # Where the trials have to drive to, written beside the stage so the
    # harness reads the course as built. Copying these constants into the
    # harness instead is how the two drift apart, and a harness aiming at a
    # lane that moved is a run that measures nothing and says nothing.
    spec = {
        "stage": os.path.basename(OUTPUT),
        "start": [start_x, start_y],
        "corridor": {
            "access_y": [divider_top, -WALL_THICKNESS],
            "gauntlet_y": [y_min + WALL_THICKNESS, divider_bottom],
            "exit_y": [lane_h + WALL_THICKNESS, y_max - WALL_THICKNESS],
            "x": [x_min + WALL_THICKNESS, x_max - WALL_THICKNESS],
        },
        "lanes": [
            {
                "width": w,
                # A metre clear of the mouth, so the approach is part of the
                # trial rather than a pose the robot is dropped into.
                "entry": [_lane_origin(i), -1.0],
                "exit": [_lane_origin(i) + LANE_JOG, lane_h + 1.2],
            }
            for i, w in enumerate(LANE_WIDTHS)
        ],
        "gaps": [
            {"gap": gap, "x": cx, "pass_y": (y_edge + (0.0 - WALL_THICKNESS)) / 2.0}
            for gap, cx, y_edge, _ in gaps
        ],
        # Driving east narrows the gauntlet monotonically, so where the robot
        # stops is the answer rather than a place it happened to fail.
        "gap_run": {
            "west": [x_min + WALL_THICKNESS + mouth / 2, divider_bottom - 0.8],
            "east": [x_max - WALL_THICKNESS - mouth / 2, divider_bottom - 0.8],
        },
    }
    with open(TARGETS, "w") as fh:
        json.dump(spec, fh, indent=2)

    print(f"wrote          : {OUTPUT}")
    print(f"targets        : {TARGETS}")
    print(f"floor          : {x_max - x_min:.1f} x {y_max - y_min:.1f} m")
    print(f"robot starts   : ({start_x:+.2f}, {start_y:+.2f})  "
          f"facing +x, on the line every gap leaves open")
    print()
    print("corner lanes   : dog-leg, two 90 degree turns each")
    print(f"    {'width':>6}  {'entry x':>9}  {'exit x':>9}  {'exit y':>7}")
    for i, w in enumerate(LANE_WIDTHS):
        xc = _lane_origin(i)
        print(f"    {w:6.2f}  {xc:+9.2f}  {xc + LANE_JOG:+9.2f}  {lane_h:7.2f}")
    print()
    print("gap obstacles  : in the approach corridor")
    print(f"    {'asked':>6}  {'centre x':>9}  {'box top y':>10}  {'actual gap':>11}")
    for gap, cx, y_edge, actual in gaps:
        flag = "" if abs(actual - gap) < 1e-6 else "   <-- MISMATCH"
        print(f"    {gap:6.2f}  {cx:+9.2f}  {y_edge:10.2f}  {actual:11.2f}{flag}")
    print()
    print("robot is 0.455 m wide; with footprint_padding 0.05 the floor is 0.555 m")
    print("in-place rotation needs 1.60 m, so no lane here allows it")
    print()
    print("Next: prepare_stage against this file, then drive it.")


main()
