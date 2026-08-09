"""Build a light 10 x 10 m test floor with corridors, and put VICA in it.

    /home/sim/isaacsim/python.sh build_vica_testroom_stage.py

Why not the hospital
--------------------
The hospital stage is the target, not the workshop. It takes minutes to open,
saturates a 12 GB card, and every tuning iteration pays that cost again. This
is the same robot in a floor plan built from twelve boxes: it opens in seconds
and leaves the GPU to the sensors.

100 m2, near enough to the 30 pyeong the robot is expected to work in.

Corridor widths are the point
-----------------------------
VICA rotates about the midpoint of its drive axle at x = +0.154, not about
base_footprint. From there the far corner of the footprint is 0.750 m away, so
turning on the spot needs 1.50 m of width, or 1.60 m once Nav2's 0.05 padding
is counted.

The loop is deliberately built around that number:

    right   2.2 m    turns freely
    top     2.0 m    turns freely
    bottom  1.6 m    exactly at the limit
    left    1.3 m    cannot turn on the spot -- must reverse out or go round

A planner that only produces in-place rotations will fail in the left corridor
and nowhere else, which is the sort of thing worth finding on a floor that
loads in seconds.

The inner block has a 1.2 m doorway off the right corridor. The robot is 0.455 m
wide, so it fits with room either side, and once inside it cannot turn around.

VICA starts at the world origin facing +Y, matching the hospital stage, so
amcl's set_initial_pose of (0,0,0) needs no changing between the two.
"""

import os
import sys

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = "/home/sim/vica_ws/src/vica_description/isaac_vica_assets"

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics


# --arm builds the same room around the arm variant, under its own name.
#
# The width and avoid course builders deliberately do not get this switch. A
# driving result is about a 20 kg robot, and a builder that can quietly produce
# a 27 kg one behind the same stage filename is how two robots end up in one
# table. This room measures nothing, so the switch is safe here and nowhere
# else.
_ARM = "--arm" in sys.argv
ROBOT = os.path.join(HERE, "robot",
                     "vica_arm" if _ARM else "vica",
                     "vica_arm.usda" if _ARM else "vica.usda")
OUTPUT = os.path.join(HERE, "vica_testroom_arm.usd" if _ARM else "vica_testroom.usd")

PHYSICS_VARIANT_SET = "Physics"
PHYSICS_VARIANT = "physx"

# Floor extent. The robot starts at (0, 0), in the middle of the wide corridor.
X_MIN, X_MAX = -9.0, 1.0
Y_MIN, Y_MAX = -5.0, 5.0

# Inner block, sized so each corridor comes out at its intended width.
BLOCK_X_MIN, BLOCK_X_MAX = -7.7, -1.2
BLOCK_Y_MIN, BLOCK_Y_MAX = -3.4, 3.0

# Doorway in the block's right wall, as a span in y.
DOOR_Y_MIN, DOOR_Y_MAX = -0.6, 0.6

# Test obstacles, placed to separate what each sensor contributes.
#
# The lidar sweeps one plane at 0.382 m. Anything below it is invisible to the
# scan and has to come from the depth camera at 0.320 m, which is the whole
# reason the camera feeds the costmap at all. A box of each kind, ahead of and
# behind the robot along the corridor, makes the difference readable straight
# off the costmap: LOW should appear only when the depth source is enabled,
# TALL should appear either way.
#
#   (x, y, side, height, name)
OBSTACLES = [
    (0.0,  2.0, 0.40, 0.25, "LowBox"),   # under the scan plane: camera only
    (0.0, -2.0, 0.40, 1.00, "TallBox"),  # through it: both sensors
]

WALL_THICKNESS = 0.1
# Tall enough that the lidar at 0.382 m sees wall rather than sky, and that
# nothing can be driven under.
WALL_HEIGHT = 1.5


def _wall(stage, path, x0, y0, x1, y1):
    """An axis-aligned wall slab, given its footprint corners."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    prim = cube.GetPrim()

    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(
        Gf.Vec3d((x0 + x1) / 2.0, (y0 + y1) / 2.0, WALL_HEIGHT / 2.0)
    )
    xf.AddScaleOp().Set(
        Gf.Vec3f(abs(x1 - x0), abs(y1 - y0), WALL_HEIGHT)
    )

    # Static collider: geometry that blocks, with no rigid body, so the solver
    # never moves it.
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim


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

    # ---- physics ----------------------------------------------------------
    # Gravity authored explicitly. Left at the defaults, UsdPhysics writes
    # direction (0,0,0) and magnitude -inf, which resolve only for code that
    # resolves them: press Play in the GUI and the product is NaN, and the
    # robot leaves through the floor on the first step.
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    # ---- floor ------------------------------------------------------------
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    fx = UsdGeom.Xformable(floor.GetPrim())
    fx.AddTranslateOp().Set(
        Gf.Vec3d((X_MIN + X_MAX) / 2.0, (Y_MIN + Y_MAX) / 2.0, -0.05)
    )
    fx.AddScaleOp().Set(Gf.Vec3f(X_MAX - X_MIN, Y_MAX - Y_MIN, 0.1))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    # ---- walls ------------------------------------------------------------
    t = WALL_THICKNESS
    walls = {
        "Outer_South": (X_MIN, Y_MIN, X_MAX, Y_MIN + t),
        "Outer_North": (X_MIN, Y_MAX - t, X_MAX, Y_MAX),
        "Outer_West": (X_MIN, Y_MIN, X_MIN + t, Y_MAX),
        "Outer_East": (X_MAX - t, Y_MIN, X_MAX, Y_MAX),
        "Block_South": (BLOCK_X_MIN, BLOCK_Y_MIN, BLOCK_X_MAX, BLOCK_Y_MIN + t),
        "Block_North": (BLOCK_X_MIN, BLOCK_Y_MAX - t, BLOCK_X_MAX, BLOCK_Y_MAX),
        "Block_West": (BLOCK_X_MIN, BLOCK_Y_MIN, BLOCK_X_MIN + t, BLOCK_Y_MAX),
        # The east face is split around the doorway.
        "Block_East_S": (BLOCK_X_MAX - t, BLOCK_Y_MIN, BLOCK_X_MAX, DOOR_Y_MIN),
        "Block_East_N": (BLOCK_X_MAX - t, DOOR_Y_MAX, BLOCK_X_MAX, BLOCK_Y_MAX),
    }
    for name, corners in walls.items():
        _wall(stage, f"/World/Walls/{name}", *corners)

    # ---- test obstacles ---------------------------------------------------
    for x, y, side, height, name in OBSTACLES:
        box = UsdGeom.Cube.Define(stage, f"/World/Obstacles/{name}")
        box.CreateSizeAttr(1.0)
        bx = UsdGeom.Xformable(box.GetPrim())
        bx.AddTranslateOp().Set(Gf.Vec3d(x, y, height / 2.0))
        bx.AddScaleOp().Set(Gf.Vec3f(side, side, height))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())

    # ---- robot ------------------------------------------------------------
    robot = UsdGeom.Xform.Define(stage, "/World/VICA")
    robot.GetPrim().GetReferences().AddReference(
        os.path.relpath(ROBOT, HERE)
    )
    robot.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    # The URDF faces +X; the corridor runs along Y.
    robot.AddRotateZOp().Set(90.0)

    # The importer hides every rigid body and joint behind a variant set with
    # "none" among the options. Selecting nothing leaves colliders and no
    # bodies: the robot is not simulated, so no floor holds it up and adding
    # one does not help. Say which one explicitly.
    vset = robot.GetPrim().GetVariantSets().GetVariantSet(PHYSICS_VARIANT_SET)
    if vset.IsValid():
        vset.SetVariantSelection(PHYSICS_VARIANT)

    # ---- light ------------------------------------------------------------
    light = UsdLux.DistantLight.Define(stage, "/World/Environment/defaultLight")
    light.CreateIntensityAttr(3000.0)
    light.CreateAngleAttr(1.0)

    stage.GetRootLayer().Save()

    # ---- report -----------------------------------------------------------
    area = (X_MAX - X_MIN) * (Y_MAX - Y_MIN)
    check = Usd.Stage.Open(OUTPUT, load=Usd.Stage.LoadAll)
    arts = [
        str(p.GetPath())
        for p in check.Traverse()
        if p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    print(f"wrote              : {OUTPUT}")
    print(f"floor              : {X_MAX - X_MIN:.0f} x {Y_MAX - Y_MIN:.0f} m"
          f"  = {area:.0f} m2  ({area / 3.305785:.0f} pyeong)")
    print(f"walls              : {len(walls)}")
    print("corridor widths    :")
    print(f"    right  {X_MAX - BLOCK_X_MAX:.1f} m   turns freely")
    print(f"    top    {Y_MAX - BLOCK_Y_MAX:.1f} m   turns freely")
    print(f"    bottom {BLOCK_Y_MIN - Y_MIN:.1f} m   at the 1.60 m limit")
    print(f"    left   {BLOCK_X_MIN - X_MIN:.1f} m   too narrow to turn on the spot")
    print(f"doorway            : {DOOR_Y_MAX - DOOR_Y_MIN:.1f} m  (robot is 0.455 m wide)")
    print("test obstacles     :")
    for x, y, side, height, name in OBSTACLES:
        seen = "camera only" if height < 0.382 else "scan and camera"
        print(f"    {name:8s} ({x:+.1f}, {y:+.1f})  {side:.2f} m square, "
              f"{height:.2f} m tall  -> {seen}")
    print(f"physics variant    : {vset.GetVariantSelection() if vset.IsValid() else 'MISSING'}")
    print(f"articulation roots : {arts}")
    if len(arts) != 1:
        print("WARNING: expected exactly one articulation root")
    print("\nNext: build_vica_ros_graphs.py against this stage.")


main()
