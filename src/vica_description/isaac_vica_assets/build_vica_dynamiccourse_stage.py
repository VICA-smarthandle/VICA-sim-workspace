#!/usr/bin/env python3
"""A corridor with someone about to step into it.

    ./make_stage.sh build_vica_dynamiccourse_stage.py vica_dynamiccourse.usd

What this course is for
-----------------------
The avoid course measures the gap beside an obstacle that is already there and
already seen. That is a planning question. This one measures the other half:
how late an obstacle can appear and still be dealt with. Three numbers come out
of it, and all three are things a guide robot's safety case needs:

    감지거리        how far away the robot first has a reading of the walker
    감속 시작거리   how far away the commanded speed starts coming down
    정지거리        how far it still travels after that

The walker starts behind the corridor wall, in a doorway, so nothing sees it
until it moves. A person already standing in the corridor is a static obstacle
and the planner routes around it before the robot is anywhere near; a person
stepping out of a doorway is the case that decides whether the robot is safe to
put in a hospital, and it is the one the physical robot's own bag from
2026-08-31 caught -- first detection at 0.28 m, stop command 0.5 s later, and a
braking distance of 0.32 m from 0.5 m/s that did not fit inside it.

The layout
----------
One straight corridor, driven from one end to the other, so speed is settled
before anything happens:

    y +1.0  ----------------+       +---------------------------
                            | 도어  |                              walker waits here
    y  0.0   robot ->                            ->  goal
    y -1.0  ------------------------------[낮은상자]--------------

The doorway is a gap in the north wall at the crossing point. The walker is a
kinematic rigid body: the runner moves it, physics does not, so it walks
through a robot that fails to stop rather than being knocked aside, and a
collision is unambiguous in the log.

Further along, against the south wall, sits a 0.25 m box. The lidar sweeps at
0.382 and the depth band starts at 0.30, so only the ultrasonic probes see it.
It is there so that one run exercises both sensors that can stop this robot.

Everything about when the walker moves belongs to the runner, not here. The
trial is repeated at different trigger distances, and rebuilding a stage for
each one would take longer than driving it.
"""

import json
import os

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else (
    os.path.dirname(os.path.abspath(os.sys.argv[0])))
ROBOT = os.path.join(HERE, "robot", "vica", "vica.usda")

NAME = os.environ.get("VICA_DYN_NAME", "vica_dynamiccourse")
OUTPUT = os.path.join(HERE, NAME + ".usd")
TARGETS = os.path.join(HERE, NAME + ".json")

PHYSICS_VARIANT_SET = "Physics"
PHYSICS_VARIANT = "physx"

WALL_THICKNESS = 0.15
WALL_HEIGHT = 1.5
FLOOR_THICKNESS = 0.5
FLOOR_MARGIN = 1.0

# 2.0 m of clear corridor. Wide enough that going round a person who has
# stopped is geometrically possible -- the padded circumscribed diameter is
# 0.945 -- so a failure to get past is about the controller and not the
# building. Hospital corridors are wider than this; a 2.0 m one is the tight
# case rather than the typical one.
CORRIDOR_WIDTH = 2.0
RUN_LENGTH = 26.0
START_X = 1.5
GOAL_X = 24.0

# Where the walker crosses. Far enough in that the robot is at its commanded
# speed and its costmap is settled: at 0.5 m/s it has had twenty seconds.
CROSS_X = 12.0
DOOR_WIDTH = 1.0

# A person, as far as a lidar is concerned: a 0.35 m cylinder at chest height.
# Kinematic, so it keeps walking through a robot that does not stop.
WALKER_RADIUS = 0.175
WALKER_HEIGHT = 1.70
WALKER_MASS = 62.0
WALKER_PARK_Y = CORRIDOR_WIDTH / 2.0 + 0.9      # behind the wall, in the doorway
# Mirrored, not chosen. play_stage moves the walker to -park_y because it
# reads the prim rather than this file, and two files disagreeing about where
# the walker ends is a discrepancy waiting to be debugged rather than a
# feature. Either value clears the corridor.
WALKER_END_Y = -WALKER_PARK_Y

# The ultrasonic-only obstacle, past the crossing, against the south wall.
LOW_X = 18.0
LOW_WIDTH = 0.30       # across the corridor
LOW_DEPTH = 0.30       # along it
LOW_HEIGHT = 0.25


def _box(stage, path, x0, y0, x1, y1, height, collide=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    prim = cube.GetPrim()
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2.0, (y0 + y1) / 2.0, height / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3f(abs(x1 - x0), abs(y1 - y0), height))
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def _wall(stage, path, x0, y0, x1, y1):
    return _box(stage, path, x0, y0, x1, y1, WALL_HEIGHT)


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

    h = CORRIDOR_WIDTH / 2.0
    y_min, y_max = -h - 2.0, h + 2.0
    x_min, x_max = -1.0, RUN_LENGTH

    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    fx = UsdGeom.Xformable(floor.GetPrim())
    fx.AddTranslateOp().Set(Gf.Vec3d((x_min + x_max) / 2.0, (y_min + y_max) / 2.0,
                                     -FLOOR_THICKNESS / 2.0))
    fx.AddScaleOp().Set(Gf.Vec3f(x_max - x_min + 2 * FLOOR_MARGIN,
                                 y_max - y_min + 2 * FLOOR_MARGIN,
                                 FLOOR_THICKNESS))
    # Drawn, not collided. The corner course records the measurement: PhysX
    # drops contacts between a floor box this size and a 65 mm wheel cylinder,
    # at some places along it and not others, and the robot ends up on its
    # belly with its wheels underground.
    plane = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
    plane.CreateAxisAttr("Z")
    plane.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

    light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    light.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))
    dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
    dome.CreateIntensityAttr(700.0)

    UsdGeom.Xform.Define(stage, "/World/Course")
    t = WALL_THICKNESS
    d0, d1 = CROSS_X - DOOR_WIDTH / 2.0, CROSS_X + DOOR_WIDTH / 2.0

    # North wall, in two pieces with the doorway between them.
    _wall(stage, "/World/Course/NorthA", x_min, h, d0, h + t)
    _wall(stage, "/World/Course/NorthB", d1, h, x_max, h + t)
    # The doorway's own side walls, so the walker is genuinely hidden until it
    # steps out rather than visible through a hole in a flat wall.
    _wall(stage, "/World/Course/DoorW", d0 - t, h, d0, h + 1.4)
    _wall(stage, "/World/Course/DoorE", d1, h, d1 + t, h + 1.4)
    _wall(stage, "/World/Course/DoorBack", d0 - t, h + 1.4, d1 + t, h + 1.4 + t)

    _wall(stage, "/World/Course/South", x_min, -h - t, x_max, -h)
    _wall(stage, "/World/Course/West", x_min, -h - t, x_min + t, h + t)
    _wall(stage, "/World/Course/East", x_max - t, -h - t, x_max, h + t)

    # The ultrasonic-only obstacle.
    _box(stage, "/World/Course/LowBox",
         LOW_X - LOW_DEPTH / 2.0, -h, LOW_X + LOW_DEPTH / 2.0, -h + LOW_WIDTH,
         LOW_HEIGHT)

    # The walker. A kinematic rigid body: the runner sets its transform every
    # frame and PhysX does not push back, so it walks through a robot that
    # fails to stop instead of being knocked over, and the log says plainly
    # whether they occupied the same place.
    walker = UsdGeom.Cylinder.Define(stage, "/World/Course/Walker")
    walker.CreateRadiusAttr(WALKER_RADIUS)
    walker.CreateHeightAttr(WALKER_HEIGHT)
    walker.CreateAxisAttr("Z")
    walker.CreateExtentAttr([
        Gf.Vec3f(-WALKER_RADIUS, -WALKER_RADIUS, -WALKER_HEIGHT / 2.0),
        Gf.Vec3f(WALKER_RADIUS, WALKER_RADIUS, WALKER_HEIGHT / 2.0)])
    wprim = walker.GetPrim()
    UsdGeom.Xformable(wprim).AddTranslateOp().Set(
        Gf.Vec3d(CROSS_X, WALKER_PARK_Y, WALKER_HEIGHT / 2.0))
    UsdPhysics.CollisionAPI.Apply(wprim)
    body = UsdPhysics.RigidBodyAPI.Apply(wprim)
    body.CreateKinematicEnabledAttr(True)
    # Mass is not optional even on a kinematic body: verify_stage refuses a
    # stage with a massless rigid body in it, having spent a month on one.
    mass = UsdPhysics.MassAPI.Apply(wprim)
    mass.CreateMassAttr(WALKER_MASS)

    robot = UsdGeom.Xform.Define(stage, "/World/VICA")
    robot.GetPrim().GetReferences().AddReference(
        os.path.relpath(ROBOT, os.path.dirname(OUTPUT)))
    UsdGeom.Xformable(robot.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(START_X, 0.0, 0.0))

    vset = robot.GetPrim().GetVariantSets().GetVariantSet(PHYSICS_VARIANT_SET)
    if vset.IsValid():
        vset.SetVariantSelection(PHYSICS_VARIANT)

    stage.GetRootLayer().Save()

    spec = {
        "stage": os.path.basename(OUTPUT),
        "start": [START_X, 0.0],
        "goal": [GOAL_X, 0.0],
        "corridor_width": CORRIDOR_WIDTH,
        "walker": {
            "prim": "/World/Course/Walker",
            "cross_x": CROSS_X,
            "park_y": WALKER_PARK_Y,
            "end_y": WALKER_END_Y,
            "radius": WALKER_RADIUS,
            "height": WALKER_HEIGHT,
        },
        "low_box": {
            "x": LOW_X, "width": LOW_WIDTH, "depth": LOW_DEPTH,
            "height": LOW_HEIGHT, "side": "south",
        },
    }
    with open(TARGETS, "w") as fh:
        json.dump(spec, fh, indent=2)

    print(f"wrote {OUTPUT}")
    print(f"wrote {TARGETS}")
    print(f"  복도 폭 {CORRIDOR_WIDTH} m, 주행 {START_X} -> {GOAL_X} m")
    print(f"  보행자 x {CROSS_X} 의 문 뒤에서 대기, y {WALKER_PARK_Y:+.2f} -> {WALKER_END_Y:+.2f}")
    print(f"  낮은 상자 x {LOW_X}, 남쪽 벽에 붙어 {LOW_WIDTH} m")


main()
