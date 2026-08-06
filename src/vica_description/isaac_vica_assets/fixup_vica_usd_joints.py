"""Restore VICA's joint drive setup after a URDF re-import.

Run from Isaac Sim's Script Editor with the Timeline stopped, once per import.

The URDF importer gives every revolute joint an angular drive. That is wrong for
the four caster joints: a caster is passive, it swivels and rolls because the
robot pushes it, and a drive fights that. Left in place the casters resist
turning, the robot ploughs instead of steering, and the two drive wheels end up
working against four powered joints.

This was fixed by hand once, directly in the USD. That fix does not survive the
next import, and neither does any record of it -- the same failure mode as the
Action Graphs that lived only in the stage. Hence this file: geometry lives in
VICA.xacro, the ROS interface in build_vica_ros_graphs.py, and the joint drives
here, so a re-import is three scripted steps rather than an archaeology exercise.

What it does:

    caster joints  drive removed, and its attributes zeroed as a fallback for
                   when the joint is composed from a reference and the schema
                   cannot actually be dropped from the root layer
    drive wheels   velocity drive, zero stiffness, damping and max force set
                   below so they can be tuned in one place

Run build_vica_ros_graphs.py after this one.
"""

from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade

import omni.timeline
import omni.usd


# --------------------------------------------------------------------------
# Joints -- names come from urdf/VICA.xacro
# --------------------------------------------------------------------------
DRIVE_JOINTS = ["left_wheel_joint", "right_wheel_joint"]

# Passive. Named front_* in the URDF even though both casters sit behind the
# drive axle at x = -0.222; the names match the physical robot's package and are
# left alone for that reason. If they ever change, the error path below prints
# every revolute joint in the stage.
CASTER_JOINTS = [
    "front_left_caster_steer_joint",
    "front_right_caster_steer_joint",
    "front_left_caster_wheel_joint",
    "front_right_caster_wheel_joint",
]

# --------------------------------------------------------------------------
# Drive tuning -- the two values worth touching after a drive test
# --------------------------------------------------------------------------
# Velocity control wants zero stiffness: stiffness is the position gain, and a
# non-zero value makes the joint fight to hold an angle it was never told to.
# The articulation carries no PhysxArticulationAPI as imported, so it solves on
# PhysX's defaults of four position iterations and one velocity iteration. That
# is thin for four wheel contacts under a drive strong enough to hold 4 rad/s,
# and under-converged contacts slip whatever friction they are given.
#
# Measured, turning in place at a commanded 0.3 rad/s: 0.00-0.13 rad/s achieved
# on the defaults, 0.17 with these counts. At 0.4, the configured wz_max, 0.20
# becomes 0.24. Above 0.5 it makes no difference, so this is not the whole of
# the rotation deficit -- see measure_rotation -- but it is the part of it that
# sits in the range the controller actually commands.
#
# Raising the drive damping a hundredfold and cutting the caster friction to a
# thousandth both changed nothing, which is what sent the search here.
ARTICULATION_POSITION_ITERATIONS = 64
ARTICULATION_VELOCITY_ITERATIONS = 16

WHEEL_STIFFNESS = 0.0
WHEEL_DAMPING = 1.0e4
WHEEL_MAX_FORCE = 1.0e5

# --------------------------------------------------------------------------
# Tyre friction
# --------------------------------------------------------------------------
# The importer binds no physics material at all, so every wheel runs on PhysX's
# default. That is enough to creep forwards -- rolling asks little of the
# contact -- and not enough to turn: rotating in place needs the drive wheels
# to push sideways against the ground hard enough to drag two casters around an
# arc, and without grip they simply spin. Commanded 0.5 rad/s produced 0.08
# degrees of yaw in ten seconds while both wheels turned in opposite
# directions, which is what no traction looks like from the outside.
#
# Rubber on a hard indoor floor. Restitution 0 because a bouncing wheel is not
# a thing this robot does.
WHEEL_STATIC_FRICTION = 0.9
WHEEL_DYNAMIC_FRICTION = 0.8
WHEEL_RESTITUTION = 0.0

# Casters get their own, much slipperier material.
#
# Binding the tyre material to all four wheels made the robot turn, but it took
# more than ten seconds to get going: 0.17 deg, then 0.8, then 2.2, then 5.
# A caster only turns because the ground drags its contact patch around behind
# the swivel axis, and grippy casters fight that -- the same friction that gives
# the drive wheels traction gives the casters resistance. Real ones are small
# hard wheels that scrub easily, which is the point of a caster.
#
# So: grip on the drive wheels, slip on the casters.
CASTER_STATIC_FRICTION = 0.1
CASTER_DYNAMIC_FRICTION = 0.05
# Placed under the stage's default prim so it lands inside the robot asset
# and travels with it into any stage that references the robot.
DRIVE_MATERIAL_NAME = "PhysicsMaterials/VicaDriveTyre"
CASTER_MATERIAL_NAME = "PhysicsMaterials/VicaCaster"

SAVE_STAGE = True


def _require_stopped_timeline():
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Stop the Timeline before editing joint drives.")


def _collect_joints(stage):
    """Map joint name -> prim for every revolute joint in the stage."""
    found = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            found.setdefault(prim.GetName(), prim)
    return found


def _set_attr(prim, name, value):
    attr = prim.GetAttribute(name)
    if attr:
        attr.Set(value)
        return True
    return False


def _disable_drive(prim):
    """Drop the angular drive, and zero it in case the schema cannot be dropped.

    A joint composed from a referenced layer keeps its referenced opinion when
    the root layer removes the API, so RemoveAPI alone is not enough. Attribute
    overrides do compose, which is why the zeroing is not merely belt and braces.
    """
    removed = False
    try:
        removed = prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
    except Exception:  # noqa: BLE001 -- older schema builds raise rather than return False
        removed = False

    zeroed = []
    for suffix, value in (
        ("stiffness", 0.0),
        ("damping", 0.0),
        ("maxForce", 0.0),
        ("targetVelocity", 0.0),
    ):
        if _set_attr(prim, f"drive:angular:physics:{suffix}", value):
            zeroed.append(suffix)
    return removed, zeroed


def _configure_velocity_drive(prim):
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(WHEEL_STIFFNESS)
    drive.CreateDampingAttr().Set(WHEEL_DAMPING)
    drive.CreateMaxForceAttr().Set(WHEEL_MAX_FORCE)
    drive.CreateTargetVelocityAttr().Set(0.0)


def _define_material(stage, root, name, static, dynamic):
    path = root.AppendPath(name)
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(static)
    api.CreateDynamicFrictionAttr().Set(dynamic)
    api.CreateRestitutionAttr().Set(WHEEL_RESTITUTION)
    return material, str(path)


def _bind_wheel_friction(stage):
    """Grip on the drive wheels, slip on the casters.

    Bound on the robot asset rather than a stage so it travels with the robot:
    a material authored in one composed stage is one more thing to remember
    when building the next.
    """
    default_prim = stage.GetDefaultPrim()
    root = default_prim.GetPath() if default_prim else Sdf.Path("/")

    drive_mat, drive_path = _define_material(
        stage, root, DRIVE_MATERIAL_NAME,
        WHEEL_STATIC_FRICTION, WHEEL_DYNAMIC_FRICTION)
    caster_mat, caster_path = _define_material(
        stage, root, CASTER_MATERIAL_NAME,
        CASTER_STATIC_FRICTION, CASTER_DYNAMIC_FRICTION)

    bound = {"drive": [], "caster": []}
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        # The collider is a child shape of the link, so the link name is the
        # parent's: .../left_wheel_1/cylinder.
        owner = prim.GetParent().GetName()
        if "wheel" not in owner:
            continue
        is_caster = "caster" in owner
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            caster_mat if is_caster else drive_mat,
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
        bound["caster" if is_caster else "drive"].append(owner)
    return bound, drive_path, caster_path


def _configure_articulation_solver(stage):
    """Give every articulation root explicit solver iteration counts."""
    touched = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        api.CreateSolverPositionIterationCountAttr().Set(
            ARTICULATION_POSITION_ITERATIONS)
        api.CreateSolverVelocityIterationCountAttr().Set(
            ARTICULATION_VELOCITY_ITERATIONS)
        touched.append(str(prim.GetPath()))
    return touched


def main():
    _require_stopped_timeline()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")

    joints = _collect_joints(stage)

    missing = [n for n in DRIVE_JOINTS if n not in joints]
    if missing:
        raise RuntimeError(
            f"Drive joints not found: {missing}. "
            f"Revolute joints in the stage: {sorted(joints)}"
        )

    print("=== passive caster joints")
    touched = 0
    for name in CASTER_JOINTS:
        prim = joints.get(name)
        if prim is None:
            continue
        removed, zeroed = _disable_drive(prim)
        touched += 1
        state = "DriveAPI removed" if removed else "DriveAPI kept (referenced)"
        print(f"    {name:34s} {state}, zeroed: {','.join(zeroed) or 'none'}")
    if touched == 0:
        print("    none found -- check the caster joint names against the URDF")
    elif touched != 4:
        print(f"    WARNING: expected 4 caster joints, handled {touched}")

    print("\n=== drive wheels")
    for name in DRIVE_JOINTS:
        _configure_velocity_drive(joints[name])
        print(
            f"    {name:34s} velocity drive  stiffness={WHEEL_STIFFNESS} "
            f"damping={WHEEL_DAMPING} maxForce={WHEEL_MAX_FORCE}"
        )

    print("\n=== wheel friction")
    bound, drive_path, caster_path = _bind_wheel_friction(stage)
    print(f"    drive  {drive_path}  "
          f"static={WHEEL_STATIC_FRICTION} dynamic={WHEEL_DYNAMIC_FRICTION}")
    print(f"           {', '.join(sorted(set(bound['drive']))) or 'NONE FOUND'}")
    print(f"    caster {caster_path}  "
          f"static={CASTER_STATIC_FRICTION} dynamic={CASTER_DYNAMIC_FRICTION}")
    print(f"           {', '.join(sorted(set(bound['caster']))) or 'NONE FOUND'}")

    print("\n=== articulation solver")
    roots = _configure_articulation_solver(stage)
    for path in roots:
        print(f"    {path}  position={ARTICULATION_POSITION_ITERATIONS} "
              f"velocity={ARTICULATION_VELOCITY_ITERATIONS}")
    if not roots:
        print("    no articulation root found")

    if SAVE_STAGE:
        stage.GetRootLayer().Save()
        print("\nStage saved.")
    else:
        print("\nSAVE_STAGE is False -- edits made in memory only.")

    print("Next: run build_vica_ros_graphs.py, then Play and check /cmd_vel drives the robot.")


main()
