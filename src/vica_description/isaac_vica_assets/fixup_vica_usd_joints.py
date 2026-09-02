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

import math
import os

from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

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
# Measured three times per rate, in open floor space, achieved yaw in rad/s:
#
#     solver     RTF      commanded 0.3           commanded 0.4
#      4/1      0.647   0.003  -0.001  -0.006   0.001  -0.007  -0.007
#     16/4      0.635  -0.000   0.002   0.007   0.022   0.110  -0.004
#     32/8      0.631  -0.002   0.006   0.015   0.236   0.201   0.198
#     64/16     0.562   0.158   0.165   0.153   0.245   0.233   0.237
#
# On the defaults the robot does not turn in place at all at its own wz_max of
# 0.4. Not slowly -- at all. That is the whole of why goals needing a heading
# change aborted on "Failed to make progress" while straight-line goals
# succeeded. 32/8 recovers 0.4 and 64/16 also recovers 0.3, for 13 % of real
# time against the defaults' 0.647.
#
# 128/32 is worse, not better: a commanded 0.3 produced 0.484, and the robot
# crossed 1.7 m in six seconds while being told to turn on the spot. Past this
# point the solver stops converging and starts injecting energy, so 64/16 is a
# ceiling reached by measurement rather than a number chosen for headroom.
#
# One trial per setting could not have shown any of this. The first attempt put
# 16/4 below the defaults, which cannot happen; near the threshold a single
# six-second trial is close to a coin toss, and the ordering only appears with
# repeats.
#
# Above 0.5 rad/s none of it helps -- every setting lands between 0.49 and 0.55
# for a commanded 0.8 -- so this is not the whole rotation deficit. See
# measure_rotation for what remains.
#
# Raising the drive damping a hundredfold and cutting the caster friction to a
# thousandth both changed nothing, which is what sent the search here.
#
# Both of those moved the gain in the same direction it was already too far in.
# On 2026-08-07 the wheels were measured against their own commanded rate for
# the first time, three trials each:
#
#     cmd wz 0.20   wheel target 0.560   wheel actual 0.823   147%
#     cmd wz 0.30   wheel target 0.840   wheel actual 1.338   159%
#
# A velocity drive that overshoots its target by half is not tracking it. That
# is a gain problem in the joint drive, and it is upstream of everything the
# rotation search has been looking at -- ground contact, tyre friction, caster
# swivel and solver counts were all checked on the same day and all correct.
# Damping is the velocity gain here, and 1e4 N m s/rad on a wheel of roughly
# 1e-3 kg m^2 is enormous.
#
# So these are overridable, to be swept rather than argued about:
#
#     VICA_WHEEL_DAMPING=100 make_stage.sh --prepare-only <stage.usd>
#
# Swept once already, and it found the thing the search had been missing.
# Same stage, same commands, three trials each:
#
#     cmd wz 0.30    damping 1e4    yaw 0.164    spread 0.175   <- does not repeat
#                    damping 100    yaw 0.154    spread 0.043
#
# The rotation is the same. The *repeatability* is four times better, and the
# runs that used to throw out -6.321 rad/s -- a revolution per second
# backwards -- stopped. A drive gain that large makes the solver unstable, and
# an unstable solver is why the same command had been answering differently
# every time it was asked. Every rotation figure recorded before 2026-08-07
# came off that, one sample each, and none of them should be cited.
#
# 100 is not adopted as the default yet, deliberately. It is measured against
# rotation only; nothing has checked what it does to straight-line driving,
# and the width sweep results all came off 1e4. Adopting it means re-running
# that sweep, not editing this line. Until then both stages stay at 1e4 so
# they can be compared with each other -- a stage that quietly differs from
# its neighbour is the failure this whole day was spent on.
ARTICULATION_POSITION_ITERATIONS = int(
    os.environ.get("VICA_POSITION_ITERATIONS", 64))
ARTICULATION_VELOCITY_ITERATIONS = int(
    os.environ.get("VICA_VELOCITY_ITERATIONS", 16))

# Arm joints, when the stage carries the arm variant. Nothing here applies to
# the driving robot: it has no joint whose name starts with the arm prefix, and
# the loop below finds none.
ARM_JOINT_PREFIX = os.environ.get("VICA_ARM_JOINT_PREFIX", "gen3_joint_")
ARM_HOLD_DEG = float(os.environ.get("VICA_ARM_HOLD_DEG", 1.0))
ARM_DAMPING_RATIO = float(os.environ.get("VICA_ARM_DAMPING_RATIO", 0.1))
# Where the arm is told to hold. Without this the target is zero, and zero on a
# Gen3 lite is straight up: the arm stands to attention and carries its mass as
# high as it can. config/arm_stow_pose.yaml has the derivation.
ARM_STOW_POSE = os.environ.get(
    "VICA_ARM_STOW_POSE",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "config", "arm_stow_pose.yaml"))


def _stow_targets():
    """joint name -> radians, or an empty dict when the file is not there."""
    if not os.path.isfile(ARM_STOW_POSE):
        return {}
    out = {}
    with open(ARM_STOW_POSE) as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            try:
                out[k.strip()] = float(v)
            except ValueError:
                pass
    return out

WHEEL_STIFFNESS = float(os.environ.get("VICA_WHEEL_STIFFNESS", 0.0))
WHEEL_DAMPING = float(os.environ.get("VICA_WHEEL_DAMPING", 1.0e4))
WHEEL_MAX_FORCE = float(os.environ.get("VICA_WHEEL_MAX_FORCE", 1.0e5))

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


def _configure_position_drive(prim, hold_deg, target=None):
    """Hold an arm joint where it is put, instead of letting gravity have it.

    The URDF importer brings the effort limit across as maxForce and stops
    there: stiffness, damping and the drive type are all unauthored, which is
    no drive at all. Measured on vica_testroom_arm.usd before this existed,
    the arm fell through 56 degrees in fourteen seconds and was already
    collapsed by the first sample.

    kortex_description carries no gains to copy. It has effort and velocity
    limits and an initial_value for fake hardware, and the numbers a real
    Gen3 lite runs on live in its controller configuration, not its
    description. So stiffness is derived from the one number that is
    published, rather than picked:

        stiffness = effort_limit / hold_deg_in_radians

    which is the gain at which the joint's own rated torque is spent holding
    hold_deg of error. At the default 1 degree a joint carrying its full
    rated load sags one degree, and a joint carrying a tenth of it sags a
    tenth of that.

    Damping is a fraction of stiffness rather than a critical-damping
    calculation, because the effective inertia at each joint depends on the
    pose of every joint outboard of it and there is no single value to
    critically damp against. ARM_DAMPING_RATIO is overridable for the same
    reason VICA_WHEEL_DAMPING is: to be swept rather than argued about.
    """
    effort = prim.GetAttribute("drive:angular:physics:maxForce")
    effort = effort.Get() if effort and effort.HasAuthoredValue() else None
    if not effort:
        return None

    stiffness = effort / math.radians(hold_deg)
    damping = stiffness * ARM_DAMPING_RATIO

    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(stiffness)
    drive.CreateDampingAttr().Set(damping)
    drive.CreateMaxForceAttr().Set(effort)
    # USD angular drive targets are in degrees.
    drive.CreateTargetPositionAttr().Set(math.degrees(target or 0.0))
    return effort, stiffness, damping


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

    # Keep the articulation awake.
    #
    # PhysX stops integrating a body that has stopped moving. That is right for
    # scenery and wrong for a robot that spends its first forty seconds waiting
    # for nav2's lifecycle, because a drive target written to a sleeping
    # articulation does not wake it.
    #
    # Written while chasing the 48 mm sink, and offered at the time as its
    # cause. It was not. The sink was a floor box PhysX would not collide the
    # wheels against, plus a camera_link with no mass; both are fixed where
    # they belong, in the course builders and in the URDF. Setting this changed
    # the settling height at no position. It stays because the reason in the
    # first paragraph is its own reason, not because it fixed anything.
    print("=== sleep")
    root = None
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root = prim
            break
    if root is None:
        print("    WARNING: no articulation root; sleep left at the default")
    else:
        api = PhysxSchema.PhysxArticulationAPI.Get(stage, root.GetPath())
        if not api:
            api = PhysxSchema.PhysxArticulationAPI.Apply(root)
        api.CreateSleepThresholdAttr().Set(0.0)
        api.CreateStabilizationThresholdAttr().Set(0.0)
        print(f"    {str(root.GetPath()).split('/')[-1]:34s} sleepThreshold 0.0 "
              f"(never sleeps)")

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

    arm = sorted(n for n in joints if n.startswith(ARM_JOINT_PREFIX))
    if arm:
        print(f"\n=== arm joints  (hold {ARM_HOLD_DEG} deg, "
              f"damping ratio {ARM_DAMPING_RATIO})")
        stow = _stow_targets()
        print(f"    자세: {ARM_STOW_POSE if stow else '없음, 0 으로 둡니다'}")
        for name in arm:
            got = _configure_position_drive(joints[name], ARM_HOLD_DEG,
                                            stow.get(name))
            if got is None:
                print(f"    {name:34s} SKIPPED: maxForce 가 없어 stiffness 를 "
                      f"유도할 수 없습니다")
                continue
            effort, stiffness, damping = got
            print(f"    {name:34s} position drive  effort={effort:g} "
                  f"stiffness={stiffness:.0f} damping={damping:.0f} "
                  f"target={math.degrees(stow.get(name, 0.0)):+.1f}deg")

    print("\n=== wheel friction")
    bound, drive_path, caster_path = _bind_wheel_friction(stage)
    print(f"    drive  {drive_path}  "
          f"static={WHEEL_STATIC_FRICTION} dynamic={WHEEL_DYNAMIC_FRICTION}")
    print(f"           {', '.join(sorted(set(bound['drive']))) or 'NONE FOUND'}")
    print(f"    caster {caster_path}  "
          f"static={CASTER_STATIC_FRICTION} dynamic={CASTER_DYNAMIC_FRICTION}")
    print(f"           {', '.join(sorted(set(bound['caster']))) or 'NONE FOUND'}")

    # Give cylinder colliders a convex hull.
    #
    # An earlier version of this comment said PhysX has no cylinder and that a
    # bare Cylinder collider gets no shape at all. That is wrong. PhysX carries
    # cylinders and cones as custom geometry, which is what a Cylinder prim
    # gets by default, and this project drove on bare cylinder wheels for weeks.
    #
    # What is true is that the wheels went a whole day without colliding with
    # the floor while the chassis box collided with it perfectly, and that the
    # floor was a 50 m box. Replacing that box with a ground plane fixed it;
    # applying these hulls, on its own, did not. They stay because a hull is
    # the ordinary, fully supported representation and the wheels measure
    # exactly right on it: commanded 2, 4, 6 and 8 rad/s all come back as
    # 100 per cent of r*omega with no slip. If they ever need to come off,
    # the thing to re-measure is that table.
    print("\n=== cylinder colliders")
    hulls = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Cylinder):
            continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not prim.HasAPI(PhysxSchema.PhysxConvexHullCollisionAPI):
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
        hulls += 1
        print(f"    {'/'.join(str(prim.GetPath()).split('/')[-2:]):44s} convex hull")
    if hulls == 0:
        print("    none found -- the wheels are not cylinders any more?")

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
