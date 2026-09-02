"""Rebuild every VICA ROS 2 interface graph in the open Isaac Sim stage.

Run once from Isaac Sim 6.0.1's Script Editor while the Timeline is stopped.

This replaces add_drive_odom_graphs.py, which only covered two of the graphs the
robot needs. Joint states, the RTX lidar and the camera were authored through
the UI, so they lived in the USD alone: re-importing the robot or rebuilding the
stage silently dropped them with no record of what went missing. Every graph the
ROS side depends on is now created here, so a broken stage recovers by running
this file again.

Graphs created (all under <ROBOT_PATH>/Graph):

    ROS_Clock              /clock
    ROS_DifferentialDrive  /cmd_vel  -> wheel velocity targets
    ROS_Odometry           /odom  and  odom -> base_footprint on /tf
    ROS_JointStates        /joint_states
    ROS_Lidar              /scan
    ROS_Camera             /rgb, /depth, /camera_info

Sensor graphs are skipped with a warning when their prim is absent, so the
script still succeeds on a stage that carries only the robot body.
"""

from pxr import Usd, UsdGeom, UsdPhysics
from usdrt import Sdf

import omni.graph.core as og
import omni.timeline
import omni.usd


# --------------------------------------------------------------------------
# Stage layout
# --------------------------------------------------------------------------
# The robot path is DERIVED from the articulation root, never hard-coded.
# Referencing the robot into an environment adds a namespace -- the working
# stage nests it as /World/vica2/vica1 rather than /World/vica1 -- and a fixed
# constant is what silently broke the earlier graphs. Everything below keys off
# whatever prim actually carries ArticulationRootAPI.
ARTICULATION_SUFFIX = "/Geometry/base_footprint/base_link"

# Graphs are authored at stage level, NOT inside the robot prim. The robot is
# brought in as a reference + payload (/World/vica2 -> /World/vica2/vica1), and
# prims composed from a reference cannot be removed by the root layer: deleting
# one only drops the local opinion while the referenced prim remains, so
# rebuilding a graph in place fails with "Failed to wrap graph in node".
#
# Keeping the graphs outside the reference also means re-importing the robot no
# longer takes the ROS interface down with it -- which is what happened before.
GRAPH_PARENT = "/World/VICA_ROS"

# Set an explicit path when the robot subtree holds more than one sensor of the
# same kind; leave None to search by prim type.
LIDAR_PRIM_HINT = None

# The RSD455 asset carries four Camera prims -- colour, pseudo-depth and the two
# stereo eyes -- so colour and depth each need their own render product. Paths
# are given as suffixes and resolved against the robot root, which keeps them
# valid however deeply the robot is referenced into an environment.
CAMERA_COLOR_SUFFIX = (
    "/Geometry/base_footprint/base_link/camera_link/rsd455/RSD455"
    "/Camera_OmniVision_OV9782_Color"
)
CAMERA_DEPTH_SUFFIX = (
    "/Geometry/base_footprint/base_link/camera_link/rsd455/RSD455/Camera_Pseudo_Depth"
)

# --------------------------------------------------------------------------
# Robot constants -- keep in step with urdf/VICA.xacro
# --------------------------------------------------------------------------
LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"

# Geometry, not the odometry calibration constant. The real robot's
# encoder_feedback wheel_base_m is 0.37, an effective tread fitted by rotation
# test; it compensates for tyre scrub that PhysX does not reproduce. Isaac must
# use the measured centre-to-centre distance instead.
WHEEL_RADIUS = 0.065
WHEEL_DISTANCE = 0.364

# --------------------------------------------------------------------------
# Publish gating
# --------------------------------------------------------------------------
# Graphs that stamp a message with simulation time publish through an
# IsaacSimulationGate rather than straight off the tick.
#
# OnPlaybackTick fires per rendered frame; simulation time only advances on a
# physics step. Frames outrun physics -- measured 84 Hz of frames against 60 Hz
# of physics -- so the extra frames publish again carrying a timestamp already
# used. On /odom that came to 86 of 299 gaps being exactly zero. It is not an
# artefact of how this was measured; it reproduces under plain timeline
# playback, which is what the GUI does.
#
# One repeat is enough to kill cartographer outright:
#
#     map_by_time.h:43  Check failed: data.time > prev
#
# and the NaN reported next to it is the pose extrapolator dividing by the
# zero-length interval between two poses.
#
# The gate lets one tick through every GATE_STEP. At 84 frames against 60
# physics steps, every second frame leaves at least one step behind it, so the
# timestamp always moves. Two things would break that and both are worth
# knowing: a frame rate more than twice the physics rate, and a physics rate
# changed without revisiting this number.
#
# OnPhysicsStep would be the direct answer and does not work here -- a graph
# built on it publishes nothing at all, under this harness or under timeline
# playback. The gate is the fallback that does work.
#
# The subscriber graph is left ungated: /cmd_vel arriving twice applies the
# same velocity twice, which costs nothing and reacts sooner. The sensor graphs
# are left alone too, being driven by render products; /scan measures a clean
# 10.0 Hz with no repeats.
GATE_NODE = "PublishGate"
GATE_TYPE = "isaacsim.core.nodes.IsaacSimulationGate"
GATE_STEP = 2

# --------------------------------------------------------------------------
# ROS frames and topics
# --------------------------------------------------------------------------
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
LIDAR_FRAME = "laser_frame"

# Camera topics and frames imitate realsense2_camera on the physical robot
# rather than being named for what they are. Anything that consumes the camera
# -- vica_nvblox_bringup remaps all four of these by name -- then runs against
# the simulator unmodified, which is the only way the sim result means anything
# about the robot.
#
#     ros2 topic list  (robot)          this file
#     /camera/camera/color/image_raw    CAMERA_RGB_TOPIC
#     /camera/camera/color/camera_info  CAMERA_COLOR_INFO_TOPIC
#     /camera/camera/depth/image_rect_raw
#     /camera/camera/depth/camera_info  CAMERA_DEPTH_INFO_TOPIC
#
# The driver supplies the two optical frames at runtime; in simulation
# VICA.xacro declares them instead.
CAMERA_COLOR_FRAME = "camera_color_optical_frame"
CAMERA_DEPTH_FRAME = "camera_depth_optical_frame"

# The velocity the robot actually executes: the collision monitor's output.
#
# Not cmd_vel_smoothed, which is the monitor's input. Subscribing there would
# work -- measured, 0.12 m to 1.05 m on a direct publish -- and it is what the
# physical robot effectively does, since its nav2_map_test.launch.py remaps
# cmd_vel_smoothed to /cmd_vel_req for its safety supervisor. But it would also
# route around the monitor, so a monitor turned back on later would be
# configured, running, and never tested. Its polygons are disabled in
# vica_nav2_params.yaml instead, which leaves it in the path and transparent.
#
# Staying on cmd_vel also keeps the recovery behaviours connected. Jazzy's
# navigation_launch.py gives the behaviour server the same cmd_vel remap as
# controller_server -- measured, /cmd_vel_nav has six publishers, one of them
# behavior_server -- so Spin, BackUp and DriveOnHeading travel the same path
# through the smoother and the monitor and reach the robot.
#
# The physical robot's Humble launch does not, which is why its /cmd_vel
# measured five publishers and zero subscribers on 2026-07-29: recovery
# behaviours had never once moved it. That is a difference in the two nav2
# versions' launch files, not something to reproduce here.
CMD_VEL_TOPIC = "cmd_vel"

SCAN_TOPIC = "scan"
CAMERA_RGB_TOPIC = "/camera/camera/color/image_raw"
CAMERA_DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
CAMERA_COLOR_INFO_TOPIC = "/camera/camera/color/camera_info"
CAMERA_DEPTH_INFO_TOPIC = "/camera/camera/depth/camera_info"
# The name realsense2_camera gives its organised cloud when pointcloud output is
# enabled. Nav2's VoxelLayer takes this as a second observation source, which is
# what makes a VoxelLayer worth having: fed only a 2D LaserScan it produces the
# same answer as an ObstacleLayer for more work.
#
# This is not nvblox. The robot runs nvblox_ros, an Isaac ROS package with no
# Jazzy build, inside a Humble container; it fuses depth into a TSDF and slices
# a costmap out of it, which handles a moving obstacle's history far better than
# marking raw points does. What this shares with it is the part that matters
# first -- the camera contributing obstacles at all, so the robot stops being
# blind to anything the lidar plane at 0.382 m passes over.
CAMERA_DEPTH_PCL_TOPIC = "/camera/camera/depth/color/points"
# Each camera renders at its own sensor's aspect ratio, not a shared one.
#
# Isaac computes fx from width and horizontalAperture, fy from height and
# verticalAperture, then forces fy to fx for square pixels. Render at a ratio
# the apertures do not share and the vertical field of view is silently
# replaced by whatever the horizontal one implies -- at 640x480 the depth
# camera's 64.9 degrees became 74.3.
#
# Depth is 848x480, the D455's native depth resolution, matching the apertures
# attach_vica_sensors.py sets for 58 degrees vertical.
# Colour is 640x400, the OV9782's 1.6 aspect, which the asset's own optics
# already suit.
CAMERA_COLOR_WIDTH = 640
CAMERA_COLOR_HEIGHT = 400
CAMERA_DEPTH_WIDTH = 848
CAMERA_DEPTH_HEIGHT = 480

# The point cloud renders far smaller than the depth image, on the same camera.
#
# A costmap does not need 407 040 points at 30 Hz. Feeding it that starved the
# controller: the loop asked for 20 Hz and got a median of 2.9, erratic between
# 1.3 and 30, and "Failed to make progress" followed. Turning the source off
# restored 20 Hz exactly, so the cloud was the cost, not the footprint checking
# it was first blamed on.
#
# 212x120 keeps the 848x480 aspect, so the field of view is identical and only
# the sampling density drops -- a sixteenth of the points. At 0.05 m costmap
# cells, 25 440 points still put several samples in every cell the camera can
# see. Resolution and field of view are separate, which is the same distinction
# that fixed the camera optics.
CAMERA_DEPTH_PCL_WIDTH = 212
CAMERA_DEPTH_PCL_HEIGHT = 120

# Set False to build the graphs without writing the stage back to disk. Used by
# the headless validation harness; leave True for normal Script Editor runs.
SAVE_STAGE = True

GRAPHS = [
    "ROS_Clock",
    "ROS_DifferentialDrive",
    "ROS_Odometry",
    "ROS_JointStates",
    "ROS_Lidar",
    "ROS_Camera",
]

# The ultrasonic probes are NOT published from a graph. There is no
# ROS2PublishRange node, and the generic ROS2Publisher resolves its message
# fields into dynamic attributes only after the node has been evaluated, so
# they cannot be set in the same edit that creates it. play_stage.py reads the
# raycast sensors and publishes sensor_msgs/Range with rclpy instead, which is
# a plainer thing to verify. See the note there.


# --------------------------------------------------------------------------
# Validation
# --------------------------------------------------------------------------
def _require_stopped_timeline():
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Stop the Timeline before rebuilding the Action Graphs.")


def _find_articulation_root(stage):
    """Return (articulation_root_path, robot_root_path).

    Matches on the path tail so the robot is found at whatever depth the stage
    references it in.
    """
    roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    vica = [p for p in roots if p.endswith(ARTICULATION_SUFFIX)]

    if len(vica) != 1:
        raise RuntimeError(
            f"Expected exactly one prim ending in {ARTICULATION_SUFFIX}, found {len(vica)}. "
            f"Articulation roots in the stage: {roots}"
        )

    articulation_root = vica[0]
    robot_root = articulation_root[: -len(ARTICULATION_SUFFIX)]
    return articulation_root, robot_root


def _require_drive_joints(stage):
    names = {p.GetName() for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
    missing = {LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT} - names
    if missing:
        raise RuntimeError(f"Missing drive joints: {sorted(missing)}")


def _require_ros2_nodes():
    """Fail early and clearly when the ROS 2 bridge did not register its nodes.

    Isaac loads librmw_implementation.so from isaacsim.ros2.core, which in turn
    needs the ROS 2 Jazzy libraries on LD_LIBRARY_PATH. Launching Isaac without
    sourcing the ROS environment leaves every isaacsim.ros2.bridge.* node type
    unresolved, and og.Controller.edit then builds a graph of empty nodes
    instead of raising -- the failure only shows up later as silent topics.
    """
    import ctypes

    try:
        ctypes.CDLL("libament_index_cpp.so")
    except OSError as exc:
        raise RuntimeError(
            "ROS 2 libraries are not on the loader path, so isaacsim.ros2.bridge "
            "cannot register its node types and every graph below would be built "
            f"from unresolved nodes.\n  ({exc})\n\n"
            "Close Isaac Sim and relaunch it from a shell that sourced ROS 2:\n"
            "    source /opt/ros/jazzy/setup.bash\n"
            "    ~/isaacsim/isaac-sim.sh"
        ) from None


def _require_single_physics_scene(stage):
    scenes = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if len(scenes) != 1:
        raise RuntimeError(f"Expected exactly one PhysicsScene, found {len(scenes)}: {scenes}")


def _find_sensor(stage, robot_root, hint, type_names, label):
    """Locate a sensor prim under the robot subtree, or return None.

    Restricting the search to the robot keeps environment and viewport cameras
    out of the way -- the working stage carries eight Camera prims in total.
    """
    if hint:
        prim = stage.GetPrimAtPath(hint)
        if not prim.IsValid():
            raise RuntimeError(f"{label} hint path does not exist: {hint}")
        return hint

    found = [
        str(p.GetPath())
        for p in stage.Traverse()
        if str(p.GetPath()).startswith(f"{robot_root}/") and p.GetTypeName() in type_names
    ]
    if not found:
        print(f"    no {label} prim under {robot_root}")
        return None
    if len(found) > 1:
        raise RuntimeError(
            f"Found {len(found)} {label} prims under {robot_root}; "
            f"set the hint constant to one of {found}"
        )
    return found[0]


def _resolve_suffix(stage, robot_root, suffix, label):
    """Resolve a robot-relative prim path, or return None with a note."""
    path = f"{robot_root}{suffix}"
    if stage.GetPrimAtPath(path).IsValid():
        return path
    print(f"    {label} prim not found at {path}")
    return None


def _replace_graph(stage, graph_root, name):
    """Clear a stage-level graph so og.Controller.edit rebuilds it cleanly."""
    path = f"{graph_root}/{name}"
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
    return path


def _deactivate_legacy_graphs(stage, robot_root):
    """Switch off ROS graphs that live inside the robot reference.

    Those prims cannot be deleted from the root layer, but they can be
    deactivated, which stops them evaluating. Leaving them running alongside the
    new stage-level graphs would double-publish every topic.
    """
    legacy = [
        p
        for p in stage.Traverse()
        if str(p.GetPath()).startswith(f"{robot_root}/Graph/") and p.GetTypeName() == "OmniGraph"
    ]
    for prim in legacy:
        prim.SetActive(False)
        print(f"    deactivated legacy graph: {prim.GetPath()}")
    return [str(p.GetPath()) for p in legacy]


# --------------------------------------------------------------------------
# Graphs
# --------------------------------------------------------------------------
def _clock_graph(path):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                (GATE_NODE, GATE_TYPE),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", f"{GATE_NODE}.inputs:execIn"),
                (f"{GATE_NODE}.outputs:execOut", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [("PublishClock.inputs:topicName", "clock")],
        },
    )


def _drive_graph(path, articulation_root):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("BreakLinear.outputs:x", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("BreakAngular.outputs:z", "DiffController.inputs:angularVelocity"),
                (
                    "DiffController.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", CMD_VEL_TOPIC),
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", WHEEL_DISTANCE),
                # Never leave jointNames empty: an empty list means *every* joint,
                # so this node would stamp zero velocity onto the casters too.
                (
                    "ArticulationController.inputs:jointNames",
                    [LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT],
                ),
                ("ArticulationController.inputs:targetPrim", [Sdf.Path(articulation_root)]),
            ],
        },
    )


def _odometry_graph(path, articulation_root):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                (GATE_NODE, GATE_TYPE),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", f"{GATE_NODE}.inputs:execIn"),
                (f"{GATE_NODE}.outputs:execOut", "ComputeOdometry.inputs:execIn"),
                (f"{GATE_NODE}.outputs:execOut", "PublishOdometry.inputs:execIn"),
                (f"{GATE_NODE}.outputs:execOut", "PublishOdomTf.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTf.inputs:timeStamp"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "PublishOdomTf.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTf.inputs:rotation"),
            ],
            keys.SET_VALUES: [
                (f"{GATE_NODE}.inputs:step", GATE_STEP),
                ("ComputeOdometry.inputs:chassisPrim", [Sdf.Path(articulation_root)]),
                ("PublishOdometry.inputs:topicName", "odom"),
                ("PublishOdometry.inputs:odomFrameId", ODOM_FRAME),
                ("PublishOdometry.inputs:chassisFrameId", BASE_FRAME),
                ("PublishOdomTf.inputs:topicName", "tf"),
                ("PublishOdomTf.inputs:parentFrameId", ODOM_FRAME),
                ("PublishOdomTf.inputs:childFrameId", BASE_FRAME),
            ],
        },
    )


def _joint_state_graph(path, articulation_root):
    """Publish /joint_states through IsaacReadJointState.

    ROS2PublishJointState still accepts a targetPrim, but its own documentation
    marks the connected inputs as the preferred path: "Joint names from Isaac
    Read Joint State (connect instead of targetPrim for preferred path)". The
    targetPrim branch is what raises the deprecation warning in the console, and
    it spins up a second tensor simulation view to do the same work.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                (GATE_NODE, GATE_TYPE),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", f"{GATE_NODE}.inputs:execIn"),
                (f"{GATE_NODE}.outputs:execOut", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                (
                    "ReadJointState.outputs:stageMetersPerUnit",
                    "PublishJointState.inputs:stageMetersPerUnit",
                ),
            ],
            keys.SET_VALUES: [
                (f"{GATE_NODE}.inputs:step", GATE_STEP),
                ("ReadJointState.inputs:prim", [Sdf.Path(articulation_root)]),
                ("PublishJointState.inputs:topicName", "joint_states"),
            ],
        },
    )


def _lidar_graph(path, lidar_prim):
    """RTX lidar -> /scan.

    RunOnce is not decoration. The first version of this graph ran
    IsaacCreateRenderProduct straight off OnPlaybackTick, so a new render
    product was created every tick. An RTX lidar accumulates a revolution
    across many ticks, and rebuilding its render product underneath it restarts
    that accumulation before it ever completes -- /scan is advertised, the node
    runs, and not one message comes out. Creating the render product on a
    single frame is what the GUI-authored graph that used to publish /scan did,
    and the difference between the two was exactly this node.

    fullScan is deliberately left alone: the node documentation states "RTX
    Lidar now always produces full scans via accumulateOutputs; this setting is
    ignored". The deprecation warning appears regardless of whether it is set.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "LidarHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),
                # The other publishers here run on the default context happily.
                # This one is wired explicitly because the graph that used to
                # publish /scan wired it, and after RunOnce it was the only
                # structural difference left between the two.
                ("Context.outputs:context", "LidarHelper.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [Sdf.Path(lidar_prim)]),
                ("LidarHelper.inputs:type", "laser_scan"),
                ("LidarHelper.inputs:topicName", SCAN_TOPIC),
                ("LidarHelper.inputs:frameId", LIDAR_FRAME),
            ],
        },
    )


def _camera_graph(path, color_prim, depth_prim):
    """Colour and depth each get their own render product and camera_info.

    The RSD455 models them as separate Camera prims, so a single shared render
    product would publish the colour sensor twice rather than colour + depth.

    Two camera_info publishers rather than one, because that is what the
    RealSense driver does: colour and depth are different imagers with
    different intrinsics, and a consumer that rectifies depth against the
    colour intrinsics gets a subtly wrong point cloud. nvblox subscribes to
    both separately.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ColorRp", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthRp", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishColorInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("DepthPclRp", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishDepthPcl", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ColorRp.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DepthRp.inputs:execIn"),
                ("ColorRp.outputs:execOut", "PublishRgb.inputs:execIn"),
                ("ColorRp.outputs:execOut", "PublishColorInfo.inputs:execIn"),
                ("DepthRp.outputs:execOut", "PublishDepth.inputs:execIn"),
                ("DepthRp.outputs:execOut", "PublishDepthInfo.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DepthPclRp.inputs:execIn"),
                ("DepthPclRp.outputs:execOut", "PublishDepthPcl.inputs:execIn"),
                ("ColorRp.outputs:renderProductPath", "PublishRgb.inputs:renderProductPath"),
                ("ColorRp.outputs:renderProductPath", "PublishColorInfo.inputs:renderProductPath"),
                ("DepthRp.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
                ("DepthRp.outputs:renderProductPath", "PublishDepthInfo.inputs:renderProductPath"),
                ("DepthPclRp.outputs:renderProductPath", "PublishDepthPcl.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("ColorRp.inputs:cameraPrim", [Sdf.Path(color_prim)]),
                ("ColorRp.inputs:width", CAMERA_COLOR_WIDTH),
                ("ColorRp.inputs:height", CAMERA_COLOR_HEIGHT),
                ("DepthRp.inputs:cameraPrim", [Sdf.Path(depth_prim)]),
                ("DepthRp.inputs:width", CAMERA_DEPTH_WIDTH),
                ("DepthRp.inputs:height", CAMERA_DEPTH_HEIGHT),
                ("DepthPclRp.inputs:cameraPrim", [Sdf.Path(depth_prim)]),
                ("DepthPclRp.inputs:width", CAMERA_DEPTH_PCL_WIDTH),
                ("DepthPclRp.inputs:height", CAMERA_DEPTH_PCL_HEIGHT),
                ("PublishRgb.inputs:type", "rgb"),
                ("PublishRgb.inputs:topicName", CAMERA_RGB_TOPIC),
                ("PublishRgb.inputs:frameId", CAMERA_COLOR_FRAME),
                ("PublishColorInfo.inputs:topicName", CAMERA_COLOR_INFO_TOPIC),
                ("PublishColorInfo.inputs:frameId", CAMERA_COLOR_FRAME),
                ("PublishDepth.inputs:type", "depth"),
                ("PublishDepth.inputs:topicName", CAMERA_DEPTH_TOPIC),
                ("PublishDepth.inputs:frameId", CAMERA_DEPTH_FRAME),
                ("PublishDepthInfo.inputs:topicName", CAMERA_DEPTH_INFO_TOPIC),
                ("PublishDepthInfo.inputs:frameId", CAMERA_DEPTH_FRAME),
                # Same render product as the depth image; one more publisher on
                # it rather than a third render product.
                ("PublishDepthPcl.inputs:type", "depth_pcl"),
                ("PublishDepthPcl.inputs:topicName", CAMERA_DEPTH_PCL_TOPIC),
                ("PublishDepthPcl.inputs:frameId", CAMERA_DEPTH_FRAME),
            ],
        },
    )


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------
def main():
    _require_stopped_timeline()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")

    articulation_root, robot_root = _find_articulation_root(stage)
    graph_root = GRAPH_PARENT
    _require_drive_joints(stage)
    _require_single_physics_scene(stage)
    _require_ros2_nodes()

    lidar_prim = _find_sensor(stage, robot_root, LIDAR_PRIM_HINT, {"OmniLidar"}, "lidar")
    color_prim = _resolve_suffix(stage, robot_root, CAMERA_COLOR_SUFFIX, "camera colour")
    depth_prim = _resolve_suffix(stage, robot_root, CAMERA_DEPTH_SUFFIX, "camera depth")

    legacy = _deactivate_legacy_graphs(stage, robot_root)

    for name in GRAPHS:
        _replace_graph(stage, graph_root, name)

    built, skipped = [], []

    _clock_graph(f"{graph_root}/ROS_Clock")
    built.append("ROS_Clock")

    _drive_graph(f"{graph_root}/ROS_DifferentialDrive", articulation_root)
    built.append("ROS_DifferentialDrive")

    _odometry_graph(f"{graph_root}/ROS_Odometry", articulation_root)
    built.append("ROS_Odometry")

    _joint_state_graph(f"{graph_root}/ROS_JointStates", articulation_root)
    built.append("ROS_JointStates")

    if lidar_prim:
        _lidar_graph(f"{graph_root}/ROS_Lidar", lidar_prim)
        built.append("ROS_Lidar")
    else:
        skipped.append("ROS_Lidar (no OmniLidar prim under the robot)")

    if color_prim and depth_prim:
        _camera_graph(f"{graph_root}/ROS_Camera", color_prim, depth_prim)
        built.append("ROS_Camera")
    else:
        skipped.append("ROS_Camera (RSD455 colour/depth prims not found)")

    if SAVE_STAGE:
        stage.GetRootLayer().Save()

    print(f"articulation root : {articulation_root}")
    print(f"lidar prim        : {lidar_prim or '-'}")
    print(f"camera colour     : {color_prim or '-'}")
    print(f"camera depth      : {depth_prim or '-'}")
    print(f"wheel distance    : {WHEEL_DISTANCE} m   radius {WHEEL_RADIUS} m")
    print(f"graph root        : {graph_root}")
    print("built             : " + ", ".join(built))
    for s in skipped:
        print(f"SKIPPED           : {s}")
    if legacy:
        print(f"deactivated       : {len(legacy)} legacy graph(s) inside the robot reference")
    if SAVE_STAGE:
        print("Stage saved. Press Play, then check /clock /cmd_vel /odom /joint_states /scan /tf.")
    else:
        print("SAVE_STAGE is False -- graphs built in memory, stage NOT written.")


main()
