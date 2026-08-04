"""Add VICA ROS 2 drive and odometry Action Graphs to the open stage.

Run once from Isaac Sim 6.0.1's Script Editor while the Timeline is stopped.
The script only replaces the two graphs named below and then saves the stage.
"""

from pxr import UsdPhysics
from usdrt import Sdf

import omni.graph.core as og
import omni.timeline
import omni.usd


ROBOT_PATH = "/World/vica1"
GRAPH_ROOT = f"{ROBOT_PATH}/Graph"
ARTICULATION_ROOT_HINT = f"{ROBOT_PATH}/Geometry/base_footprint/base_link"
DRIVE_GRAPH = f"{GRAPH_ROOT}/ROS_DifferentialDrive"
ODOM_GRAPH = f"{GRAPH_ROOT}/ROS_Odometry"

LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"
WHEEL_RADIUS = 0.065
WHEEL_DISTANCE = 0.293


def _require_stopped_timeline():
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Stop the Timeline before modifying the Action Graphs.")


def _validate_stage(stage):
    articulation_candidates = [
        prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    exact = stage.GetPrimAtPath(ARTICULATION_ROOT_HINT)
    if exact.IsValid() and exact.HasAPI(UsdPhysics.ArticulationRootAPI):
        articulation = exact
    else:
        vica_candidates = [
            prim
            for prim in articulation_candidates
            if str(prim.GetPath()).startswith(f"{ROBOT_PATH}/")
            and str(prim.GetPath()).endswith("/Geometry/base_footprint/base_link")
        ]
        if len(vica_candidates) != 1:
            paths = [str(prim.GetPath()) for prim in articulation_candidates]
            raise RuntimeError(
                "Could not identify one VICA articulation root. "
                f"Articulation roots found in the Stage: {paths}"
            )
        articulation = vica_candidates[0]

    joint_names = {
        prim.GetName()
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.RevoluteJoint)
    }
    missing = {LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT} - joint_names
    if missing:
        raise RuntimeError(f"Missing drive joints: {sorted(missing)}")

    physics_scenes = [prim.GetPath() for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if len(physics_scenes) != 1:
        raise RuntimeError(
            f"Expected exactly one PhysicsScene, found {len(physics_scenes)}: {physics_scenes}"
        )

    return str(articulation.GetPath())


def _replace_graph(stage, graph_path):
    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)


def _create_drive_graph(articulation_root):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": DRIVE_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinearVelocity", "omni.graph.nodes.BreakVector3"),
                ("BreakAngularVelocity", "omni.graph.nodes.BreakVector3"),
                (
                    "DifferentialController",
                    "isaacsim.robot.wheeled_robots.DifferentialController",
                ),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinearVelocity.inputs:tuple"),
                ("BreakLinearVelocity.outputs:x", "DifferentialController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngularVelocity.inputs:tuple"),
                ("BreakAngularVelocity.outputs:z", "DifferentialController.inputs:angularVelocity"),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", "cmd_vel"),
                ("DifferentialController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DifferentialController.inputs:wheelDistance", WHEEL_DISTANCE),
                (
                    "ArticulationController.inputs:jointNames",
                    [LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT],
                ),
                ("ArticulationController.inputs:targetPrim", [Sdf.Path(articulation_root)]),
            ],
        },
    )


def _create_odometry_graph(articulation_root):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": ODOM_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                (
                    "PublishOdomToBaseFootprint",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdomToBaseFootprint.inputs:execIn"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                (
                    "ReadSimulationTime.outputs:simulationTime",
                    "PublishOdomToBaseFootprint.inputs:timeStamp",
                ),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdomToBaseFootprint.inputs:translation",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdomToBaseFootprint.inputs:rotation",
                ),
            ],
            keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", [Sdf.Path(articulation_root)]),
                ("PublishOdometry.inputs:topicName", "odom"),
                ("PublishOdometry.inputs:odomFrameId", "odom"),
                ("PublishOdometry.inputs:chassisFrameId", "base_footprint"),
                ("PublishOdomToBaseFootprint.inputs:topicName", "tf"),
                ("PublishOdomToBaseFootprint.inputs:parentFrameId", "odom"),
                ("PublishOdomToBaseFootprint.inputs:childFrameId", "base_footprint"),
            ],
        },
    )


def main():
    _require_stopped_timeline()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")

    articulation_root = _validate_stage(stage)
    _replace_graph(stage, DRIVE_GRAPH)
    _replace_graph(stage, ODOM_GRAPH)
    _create_drive_graph(articulation_root)
    _create_odometry_graph(articulation_root)

    stage.GetRootLayer().Save()
    print(f"Created {DRIVE_GRAPH}")
    print(f"Created {ODOM_GRAPH}")
    print(f"Using articulation root: {articulation_root}")
    print("Saved the current stage. Press Play, then verify /cmd_vel, /odom, and odom -> base_footprint.")


main()
