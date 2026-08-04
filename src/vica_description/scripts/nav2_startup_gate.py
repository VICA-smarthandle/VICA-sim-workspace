#!/usr/bin/env python3

"""Start the Nav2 lifecycle only after the complete localization TF is stable."""

import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class Nav2StartupGate(Node):
    """Keep Nav2 inactive until Isaac Sim and AMCL provide a stable TF chain."""

    def __init__(self) -> None:
        super().__init__("nav2_startup_gate")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("source_frame", "base_footprint")
        self.declare_parameter("stable_duration", 3.0)
        self.declare_parameter("retry_delay", 3.0)
        self.declare_parameter("startup_timeout", 90.0)

        self.target_frame = self.get_parameter("target_frame").value
        self.source_frame = self.get_parameter("source_frame").value
        self.stable_duration = float(self.get_parameter("stable_duration").value)
        self.retry_delay = float(self.get_parameter("retry_delay").value)
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )

    def wait_until_ready(self) -> bool:
        stable_since = None
        last_log = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            tf_ready = self.tf_buffer.can_transform(
                self.target_frame,
                self.source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            service_ready = self.lifecycle_client.service_is_ready()

            if tf_ready and service_ready:
                if stable_since is None:
                    stable_since = now
                    self.get_logger().info(
                        f"TF {self.target_frame} -> {self.source_frame} is available; "
                        f"waiting {self.stable_duration:.1f}s for startup stability"
                    )
                if now - stable_since >= self.stable_duration:
                    return True
            else:
                stable_since = None
                if now - last_log >= 5.0:
                    missing = []
                    if not tf_ready:
                        missing.append(
                            f"TF {self.target_frame} -> {self.source_frame}"
                        )
                    if not service_ready:
                        missing.append("Nav2 lifecycle service")
                    self.get_logger().info(
                        "Waiting for " + " and ".join(missing) + "; RViz remains usable"
                    )
                    last_log = now

        return False

    def call_lifecycle(self, command: int) -> bool:
        request = ManageLifecycleNodes.Request()
        request.command = command
        future = self.lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.startup_timeout,
        )
        if not future.done() or future.result() is None:
            return False
        return bool(future.result().success)

    def start_navigation(self) -> bool:
        while rclpy.ok():
            if not self.wait_until_ready():
                return False

            self.get_logger().info("Starting Nav2 managed nodes")
            if self.call_lifecycle(ManageLifecycleNodes.Request.STARTUP):
                self.get_logger().info("Nav2 managed nodes are active")
                return True

            self.get_logger().warning(
                "Nav2 startup failed; resetting managed nodes before retry"
            )
            self.call_lifecycle(ManageLifecycleNodes.Request.RESET)
            retry_until = time.monotonic() + self.retry_delay
            while rclpy.ok() and time.monotonic() < retry_until:
                rclpy.spin_once(self, timeout_sec=0.2)

        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2StartupGate()
    should_fail = False
    try:
        success = node.start_navigation()
        should_fail = not success and rclpy.ok()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if should_fail:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
