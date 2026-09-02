#!/usr/bin/env python3
"""Convert the simulated ultrasonic fans from LaserScan to Range.

    ros2 run vica_description ultrasonic_range

Isaac publishes each probe as a seven-beam LaserScan on
/ultrasonic/front_left_scan and /ultrasonic/front_right_scan. nav2's
RangeSensorLayer wants sensor_msgs/Range on /ultrasonic/front_left and
/ultrasonic/front_right, which is what the robot's own driver publishes. This
is the whole difference between the two.

Why the conversion is a separate process
----------------------------------------
Two shorter routes were tried first and neither works here.

  - The ROS 2 bridge has no Range publisher. It offers LaserScan, PointCloud,
    Image, Odometry, JointState, TF and a few more; Range is not among them.
    The generic ROS2Publisher can name any message, but it resolves that
    message into dynamic input attributes only after the node has been
    evaluated, so the fields cannot be set in the edit that creates it.

  - Publishing from inside the Isaac process with rclpy takes the process
    down. Isaac's bridge has already loaded its own rcl, and the second load
    breaks the dynamic linker outright:

        Inconsistency detected by ld.so: dl-close.c: 202: _dl_close_worker:
        Assertion `(*lp)->l_idx >= 0 && (*lp)->l_idx < nloaded' failed!

    The probes initialised, announced both topics, and the simulator died at
    shutdown with nothing published.

So the message crosses the process boundary as something the bridge does
publish, and becomes a Range out here.

What an ultrasonic reports
--------------------------
The nearest echo in its cone, not a profile: that is the physical difference
between it and a lidar, and it is why the fan collapses to one number. Beams
that returned nothing come back as inf, and an ultrasonic with nothing in
front of it reports its maximum range -- which is what clear_on_max_reading in
the costmap layer is waiting for, so it is reported as max rather than
dropped.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range

# DYP-A22, from its datasheet and the URDF comment on the robot.
FIELD_OF_VIEW = 0.5236     # 30 degrees, the fan Isaac casts
MIN_RANGE = 0.02
MAX_RANGE = 4.0

PROBES = (
    ("/ultrasonic/front_left_scan", "/ultrasonic/front_left"),
    ("/ultrasonic/front_right_scan", "/ultrasonic/front_right"),
)


class UltrasonicRange(Node):
    def __init__(self):
        super().__init__("ultrasonic_range")
        # Best effort, matching what a sensor driver publishes and what the
        # costmap layer subscribes with. Reliable here would leave the
        # subscription incompatible and the layer would see nothing at all,
        # silently -- the failure looks like a probe that reports no obstacle.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pubs = {}
        for scan_topic, range_topic in PROBES:
            pub = self.create_publisher(Range, range_topic, qos)
            self._pubs[scan_topic] = pub
            self.create_subscription(
                LaserScan, scan_topic,
                lambda msg, t=scan_topic: self._on_scan(msg, t), qos)
            self.get_logger().info(f"{scan_topic} -> {range_topic}")

    def _on_scan(self, msg, scan_topic):
        hits = [r for r in msg.ranges
                if r == r and not math.isinf(r) and MIN_RANGE <= r <= MAX_RANGE]
        out = Range()
        out.header = msg.header
        out.radiation_type = Range.ULTRASOUND
        out.field_of_view = FIELD_OF_VIEW
        out.min_range = MIN_RANGE
        out.max_range = MAX_RANGE
        out.range = min(hits) if hits else MAX_RANGE
        self._pubs[scan_topic].publish(out)


def main():
    rclpy.init()
    node = UltrasonicRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
