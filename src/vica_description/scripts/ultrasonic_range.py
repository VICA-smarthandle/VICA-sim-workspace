#!/usr/bin/env python3
"""Convert the simulated ultrasonic fans from LaserScan to Range.

    ros2 run vica_description ultrasonic_range

Isaac publishes each probe as a LaserScan on /ultrasonic/front_left_scan and
/ultrasonic/front_right_scan, from an RTX lidar restricted to the probe's 50
degree beam. nav2's
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

# DYP-A22, copied from the robot's own user_guidance.yaml rather than guessed.
# Every value below has to stay equal to the one there, because the whole point
# of the simulated probe is that nav2 cannot tell which one it is reading.
#
# field_of_view is NOT the angle the probe listens over. The probe hears about
# 50 degrees, firmware US_ANGLE_LEVEL 3, and Isaac's sensor is built that wide.
# This number is the arc the costmap draws, and the robot deliberately draws
# narrower than it hears: RangeSensorLayer computes its arc from the message's
# fov, and a 50 degree arc at 1.5 m is 1.31 m wide, which put marks well past
# the real silhouette and closed a 10 cm corner gap in the map. The A/B on
# 2026-09-02, same detection criterion, was 1.6 partial stops a minute at 50
# degrees against 0.2 at 30, with average speed unchanged at 0.40 and 0.39.
# Detection is unaffected because the physical beam decides that.
#
# max_range 1.50 rather than the sensor's own reach: past that the robot does
# not act on the reading, and a longer horizon only marks things the lidar has
# already accounted for.
FIELD_OF_VIEW = 0.524      # 30 degrees of marking arc; the beam is 50
MIN_RANGE = 0.02
MAX_RANGE = 1.50

# The listening beam, radians, half-width about the probe's forward axis.
# 50 degrees total, firmware US_ANGLE_LEVEL 3.
#
# Cut here rather than on the sensor. Isaac's probe sweeps the full circle, so
# without this the reported range would be the nearest thing in any direction
# at all -- including the corridor wall beside the robot and its own chassis
# behind. attach_vica_sensors records the two ways of cutting it at the sensor
# that were tried first: both stop the probe publishing, one immediately and
# one after a handful of messages, neither with an error.
BEAM_HALF_RAD = math.radians(50.0) / 2.0

PROBES = (
    ("/ultrasonic/front_left_scan", "/ultrasonic/front_left"),
    ("/ultrasonic/front_right_scan", "/ultrasonic/front_right"),
)


class UltrasonicRange(Node):
    def __init__(self):
        super().__init__("ultrasonic_range")
        # Default QoS on the Range, depth 10, which is exactly what the robot's
        # own driver uses: create_publisher(Range, t, 10) in
        # user_guidance_driver_node. Anything else here is a difference between
        # sim and hardware that nav2 can see.
        #
        # An earlier version published best effort, on the reasoning that a
        # sensor driver does and that reliable would leave the costmap layer
        # incompatible. Both halves were wrong. The driver publishes reliable,
        # and compatibility only fails the other way round: a reliable
        # publisher feeds a best effort subscriber perfectly well, while a best
        # effort publisher feeds a reliable one nothing at all. RViz asks for
        # reliable, so the cones simply never appeared:
        #
        #   New publisher discovered on topic '/ultrasonic/front_left',
        #   offering incompatible QoS. No messages will be sent to it.
        #   Last incompatible policy: RELIABILITY_QOS_POLICY
        #
        # The subscription keeps best effort. Isaac's LaserScan is published
        # reliable, and a best effort subscriber takes that happily; asking for
        # reliable off a sensor stream only buys retransmissions of scans that
        # are already stale by the time they arrive.
        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pubs = {}
        for scan_topic, range_topic in PROBES:
            pub = self.create_publisher(Range, range_topic, 10)
            self._pubs[scan_topic] = pub
            self.create_subscription(
                LaserScan, scan_topic,
                lambda msg, t=scan_topic: self._on_scan(msg, t), sub_qos)
            self.get_logger().info(f"{scan_topic} -> {range_topic}")

    def _on_scan(self, msg, scan_topic):
        hits = []
        for i, r in enumerate(msg.ranges):
            if r != r or math.isinf(r):
                continue
            if not (MIN_RANGE <= r <= MAX_RANGE):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # Wrap into (-pi, pi] before comparing, so a scan that runs 0..2pi
            # and one that runs -pi..pi are treated the same.
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            if abs(angle) <= BEAM_HALF_RAD:
                hits.append(r)
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
