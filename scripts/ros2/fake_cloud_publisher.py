#!/usr/bin/env python3
"""Publish a synthetic PointCloud2 of a flat wall, for testing without a sensor.

The wall sits at ``x = WALL_X`` in the publisher's own frame and spans a square
in y and z. Feeding this to a mapping node gives a map whose correct contents
are known in advance: the wall's voxels occupied, the volume between the origin
and the wall free, and everything else unknown — which is what
``smoke_test.sh`` then checks against.

    python3 fake_cloud_publisher.py --frame sensor --topic /cloud_in --rate 5
"""

import argparse
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

WALL_X = 2.0
WALL_HALF_EXTENT = 0.5
WALL_SPACING = 0.05


def wall_points(nan_fraction: float):
    """The wall, as (x, y, z) tuples in the publisher's frame.

    A share of the points is replaced with NaN, which is what a depth camera
    emits for a pixel that got no return. A consumer that does not skip those
    ends up with a map full of garbage at the origin, so it is worth having in
    the test data rather than only in a unit test.
    """
    steps = int(2 * WALL_HALF_EXTENT / WALL_SPACING) + 1
    points = []
    for iy in range(steps):
        for iz in range(steps):
            y = -WALL_HALF_EXTENT + iy * WALL_SPACING
            z = -WALL_HALF_EXTENT + iz * WALL_SPACING
            drop = nan_fraction > 0 and (iy * steps + iz) % max(
                1, int(1 / nan_fraction)
            ) == 0
            if drop:
                points.append((math.nan, math.nan, math.nan))
            else:
                points.append((WALL_X, y, z))
    return points


def to_pointcloud2(points, frame_id, stamp):
    data = bytearray()
    for x, y, z in points:
        data += struct.pack("<fff", x, y, z)

    fields = [
        PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
        for n, o in (("x", 0), ("y", 4), ("z", 8))
    ]

    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(points)
    message.fields = fields
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(points)
    message.data = bytes(data)
    message.is_dense = False
    return message


class FakeCloudPublisher(Node):
    def __init__(self, topic, frame_id, rate, nan_fraction):
        super().__init__("fake_cloud_publisher")
        self.frame_id = frame_id
        self.points = wall_points(nan_fraction)
        # Reliable, which a best-effort subscriber accepts too.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        self.create_timer(1.0 / rate, self.tick)
        finite = sum(1 for p in self.points if not math.isnan(p[0]))
        self.get_logger().info(
            f"publishing {len(self.points)} points ({finite} finite) "
            f"on {topic} in frame {frame_id!r} at {rate} Hz"
        )

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        self.publisher.publish(to_pointcloud2(self.points, self.frame_id, stamp))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/cloud_in")
    parser.add_argument("--frame", default="sensor")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument(
        "--nan-fraction",
        type=float,
        default=0.1,
        help="share of points replaced with NaN, as a depth camera would",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = FakeCloudPublisher(args.topic, args.frame, args.rate, args.nan_fraction)
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
