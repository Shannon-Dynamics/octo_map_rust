#!/usr/bin/env python3
"""Wait for one octomap_msgs/Octomap and write its payload to a file.

The point is to get the exact bytes the node put on the wire out to where a
C++ program can be pointed at them, without that program needing to be a ROS
node itself. ``decode_octomap_payload.cpp`` is the other half.

Writes two files: ``<out>`` holding ``data`` verbatim, and ``<out>.meta``
holding the ``binary``, ``id`` and ``resolution`` fields, one per line.

    python3 capture_octomap.py --topic /octomap_binary --out /tmp/map.payload
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from octomap_msgs.msg import Octomap


class Capture(Node):
    def __init__(self, topic, out, require_nonempty):
        super().__init__("capture_octomap")
        self.out = out
        self.require_nonempty = require_nonempty
        self.captured = False

        # Matching the publisher: transient-local so a map published before
        # this subscriber existed still arrives.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Octomap, topic, self.on_map, qos)
        self.get_logger().info(f"waiting for a map on {topic}")

    def on_map(self, message):
        payload = bytes(bytearray((b & 0xFF) for b in message.data))
        if self.require_nonempty and not payload:
            self.get_logger().info("got an empty map, still waiting for a real one")
            return

        with open(self.out, "wb") as handle:
            handle.write(payload)
        with open(self.out + ".meta", "w") as handle:
            handle.write(f"{'true' if message.binary else 'false'}\n")
            handle.write(f"{message.id}\n")
            handle.write(f"{message.resolution!r}\n")
            handle.write(f"{message.header.frame_id}\n")

        self.get_logger().info(
            f"captured {len(payload)} bytes: binary={message.binary} "
            f"id={message.id!r} resolution={message.resolution} "
            f"frame_id={message.header.frame_id!r}"
        )
        self.captured = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/octomap_binary")
    parser.add_argument("--out", required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="accept a map with no nodes; by default keep waiting for one",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Capture(args.topic, args.out, not args.allow_empty)

    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while rclpy.ok() and not node.captured:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > deadline:
                node.get_logger().error(f"no map within {args.timeout} s")
                break
    finally:
        captured = node.captured
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(0 if captured else 1)


if __name__ == "__main__":
    main()
