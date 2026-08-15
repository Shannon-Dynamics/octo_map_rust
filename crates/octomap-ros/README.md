# octomap-ros

ROS 2 message conversions for [`octomap-core`](https://crates.io/crates/octomap-core):
`sensor_msgs/PointCloud2` in, `octomap_msgs/Octomap` out, plus occupied cells
ready to become a `visualization_msgs/MarkerArray`.

**This crate has no ROS dependency.** Every entry point takes the plain data a
message carries — a byte blob, a list of field descriptors, a resolution —
rather than a generated message type. So it builds and tests on a machine with
no ROS installed, the client library stays the caller's choice (`r2r`, `rclrs`,
a rosbag reader, a fixture in a test), and the conversions are unit-testable
without a running middleware.

`ros2/octomap_server_rs` in the repository is the `r2r` side of that boundary:
it does nothing but move fields between `r2r` structs and the functions here.

- Repository: <https://github.com/Shannon-Dynamics/octo_map_rust>
- ROS 2 reference: <https://github.com/Shannon-Dynamics/octo_map_rust/blob/main/docs/07-ros2.md>

## Installation

Not yet published to crates.io. Until it is:

```toml
[dependencies]
octomap-ros = { git = "https://github.com/Shannon-Dynamics/octo_map_rust" }
```

## What it covers

| Module | Direction | What it does |
|---|---|---|
| `pointcloud2` | in | Decodes a `PointCloud2` blob against its field descriptors, including non-`f32` layouts and row padding |
| `ScanFilter` | in | Turns a decoded cloud into an `octomap_core::PointCloud` in the map frame, applying range, height and subsampling limits |
| `msg` | out | Builds and parses the headerless `data` payload of an `octomap_msgs/Octomap` message |
| `voxels` | out | Occupied leaves as centres and edge lengths, with a height colour ramp |
| `Transform3` | both | The sensor → map transform, applied without pulling in a linear-algebra dependency |

## Safety

No `unsafe`: `unsafe_code` is `forbid`-ed at the workspace level. Message
decoding is the part of a ROS node most exposed to input it did not produce, so
every offset in `PointCloud2` decoding is bounds-checked and a malformed message
becomes a `CloudError`, never an out-of-range read.

## License

Apache-2.0. See `LICENSE`, and `NOTICE` for the upstream OctoMap attribution
that must travel with any redistribution.
