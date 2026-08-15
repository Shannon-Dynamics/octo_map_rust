# Changelog — octomap-ros

User-visible changes to this crate. Repository-level changes are in the
[root changelog](../../CHANGELOG.md).

The format follows [Keep a Changelog](https://keepachangelog.com/). Versions
follow [semantic versioning](https://semver.org/), with the 0.x convention that
the minor version is the breaking one until 1.0.

## [Unreleased]

Not yet published. This section describes the API the first release will
contain.

### Added

- `pointcloud2` — decodes a `sensor_msgs/PointCloud2` blob against its field
  descriptors: `Cloud`, `FieldRef`, `PointIter`, `CloudError`. Field offsets,
  datatypes, row stride and blob length are validated against each other before
  any point is read, and both `FLOAT32` and `FLOAT64` layouts are supported.
- `ScanFilter` and `ScanStats` — turn a decoded cloud into an
  `octomap_core::PointCloud` in the map frame, applying range, height and
  subsampling limits, and reporting what was dropped.
- `msg` — builds and parses the headerless `data` payload of an
  `octomap_msgs/Octomap`, in both binary and full form, including the `int8[]`
  reinterpretation the message type requires.
- `voxels` — occupied and free cells as centres and edge lengths, ready to
  become a `visualization_msgs/MarkerArray`, plus a height colour ramp.
- `Transform3` — the sensor → map transform, without a linear-algebra
  dependency.

### Safety

- The crate contains no `unsafe`.
- **No ROS dependency.** Every entry point takes the plain data a message
  carries, so the crate builds and is tested on machines with no ROS installed
  — including the decoding path, which is the part most exposed to input it did
  not produce.
- Integer arithmetic in `Cloud::new` and field resolution is checked rather
  than wrapping. A message declaring an extreme `width`, `point_step` or field
  offset is rejected with `CloudError::GeometryOverflow` instead of wrapping a
  length computation on a 32-bit target.
- `CloudError` is `#[non_exhaustive]`, so a message shape this decoder learns
  to reject later does not become a breaking change.
- The unit tests and the `PointCloud2` robustness suite pass under Miri with
  strict provenance. The robustness suite asserts that for arbitrary field
  descriptors, dimensions and blob lengths, decoding either fails or produces a
  cloud that can be iterated to the end.

### Known limitations

- The API is pre-1.0 and may change.
- `ColorOcTree` payloads are rejected by tree id rather than decoded as a plain
  occupancy tree.
- Transforms are applied as given; there is no TF graph or time interpolation
  here. That belongs to the caller.
