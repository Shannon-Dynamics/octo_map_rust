# Changelog — octomap-core

User-visible changes to this crate. Repository-level changes are in the
[root changelog](../../CHANGELOG.md).

The format follows [Keep a Changelog](https://keepachangelog.com/). Versions
follow [semantic versioning](https://semver.org/), with the 0.x convention that
the minor version is the breaking one until 1.0.

## [Unreleased]

Not yet published. This section describes the API the first release will
contain.

### Added

- `Point3`, `Pose6`, `Quaternion` — coordinates, sensor poses and rotations.
- `OcTreeKey`, `KeyRay`, `TreeGeometry` — voxel addressing and every
  coordinate ↔ key conversion, including depth-limited and bounds-checked
  variants.
- `OctreeCore<T>` — a generic octree: insert, search, delete, expand, prune,
  and tree / leaf / depth-limited iterators. Not tied to occupancy.
- `OcTree` — the probabilistic occupancy map: log-odds updates, clamping, a
  configurable `SensorModel`, lazy insertion, inner-node propagation,
  max-likelihood collapse, and change detection.
- **Three-state queries.** `is_occupied_at`, `get_occupancy_at` and
  `get_log_odds_at` return `Option`, keeping *never observed* distinct from
  *free*.
- `PointCloud`, ray traversal (`compute_ray_keys`), ray casting (`cast_ray`
  with a typed miss reason), `insert_ray`, `insert_point_cloud`,
  `insert_point_cloud_rays`, and the `compute_update` pair that works out what a
  scan would change without applying it.
- `io` — `.bt` and `.ot` reading and writing, as files, as generic
  `Read`/`Write`, and as the headerless payloads an `octomap_msgs/Octomap`
  carries.
- `OctomapError` and `IoError`, both implementing `std::error::Error`.

### Safety

- The crate contains no `unsafe`: `unsafe_code = "forbid"` is set at the
  workspace level, so the compiler rejects it rather than a reviewer having to.
- No runtime dependencies, so no third-party `unsafe` is inherited.
- The unit test suite passes under Miri with strict provenance.
- File parsing is bounded by the input it is given: header fields are metadata
  and never drive an allocation, and the parser rejects nesting deeper than the
  tree depth rather than recursing until the stack runs out.

### Documentation

- The crate README is the rustdoc front page, so its quick start is a doc-test.
- Every public item is documented; `missing_docs` is denied at the workspace
  level.
- Examples: `insert_and_query`, `save_load`, `ray_cast`, `write_scene`,
  `dump_bench_fixture`.

### Known limitations

- The API is pre-1.0 and may change.
- Derived tree types (`ColorOcTree`, `OcTreeStamped`, `CountingOcTree`,
  `ScanGraph`, `MapCollection`) are not implemented.
- Bounding-box-limited insertion (`setBBXMin` / `setBBXMax`) is not
  implemented.
- The legacy headerless `.bt` format is rejected with a typed error rather than
  guessed at.
- Insertion is single-threaded.
- Node values are read and written little-endian unconditionally, so big-endian
  platforms are unsupported.
