# 6. Code tour

What each module does, its main public API, and the decisions embedded in it.
For the larger picture see [`01-architecture.md`](01-architecture.md).

---

## 6.1 `octomap-core` — 11 modules, ~6,300 lines

### [`error.rs`](../crates/octomap-core/src/error.rs) (67)

`OctomapError` and the `Result<T>` alias. Failures that are an `assert` or
undefined behaviour in C++ appear here as typed errors.

### [`point.rs`](../crates/octomap-core/src/point.rs) (239)

`Point3` — three `f32`, vector arithmetic, `const fn new`. Deliberately plain:
this type appears in every signature and must not carry anything with it.

`f32` rather than `f64` is a correctness requirement, not a storage choice.
The reference's `point3d` is `octomath::Vector3`, which is `float`-backed;
widening here would silently diverge whenever a coordinate is rounded on its way
into a key.

### [`key.rs`](../crates/octomap-core/src/key.rs) (302)

| Item | What it is |
|---|---|
| `KeyScalar = u16`, `OcTreeKey` | The discrete address of a voxel |
| `compute_child_key`, `compute_child_index`, `compute_index_key` | Child and level navigation, following `OcTreeKey.h` |
| `KEY_RAY_MAX_SIZE = 100_000`, `KeyRay` | The ray buffer |

`KeyRay` **stops** at the 100,000-key limit rather than writing past the buffer.
C++ uses an `assert`, which is compiled out in a release build — one of the two
places this port is deliberately stricter
([ADR-0008](decisions/0008-stricter-than-reference.md)).

### [`node.rs`](../crates/octomap-core/src/node.rs) (400)

`Node<T>` with `CHILD_COUNT = 8`. Its children are
`Option<Box<[Option<Node<T>>; 8]>>` — **lazily allocated, and all eight in one
allocation**. C++ allocates each node separately through
`AbstractOcTreeNode**`.

That layout difference is the likely explanation for the key-query timing
difference recorded in [`05-regression-baselines.md`](05-regression-baselines.md): an octree
descent is pointer chasing, and eight children adjacent in memory help there.

### [`pose.rs`](../crates/octomap-core/src/pose.rs) (505)

`Quaternion` (`from_euler` via a rotation matrix, `to_euler`, `rotate`,
axis-angle, `normalize`) and `Pose6` (transform, inverse, composition).

Two things that must not be changed:

- **`Pose6` stores a quaternion, not Euler angles.** The learning guide
  described it as `x, y, z, roll, pitch, yaw`; the C++ source stores a
  quaternion and derives Euler angles per call through a rotation matrix and
  `atan2`. Storing Euler angles would change the result of `pose * pose` and
  `pose.inv()`, and would introduce gimbal lock the reference does not have.
  [ADR-0011](decisions/0011-pose6-stores-quaternion.md).
- **`Quaternion::norm` squares in `f32`** then accumulates into `f64`. Promoting
  first shifts the result by exactly one ULP.
  [ADR-0006](decisions/0006-quaternion-norm-f32.md).

### [`geometry.rs`](../crates/octomap-core/src/geometry.rs) (515)

`TreeGeometry` and every coord ↔ key conversion, including the `at_depth`,
`checked` and `adjust_key_at_depth` variants. `DEFAULT_TREE_DEPTH = 16`,
`DEFAULT_TREE_MAX_VAL = 32768`.

**Scaling multiplies by a cached `1.0 / resolution`; it does not divide.**
Dividing moves points across voxel boundaries
([ADR-0005](decisions/0005-reciprocal-multiply.md)).

`coord_to_key_axis_checked` adds an `is_finite()` check per axis that C++ does
not have — three extra branches per query, and a measurably more expensive
conversion than the reference's. That cost is paid deliberately, to reject NaN
input that is undefined behaviour on the C++ path.

### [`io.rs`](../crates/octomap-core/src/io.rs) (1023)

Four pairs of functions that are easy to confuse:

| Function | Produces |
|---|---|
| `write_binary_file` / `read_binary_file` | A `.bt` file **with the header** |
| `write_full_file` / `read_full_file` | An `.ot` file **with the header** |
| `write_binary_data` / `read_binary_data` | The node payload **without a header** — the contents of the `data` field in `octomap_msgs` |
| `write_full_data` / `read_full_data` | The same, in the full format |

Plus `write_full` / `read_full` / `write_binary` / `write_binary_const` /
`read_binary` for a generic `Write`/`Read`.

Piping the contents of a `.bt` file into a message's `data` field produces a
message nobody can decode, and the symptom does not point at the cause.

The resolution is written with **six significant digits**, following C++
`ostream` defaults — lossy for fine resolutions, and required for byte-identical
files ([ADR-0007](decisions/0007-resolution-six-digits.md)).

The parser **rejects** a payload nested deeper than the tree depth rather than
recursing until the stack runs out. There is a fuzz test for it
([ADR-0008](decisions/0008-stricter-than-reference.md)).

### [`tree.rs`](../crates/octomap-core/src/tree.rs) (937)

`OctreeCore<T>` — the tree, knowing nothing about occupancy: insert, search,
delete, expand, prune, plus three iterators (tree, leaf, depth-limited).

**`prune()` stops at the first level that merges nothing**, so a partially
pruned tree can leave a collapsible level above it untouched. C++ marks this
`FIXME`; it is reproduced deliberately because fixing it would change pruned
output relative to the reference
([ADR-0003](decisions/0003-prune-stops-early.md)).

### [`ray.rs`](../crates/octomap-core/src/ray.rs) (1012)

`PointCloud`, `RayCast`, `RayCastMiss`, `compute_ray_keys` (Amanatides–Woo
DDA), `cast_ray`, `insert_ray`, `compute_update` / `compute_discrete_update`,
`insert_point_cloud`, `insert_point_cloud_rays`.

`compute_ray_keys` and `cast_ray` are **inconsistent with each other on
purpose** — the first narrows the voxel-border offset to `float`, the second
keeps `double`. The reference does this, and both spellings are reproduced as
found ([ADR-0004](decisions/0004-raykeys-castray-narrowing.md)).

`compute_update` is the hottest path and where the timing difference against C++
lives. Untested hypothesis: Rust's default `HashSet` uses SipHash-1-3 while C++
uses a trivial hash. See [`05-regression-baselines.md`](05-regression-baselines.md).

### [`occupancy.rs`](../crates/octomap-core/src/occupancy.rs) (1232)

| Item | What it is |
|---|---|
| `log_odds()`, `probability()` | The conversions, using `log`/`exp` — the path most dependent on `libm` |
| `OccupancyValue` | The value stored in a node |
| `SensorModel` | `prob_hit`, `prob_miss`, `occupancy_thres`, clamping, with validated setters |
| `OcTree` | The occupancy map |
| `max_child_log_odds`, `mean_child_log_odds` | Propagation to inner nodes |

The most-used parts of the `OcTree` API:

```rust
update_node(key, occupied)          update_node_at(point, occupied)
set_node_value(key, log_odds, lazy) update_node_log_odds(...)
is_occupied(key)                    is_occupied_at(point)
get_log_odds(key)                   get_log_odds_at(point)
get_occupancy(key)                  get_occupancy_at(point)
update_inner_occupancy()            to_max_likelihood()
prune()                             expand_to_depth(d)
iter_nodes()  iter_leaves()         iter_leaves_to_depth(d)
enable_change_detection(b)          changed_keys()
```

The `*_at` variants take a world coordinate and return an `Option` — **`None`
means unknown, not free**. Telling those apart is the point of an occupancy map,
and an API returning a plain `bool` would erase the distinction.

**`update_node` silently auto-prunes** on the way back up the recursion.
Structural fixtures must be generated with
`set_node_value(..., lazy_eval = true)` instead.

---

## 6.2 `octomap-ros` — 5 modules, 47 tests

There is not one ROS type in here. Everything takes raw data.

| Module | Public items | What it does |
|---|---|---|
| [`pointcloud2.rs`](../crates/octomap-ros/src/pointcloud2.rs) | `Cloud`, `FieldRef`, `PointIter`, `CloudError` | Decodes a `sensor_msgs/PointCloud2` blob: offsets are read from the field descriptors, not assumed |
| [`msg.rs`](../crates/octomap-ros/src/msg.rs) | `OctomapPayload`, `binary_payload`, `binary_payload_collapsed`, `full_payload`, `decode`, `decode_i8`, `to_i8`, `from_i8`, `TREE_ID` | The `octomap_msgs/Octomap` payload, both directions |
| [`filter.rs`](../crates/octomap-ros/src/filter.rs) | `ScanFilter`, `ScanStats` | `max_range`, `min_range`, `min_z`/`max_z`, `point_stride` |
| [`transform.rs`](../crates/octomap-ros/src/transform.rs) | `Transform3` | Pose transform composition and application |
| [`voxels.rs`](../crates/octomap-ros/src/voxels.rs) | `Voxel`, `occupied_voxels`, `free_voxels`, `voxels_by_depth`, `height_color` | Extracting cells as markers, coloured by height |

`to_i8` / `from_i8` exist because the `data` field in `octomap_msgs` is typed
`int8[]` while the serializer works in `u8`. It is a bit reinterpretation, not a
numeric conversion.

`ScanFilter` treats `max_range` and `min_range` **asymmetrically, on purpose**:
a point beyond `max_range` still clears space along its ray up to that distance,
while a point inside `min_range` is discarded entirely. Truncating a ray and
discarding a measurement are different operations, and conflating them leaves
unknown holes in places the sensor plainly saw.

---

## 6.3 `ros2/octomap_server_rs` — 4 modules

| Module | What it does |
|---|---|
| [`main.rs`](../ros2/octomap_server_rs/src/main.rs) | The node, subscriptions, the publish timer, the `reset` service |
| [`params.rs`](../ros2/octomap_server_rs/src/params.rs) | Parameter declaration and reading. Read **once at start** |
| [`publish.rs`](../ros2/octomap_server_rs/src/publish.rs) | `octomap_binary`, `octomap_full`, the marker array, cell centres |
| [`tf.rs`](../ros2/octomap_server_rs/src/tf.rs) | A per-edge transform cache |

Parameters are read once at start and are **not** reconfigured at runtime: the
map has already been built with the old resolution and sensor model, and
changing either midway corrupts it. Restart instead.

`tf.rs` **does not interpolate in time** — it keeps the latest transform per
edge and ignores the cloud stamp. On a fast robot, scans smear along the
trajectory. This is a consequence of not binding to `tf2`, which has no Rust
binding.

Publishing runs on a timer, independent of the scan rate: serializing a large
map at 30 Hz costs more than building it.

---

## 6.4 Benchmarks, examples and scripts

| File | What it is |
|---|---|
| [`benches/shared/fixture.rs`](../crates/octomap-core/benches/shared/fixture.rs) | The **only** benchmark scene generator: `generate`, `serialize`, `deserialize`, `load`, `write_fixture`. 50,176 points (a 224×224 grid), 0.1 m resolution, sensor at (0.05, 0.05, 0.05) |
| [`benches/insert_point_cloud.rs`](../crates/octomap-core/benches/insert_point_cloud.rs) | The criterion benchmark: three insertion modes, two query modes, ray casting |
| [`examples/insert_and_query.rs`](../crates/octomap-core/examples/insert_and_query.rs) | The runtime usage pattern, all three occupancy states |
| [`examples/save_load.rs`](../crates/octomap-core/examples/save_load.rs) | Both file formats, what each keeps, and why the write order matters |
| [`examples/ray_cast.rs`](../crates/octomap-core/examples/ray_cast.rs) | A hit, and the four distinct ways a cast can miss |
| [`examples/write_scene.rs`](../crates/octomap-core/examples/write_scene.rs) | Writes files for the reverse-direction interop check |
| [`examples/dump_bench_fixture.rs`](../crates/octomap-core/examples/dump_bench_fixture.rs) | Writes the shared scene for the C++ side |
| [`scripts/gen_golden_*.cpp`](../scripts/README.md) | Six fixture generators |
| [`scripts/verify_rust_io.cpp`](../scripts/verify_rust_io.cpp) | Hands Rust-written files to C++ |
| [`scripts/bench_cpp.cpp`](../scripts/bench_cpp.cpp) | The comparison benchmark on the C++ side |
| [`scripts/linux_verify.sh`](../scripts/linux_verify.sh) | Cross-platform verification |
| [`scripts/ros2/smoke_test.sh`](../scripts/ros2/smoke_test.sh) | Tests the node without hardware, decoding with C++ OctoMap |

`fixture.rs` is one generator read by **both** benchmarks — there is no second
one on the C++ side. Two generators that are merely *supposed* to agree are a
failure mode this project's audit has already caught three times.
