# 1. Architecture

Three layers, separated by **what is required to build them** rather than by
conceptual tidiness. That separation is what lets most of this project be tested
on any machine.

```text
┌──────────────────────────────────────────────────────────────┐
│ ros2/octomap_server_rs      needs ROS 2 sourced (r2r)        │
│   mapping node: topics, services, parameters, TF             │
├──────────────────────────────────────────────────────────────┤
│ crates/octomap-ros          NO ROS dependency                │
│   PointCloud2 in → octomap_msgs payload out, filter, markers │
├──────────────────────────────────────────────────────────────┤
│ crates/octomap-core         NO dependencies at all           │
│   Point3, Pose6, OcTreeKey, OctreeCore<T>, OcTree, ray, I/O  │
└──────────────────────────────────────────────────────────────┘
```

---

## 1.1 `octomap-core` — the port itself

Eleven modules, ~6,300 lines including in-module tests, **zero runtime
dependencies** — `std` only.

```text
error.rs       (67)    error types
lib.rs         (72)    re-exports and crate documentation
point.rs      (239)    Point3
key.rs        (302)    OcTreeKey, KeyRay, child helpers
node.rs       (400)    Node<T>, lazily allocated 8-child array
pose.rs       (505)    Quaternion, Pose6
geometry.rs   (515)    TreeGeometry, coord ↔ key conversions
tree.rs       (937)    OctreeCore<T>, three iterators
ray.rs       (1012)    PointCloud, DDA, ray casting, point-cloud integration
io.rs        (1023)    .bt / .ot, as files and as headerless payloads
occupancy.rs (1232)    SensorModel, OcTree
```

Zero dependencies is a constraint that is held, not a coincidence: every
consumer inherits whatever is added here. Criterion is present as a
*dev*-dependency for `cargo bench` only, and dev-dependencies are not propagated.

The generic/concrete split follows the C++ source. `OctreeCore<T>` is the tree —
insert, search, delete, expand, prune, iterate — and knows nothing about
occupancy. `OcTree` is the occupancy map built on top of it, carrying log-odds,
the sensor model and clamping.

## 1.2 `octomap-ros` — conversions, without ROS

Five modules: `pointcloud2.rs`, `msg.rs`, `filter.rs`, `transform.rs`,
`voxels.rs`.

This crate **never touches a generated message type**. It takes the raw data a
message carries — a byte blob, a list of field descriptors, a resolution — and
returns raw data too. Three things follow:

- It builds and passes its tests on a machine with no ROS installed, Windows
  included.
- Its logic is covered by ordinary unit tests that run under
  `cargo test --workspace`. **47 tests.**
- Which ROS client library you use stays your decision.

If you want an octree inside your own node, depend on `octomap-ros` and skip
`octomap_server_rs` entirely.

See [ADR-0009](decisions/0009-ros-split.md).

## 1.3 `ros2/octomap_server_rs` — the node

Its own Cargo workspace, **excluded from the root workspace**. Four modules:
`main.rs`, `params.rs`, `publish.rs`, `tf.rs`.

`r2r` builds its message bindings from `AMENT_PREFIX_PATH` at compile time, so
this crate only builds on a machine with ROS sourced. If it were in the root
workspace's `members`, `cargo test --workspace` would fail on every machine that
has Rust but not ROS — including the machine this port was developed on.

Its topics, services and parameter names match the C++ `octomap_server`, so it
can replace that node without changing anything downstream. Details in
[`07-ros2.md`](07-ros2.md).

---

## 1.4 The boundaries that are enforced

| Boundary | Rule | What breaks if it is crossed |
|---|---|---|
| `octomap-core` → the outside world | Zero runtime dependencies | Every consumer inherits it |
| `octomap-ros` → ROS | None permitted | The crate stops being testable on a machine without ROS |
| `octomap_server_rs` → root workspace | Stays `exclude`d | `cargo test --workspace` dies on any machine without ROS |
| Everything → the C++ reference | Identical behaviour, bit-exact | The differential tests fail, and that means the port is wrong |

## 1.5 Why there are two serialization shapes

`io.rs` offers two pairs of functions that are easy to confuse:

| Function | Produces |
|---|---|
| `write_binary_file` / `write_full_file` | The contents of a `.bt` / `.ot` file — **with the header** |
| `write_binary_data` / `write_full_data` | The node payload **without a header** |

The second is what the `data` field of an `octomap_msgs/Octomap` message
carries. Piping the contents of a `.bt` file into it produces a message nobody
can decode — RViz included — and the symptom does not point at the cause. The
`read_*` functions mirror both directions.

## 1.6 What `tests/golden/` and `scripts/` are for

Neither is part of the library, and both are part of its correctness claim.

- **`scripts/`** — seven C++ programs: six fixture generators, one interop
  verifier, one comparison benchmark. Plus `linux_verify.sh`.
- **`tests/golden/`** — 174 KB of answers produced by a C++ binary that actually
  ran. **Committed**, so that `cargo test` demonstrates equivalence with no C++
  toolchain at all.

The C++ source itself is **not vendored** — a script clones it and it is
gitignored. The reasoning is in
[ADR-0002](decisions/0002-commit-fixtures-not-reference.md).

The methodology, and the three bugs it caught, are in
[`03-verification.md`](03-verification.md).
