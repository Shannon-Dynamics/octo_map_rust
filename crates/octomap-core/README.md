# octomap-core

A Rust-native 3D occupancy mapping library: an octree of log-odds occupancy
values, ray traversal, point-cloud integration, and file compatibility with the
[OctoMap](https://octomap.github.io/) C++ library.

It is a **port, not a wrapper** — no C++ is compiled into the build and there is
no C++ runtime to install. The reference is OctoMap C++ 1.10.0 (commit
`f012f5f`), and behavioral equality with it is checked by a differential test
suite rather than assumed.

- Repository: <https://github.com/Shannon-Dynamics/octo_map_rust>
- Tutorial: <https://github.com/Shannon-Dynamics/octo_map_rust/blob/main/docs/TUTORIAL.md>
- Safety model: <https://github.com/Shannon-Dynamics/octo_map_rust/blob/main/SAFETY.md>

## Why this crate

- **No `unsafe`.** `unsafe_code` is `forbid`-ed for the whole workspace, so the
  compiler rejects it rather than a reviewer having to catch it. There is no FFI
  boundary and no raw pointer arithmetic anywhere in the tree.
- **No runtime dependencies.** `std` only. Nothing is inherited by a consumer
  beyond the standard library.
- **Unknown space is a distinct answer.** Occupancy queries return
  `Option<bool>`: `Some(true)` occupied, `Some(false)` free, `None` never
  observed. A ray-cast map cannot say anything about space its rays never
  reached, and flattening that into "free" is how a planner ends up driving
  through a wall.
- **Fallible construction is fallible in the type system.** A resolution that is
  not finite and positive, a depth beyond the tree, a coordinate outside the
  addressable volume — each is a `Result` or an `Option`, not a panic and not a
  silently clamped value.

## Installation

Not yet published to crates.io. Until it is, depend on it from git:

```toml
[dependencies]
octomap-core = { git = "https://github.com/Shannon-Dynamics/octo_map_rust" }
```

Once published, this becomes `octomap-core = "0.1"`.

Requires Rust 1.75 or newer.

## 60-second quick start

Integrate one scan from a known sensor position, then ask the map about three
places in the world:

```rust
use octomap_core::{OcTree, Point3, PointCloud};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 10 cm voxels. Fails only if the resolution is not finite and positive.
    let mut map = OcTree::new(0.1)?;

    let sensor = Point3::new(0.05, 0.05, 0.05);
    let scan: PointCloud = vec![
        Point3::new(1.05, 0.05, 0.05),
        Point3::new(0.05, 1.05, 0.05),
    ]
    .into_iter()
    .collect();

    // Each ray frees the space it crosses and marks its endpoint occupied.
    // -1.0 means "no range limit"; the two flags are lazy evaluation and
    // endpoint discretization, both off here.
    map.insert_point_cloud(&scan, sensor, -1.0, false, false);

    // Three states, and the third is the one that matters.
    assert_eq!(map.is_occupied_at(Point3::new(1.05, 0.05, 0.05)), Some(true));  // hit
    assert_eq!(map.is_occupied_at(Point3::new(0.55, 0.05, 0.05)), Some(false)); // passed through
    assert_eq!(map.is_occupied_at(Point3::new(5.05, 0.05, 0.05)), None);        // never seen

    Ok(())
}
```

`cargo run --example insert_and_query` is a worked, commented version of the
same thing.

## Core concepts

| Concept | Type | In one line |
|---|---|---|
| Resolution | `OcTree::new(res)` | Edge length of the smallest voxel, in metres. Fixed for the life of the map |
| Coordinate | `Point3` | A world position in metres, `f32` components |
| Key | `OcTreeKey` | The integer address of a voxel. Stable, hashable, cheaper to query than a coordinate |
| Occupancy | log-odds `f32` | Updated additively per observation, clamped at both ends so a voxel stays revisable |
| Sensor model | `SensorModel` | Hit / miss probabilities, clamping and the occupancy threshold |
| Scan | `PointCloud` | The endpoints of one sensor reading, in world coordinates |

Two things follow from the octree representation and surprise people otherwise:

- Uniform sibling blocks are **pruned** into their parent, so the number of
  nodes is not the number of observed voxels. Iterate leaves, not nodes, when
  you want cells.
- `insert_point_cloud(.., lazy_eval = true)` skips inner-node maintenance during
  insertion. Call `update_inner_occupancy()` before querying inner nodes or
  writing the map out.

## Saving and loading

`.bt` (binary, occupancy thresholded) and `.ot` (full, exact log-odds) are the
two OctoMap file formats. Both are written byte-identically to what the C++
reference produces, and both directions are covered by tests:

```rust
use octomap_core::{io, OcTree, Point3, PointCloud};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;
    let scan: PointCloud = vec![Point3::new(1.05, 0.05, 0.05)].into_iter().collect();
    map.insert_point_cloud(&scan, Point3::new(0.05, 0.05, 0.05), -1.0, false, false);

    let path = std::env::temp_dir().join("octomap-core-readme.bt");
    io::write_binary_file(&mut map, &path)?;
    let reloaded = io::read_binary_file(&path)?;

    assert_eq!(reloaded.resolution(), map.resolution());
    std::fs::remove_file(&path).ok();
    Ok(())
}
```

`write_binary_data` / `read_binary_data` are the **headerless** payloads that an
`octomap_msgs/Octomap` message carries. They are not the contents of a `.bt`
file — confusing the two produces a message nobody can decode.

## What is implemented

- `Point3`, `Pose6`, `Quaternion`
- `OcTreeKey`, `KeyRay`, `TreeGeometry` — every coordinate/key conversion
- `OctreeCore<T>` — a generic octree: insert, search, delete, expand, prune, and
  tree / leaf / depth-limited iterators
- `OcTree` — the occupancy map: log-odds updates, clamping, sensor model, lazy
  insertion, inner-node propagation, max-likelihood collapse, change detection,
  and world-coordinate queries that keep unknown distinct from free
- `PointCloud`, ray traversal, ray casting, point-cloud integration
- `.bt` and `.ot` reading and writing, as files and as headerless payloads

## Current limitations

- The public API is pre-1.0 and may still change.
- No derived tree types: `ColorOcTree`, `OcTreeStamped`, `CountingOcTree`,
  `ScanGraph`, `MapCollection`.
- No bounding-box-limited insertion (`setBBXMin` / `setBBXMax`).
- The legacy headerless `.bt` format is rejected with a clear error rather than
  guessed at. The reference reads it but tells you to convert those files.
- Single-threaded. The C++ reference has an OpenMP path on insertion; this port
  has no equivalent.
- Little-endian node values are written and read unconditionally, so big-endian
  platforms are unsupported.

## License

Apache-2.0. See `LICENSE`.

This is a port, not an independent implementation: OctoMap's original
BSD-3-Clause copyright is retained and reproduced in full in `NOTICE`, which
ships with this crate and must travel with any redistribution.
