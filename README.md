# octo_map_rust

[![CI](https://github.com/Shannon-Dynamics/octo_map_rust/actions/workflows/ci.yml/badge.svg)](https://github.com/Shannon-Dynamics/octo_map_rust/actions/workflows/ci.yml)
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![MSRV](https://img.shields.io/badge/rustc-1.75%2B-orange.svg)](#installation)

A Rust-native 3D occupancy mapping library focused on a clear API, memory-safe
abstractions, and practical robotics and spatial-mapping workflows.

It is a **port of [OctoMap](https://octomap.github.io/) C++ 1.10.0, not a
wrapper**: no C++ is compiled into the build, and there is no C++ runtime to
install. Files it writes are byte-identical to the reference's, so it drops into
an existing OctoMap toolchain.

```rust
use octomap_core::{OcTree, Point3, PointCloud};

let mut map = OcTree::new(0.1)?;                       // 10 cm voxels
let sensor = Point3::new(0.05, 0.05, 0.05);
let scan: PointCloud = vec![Point3::new(1.05, 0.05, 0.05)].into_iter().collect();

map.insert_point_cloud(&scan, sensor, -1.0, false, false);

assert_eq!(map.is_occupied_at(Point3::new(1.05, 0.05, 0.05)), Some(true));   // occupied
assert_eq!(map.is_occupied_at(Point3::new(0.55, 0.05, 0.05)), Some(false));  // free
assert_eq!(map.is_occupied_at(Point3::new(5.05, 0.05, 0.05)), None);         // unknown
```

| | |
|---|---|
| 🚀 Start here | [60-second quick start](#60-second-quick-start) |
| 📚 Step-by-step tutorial | [`docs/TUTORIAL.md`](docs/TUTORIAL.md) |
| 🛡️ Safety model | [`SAFETY.md`](SAFETY.md) |
| 🔬 How correctness is measured | [`docs/03-verification.md`](docs/03-verification.md) |
| 🧭 Where to start reading the code | [`docs/README.md`](docs/README.md) |

---

## Why This Project Exists

Occupancy mapping is used by software that moves machines through space. The
established implementation is a C++ library, and using it from Rust means an FFI
boundary: raw pointers across the wall, ownership rules the compiler cannot see,
and a C++ toolchain in every build.

This project removes that boundary rather than wrapping it. The algorithms are
reimplemented in Rust, so ownership, lifetimes and bounds checking apply to the
whole map — including the parts that decode files and messages produced
elsewhere, which is where a mapping stack is most exposed to input it did not
create.

The port keeps the reference's **observable behaviour**, not its design. Where
those two conflict, the reference wins and the divergence is recorded as an
[ADR](docs/decisions/README.md). Seven such places exist; each is commented at
the point of definition, because without an explanation they read as bugs.

## Key Features

- **Octree occupancy map** — log-odds updates, clamping, a configurable sensor
  model, lazy insertion, inner-node propagation, max-likelihood collapse, and
  change detection.
- **Three-state queries** — occupied, free, and *never observed*, kept distinct
  in the type system as `Option<bool>`.
- **Rays and point clouds** — DDA ray traversal, ray casting, and scan
  integration with an optional discretization pass.
- **OctoMap file compatibility** — `.bt` and `.ot`, read and written
  byte-identically to the C++ reference, plus the headerless payloads that
  `octomap_msgs` carries.
- **A generic octree underneath** — `OctreeCore<T>` is not tied to occupancy;
  the occupancy map is one instantiation of it.
- **ROS 2 conversions that do not depend on ROS** — see [ROS 2](#ros-2).
- **Zero runtime dependencies** — `std` only.

## Memory Safety

Memory safety is the reason this library exists in Rust rather than as a binding,
so it is worth being precise about what is and is not claimed.

**What is enforced by the compiler:**

- `unsafe_code = "forbid"` is set at the workspace level. Not "we avoid unsafe" —
  the compiler rejects an `unsafe` block anywhere in this repository.
- No FFI, no `extern` blocks, no raw pointers, no transmutes. There is no
  C++ library in the build, so there is no boundary where ownership stops being
  checked.
- Every dependency is `std`. `octomap-core` has no runtime dependencies at all,
  and `octomap-ros` depends only on `octomap-core`, so there is no third-party
  `unsafe` inherited through the dependency tree either. (`std` itself contains
  `unsafe` internally, as every Rust program's does.)

**What the API design contributes:**

- **Ownership is explicit.** A map owns its nodes; queries borrow. Iterators
  borrow the tree, so it cannot be mutated while being walked.
- **Invalid states are rejected at construction.** `OcTree::new` returns a
  `Result`; a non-finite or non-positive resolution never becomes a live map.
- **Untrusted input has a total decoder.** `.bt` / `.ot` parsing and
  `PointCloud2` decoding are the paths that read bytes somebody else produced.
  Every offset is bounds-checked, and malformed input becomes a typed error, not
  an out-of-range read.
- **No silent clamping of out-of-range coordinates.** A world coordinate outside
  the addressable volume is `None` or an error, not a wrapped key pointing at an
  unrelated voxel.

**What is *not* claimed:** this does not make an application built on it correct,
and it does not extend to any other crate a downstream binary links. A robot
built on this library can still drive into a wall for reasons that have nothing
to do with memory safety. The claim is bounded and it is this: **this repository
contains no `unsafe` code, and no way to reach undefined behaviour through its
public API.**

The full policy — invariants, panic expectations, malformed-input handling, what
would have to happen for `unsafe` to be accepted — is in [`SAFETY.md`](SAFETY.md).

## Installation

Requires **Rust 1.75 or newer**.

### Current development usage

The crates are **not yet published to crates.io**, so `cargo add octomap-core`
will not find them. Depend on the repository directly:

```toml
[dependencies]
octomap-core = { git = "https://github.com/Shannon-Dynamics/octo_map_rust" }

# Only if you are moving data in and out of ROS 2 messages:
octomap-ros = { git = "https://github.com/Shannon-Dynamics/octo_map_rust" }
```

Or work in a clone:

```bash
git clone https://github.com/Shannon-Dynamics/octo_map_rust
cd octo_map_rust
cargo test --workspace
cargo run --example insert_and_query
```

### crates.io usage — after the first release

```toml
[dependencies]
octomap-core = "0.1"
```

Publishing is tracked as Phase 2 in the [roadmap](ROADMAP.md); the process is
written down in [`docs/RELEASING.md`](docs/RELEASING.md).

## 60-Second Quick Start

A sensor at the origin sees a wall two metres away. Build the map, then ask it
about three places:

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
    // -1.0 means "no range limit"; the flags are lazy evaluation and endpoint
    // discretization, both off here.
    map.insert_point_cloud(&scan, sensor, -1.0, false, false);

    // Three states, and the third is the one that matters: space no ray
    // reached is unknown, not free.
    assert_eq!(map.is_occupied_at(Point3::new(1.05, 0.05, 0.05)), Some(true));
    assert_eq!(map.is_occupied_at(Point3::new(0.55, 0.05, 0.05)), Some(false));
    assert_eq!(map.is_occupied_at(Point3::new(5.05, 0.05, 0.05)), None);

    Ok(())
}
```

Run it as `cargo run --example insert_and_query`, which is the same scene with
commentary and printed output. This code is compiled and its assertions checked
on every CI run: it is the doc-test carried by
[`crates/octomap-core/README.md`](crates/octomap-core/README.md), which is also
the crate's rustdoc front page.

Next step: [`docs/TUTORIAL.md`](docs/TUTORIAL.md).

## Core Concepts

| Concept | Type | In one line |
|---|---|---|
| Resolution | argument to `OcTree::new` | Edge length of the smallest voxel, in metres. Fixed for the life of the map |
| Coordinate | `Point3` | A world position in metres |
| Key | `OcTreeKey` | The integer address of a voxel — stable, hashable, and cheaper to query than a coordinate |
| Occupancy | log-odds `f32` | Updated additively per observation and clamped at both ends, so a voxel stays revisable |
| Sensor model | `SensorModel` | Hit / miss probabilities, clamping bounds, and the occupancy threshold |
| Scan | `PointCloud` | The endpoints of one sensor reading, in world coordinates |
| Sensor pose | `Pose6`, `Quaternion` | Position and orientation, for transforming a scan into the map frame |

Two consequences of the octree representation that surprise people otherwise:

- Uniform sibling blocks are **pruned** into their parent, so the node count is
  not the voxel count. Iterate leaves when you want cells.
- `lazy_eval = true` skips inner-node maintenance during insertion. Call
  `update_inner_occupancy()` before querying inner nodes or writing the map out.

## Examples

All under [`crates/octomap-core/examples/`](crates/octomap-core/examples), all
run with `cargo run --example <name>`:

| Example | What it shows |
|---|---|
| `insert_and_query` | The runtime pattern: integrate a scan, query the three occupancy states |
| `save_load` | Writing `.bt` and `.ot`, reading them back, and what each format keeps |
| `ray_cast` | Casting a ray into the map and telling a hit from the three ways a cast can miss |
| `write_scene` | Writes the scene used by the cross-language file check |
| `dump_bench_fixture` | Regenerates the shared benchmark scene from a fixed seed |

## Tutorial

[`docs/TUTORIAL.md`](docs/TUTORIAL.md) goes from an empty project to a saved map
in nine parts: installation, creating a map, resolution and coordinates,
inserting data, querying, iterating, saving and loading, point clouds, and the
Candi scanning application built on this library.

It assumes no prior knowledge of OctoMap.

## Current Capabilities

- `Point3`, `Pose6`, `Quaternion`
- `OcTreeKey`, `KeyRay`, `TreeGeometry` — all coordinate/key conversions
- `OctreeCore<T>` — generic octree: insert, search, delete, expand, prune, and
  tree / leaf / depth-limited iterators
- `OcTree` — the occupancy map: log-odds updates, clamping, sensor model, lazy
  insertion, inner-node propagation, max-likelihood collapse, change detection,
  and world-coordinate queries (`is_occupied_at`, `get_log_odds_at`,
  `get_occupancy_at`) that keep unknown distinct from free
- `PointCloud`, ray traversal, ray casting, point-cloud integration
- `.bt` and `.ot` reading and writing, as files and as headerless payloads
- ROS 2: `sensor_msgs/PointCloud2` decoding, `octomap_msgs/Octomap` payloads,
  and a mapping node — see [`docs/07-ros2.md`](docs/07-ros2.md)

## Current Limitations

Stated plainly, because an early open-source project that hides these is harder
to trust than one that does not:

- **The public API is pre-1.0 and still evolving.** Breaking changes are
  possible before 1.0; each one will be in the [changelog](CHANGELOG.md).
- **Not published to crates.io yet** — see [Installation](#installation).
- **Not all OctoMap features are implemented.** Missing: `ColorOcTree`,
  `OcTreeStamped`, `CountingOcTree`, `ScanGraph`, `MapCollection`;
  bounding-box-limited insertion (`setBBXMin` / `setBBXMax`);
  `getRayIntersection`.
- **The legacy headerless `.bt` format is rejected** with a clear error rather
  than guessed at. The reference still reads it, but tells you to convert those
  files.
- **Single-threaded.** The reference has an OpenMP path on insertion and this
  port has no equivalent.
- **No CLI, no Python or C bindings.**
- **Little-endian only.** Node values are read and written little-endian
  unconditionally, matching every OctoMap file in circulation, so big-endian
  platforms are unsupported.
- **Verified on Windows and Linux x86-64 only.** No evidence yet from ARM64 or
  macOS, which matters more than usual here — see
  [Correctness](#correctness).

Each of these is tracked in the [roadmap](ROADMAP.md).

## Correctness

Behavioural compatibility with the C++ library is the whole point of a port, so
it is **measured rather than assumed**. The C++ reference is built, driven
through the same inputs, and its answers are captured as fixtures under
`tests/golden/`.

**284 tests** run in the workspace — 206 unit, 63 differential against those
fixtures, 12 parser-robustness, 3 doc-tests —
comparing floating point as **raw IEEE-754 bit patterns** rather than with a
tolerance. Both implementations perform the same operations in the same order,
so anything less than bit equality is a real divergence, and a tolerance would
hide exactly the class of bug these tests exist to find. Three real bugs were
caught this way; all three would have passed any reasonable epsilon.

| Area | What is pinned against C++ |
|---|---|
| Geometry & keys | 938 rows: coordinate ↔ key, depth variants, bounds checks, node sizes |
| Octree structure | Node counts, leaf and tree iteration order with keys and depths, prune, delete, depth-limited views |
| Occupancy | 43 sequential updates bit-for-bit, clamping, auto-prune, block reopen, change detection |
| Rays | DDA key sequences for 12 ray shapes (one spanning 653 voxels), 8 ray-cast cases, point-cloud integration |
| Pose | Euler ↔ quaternion, rotation, axis-angle, pose transform / inverse / composition |
| File I/O | Byte-identical `.ot` and `.bt` output, plus decoding files the reference wrote |

File interoperability is verified **in both directions**. Rust reads
C++-written `.bt` and `.ot` files, and Rust's own output is byte-for-byte
identical to what the reference produces — a stronger statement than "C++ can
parse it", and one that needs no C++ toolchain at `cargo test` time.
`scripts/verify_rust_io.cpp` closes the loop by handing the reference Rust's
files directly.

The suite passes on **Windows x86-64 and Linux x86-64**. That matters more than
usual here: bit-exact comparison puts the port at the mercy of `libm`, since
`log`/`exp`/`atan2` are not guaranteed identical across implementations. They
turned out to agree — see [`docs/03-verification.md`](docs/03-verification.md),
and `scripts/linux_verify.sh` to reproduce.

## Deliberate Divergences

Seven places where matching C++ exactly was chosen over what a fresh design
would do. Each is commented at the point of definition and recorded as an ADR in
[`docs/decisions/`](docs/decisions/README.md) — without an explanation they look
like bugs:

| | |
|---|---|
| [ADR-0003](docs/decisions/0003-prune-stops-early.md) | `prune()` stops at the first level that merges nothing. The reference carries a `FIXME` about it |
| [ADR-0004](docs/decisions/0004-raykeys-castray-narrowing.md) | `computeRayKeys` narrows the voxel-border offset to `float`, `castRay` keeps it `double`. Both spellings reproduced |
| [ADR-0005](docs/decisions/0005-reciprocal-multiply.md) | Scaling multiplies by `1.0 / resolution`. Dividing moves points across voxel boundaries |
| [ADR-0006](docs/decisions/0006-quaternion-norm-f32.md) | `Quaternion::norm` squares in `f32`, then accumulates in `f64`. One ULP, propagated through every composed pose |
| [ADR-0007](docs/decisions/0007-resolution-six-digits.md) | Resolution written at six significant digits, matching C++ stream defaults |
| [ADR-0008](docs/decisions/0008-stricter-than-reference.md) | Two places stricter than the reference — both to avoid undefined behaviour, not to change semantics |
| [ADR-0011](docs/decisions/0011-pose6-stores-quaternion.md) | `Pose6` stores a quaternion, not Euler angles — following the C++ source over the plan |

## Architecture

```
octo_map_rust/
├─ crates/
│  ├─ octomap-core/     # The library. Zero runtime dependencies — std only
│  │  ├─ src/           # 10 modules, ~6,100 lines
│  │  ├─ tests/         # 6 differential suites against the C++ fixtures
│  │  ├─ benches/       # criterion regression baselines + the scene generator
│  │  └─ examples/
│  └─ octomap-ros/      # ROS 2 conversions — no ROS dependency, builds anywhere
├─ ros2/octomap_server_rs/   # A mapping node on r2r. Its own cargo workspace
├─ scripts/             # 7 C++ programs: fixture generators, verifier, benchmark
├─ tests/golden/        # 174 KB of fixtures produced by a real C++ binary
├─ benchmarks/          # Raw logs from internal regression runs
└─ docs/                # Architecture, verification, tutorial, ADRs, runbooks
```

Why the split: [`docs/01-architecture.md`](docs/01-architecture.md) and
[ADR-0009](docs/decisions/0009-ros-split.md).

## Use Cases

What the library provides, and what could be built on it. The distinction
matters — none of the applications below ship in this repository:

| Library capability | Potential downstream use case |
|---|---|
| Occupied / free / unknown representation of a volume | **Robotics** — an environment model a robot can reason about, with unknown space distinguishable from empty space |
| Point-cloud integration from a sensor origin | **LiDAR and depth-sensor mapping** — turning spatial observations into a structured 3D representation |
| Occupancy queries by coordinate and by key | **Navigation and planning** — a collision check a planner can consume. *This library does not include a planner* |
| Byte-identical `.bt` / `.ot` files | **Inspection robotics** — feeding maps into existing OctoMap tooling and viewers |
| Deterministic voxelization of scans | **Digital reconstruction** — structured occupancy from spatial scans |
| All of the above, applied | **Cultural heritage scanning** — [`scan_candi_with_octomap_rust`](https://github.com/Shannon-Dynamics/scan_candi_with_octomap_rust) is the concrete reference implementation |

SLAM, path planning, sensor drivers and localization are **not** implemented
here. This library is the map, not the stack around it.

## ROS 2

The map can be built from live sensor data and published to the rest of a ROS 2
graph. Two pieces, split so that only one of them needs ROS:

- **`crates/octomap-ros`** — the conversions. It takes the plain data a message
  carries rather than a generated message type, so it has **no ROS dependency**,
  builds anywhere, and leaves the client library up to the caller.
- **`ros2/octomap_server_rs`** — a mapping node on top of it, using `r2r`. Same
  topics, services and parameters as the C++ `octomap_server`.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2/octomap_server_rs && cargo build --release
./target/release/octomap_server_rs --ros-args -r cloud_in:=/camera/depth/points
```

`scripts/ros2/smoke_test.sh` checks the whole path without hardware and decodes
what the node publishes with the **C++** OctoMap library — the interoperability
claim is measured, not assumed.

[`docs/07-ros2.md`](docs/07-ros2.md) covers parameters, the colcon build, RViz,
tuning and troubleshooting.

## Roadmap

[`ROADMAP.md`](ROADMAP.md) — five phases, no dates. In short: stabilize the API
and the safety policy (Phase 1), publish to crates.io (Phase 2), fill in the
mapping workflow gaps (Phase 3), add optional ecosystem integrations behind
Cargo features (Phase 4), then stabilize for 1.0 (Phase 5).

## Documentation

| | |
|---|---|
| [`docs/TUTORIAL.md`](docs/TUTORIAL.md) | Step-by-step introduction, no prior OctoMap knowledge assumed |
| [`docs/README.md`](docs/README.md) | Index of the deep documentation |
| [`docs/01-architecture.md`](docs/01-architecture.md) | Module map and why the crates are split this way |
| [`docs/02-tech-stack.md`](docs/02-tech-stack.md) | Dependencies (there are none) and toolchain |
| [`docs/03-verification.md`](docs/03-verification.md) | The differential method, and the three bugs it caught |
| [`docs/04-running.md`](docs/04-running.md) | Every command, on both platforms |
| [`docs/05-regression-baselines.md`](docs/05-regression-baselines.md) | Internal timing baselines and their methodology — a regression tool, not a claim |
| [`docs/06-code-tour.md`](docs/06-code-tour.md) | What each module does |
| [`docs/07-ros2.md`](docs/07-ros2.md) | ROS 2 reference |
| [`docs/reference-audit.md`](docs/reference-audit.md) | The C++ source audit the port was written from |
| [`docs/decisions/`](docs/decisions/README.md) | 11 ADRs |
| [`docs/runbooks/`](docs/runbooks/README.md) | Fixtures, benchmarks, cross-platform, ROS 2 |
| [`docs/RELEASING.md`](docs/RELEASING.md) | The release process |
| [`CHANGELOG.md`](CHANGELOG.md) | Project-level changes; each crate has its own |

## Contributing

Contributions are welcome. [`CONTRIBUTING.md`](CONTRIBUTING.md) covers the build
and test commands, the rules that make this a port rather than a rewrite, and
which of the two repositories a given change belongs in.

## Security

The project aims to reduce memory-safety risks through safe Rust abstractions,
explicit input validation, documented boundaries and continuous dependency
checks. It does not claim to be free of vulnerabilities.

What is checked, and how:

| | |
|---|---|
| `unsafe` | Forbidden by the compiler at the workspace level; none in the repository |
| Undefined behaviour | Both crates' unit suites and the `PointCloud2` robustness tests run under Miri with strict provenance |
| Malformed input | Property tests: every truncation of a valid map file, thousands of byte corruptions, and randomized `PointCloud2` geometry, all asserting an error rather than a panic |
| Integer overflow | Checked arithmetic on every message-supplied length and offset |
| Dependencies | `cargo audit` in CI; `cargo deny` configuration in [`deny.toml`](deny.toml) |
| Workflows | Least-privilege permissions, third-party actions pinned to commit SHAs |

Reporting a vulnerability: [`SECURITY.md`](SECURITY.md). The trust boundaries,
the invariants and what is explicitly *not* claimed are in
[`SAFETY.md`](SAFETY.md).

## License

**Apache-2.0** — see [`LICENSE`](LICENSE).

This is a port, not an independent implementation. The Apache-2.0 terms cover
this port's own source code; OctoMap's original copyright and its
**BSD-3-Clause** conditions are retained and reproduced in full in
[`NOTICE`](NOTICE), as BSD-3-Clause clauses 1 and 2 require — and Apache-2.0
§4(d) independently requires that NOTICE travel with any redistribution.
**Redistribute both files together.** Copies of both ship inside each published
crate for the same reason.

OctoMap is copyright (c) 2009-2013 K. M. Wurm and A. Hornung, University of
Freiburg. This project is not endorsed by or affiliated with them — report bugs
here, not upstream.
