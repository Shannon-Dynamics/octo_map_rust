# Roadmap

Phases, not dates. Each one is a state the project can sit in indefinitely
without being half-finished, and each is ordered by what the next one needs.

Current version: **0.1.0**, unpublished. The public API is pre-1.0 and may still
change; see [Phase 5](#phase-5--api-stabilization) for what would end that.

Nothing below promises a feature. Items marked *not started* have no
implementation behind them, and the ones that do exist are named as such.

---

## Phase 1 — Foundation

**Goal: the repository is safe to depend on from git and to contribute to.**

| | Item |
|---|---|
| ✅ | Core API implemented — octree, occupancy, rays, point clouds, file I/O |
| ✅ | Differential test suite against OctoMap C++ 1.10.0, bit-exact |
| ✅ | Memory-safety policy written down and enforced by the compiler ([`SAFETY.md`](SAFETY.md)) |
| ✅ | Error model: `OctomapError` and `IoError`, no panics from library input |
| ✅ | Public API documented, `cargo doc` clean with warnings denied |
| ✅ | Working examples, each runnable with `cargo run --example` |
| ✅ | Cargo metadata complete, `LICENSE` and `NOTICE` shipped inside each crate |
| ✅ | CI on Linux and Windows: format, clippy, tests, docs, MSRV, packaging |
| ✅ | Contributor documentation, security policy, per-crate changelogs |
| ✅ | Parser robustness tests: truncation, corruption and randomized message geometry |
| ✅ | Undefined-behaviour check under Miri with strict provenance |
| ✅ | Supply-chain checks: `cargo audit` in CI, `cargo deny` configuration |
| ⬜ | CI observed green on a real push — the workflow exists but has not run yet |
| ⬜ | Miri in CI, on a schedule rather than per push — it takes minutes |

## Phase 2 — First public crate release

**Goal: `cargo add octomap-core` works.**

| | Item |
|---|---|
| ✅ | `cargo package` and `cargo publish --dry-run` clean for `octomap-core`; `octomap-ros` verified with `cargo package --list`, since packaging it cannot resolve `octomap-core` from the registry until that crate is published |
| ✅ | Release process written down ([`docs/RELEASING.md`](docs/RELEASING.md)) |
| ✅ | Quick start and tutorial that a newcomer can follow without prior OctoMap knowledge |
| ⬜ | Crate names claimed on crates.io. A search currently returns nothing for either name, so they appear free |
| ⬜ | 0.1.0 published, `octomap-core` first, then `octomap-ros` |
| ⬜ | docs.rs build verified after publication |
| ⬜ | Compatibility policy stated: what 0.x means here, and what would be a breaking change |
| ⬜ | crates.io and docs.rs badges added to the README, once they point at something |

## Phase 3 — Mapping workflow

**Goal: close the gaps between this port and the reference that a real mapping
job actually hits.** Ordered by how often they come up.

| | Item |
|---|---|
| ⬜ | **Bounding-box-limited insertion** — `setBBXMin` / `setBBXMax` and the `inBBX` branch in `computeUpdate`. `compute_update` is already structured to take it, and it is what blocks `BoundingBoxQuery` (`~/clear_bbx`) in the ROS 2 node |
| ⬜ | **`ColorOcTree`** — the most-requested derived tree type. Payloads with `id: "ColorOcTree"` are currently rejected by name rather than silently decoded as plain occupancy, which is the honest failure but not a useful one |
| ⬜ | **`getRayIntersection`** — ray/voxel-plane intersection, for callers that need the surface point rather than the voxel centre |
| ⬜ | **Unknown-space queries** — `getUnknownLeafCenters` and the volume queries around it |
| ⬜ | **Input validation review** — a pass over every public entry point asking what a hostile file or an adversarial scan does to it, feeding [`SECURITY.md`](SECURITY.md) |
| ⬜ | Other derived trees: `OcTreeStamped`, `CountingOcTree`, `ScanGraph`, `MapCollection` |

## Phase 4 — Ecosystem integration

**Goal: fit into the Rust robotics ecosystem without dragging it into the core
crate.** Everything here is a candidate for an **optional Cargo feature**, off
by default. `octomap-core`'s zero-dependency property is worth more than any
single integration, and a core crate that depends on a robotics framework is
one that cannot be used outside it.

| | Item |
|---|---|
| ⬜ | `serde` behind a feature, for `Point3`, `OcTreeKey`, `SensorModel` and map snapshots |
| ⬜ | Conversions for common Rust math types (`glam`, `nalgebra`) behind features — traits and `From` impls only, never a dependency in the default build |
| ⬜ | Point-cloud ecosystem interop: reading the formats Rust point-cloud crates emit |
| ⬜ | A visualization adapter, so a map can be handed to a viewer without hand-writing the marker conversion. The `voxels` module in `octomap-ros` is the shape this would take |
| ⬜ | ROS 2 beyond the current node: `rclrs` support alongside `r2r`, once `rclrs` stabilizes |
| ⬜ | Parallel insertion behind a feature — the reference has an OpenMP path and this port has no equivalent. A feature, because the dependency (`rayon`) would otherwise be inherited by everyone |

## Phase 5 — API stabilization

**Goal: 1.0, meaning the API stops moving.**

| | Item |
|---|---|
| ⬜ | Public API review — every `pub` item justified, accidental surface removed. `octomap-core` currently re-exports broadly; some of that is convenience and some is leakage |
| ⬜ | Ergonomics pass on the argument-heavy entry points. `insert_point_cloud(&scan, origin, max_range, lazy_eval, discretize)` mirrors the C++ signature, which is right for a port and wrong for a Rust API a newcomer meets first. If this changes, the C++-shaped call stays available and the new one is additive |
| ⬜ | Error stability: which variants are exhaustive, which are `#[non_exhaustive]`, and what a caller may match on |
| ⬜ | MSRV policy stated explicitly — currently 1.75, but "how far back, and what bumps it" is not written down |
| ⬜ | Platform coverage: ARM64 and macOS evidence. Bit-exact comparison depends on `libm` agreeing, and that has only been observed on x86-64 Windows and Linux |
| ⬜ | Decide the big-endian question. Node values are little-endian unconditionally today; either support it or state the exclusion in the manifest |
| ⬜ | 1.0 with a documented compatibility promise |

---

## Not on the roadmap

Named because their absence is a design choice, not an oversight:

- **SLAM, localization, path planning, sensor drivers.** This is the map, not
  the stack around it. A planner belongs in a crate that depends on this one.
- **Beating the C++ implementation on speed.** Timing baselines exist under
  [`benchmarks/`](benchmarks/README.md) as a regression tool, and they are not a
  goal of the project.
- **Vendoring the C++ source.** The reference is cloned by a script to generate
  fixtures and is never part of the build —
  [ADR-0002](docs/decisions/0002-commit-fixtures-not-reference.md).
