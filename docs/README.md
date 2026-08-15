# Documentation

These documents describe the port as it stands: what was built, how its
correctness is demonstrated, what has been measured, and what is missing.

Reference: **OctoMap C++ 1.10.0**, commit
`f012f5f0a4f58cad19501833f9c0ea9d864427b6`.

New to the library? Start with [`TUTORIAL.md`](TUTORIAL.md) instead — it assumes
no prior knowledge of OctoMap. This index is for people working *on* the port.

---

## Reading order

| # | Document | For |
|---|---|---|
| 0 | [`TUTORIAL.md`](TUTORIAL.md) | Anyone using the library for the first time |
| 1 | [`01-architecture.md`](01-architecture.md) | The module map, the crate split, and why it is drawn there |
| 2 | [`02-tech-stack.md`](02-tech-stack.md) | Dependencies (there are none) and the toolchain |
| 3 | [`03-verification.md`](03-verification.md) | **The bit-exact differential method** — the core of this project |
| 4 | [`04-running.md`](04-running.md) | Every command, on both platforms |
| 5 | [`05-regression-baselines.md`](05-regression-baselines.md) | Internal timing baselines and their methodology. A regression tool, not a claim |
| 6 | [`06-code-tour.md`](06-code-tour.md) | What each module actually does |
| 7 | [`07-ros2.md`](07-ros2.md) | ROS 2 reference: parameters, RViz, tuning |

## Reference material

| File / folder | Contents |
|---|---|
| [`../SAFETY.md`](../SAFETY.md) | The memory-safety model: unsafe policy, ownership, input boundaries, what is not claimed |
| [`../ROADMAP.md`](../ROADMAP.md) | Five phases, no dates |
| [`RELEASING.md`](RELEASING.md) | The release process, step by step |
| [`RELEASE_NOTES_TEMPLATE.md`](RELEASE_NOTES_TEMPLATE.md) | The shape of a release note |
| [`reference-audit.md`](reference-audit.md) | The C++ source audit the port was written from — every constant and formula with file and line references |
| [`decisions/`](decisions/README.md) | 11 ADRs, including the 7 deliberate divergences that look like bugs without an explanation |
| [`runbooks/`](runbooks/README.md) | Regenerating fixtures, benchmarking, cross-platform verification, the ROS 2 node, troubleshooting |
| [`../benchmarks/`](../benchmarks/README.md) | Raw logs from benchmark and verification runs |

---

## One screen

A pure-Rust port of OctoMap, the probabilistic 3D occupancy mapping library.
**Not a wrapper**: no C++ in the build, no C++ runtime to install.

Three properties define the shape of this project:

- **The goal is behavioural equality, not a cleaner design.** Where the two
  conflict, the reference wins and the divergence is recorded as an ADR. There
  are seven.
- **Correctness is measured, not assumed.** The C++ reference is built, driven
  with the same inputs, and its answers are kept as fixtures. Floating point is
  compared as **raw IEEE-754 bit patterns**, not with a tolerance.
- **`octomap-core` has no runtime dependencies at all** — `std` only. Every
  consumer inherits whatever is added here, so nothing is.

## Key numbers

| | |
|---|---|
| Tests passing | **284** (206 unit + 63 differential + 12 robustness + 3 doc-tests) |
| Verified platforms | Windows x86-64 **and** Linux x86-64 |
| Clippy / `fmt` / `cargo doc` | 0 warnings |
| `unsafe` | 0 — `forbid`-ed at the workspace level |
| Implementation | ~6,300 lines of Rust across 11 modules, plus 5 in `octomap-ros` |
| Golden fixtures | 174 KB, from a C++ binary that actually ran |
| Runtime dependencies | **none** |

## Phase status

| Phase | Contents | Status |
|---|---|---|
| 1 | C++ source audit | ✅ Done |
| 2 | Geometry & keys | ✅ Done |
| 3 | Generic octree core | ✅ Done |
| 4 | Occupancy model | ✅ Done |
| 5 | Rays & point clouds | ✅ Done |
| 6 | `.bt` / `.ot` serialization | ✅ Done |
| P1 | `Pose6` & `Quaternion` | ✅ Done |
| 7 | Simulation readiness | ✅ Done |
| — | ROS 2 integration | ✅ Done |
| — | Open-source release preparation | ✅ Done — see [`../ROADMAP.md`](../ROADMAP.md) Phase 1 |
| — | crates.io publication | ⬜ Not yet |
| — | CLI, Python / C bindings | ⬜ Not planned for now |

What is not implemented, and why, is in [`../ROADMAP.md`](../ROADMAP.md).
