# 2. Tech stack

The most important part of this page is the column that is empty:
**`octomap-core` has no runtime dependencies at all.**

---

## 2.1 Runtime

| Layer | Technology | Note |
|---|---|---|
| Language | **Rust**, edition 2021, MSRV **1.75** | Developed on `x86_64-pc-windows-gnu` with 1.97.1 |
| `octomap-core` | `std` only | **Zero dependencies** |
| `octomap-ros` | `octomap-core` | No ROS — it takes the raw data a message carries, not a generated type |
| `ros2/octomap_server_rs` | `r2r` (ROS 2 Jazzy), `tokio`, `futures` | Separate workspace; message bindings are generated from `AMENT_PREFIX_PATH` at compile time |
| Dev-dependency | `criterion` | `cargo bench` only, not propagated to users of the crate |

The GNU target was chosen over MSVC because the development machine had g++ from
MSYS2 and no Visual Studio — **and** because the same toolchain later built the
C++ reference, so both sides of the comparison share an environment.

## 2.2 Lints pinned at the workspace level

```toml
[workspace.lints.rust]
missing_docs = "warn"
unsafe_code = "forbid"

[workspace.lints.clippy]
all = { level = "warn", priority = -1 }
```

`unsafe_code = "forbid"` is not a style preference. This port reproduces C++
pointer arithmetic with `Option<Box<[Option<Node<T>>; 8]>>` and typed indices;
if `unsafe` were permitted, a literal translation of the C++ would be the path
of least resistance and most of Rust's guarantees would go with it. `forbid`
rather than `deny` means no `#[allow]` further down the tree can re-enable it.

`missing_docs = "warn"` is held at zero warnings, so the whole public API is
documented. The full safety policy is in [`../SAFETY.md`](../SAFETY.md).

## 2.3 Build profiles

```toml
[profile.bench]
lto = true
codegen-units = 1
```

This is a condition for a re-measurement to mean anything. On the C++ side the
whole OctoMap implementation is header templates landing in one translation
unit, so `-O3` inlines freely across what is a crate boundary in Rust. Without
LTO a run is dominated by missing inlining rather than by anything in the code,
which makes it useless as a regression signal.

**For users of the crate:** this profile applies to `cargo bench` in this
repository only. An application that depends on `octomap-core` and builds with
plain `--release` gets meaningfully slower insertion unless it enables LTO
itself. See [ADR-0010](decisions/0010-lto-in-bench-profile.md).

## 2.4 Verification toolchain (optional)

None of this is needed to **use** the crate. It exists only to verify or
re-measure it.

| Tool | Version | Role |
|---|---|---|
| g++ | 10.3.0 (MSYS2 MinGW-w64) | Builds the C++ reference and the seven programs in `scripts/` |
| CMake + Ninja | 3.31.3 | Builds `liboctomap.a` / `liboctomath.a` |
| OctoMap C++ | 1.10.0, commit `f012f5f` | The reference. Cloned, **not vendored** |
| ROS 2 | Jazzy | For `ros2/octomap_server_rs` and the smoke test only |
| `octomap_msgs` (ROS) | Jazzy | The message types the node publishes |
| `octomap` (ROS, C++) | Jazzy | **Only** for the smoke test — decodes the node's output with the reference library |

`cargo test --workspace` deliberately requires **none** of the above, because the
generated fixtures are committed
([ADR-0002](decisions/0002-commit-fixtures-not-reference.md)).

## 2.5 Measurement environment

The internal timing baselines in [`05-regression-baselines.md`](05-regression-baselines.md) were
recorded on:

| | |
|---|---|
| CPU | AMD Ryzen 7 5800H, 8 cores / 16 threads, up to 3.2 GHz |
| OS | Windows 11 Home Single Language 10.0.26200 |
| C++ | g++ 10.3.0, benchmark flags `-O3 -DNDEBUG -std=c++11` |
| C++ OpenMP | **OFF** (`OCTOMAP_USE_OPENMP=OFF`, zero `omp_` symbols in `liboctomap.a`) |
| Rust | rustc 1.97.1, `[profile.bench]` with LTO + `codegen-units = 1` |
| Threading | **Single-threaded on both sides** |

OctoMap has no explicit template instantiation for
`OccupancyOcTreeBase<OcTreeNode>`, so the entire template implementation is
compiled into the benchmark's translation unit and **the `-O3` above is what
applies**, not the library's own flags.

## 2.6 Platforms with evidence behind them

| Platform | Toolchain | Result |
|---|---|---|
| Windows 11 x86-64 | `x86_64-pc-windows-gnu`, MinGW libm | Full suite passing, clippy clean |
| Ubuntu 24.04 x86-64 (WSL2) | `x86_64-unknown-linux-gnu`, glibc 2.39 | Full suite passing, clippy clean |

There is no evidence yet from ARM64 or macOS. Why that matters more here than in
most projects — and why the `libm` hypothesis did not materialise — is in
[`03-verification.md`](03-verification.md).

Big-endian is **not supported**: node values are read and written little-endian
unconditionally. The reference uses raw `memcpy`, so files produced by a
big-endian build do not interoperate with a little-endian C++ build either.
