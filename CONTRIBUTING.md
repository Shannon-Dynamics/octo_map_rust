# Contributing

Thanks for looking. This is an early open-source project: the code is complete
enough to use, the API is not frozen, and the rules that keep it a *port* rather
than a rewrite are the part most worth reading before you start.

## Getting set up

```bash
git clone https://github.com/Shannon-Dynamics/octo_map_rust
cd octo_map_rust

cargo build --workspace
cargo test --workspace            # 284 tests, no C++ toolchain needed
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo doc --workspace --no-deps --open
```

Requires Rust 1.75 or newer — that is the MSRV, and CI checks it separately from
the stable build. Nothing else: `octomap-core` has no dependencies, and the
differential fixtures are committed so the tests need no C++ compiler.

Two optional extras:

- **Regenerating the golden fixtures** needs CMake and a C++ toolchain. See
  [`docs/runbooks/regenerate-fixtures.md`](docs/runbooks/regenerate-fixtures.md).
  You only need this if you are adding a differential test.
- **Building the ROS 2 node** needs a sourced ROS 2 installation. See
  [`docs/07-ros2.md`](docs/07-ros2.md).

## Which repository does my change belong in?

There are two, and the split is deliberate:

| Change | Repository |
|---|---|
| Octree, occupancy, rays, geometry, file I/O, the public API | **`octo_map_rust`** → `crates/octomap-core` |
| ROS 2 message conversions, `PointCloud2` decoding, marker output | **`octo_map_rust`** → `crates/octomap-ros` |
| The ROS 2 mapping node itself | **`octo_map_rust`** → `ros2/octomap_server_rs` |
| Library documentation, tutorial, examples | **`octo_map_rust`** |
| The drone simulation, the scanning pipeline, the Candi demo | **[`scan_candi_with_octomap_rust`](https://github.com/Shannon-Dynamics/scan_candi_with_octomap_rust)** |
| "The example application should show X" | **`scan_candi_with_octomap_rust`** |

Rule of thumb: if it would still be true for someone mapping a warehouse, it
belongs here. If it is about a temple, a drone or MuJoCo, it belongs in the
application repository.

## The rules that make this a port

These are not style preferences. Breaking one changes output relative to the C++
reference, which is the thing the whole test suite exists to prevent.

1. **Never relax a differential test to a tolerance.** Floating point is
   compared as raw IEEE-754 bit patterns because both implementations perform
   the same operations in the same order. Three real bugs were caught by exactly
   that strictness and every one would have passed a reasonable epsilon. If a
   comparison fails, assume the port is wrong until proven otherwise.
2. **Do not "fix" a deliberate divergence** without reading its ADR first. Seven
   exist in [`docs/decisions/`](docs/decisions/README.md). Each looks like a bug
   and is not.
3. **`unsafe` is `forbid`-ed at the workspace level.** See
   [`SAFETY.md`](SAFETY.md) for what it would take to change that.
4. **`octomap-core` has zero runtime dependencies.** Adding one is a design
   decision, not a convenience: every consumer inherits it. Open an issue first.
5. **`octomap-ros` must never depend on ROS.** It takes the plain data a message
   carries, so it builds and tests anywhere —
   [ADR-0009](docs/decisions/0009-ros-split.md).
6. **`ros2/octomap_server_rs` stays out of the root workspace.** Pulling it in
   breaks `cargo test --workspace` on every machine that has Rust but not ROS.
7. **Fixtures are regenerated from a real C++ binary, never hand-edited.**

## Submitting a change

1. Open an issue first for anything that changes the public API, adds a
   dependency, or diverges further from the reference. For a bug fix or a
   documentation improvement, go straight to a pull request.
2. Keep the commit history readable — one logical change per commit.
3. Before pushing:

   ```bash
   cargo fmt --all -- --check
   cargo clippy --workspace --all-targets -- -D warnings
   cargo test --workspace
   cargo doc --workspace --no-deps       # with RUSTDOCFLAGS="-D warnings"
   ```

   CI runs exactly these on Linux and Windows, plus an MSRV check, `cargo
   audit`, a documentation-link check and `cargo package`.

   If you touched a parser, geometry, or anything else in the core, also run:

   ```bash
   cargo +nightly miri test -p octomap-core --lib
   cargo deny check
   ```

   Miri takes minutes rather than seconds, which is why CI does not run it on
   every push. It is the check that would catch undefined behaviour, and this
   project's main claim is that there is none.
4. New behaviour needs a test. New *public* behaviour needs a doc comment with
   an example, and the example is a doc-test, so it has to compile.
5. Add a changelog entry under `## [Unreleased]` in the matching category. A
   change to a crate's API goes in that crate's changelog
   (`crates/*/CHANGELOG.md`); anything repository-wide goes in the root one.
6. If your change alters output relative to the C++ reference, it needs an ADR
   in `docs/decisions/` **and** a comment at the point of definition. Without
   the comment, the next reader files it as a bug.

## Documentation

All documentation is written in English.

Documentation is expected to match the code exactly. Every command in a README
or a tutorial should run as written, every code sample should compile, every
type name should exist. If you find one that does not, that is a bug worth a
pull request on its own.

## Lockfile policy

`Cargo.lock` is **committed** for this repository. The workspace contains
binaries and examples as well as libraries, and a committed lockfile is what
makes `cargo test` resolve the same dependency versions for everyone — which
matters here, because the differential suites compare exact floating-point
results.

It has no effect on anyone depending on `octomap-core`: a lockfile in a
dependency is ignored by the crate that consumes it.

## Code style

`rustfmt` defaults, no exceptions. Beyond that, the existing code has a habit
worth continuing: comments explain *why*, especially where an expression looks
wrong. `1.0 / resolution` multiplied rather than divided, a `f32` narrowing in
the middle of `f64` arithmetic, a prune that stops early — each of those is
correct, deliberate, and commented at the point of definition. Add the comment
when you add the surprise.
