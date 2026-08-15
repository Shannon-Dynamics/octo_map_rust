# 4. Running everything

The library and its whole test suite need nothing but **Rust 1.75+**. A C++
toolchain, CMake and ROS 2 are only required for regenerating fixtures, running
the comparison benchmark, and building the ROS 2 node.

---

## 4.1 The basics

```bash
cargo test --workspace --all-features                # 284 tests, no C++ toolchain
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo fmt --all -- --check
cargo doc --workspace --no-deps --all-features       # 0 warnings
cargo run --example insert_and_query                 # the runtime usage pattern
```

All of the above are expected to be clean. If a differential test fails,
**assume the port is wrong until proven otherwise** — do not widen the
comparison to a tolerance. The reasoning is in
[`03-verification.md`](03-verification.md).

## 4.2 Examples

| Command | What it does |
|---|---|
| `cargo run --example insert_and_query` | Integrates a scan and queries the three occupancy states |
| `cargo run --example save_load` | Writes `.bt` and `.ot` to a temp directory, reads both back, shows what each format keeps |
| `cargo run --example ray_cast` | A hit, and the four distinct ways a cast can miss |
| `cargo run --example write_scene -- tests/golden` | Writes `rust_scene.bt` / `.ot` for the reverse-direction interop check |
| `cargo run --release --example dump_bench_fixture` | Writes the shared benchmark scene (~1.9 MB, deterministic from a fixed seed) |

`dump_bench_fixture` is the **only** benchmark scene generator. There is no
second one on the C++ side — two generators that are merely *supposed* to
produce the same points are the kind of silent divergence this project's own
audit has already caught three times.

## 4.3 Benchmarks

Benchmarks here are a regression tool: they exist so that a change to a hot path
can be checked against the same measurement afterwards. They are not a claim
about this library relative to anything else.

```bash
cargo bench                               # everything
cargo bench --bench insert_point_cloud    # the one used in the timing document
```

Criterion's medians are in `target/criterion/*/new/estimates.json` — **the
number printed to the screen is the mean**, not the median, and `bench_cpp`
reports the median. Comparing the two means comparing different statistics.

The full procedure is in [`runbooks/benchmark.md`](runbooks/benchmark.md).

## 4.4 Regenerating the fixtures

Needs CMake and a C++ toolchain. Not required to use or test this crate — only
if the fixtures have to be rebuilt.

```bash
git clone --depth 1 https://github.com/OctoMap/octomap.git reference-cpp
cmake -S reference-cpp/octomap -B build-cpp -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-cpp -j 4
```

The complete steps, generator by generator, are in
[`runbooks/regenerate-fixtures.md`](runbooks/regenerate-fixtures.md).

## 4.5 Linux verification

```bash
bash scripts/linux_verify.sh
```

The script copies the source to `$HOME` first — building under `/mnt` is slow
and collides with the Windows `target/` — and needs **no C++ toolchain**,
because the fixtures are committed. The procedure is in
[`runbooks/linux-verify.md`](runbooks/linux-verify.md).

## 4.6 The ROS 2 node

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2/octomap_server_rs
cargo build --release
./target/release/octomap_server_rs --ros-args -r cloud_in:=/camera/depth/points
```

Testing it without hardware:

```bash
bash scripts/ros2/smoke_test.sh
```

Parameters, the colcon build, RViz, tuning and troubleshooting are in
[`07-ros2.md`](07-ros2.md); the operational summary is in
[`runbooks/ros2-node.md`](runbooks/ros2-node.md).

## 4.7 Environment notes

- Rust installs into `~/.cargo` without changing the system PATH. Add
  `%USERPROFILE%\.cargo\bin` to PATH to make it permanent, or prefix each
  command with `export PATH="$HOME/.cargo/bin:$PATH"`.
- `cargo bench` uses `[profile.bench]` with `lto = true` and
  `codegen-units = 1`. An application that depends on this crate and builds with
  plain `--release` gets meaningfully slower insertion unless it enables LTO
  itself — [ADR-0010](decisions/0010-lto-in-bench-profile.md).
