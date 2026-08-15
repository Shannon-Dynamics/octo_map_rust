# Reference tooling

These scripts build the C++ reference and capture its output as golden fixtures
for the differential tests. Nothing here is needed to *use* `octomap-core` —
only to verify it.

## 1. Fetch the reference

```bash
git clone --depth 1 https://github.com/OctoMap/octomap.git reference-cpp
```

Pinned to OctoMap **1.10.0**, commit `f012f5f0a4f58cad19501833f9c0ea9d864427b6`.
`reference-cpp/` is gitignored on purpose — it is a build input, not vendored
source.

## 2. Build it

Requires CMake and a C++ toolchain. On Windows with MSYS2 the GNU toolchain
works and matches the `x86_64-pc-windows-gnu` Rust host:

```bash
cmake -S reference-cpp/octomap -B build-cpp -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build build-cpp -j 4
```

Produces `build-cpp/liboctomap.a` and `build-cpp/liboctomath.a`.

## 3. Regenerate the golden fixtures

Generators that write to stdout:

```bash
for gen in geometry tree occupancy ray; do
  g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
      "scripts/gen_golden_$gen.cpp" -o "build-cpp/gen_golden_$gen.exe" \
      -L build-cpp -loctomap -loctomath
  "./build-cpp/gen_golden_$gen.exe" > "tests/golden/$gen.csv"
done
```

Generators that take an output path, because the reference prints debug output
to stdout during the calls they make:

```bash
for gen in io pose; do
  g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
      "scripts/gen_golden_$gen.cpp" -o "build-cpp/gen_golden_$gen.exe" \
      -L build-cpp -loctomap -loctomath
done
./build-cpp/gen_golden_io.exe tests/golden tests/golden/io.csv
./build-cpp/gen_golden_pose.exe tests/golden/pose.csv
```

Then `cargo test --workspace`.

| Generator | Fixture | Covers |
|---|---|---|
| `gen_golden_geometry.cpp` | `geometry.csv` | coordinate ↔ key, depth variants, bounds checking, node sizes, sensor defaults |
| `gen_golden_tree.cpp` | `tree.csv` | node counts, leaf and tree iteration order, prune, delete, depth-limited views |
| `gen_golden_occupancy.cpp` | `occupancy.csv` | log-odds updates, clamping, lazy insertion, inner-node propagation, max likelihood, change detection |
| `gen_golden_ray.cpp` | `ray.csv` | DDA key sequences, ray casting, point-cloud integration |
| `gen_golden_pose.cpp` | `pose.csv` | Euler ↔ quaternion, rotation, axis-angle, pose transform / inverse / composition |
| `gen_golden_io.cpp` | `io.csv`, `cpp_scene.ot`, `cpp_scene.bt` | map files written by the reference, and what they decode to |

## 4. Cross-language file check

Byte equality against `cpp_scene.*` already implies the reference can read what
this crate writes. This closes the loop explicitly by handing the reference
Rust's own files:

```bash
g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
    scripts/verify_rust_io.cpp -o build-cpp/verify_rust_io.exe \
    -L build-cpp -loctomap -loctomath

cargo run --example write_scene -- tests/golden
./build-cpp/verify_rust_io.exe tests/golden
```

It loads `rust_scene.ot` and `rust_scene.bt`, checks the two agree about
occupancy, and confirms a C++ rewrite of the `.ot` is byte-identical. Exits
non-zero on any failure. The `rust_scene.*` files are gitignored — they are
output, not fixtures.

## 5. Timing comparison

`bench_cpp.cpp` measures the same operations the Rust benchmark measures, on
the same points. Results and interpretation live in
[`../docs/05-regression-baselines.md`](../docs/05-regression-baselines.md) — which is an internal
regression record, not a claim about either implementation.

Both sides read one scene file, written by the Rust generator. There is no
second generator for the C++ side — two generators that are only *supposed* to
agree is the failure mode this project's audit already caught three times.

```bash
# 1. Write the shared scene (~1.9 MB, gitignored, deterministic)
cargo run --release --example dump_bench_fixture

# 2. Build the C++ benchmark at -O3, matching the library's Release flags
g++ -O3 -DNDEBUG -std=c++11 -I reference-cpp/octomap/include \
    scripts/bench_cpp.cpp -o build-cpp/bench_cpp.exe \
    -L build-cpp -loctomap -loctomath

# 3. Run both, back to back, on the same machine
./build-cpp/bench_cpp.exe tests/bench/scene.txt
cargo bench --bench insert_point_cloud
```

Both print the fixture's checksum, and both print the node and leaf count of
the map they built from it. **If either pair disagrees, the comparison is void**
— the two sides are not measuring the same thing, and the numbers must not be
put in a table next to each other.

The C++ benchmark emits machine-readable lines:

```text
result,<name>,<elements>,<samples>,<median_ns>,<min_ns>,<max_ns>
```

Criterion's medians come from `target/criterion/*/new/estimates.json`. Use the
median on both sides — criterion's headline figure is the *mean*, which is not
what `bench_cpp` reports.

## Why the fixtures are committed but the reference is not

Contributors should be able to run the differential tests without a C++
toolchain. Committing the CSV makes the reference's answers available
everywhere; committing the C++ source would fork it.

## On exact comparison

The differential tests compare floating point with `==`, deliberately. Both
implementations perform the same sequence of IEEE-754 operations in the same
order, so the results are bit-identical when the port is correct. A tolerance
would mask exactly the class of bug these tests exist to find — for example
dividing by `resolution` where the reference multiplies by `1.0 / resolution`,
which shifts points across voxel boundaries while staying well inside any
reasonable epsilon.

All values are printed with `%.17g`, which round-trips a `double` exactly.
