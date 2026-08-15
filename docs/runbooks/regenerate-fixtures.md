# Runbook — regenerating the golden fixtures

**When to use it:** the C++ reference moved version, or a new divergence changes
pinned output. **Not** needed to use or test this crate — the fixtures are
committed
([ADR-0002](../decisions/0002-commit-fixtures-not-reference.md)).

**Prerequisites:** CMake, Ninja and a C++ toolchain. On Windows, the GNU
toolchain from MSYS2 matches the `x86_64-pc-windows-gnu` Rust host.

---

## 1. Fetch the reference

```bash
git clone --depth 1 https://github.com/OctoMap/octomap.git reference-cpp
```

Pinned to OctoMap **1.10.0**, commit
`f012f5f0a4f58cad19501833f9c0ea9d864427b6`. `reference-cpp/` is gitignored **on
purpose** — it is a build input, not vendored source.

## 2. Build it

```bash
cmake -S reference-cpp/octomap -B build-cpp -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build build-cpp -j 4
```

Produces `build-cpp/liboctomap.a` and `build-cpp/liboctomath.a`.

## 3. Regenerate the fixtures

Generators that write to stdout:

```bash
for gen in geometry tree occupancy ray; do
  g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
      "scripts/gen_golden_$gen.cpp" -o "build-cpp/gen_golden_$gen.exe" \
      -L build-cpp -loctomap -loctomath
  "./build-cpp/gen_golden_$gen.exe" > "tests/golden/$gen.csv"
done
```

Generators that take an output path — because the reference prints debug output
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

| Generator | Fixture | Coverage |
|---|---|---|
| `gen_golden_geometry.cpp` | `geometry.csv` | coord ↔ key, depth variants, bounds checks, node sizes, sensor defaults |
| `gen_golden_tree.cpp` | `tree.csv` | node counts, leaf and tree iteration order, prune, delete, depth-limited views |
| `gen_golden_occupancy.cpp` | `occupancy.csv` | log-odds updates, clamping, lazy insert, inner propagation, max likelihood, change detection |
| `gen_golden_ray.cpp` | `ray.csv` | DDA key sequences, ray casting, point-cloud integration |
| `gen_golden_pose.cpp` | `pose.csv` | Euler ↔ quaternion, rotation, axis-angle, transform / inverse / composition |
| `gen_golden_io.cpp` | `io.csv`, `cpp_scene.ot`, `cpp_scene.bt` | reference-written map files and what they decode to |

## 4. The cross-language check

Byte equality against `cpp_scene.*` already implies the reference can read what
this crate writes. This step closes the loop explicitly by handing Rust-written
files to the reference:

```bash
g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
    scripts/verify_rust_io.cpp -o build-cpp/verify_rust_io.exe \
    -L build-cpp -loctomap -loctomath

cargo run --example write_scene -- tests/golden
./build-cpp/verify_rust_io.exe tests/golden
```

It loads `rust_scene.ot` and `rust_scene.bt`, checks that they agree on
occupancy, and confirms that the C++ rewrite of the `.ot` is byte-identical. It
exits non-zero on any failure. The `rust_scene.*` files are gitignored — they
are output, not fixtures.

---

## Verification

```bash
cargo test --workspace
```

All of them must pass. If something fails after regeneration, the reference
changed behaviour — it does not mean the port broke. Find out **what** changed
before adjusting code: if it is a fix to one of the divergences recorded in
[`../decisions/`](../decisions/README.md), that ADR becomes superseded.

Reference values:

| | |
|---|---|
| Total fixtures | ~174 KB |
| `geometry.csv` | 938 rows |
| `interop_io.rs` | Byte-identical output in both directions |
| `verify_rust_io` | Occupancy agrees on **99 of 99 leaves** |

## If it fails

| Symptom | Cause | What to do |
|---|---|---|
| Fixtures change on many rows at once after a fresh clone | The reference is not 1.10.0 | Check the commit; `--depth 1` fetches HEAD, not the tag |
| `undefined reference to octomath::...` | Library order reversed | `-loctomap` must come before `-loctomath` |
| The CSVs have CRLF | Windows checkout without `.gitattributes` | `.gitattributes` forces `eol=lf`; do not re-normalize these files |
| Reference debug output lands in the CSV | The `io`/`pose` generators were run with `>` | Both take an **output path**, not a redirect |
