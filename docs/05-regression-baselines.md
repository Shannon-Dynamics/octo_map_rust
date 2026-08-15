# 5. Validation and regression baselines

**This document is internal maintainer tooling, not a claim about this
library.** Nothing here is a project feature, and none of it belongs in a
README.

It exists for two reasons, both of them about correctness:

1. **Reference regression comparison.** Driving this port and the C++ reference
   through the same points and checking that both build an *identical tree* is a
   correctness check that the differential fixtures do not cover — those pin
   individual operations, this pins the result of forty thousand of them in
   sequence.
2. **Regression baselines.** A change to a hot path can be re-measured against
   the same procedure afterwards. Timings are recorded so that a later run has
   something to be compared against; they are not a target and not a result.

Read [§5.7](#57-what-was-not-controlled) before quoting any number from here.
Most of the ways these numbers could mislead are listed there.

Both implementations were measured on the same machine, with the same dataset,
back to back. Raw output from both runs is committed:
[`../benchmarks/cpp-bench-run.log`](../benchmarks/cpp-bench-run.log) and
[`../benchmarks/rust-bench-run.log`](../benchmarks/rust-bench-run.log).

---

## 5.1 The equivalence check

This is the part that matters, and it is a correctness result rather than a
timing one. Both implementations read the same scene file and report what they
built from it. **Both must agree, or the run is void:**

| Check | Value |
|---|---|
| Fixture checksum (sum of the `f32` bits of every coordinate) | `328789090342991` on both sides |
| The resulting map | **34,105 nodes, 28,614 leaves** on both sides |

The checksum only proves the input was the same. The node count proves the two
implementations built **the same tree** from it — 50,176 points, tens of
thousands of insertions, one identical structure at the end.

If a change to this port ever breaks that agreement, it is a correctness
problem, and no timing number from the same run means anything until it is
resolved.

---

## 5.2 Recorded baselines

Medians, in the configuration described in [§5.3](#53-environment). The C++
column is present because the two implementations were run on identical input
in the same session; it is context for the recorded numbers, not a ranking.

### Point-cloud integration — 50,176 points, 0.1 m resolution

| Operation | C++ reference | This port |
|---|---:|---:|
| Eager | 63.8 ms | 93.9 ms |
| Lazy + inner + prune | 55.9 ms | 85.4 ms |
| Discretized | 48.0 ms | 58.6 ms |

### Queries — 10,000 lookups

| Operation | C++ reference | This port | Per query, reference | Per query, here |
|---|---:|---:|---:|---:|
| From a world coordinate | 512 µs | 523 µs | 51.2 ns | 52.3 ns |
| From a key | 420 µs | 324 µs | 42.0 ns | 32.4 ns |

### Ray casting — 1,000 casts

| Operation | C++ reference | This port | Per cast, reference | Per cast, here |
|---|---:|---:|---:|---:|
| `cast_ray` | 2.23 ms | 2.36 ms | 2.23 µs | 2.36 µs |

### Raw values (nanoseconds, median)

| Operation | C++ reference | This port (release) | This port (release + LTO) |
|---|---:|---:|---:|
| `insert_eager` | 63,816,500 | 140,974,040 | 93,891,915 |
| `insert_lazy_then_inner` | 55,946,050 | — | — |
| `insert_lazy_then_inner_and_prune` | 55,874,050 | 125,110,706 | 85,402,734 |
| `insert_discretized` | 47,954,850 | 85,955,467 | 58,558,715 |
| `query_by_coordinate` ×10k | 511,950 | 563,604 | 523,231 |
| `query_by_key` ×10k | 419,750 | 352,334 | 324,003 |
| `cast_ray` ×1000 | 2,226,050 | 2,705,309 | 2,362,939 |

The column carried into the tables above is **release + LTO** — see
[§5.4](#54-why-lto-changes-what-is-being-measured).

---

## 5.3 Environment

| | |
|---|---|
| CPU | AMD Ryzen 7 5800H, 8 cores / 16 threads, up to 3.2 GHz |
| OS | Windows 11 Home Single Language 10.0.26200 |
| **C++** | g++ 10.3.0 (MSYS2 MinGW-w64) |
| C++ flags (benchmark) | `-O3 -DNDEBUG -std=c++11` |
| C++ flags (`liboctomap.a`) | CMake `Release` → `-O3 -DNDEBUG` |
| C++ OpenMP | **OFF** (`OCTOMAP_USE_OPENMP=OFF`; zero `omp_` symbols) |
| **Rust** | rustc 1.97.1, host `x86_64-pc-windows-gnu` |
| Rust profile | `[profile.bench]`: `lto = true`, `codegen-units = 1`, opt-level 3 |
| Threading | **Single-threaded on both sides** |
| Reference | OctoMap C++ 1.10.0, commit `f012f5f` |

OctoMap has no explicit template instantiation for
`OccupancyOcTreeBase<OcTreeNode>`, so the whole template implementation is
compiled into the benchmark's translation unit and **the `-O3` above is what
applies**, not the library's own flags.

---

## 5.4 Why LTO changes what is being measured

Rust's default `release` profile has no LTO and uses `codegen-units = 16`. On
the C++ side the whole OctoMap implementation is header templates landing in
one translation unit, so `-O3` inlines freely across what is a crate boundary
in Rust.

Without LTO, a re-measurement is dominated by missing inlining rather than by
anything in the algorithms — which makes it useless as a regression signal,
because a change to the code would be invisible next to it:

| Operation | Without LTO | With LTO |
|---|---:|---:|
| `insert_eager` | 141.0 ms | 93.9 ms |
| `insert_lazy_then_finish` | 125.1 ms | 85.4 ms |
| `insert_discretized` | 86.0 ms | 58.6 ms |
| `query_by_coordinate` | 563.6 µs | 523.2 µs |
| `query_by_key` | 352.3 µs | 324.0 µs |
| `cast_ray` | 2.71 ms | 2.36 ms |

That is a third of the insertion figure moving on a build setting alone, which
is why `[profile.bench]` pins `lto = true` and `codegen-units = 1`: two runs
have to be built the same way to be comparable at all.

**For users of the crate:** that profile applies to `cargo bench` in this
repository only. It is not inherited by anything depending on `octomap-core`,
and an application built with plain `--release` behaves like the left-hand
column. See [ADR-0010](decisions/0010-lto-in-bench-profile.md).

---

## 5.5 Notes for a maintainer re-measuring

What to know before concluding anything from a fresh run.

### Insertion is where the two implementations differ most

The gap is real and it has an untested explanation. `computeUpdate` builds sets
of tens of thousands of `OcTreeKey` per scan. C++ uses `unordered_set` with a
trivial hash, `k0 + 1447·k1 + 345637·k2`. This port uses the default `HashSet`,
which uses SipHash-1-3 — resistant to hash flooding, and considerably more work
per key for a 6-byte key.

The shape of the data is consistent with that: the **discretized** variant,
which touches the set least because it casts one ray per endpoint voxel, is
closest to the reference; **eager**, which touches it most, is furthest.

Consistent is not proven. What would test it: swap the `HashSet` hasher in
`compute_update` for the same trivial one, and re-measure. That has not been
done, because changing a hasher for timing reasons is a change nobody has
needed and the default one is the safer choice.

### Key lookups and coordinate lookups measure different things

The difference between the two is the `coord → key` conversion:

| | C++ reference | This port |
|---|---:|---:|
| Query from a coordinate | 51.2 ns | 52.3 ns |
| Query from a key | 42.0 ns | 32.4 ns |
| **Conversion cost** | **9.2 ns** | **19.9 ns** |

The conversion here does more work than the reference's, and deliberately:
`coord_to_key_axis_checked` performs an `is_finite()` check per axis that
`coordToKeyChecked` does not, plus the `Option` wrapping. The C++ path has
undefined behaviour for NaN input; this port rejects it instead. **That is a
correctness decision with a cost attached, not an oversight**, and it is the
reason the two query rows move differently when either side changes.

The practical consequence is in the API guidance rather than in the numbers:
code that queries the same place repeatedly should convert once and keep the
key.

### Ray casting sits inside the measurement noise

The two are within a few percent on an arithmetically identical code path —
the range where allocator differences, code layout and run-to-run variation
live. Nothing can be concluded from a difference that size, in either
direction.

---

## 5.6 Sizing a sensor workload

Not a property of this library — a property of the workload. A 50k-point frame
at 30 Hz allows 33 ms, and at 0.1 m resolution neither implementation fits that
budget single-threaded: the reference needs about twice it and this port about
three times.

The conclusion is about configuration, not about implementations. A
full-resolution 30 Hz camera needs downsampling, a coarser map, a capped
`max_range`, or parallelism, in any language. The knobs are in
[`07-ros2.md`](07-ros2.md), cheapest first: `point_stride`, `max_range`,
`resolution`, then `publish_period`.

---

## 5.7 What was not controlled

Required reading before quoting any number above.

**Parallelism — controlled for this measurement, but not for real potential.**
C++ OctoMap has an OpenMP path in `computeUpdate` and `insertPointCloudRays`.
The build used here has `OCTOMAP_USE_OPENMP=OFF` and zero `omp_` symbols, so
both sides really are single-threaded and were doing the same work. **But:**
with OpenMP enabled, the reference can use 8–16 cores on the insertion path and
this port has no equivalent at all. **That is a limitation of the port**, and it
is on the roadmap rather than hidden here.

**One machine, one OS, one architecture.** Ryzen 7 5800H / Windows 11 / x86-64
only. The test suite passes on Linux, but the baselines were **not** re-taken
there. MinGW and glibc have different allocators, and insertion allocates
heavily.

**Different allocators.** This port uses the system allocator (MinGW `malloc`),
the reference uses MinGW's `operator new`. Both allocate tens of thousands of
nodes per scan. Neither was normalized.

**`insert_lazy_then_inner` has no direct counterpart on the C++ side.** The
reference call was `insertPointCloud(..., true, false)` +
`updateInnerOccupancy()`, while the Rust benchmark also calls `prune()`. Both
were measured on the C++ side: 55.946 ms without prune, 55.874 ms with — a
0.13% difference, below noise. The tables use the variant **with** prune, so
both sides do the same work.

**Differing C++ defaults, passed explicitly.** C++'s `castRay` defaults to
`ignoreUnknown = false, maxRange = -1.0`. The Rust benchmark calls it with
`ignore_unknown = true`, so `true` was passed explicitly on the C++ side rather
than left to the default.

**Different sample counts.** Criterion collects 10 samples for insertion (with
many iterations each) and 100 for queries; `bench_cpp` uses 20 and 50. Both
report medians, so this affects the width of the confidence interval, not the
point estimate.

**One run, not an average of sessions.** Thermals, boost clock and background
load were not controlled. A few percent sits inside that uncertainty; a factor
of one and a half does not.

---

## 5.8 Reproducing

```bash
# The shared scene (deterministic, ~1.9 MB, gitignored)
cargo run --release --example dump_bench_fixture

# C++
g++ -O3 -DNDEBUG -std=c++11 -I reference-cpp/octomap/include \
    scripts/bench_cpp.cpp -o build-cpp/bench_cpp.exe \
    -L build-cpp -loctomap -loctomath
./build-cpp/bench_cpp.exe tests/bench/scene.txt

# Rust
cargo bench --bench insert_point_cloud
```

**Check the equivalence values from [§5.1](#51-the-equivalence-check) before
trusting anything else.** If the checksum or the node count disagrees, the two
sides are not doing the same work and their numbers must not share a table.

The full procedure is in [`runbooks/benchmark.md`](runbooks/benchmark.md);
rebuilding the C++ reference from scratch is in
[`runbooks/regenerate-fixtures.md`](runbooks/regenerate-fixtures.md).

---

## 5.9 What has not been measured

- **Linux.** The test suite passes there, but these baselines were only
  recorded on Windows.
- **Parallelism.** This port is single-threaded and has no counterpart to the
  reference's OpenMP path.
- **The cause of the insertion gap.** The hasher explanation in
  [§5.5](#55-notes-for-a-maintainer-re-measuring) fits the data but is
  **untested**.
- **`octomap-ros` and the ROS 2 node.** Only core numbers were recorded; the
  cost of `PointCloud2` decoding and message serialization has not been
  separated out.

The complete list of what is open is in [`../ROADMAP.md`](../ROADMAP.md).
