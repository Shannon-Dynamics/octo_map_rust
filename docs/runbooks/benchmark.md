# Runbook — re-measuring the timing baselines

**When to use it:** a hot path changed, or the numbers in
[`../05-regression-baselines.md`](../05-regression-baselines.md) need re-taking on another machine.

These are internal regression measurements, not a claim about this library
relative to anything else. See the top of
[`../05-regression-baselines.md`](../05-regression-baselines.md).

**Prerequisites:** the C++ reference is built
([`regenerate-fixtures.md`](regenerate-fixtures.md) steps 1–2).

---

## Steps

```bash
# 1. The shared scene — deterministic from a fixed seed, ~1.9 MB, gitignored
cargo run --release --example dump_bench_fixture

# 2. The C++ benchmark at -O3, matching the library's Release flags
g++ -O3 -DNDEBUG -std=c++11 -I reference-cpp/octomap/include \
    scripts/bench_cpp.cpp -o build-cpp/bench_cpp.exe \
    -L build-cpp -loctomap -loctomath

# 3. Run both, back to back, on the same machine
./build-cpp/bench_cpp.exe tests/bench/scene.txt
cargo bench --bench insert_point_cloud
```

**One generator, not two.** Both benchmarks read `tests/bench/scene.txt`, which
the Rust generator writes. There is no second generator on the C++ side — two
generators that are merely *supposed* to produce the same points are a failure
mode this project's audit has already caught three times.

---

## Verification — do this BEFORE trusting any number

Both sides print two values. **If either pair disagrees, the comparison is
void** and the numbers must not share a table.

| Check | Correct value |
|---|---|
| Fixture checksum (sum of the `f32` bits of every coordinate) | `328789090342991` |
| The resulting map | **34,105 nodes, 28,614 leaves** |

The checksum only proves the input was the same. The node count proves **both
implementations built an identical tree** from it — that is the stronger check,
and it is the one to look at.

Both appear at the head of the log:
[`../../benchmarks/cpp-bench-run.log`](../../benchmarks/cpp-bench-run.log).

## Reading the numbers off

`bench_cpp` prints a machine-readable line:

```text
result,<name>,<elements>,<samples>,<median_ns>,<min_ns>,<max_ns>
```

**Criterion does not.** The number printed to the screen is the **mean**; the
median is in `target/criterion/*/new/estimates.json`. Use the median on both
sides, or you are comparing two different statistics.

C++ discards 3 warm-up iterations then measures 20 (insertion) or 50 (queries
and casts). Criterion has its own warm-up and sampling: 10 samples for
insertion, 100 for queries. Since both report medians, the differing sample
counts affect the width of the confidence interval, not the point estimate.

## What must be stated when reporting

The numbers mean nothing without these four pieces of context — all expanded in
[`../05-regression-baselines.md`](../05-regression-baselines.md):

1. **Rust is built with LTO** (`[profile.bench]`). Without it insertion is
   substantially slower and the ratio reads 2.2× —
   [ADR-0010](../decisions/0010-lto-in-bench-profile.md).
2. **OpenMP is off on the C++ side** (`OCTOMAP_USE_OPENMP=OFF`, zero `omp_`
   symbols) so that both are single-threaded. The reference has an OpenMP path
   and this port does not, so the real gap in a tuned deployment can be larger.
3. **One machine, one OS, one architecture.** MinGW and glibc allocators differ,
   and insertion allocates heavily.
4. **One run.** Thermals and boost clock were not controlled. A 6% difference
   sits inside that uncertainty; a 47% one does not.

## If it fails

| Symptom | Cause | What to do |
|---|---|---|
| The checksums differ between sides | The scene was rewritten between runs | Write once, run both, do not regenerate in between |
| The node counts differ | One implementation changed behaviour | **Stop.** This is a correctness problem, not a timing one — run `cargo test --workspace` |
| Rust roughly 2× slower on insertion | LTO is not active | Make sure you are using `cargo bench`, not `cargo run --release` |
| `tests/bench/scene.txt` is missing | Not generated yet | `cargo run --release --example dump_bench_fixture`. The Rust benchmark also writes it if absent |
| Criterion numbers do not match the reported ones | Reading the mean off the screen | Take the median from `estimates.json` |
