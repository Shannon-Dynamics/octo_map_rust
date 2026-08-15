# Benchmarks — raw output

Logs from the runs behind the numbers in
[`../docs/05-regression-baselines.md`](../docs/05-regression-baselines.md). Committed so that the
numbers can be checked rather than merely believed.

These are internal regression measurements. They are not a claim about this
library relative to anything else — see the top of the timing document.

| File | Contents |
|---|---|
| [`cpp-bench-run.log`](cpp-bench-run.log) | `bench_cpp` — OctoMap C++ 1.10.0, g++ 10.3.0 `-O3 -DNDEBUG`, OpenMP OFF |
| [`rust-bench-run.log`](rust-bench-run.log) | `cargo bench --bench insert_point_cloud` — criterion, bench profile with LTO |

Taken on an AMD Ryzen 7 5800H running Windows 11, both sides back to back on
the same machine.

---

## How to read them

`bench_cpp` emits a comment header then machine-readable result lines:

```text
# openmp          disabled -- single-threaded
# checksum        328789090342991
# format          result,name,elements,samples,median_ns,min_ns,max_ns
result,insert_eager,50176,20,63816500,60953400,66349900
...
# populated map   34105 nodes, 28614 leaves
```

Those two comment lines matter most, and both **must match the Rust side**:

| Check | Value | What it proves |
|---|---|---|
| `checksum` | `328789090342991` | Both sides read the same points |
| `populated map` | **34,105 nodes, 28,614 leaves** | Both sides **built an identical tree** from those points |

The second is far stronger. If either pair disagrees, the two sides are not
measuring the same thing and their numbers **must not share a table**.

The `openmp disabled` line is the evidence that the comparison is
single-threaded on both sides — the reference build has
`OCTOMAP_USE_OPENMP=OFF` and zero `omp_` symbols.

## A warning about medians

`bench_cpp` reports the **median**. Criterion prints the **mean** to the screen;
its median is in `target/criterion/*/new/estimates.json`. The numbers in the
timing document use the median on both sides.

## What is not here

- **`tests/bench/scene.txt`** (~1.9 MB) — the shared scene both benchmarks read.
  Deterministic from a fixed seed, so it is regenerated rather than committed:
  `cargo run --release --example dump_bench_fixture`.
- **`target/criterion/`** — criterion's HTML output.
- **Linux numbers.** The test suite passes there; the benchmarks were only run
  on Windows.

## Re-taking them

The full procedure, including the four pieces of context that must be stated
when reporting the numbers, is in
[`../docs/runbooks/benchmark.md`](../docs/runbooks/benchmark.md).
