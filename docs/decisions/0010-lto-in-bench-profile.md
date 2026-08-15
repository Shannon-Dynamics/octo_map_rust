# ADR-0010 — `[profile.bench]` uses LTO and one codegen unit

- **Status:** Accepted

## Context

The first timing comparison against C++ was run with Rust's default `bench`
profile: no LTO, `codegen-units = 16`. The result read as **2.2× slower** on
insertion.

That number looks like a finding about the port. It was not.

On the C++ side the whole OctoMap implementation is header templates landing in
**one translation unit**, so `-O3` inlines freely across what is a crate
boundary in Rust. Rust's default profile does not inline across that boundary.

## Decision

`[profile.bench]` pins `lto = true` and `codegen-units = 1`, and the Rust column
in every comparison table uses that configuration.

## Evidence

| Operation | Without LTO | With LTO | Improvement |
|---|---:|---:|---:|
| `insert_eager` | 141.0 ms | 93.9 ms | **33%** |
| `insert_lazy_then_finish` | 125.1 ms | 85.4 ms | **32%** |
| `insert_discretized` | 86.0 ms | 58.6 ms | **32%** |
| `query_by_coordinate` | 563.6 µs | 523.2 µs | 7% |
| `query_by_key` | 352.3 µs | 324.0 µs | 8% |
| `cast_ray` | 2.71 ms | 2.36 ms | 13% |

A third of the insertion time. Without this adjustment the insertion ratio reads
**2.2× instead of 1.5×** — blaming the port for what is actually a build
setting.

The shape of the numbers supports the diagnosis: the largest improvements are on
the insertion path, which calls across crate boundaries most, while queries and
casts — with shorter paths — move only 7–13%.

## Consequences

- **The numbers in [`../05-regression-baselines.md`](../05-regression-baselines.md) do not
  represent an application using this crate with plain `--release`.** Such an
  application gets the "without LTO" column unless it enables LTO itself. This
  is stated in the timing document, in the README, and here.
- The `bench` profile applies to this repository only; it is not inherited by
  crates that depend on `octomap-core`.
- `cargo bench` compiles more slowly. That is paid once per change, not per run.
- If someone re-measures without LTO and reports 2.2×, their number is not
  wrong — the context is missing.
