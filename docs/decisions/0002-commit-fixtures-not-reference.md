# ADR-0002 — Fixtures are committed, the C++ source is not vendored

- **Status:** Accepted

## Context

[ADR-0001](0001-differential-bit-exact.md) requires the C++ reference's answers
to be available when the tests run. There are three ways to provide them:

1. **Vendor the C++ source** into the repository and build it during the tests.
2. **Clone and build** the reference on every machine before testing.
3. **Commit the answers** as fixtures; the source is only needed to regenerate
   them.

## Decision

Option 3. `tests/golden/` holds **174 KB** of CSV and map files produced by a
C++ binary that actually ran, and it is **committed**. `reference-cpp/` and
`build-cpp/` are gitignored.

## Evidence

Option 1 forks the reference: once the C++ source is inside the repository it
starts to age, and "which version is this port equivalent to" stops having a
clear answer. It also changes the licensing position from "a port that
reproduces behaviour" to "a distribution containing the original work".

Option 2 requires CMake and a C++ toolchain on every machine that wants to run
`cargo test`. For a contributor who only wants to fix one function, that is an
entry cost out of proportion to the change.

Option 3 gives both: `cargo test --workspace` demonstrates equivalence **with no
C++ toolchain at all**, and the reference stays one explicitly named commit —
`f012f5f0a4f58cad19501833f9c0ea9d864427b6`, OctoMap 1.10.0.

It stays small because what is stored is the answers, not the program: 938 rows
of geometry, 43 occupancy steps, 12 ray shapes, and two map files.

## Consequences

- **Fixtures must never be hand-edited.** They are the output of the reference
  binary; if one needs to change, regenerate it through
  [`../runbooks/regenerate-fixtures.md`](../runbooks/regenerate-fixtures.md).
- CI needs nothing but Rust — which is what makes the Windows + Linux matrix in
  [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml) cheap.
- `.gitattributes` forces `eol=lf` on the CSVs and `binary` on `.bt`/`.ot`.
  Line-ending translation would break the byte-identical comparison.
- The `rust_scene.*` files are **not** fixtures — they are interop-check output,
  so they are gitignored. What is committed is `cpp_scene.*`.
- The published `octomap-core` crate excludes `tests/`, because the fixtures
  live outside the package directory and the tests resolve them with
  `include_str!` at compile time. The differential suites are a repository-level
  activity.
