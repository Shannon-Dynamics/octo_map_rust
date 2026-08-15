# ADR-0007 — Resolution is written with six significant digits

- **Status:** Accepted
- **C++ source:** the `.ot` / `.bt` header, written through `std::ostream <<`

## Context

The header of a `.ot` or `.bt` file carries a resolution line. The reference
writes it with a plain `std::ostream <<`, which defaults to `%g` with **six
significant digits**.

For a resolution like `0.05` or `0.1` that is fine. For one needing more digits,
the written value **loses precision** — and a file read back has a slightly
different resolution from the one that wrote it.

The sensible design: write at full precision (`%.17g`), which round-trips a
`double` exactly.

## Decision

Six significant digits, following the C++ default. Commented in
[`io.rs`](../../crates/octomap-core/src/io.rs).

## Evidence

A core claim of this project is that `.bt` and `.ot` output is **byte-for-byte
identical** to the reference's
([`../03-verification.md`](../03-verification.md)). The header is part of those
bytes. Writing full precision makes the header differ, and the byte-identical
claim collapses entirely — not just for the problematic resolutions, but for
every file whose resolution happens to need more than six digits.

Byte-identical is stronger than "C++ can parse it": identical files cannot
decode differently, and that is what lets `cargo test` demonstrate interop with
no C++ toolchain.

The downside is real but bounded: precision is lost only for resolutions
requiring more than six significant digits, which is unusual in practice.

## Consequences

- A file with a resolution like `0.0333333333` reads back as `0.0333333`. The
  reference behaves the same way, so the files still interoperate.
- **Do not raise the precision "to be more correct".** The tests in
  `interop_io.rs` compare bytes, and they would all fail.
- If upstream raises its precision, this ADR is superseded and the fixtures are
  regenerated.
