# ADR-0001 — Correctness verified differentially, compared bit-exact

- **Status:** Accepted

## Context

Behavioural compatibility with OctoMap C++ is the **point** of this port, not a
hoped-for side effect. The question was how to prove it.

The options:

1. Hand-written unit tests against the author's expectations.
2. Differential tests against the reference's output, compared with a
   **tolerance**.
3. Differential tests against the reference's output, compared **bit-exact**.

Option 1 only tests the author's understanding, and that understanding is
exactly what is most likely to be wrong. Option 2 looks safe.

## Decision

Option 3. The C++ reference is built, driven with the same inputs, its answers
are stored as fixtures under `tests/golden/`, and floating point is compared as
**raw IEEE-754 bit patterns** with `==`.

Every value is printed with `%.17g`, which round-trips a `double` exactly.

## Evidence

Both implementations perform the same sequence of operations on the same data,
so the results are bit-identical **if the port is correct**. Anything short of
bit equality is a real divergence, not noise.

A tolerance would hide the class of bug these tests are looking for, and that is
not a theoretical worry. Three bugs were caught precisely because of this
strictness, and all three would have passed any epsilon:

| Bug | Size of the difference |
|---|---|
| The reciprocal trap ([ADR-0005](0005-reciprocal-multiply.md)) | `1.2/0.1` = 11.999999999999998 vs 12.0 — one voxel, on the wrong side |
| `Quaternion::norm` ([ADR-0006](0006-quaternion-norm-f32.md)) | **Exactly one ULP**, propagating into every composed pose |
| `updateNode` auto-prune | Different tree structure, same values |

The whole test suite rests on this method: 63 differential tests alongside the
in-module unit tests, detailed per suite in
[`../03-verification.md`](../03-verification.md).

## Consequences

- **If a differential test fails, the port is wrong until proven otherwise.**
  Do not widen the comparison to a tolerance to make it green.
- The port depends on `libm`: `log`/`exp`/`atan2` are not guaranteed identical
  across implementations. This was measured, and they agreed on both MinGW and
  glibc — see [`../03-verification.md`](../03-verification.md). If a platform
  ever cracks, the answer is to move the two affected suites to a small ULP
  tolerance **and record it as a divergence**, not to declare the port wrong.
- Every deliberate divergence has to be recorded, because the tests will not let
  one pass silently. That is where the seven divergence ADRs in this folder came
  from.
