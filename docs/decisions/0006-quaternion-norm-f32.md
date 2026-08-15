# ADR-0006 — `Quaternion::norm` squares in `f32`

- **Status:** Accepted
- **C++ source:** `Quaternion.cpp::norm()`

## Context

`Quaternion` stores its components as `float`. To compute the norm, the
reference squares them **in `float`**, then accumulates the results into
`double`.

This port's first implementation did the numerically better thing: promote each
component to `f64` **first**, then square. That avoids one rounding step.

Three differential tests failed.

## Decision

Square in `f32` then accumulate into `f64`, exactly as the reference does.
Commented in [`pose.rs`](../../crates/octomap-core/src/pose.rs).

## Evidence

The difference is **exactly one ULP**.

One ULP sounds unimportant until you follow where it goes: `norm()` is used by
`normalized()`, `normalized()` is used by every pose construction and
composition, and poses transform entire point clouds. The shift propagates.

The three failing tests — in `golden_pose.rs` — all differed by one ULP. That is
precisely the class of divergence a tolerance-based comparison would let
through, and the reason [ADR-0001](0001-differential-bit-exact.md) chose bit
comparison.

Worth stating plainly: the **numerically more accurate version is the wrong one
here**. The goal of this port is behavioural equality, not the best arithmetic.

## Consequences

- Anyone reading `norm()` will see a type promotion that looks redundant. The
  comment explains it; **do not remove either one**.
- It applies to `normalized()`, `Pose6::transform`, `Pose6::inv` and pose
  composition — the 11 tests in `golden_pose.rs` rest on it.
- If OctoMap C++ ever moves to `double` storage, this ADR is superseded and the
  fixtures are regenerated.
