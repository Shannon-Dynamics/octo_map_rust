# ADR-0011 — `Pose6` stores a quaternion, not Euler angles

- **Status:** Accepted
- **C++ source:** `Pose6D.h` / `Pose6D.cpp`

## Context

The learning guide that served as the plan for this port describes `Pose6` as
`x, y, z, roll, pitch, yaw` — six numbers, as the name suggests.

The C++ source does not. `Pose6D` stores a **translation plus a quaternion**,
and derives Euler angles on request through a rotation matrix and `atan2`.

Two documents disagreeing, and one of them has to win.

## Decision

Follow the **source**, not the guide. `Pose6` stores a `Point3` and a
`Quaternion`. `to_euler()` derives the angles when called.

## Evidence

Storing Euler angles is not merely a different representation — it changes
results:

- **`pose * pose` (composition)** and **`pose.inv()`** are computed through
  quaternions in the reference. Doing them through Euler angles means converting
  back and forth on every operation, and each conversion carries its own
  rounding. The results differ, and the differences accumulate along a chain of
  poses.
- **Gimbal lock** exists in an Euler representation and not in a quaternion one.
  A port storing Euler angles would have singular configurations the reference
  does not.

The 11 tests in `golden_pose.rs` carry transform, inverse and composition
results straight from the reference. An Euler representation would not pass
them.

This is also where [ADR-0006](0006-quaternion-norm-f32.md) does its work: the
stored quaternion is normalized through `norm()`, which squares in `f32`. Both
decisions have to be right at once for pose composition to be bit-exact.

## Consequences

- `Pose6::to_euler()` calls `atan2` every time. Code that needs it repeatedly
  should cache the result.
- The API departs from the description in the port's original plan. That is
  deliberate, and it is recorded here so the difference is traceable.
- The general lesson, which applies across the project: **when the plan and the
  source disagree, the source wins** — and the difference is recorded, not left
  silent.
