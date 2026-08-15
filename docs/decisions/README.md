# Architecture Decision Records

Eleven decisions. **Seven of them are deliberate divergences from what a
sensible Rust design would do** — places where matching C++ was chosen over
tidier code. Each is commented at the point of definition as well, because
without an explanation they look like bugs.

The format is in [`_template.md`](_template.md).

| ADR | Title | Kind |
|---|---|---|
| [0001](0001-differential-bit-exact.md) | Correctness verified differentially, compared bit-exact | Methodology |
| [0002](0002-commit-fixtures-not-reference.md) | Fixtures are committed, the C++ source is not vendored | Methodology |
| [0003](0003-prune-stops-early.md) | `prune()` stops early, like the reference | **Divergence** |
| [0004](0004-raykeys-castray-narrowing.md) | `compute_ray_keys` and `cast_ray` disagree, like the reference | **Divergence** |
| [0005](0005-reciprocal-multiply.md) | Scaling multiplies by `1.0 / resolution` rather than dividing | **Divergence** |
| [0006](0006-quaternion-norm-f32.md) | `Quaternion::norm` squares in `f32` | **Divergence** |
| [0007](0007-resolution-six-digits.md) | Resolution is written with six significant digits | **Divergence** |
| [0008](0008-stricter-than-reference.md) | Two places are stricter than the reference | **Divergence** |
| [0009](0009-ros-split.md) | ROS integration split: conversions without ROS, node separate | Architecture |
| [0010](0010-lto-in-bench-profile.md) | `[profile.bench]` uses LTO and one codegen unit | Measurement |
| [0011](0011-pose6-stores-quaternion.md) | `Pose6` stores a quaternion, not Euler angles | **Divergence** |

## If you are tempted to "fix" one of them

Read the ADR first. Every divergence in that table **changes output relative to
the reference** if touched, and most of them fail non-uniformly — most tests stay
green and the difference only appears at particular geometries or resolutions.

The general rule: **when the plan and the C++ source disagree, the source
wins**, and the difference is recorded here.

The two exceptions are in [ADR-0008](0008-stricter-than-reference.md), where
following the reference would mean reproducing *undefined behaviour*. The next
deviation for that reason needs its own ADR.

## Decisions that do not stand alone as ADRs

Summarized here so they are not lost:

- **Targeting `x86_64-pc-windows-gnu` rather than MSVC** on Windows — the same
  GNU toolchain builds the C++ reference, so both sides of a differential
  comparison share an environment.
- **`unsafe_code = "forbid"` at the workspace level** — without it, a literal
  translation of the C++ pointer arithmetic would be the path of least
  resistance. The full policy is in [`../../SAFETY.md`](../../SAFETY.md).
- **`missing_docs = "warn"`, held at zero** — the whole public API is
  documented.
- **Big-endian is not supported** — node values are read and written
  little-endian unconditionally. The reference uses raw `memcpy`, so files from a
  big-endian build do not interoperate with a little-endian C++ build either.
