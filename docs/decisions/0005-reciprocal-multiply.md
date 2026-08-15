# ADR-0005 — Scaling multiplies by `1.0 / resolution` rather than dividing

- **Status:** Accepted
- **C++ source:** `OcTreeBaseImpl.hxx::coordToKey`, using a cached `resolution_factor`

## Context

`coordToKey` converts a world coordinate into a voxel index. The obvious
spelling:

```rust
let scaled = coord / resolution;
```

The reference does not write it that way. It caches `1.0 / resolution` once and
**multiplies**.

This port's first implementation divided. The differential tests failed.

## Decision

Multiply by the cached reciprocal, exactly as the reference does. Commented in
[`geometry.rs`](../../crates/octomap-core/src/geometry.rs).

## Evidence

In IEEE-754 these are **not the same operation**:

```
1.2 / 0.1        == 11.999999999999998   → floor 11
1.2 * (1.0/0.1)  == 12.0                 → floor 12
```

Dividing puts that point in the **wrong voxel**. Not approximately wrong — on
the other side of the boundary.

This is also the clearest illustration of why bit-exact comparison was chosen
([ADR-0001](0001-differential-bit-exact.md)): the difference between `1.2/0.1`
and `12.0` is about 1.8e-15, far inside any reasonable epsilon, and the
consequence is a voxel shifted by one. A tolerance would have let it through.

What was wrong in this case was not the code but the author's expectation. The
test that found it compares against the reference, not against an expectation.

## Consequences

- `TreeGeometry` stores the reciprocal factor as a field. Replacing it with a
  division "for clarity" would move some points by one voxel and break **938
  rows** of geometry fixtures.
- It applies to every conversion variant, including `at_depth` and `checked`.
- If the resolution were changeable at runtime the factor would have to be
  recomputed with it — in this port the resolution is fixed at construction, so
  there is no path for that.
