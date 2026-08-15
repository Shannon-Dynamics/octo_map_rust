# ADR-0004 — `compute_ray_keys` and `cast_ray` disagree, like the reference

- **Status:** Accepted
- **C++ source:** `OcTreeBaseImpl.hxx::computeRayKeys`, `OccupancyOcTreeBase.hxx::castRay`

## Context

Both functions set up the same Amanatides–Woo DDA. The setup is identical
**except for one line**:

```cpp
voxelBorder += (float) (step[i] * resolution * 0.5);   // computeRayKeys
voxelBorder += double(step[i] * resolution * 0.5);     // castRay
```

Narrowing to `float` in the first shifts `tMax` slightly, and that shift can
move a ray point **across a voxel boundary**. For the same ray, the two
functions can return different key sequences.

The sensible design: pick one, use it in both.

## Decision

Both are reproduced as found, each with a comment at the point of definition in
[`ray.rs`](../../crates/octomap-core/src/ray.rs).

## Evidence

`golden_ray.rs` carries 12 ray shapes from `computeRayKeys` — one of them
spanning **653 voxels** — and 8 `castRay` cases, all from the reference.
Unifying the two paths would make one of those groups fail, and it would fail
non-uniformly: only rays that happen to pass close to a voxel boundary change.

That failure shape is exactly why this divergence is worth recording. If someone
"tidies up" this one line, most tests stay green and the difference only appears
at particular geometries.

## Consequences

- Code that compares `compute_ray_keys` output with a `cast_ray` trace for the
  same ray **must not assume they are identical**. That is true in the reference
  too.
- Both spellings are kept until upstream unifies them. If that happens, the
  fixtures are regenerated and this ADR is marked superseded.
- This is a reproduced C++-against-itself divergence, not a Rust-against-C++
  one. The port adds no inconsistency of its own.
