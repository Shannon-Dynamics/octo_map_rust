# ADR-0008 — Two places are stricter than the reference

- **Status:** Accepted

## Context

Every other divergence ADR in this folder chooses to **follow** the reference
even where its spelling is odd. Two places do not, because following it would
mean reproducing *undefined behaviour* rather than behaviour.

Both are paths where C++ relies on an `assert` or on unbounded recursion —
neither of which protects anything in a release build.

## Decision

Two deviations, both of which **narrow the input that is accepted** and neither
of which changes the result for valid input:

### 1. `KeyRay` stops at its limit

`KEY_RAY_MAX_SIZE = 100_000`. When a ray exceeds it, this port **stops adding
keys**. C++ uses an `assert`, which disappears in an `NDEBUG` build — and the
write goes past the end of the buffer.

### 2. The file parser rejects excessive nesting

The `.bt`/`.ot` parser rejects a payload nested deeper than the tree depth
rather than recursing until the stack runs out. There is a **fuzz test** for it.

## Evidence

Both can only be triggered by input that is **outside the valid range**: a ray
longer than 100,000 voxels, and a file that is corrupt or maliciously
constructed. For every input the fixtures use — including the 653-voxel ray in
`golden_ray.rs` — the behaviour is identical to the reference.

The consequence of not doing this is not "a slightly different result" but a
buffer overflow and stack exhaustion. For a crate with `unsafe` `forbid`-ed,
reproducing either one would be the one way to lose that guarantee through the
back door. See [`../../SAFETY.md`](../../SAFETY.md).

## Consequences

- A very long ray is silently truncated at 100,000 keys rather than exploding.
  If that ever needs reporting as an error, that is an API change — not a change
  in behaviour relative to the reference.
- A corrupt file produces an `IoError`, not a crash. The parser returns a
  `Result`; it does not panic.
- **These are the only two places** permitted to deviate for safety reasons. The
  next one needs its own ADR, and the burden is on whoever proposes it.
