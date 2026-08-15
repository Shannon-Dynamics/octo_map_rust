# ADR-0003 — `prune()` stops early, like the reference

- **Status:** Accepted
- **C++ source:** `OcTreeBaseImpl.hxx`, `prune()` — the reference marks it `FIXME`

## Context

`prune()` sweeps the tree from the deepest level upward, merging nodes whose
eight children are identical.

The reference's sweep **stops at the first level that merges nothing**. As a
result a partially pruned tree can leave a level above it that is still
collapsible — the pruned result depends on history, not only on the tree's
contents.

C++ itself marks this with a `FIXME` comment.

## Decision

Reproduced as found, and commented at the point of definition in
[`tree.rs`](../../crates/octomap-core/src/tree.rs).

## Evidence

`golden_tree.rs` carries node counts and iteration order after pruning, straight
from the reference. Fixing the sweep to cover every level would change those
numbers — **the differential tree tests would fail** — and the resulting
`.bt`/`.ot` files would stop being byte-identical to C++'s output for the same
tree.

Byte-identical file compatibility is one of this project's core claims
([`../03-verification.md`](../03-verification.md)), and it is what gets
sacrificed if pruning is unilaterally improved.

## Consequences

- Trees here can use more nodes than they theoretically need. For the workloads
  in [`../05-regression-baselines.md`](../05-regression-baselines.md) the effect was not
  measurable.
- If upstream fixes its `FIXME`, this port follows — the fixtures are
  regenerated from the new version and the reference commit named in
  [`../README.md`](../README.md) is updated with them.
- **Do not "clean this up".** It looks like a bug and it behaves like one; it is
  here so that the port and the reference produce the same tree.
