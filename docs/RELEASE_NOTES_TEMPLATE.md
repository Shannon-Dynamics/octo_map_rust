# Release notes template

Copy the block below into a GitHub release, fill it in from the matching
[`../CHANGELOG.md`](../CHANGELOG.md) section, and delete every heading that has
nothing under it. An empty "Fixed" heading reads as though something was hidden.

Rules for filling it in:

- Say what a reader can now do that they could not before. Not what was
  refactored.
- No section about timing, throughput or benchmark results. Those baselines are
  maintainer tooling for spotting regressions, not something a release
  delivers.
- Do not claim an achievement the release does not contain.
- Link to the issue or PR where one exists.
- Migration notes only when something actually breaks.

---

```markdown
# octomap-core vX.Y.Z

## Highlights

One short paragraph: what this release gives someone who upgrades. If you
cannot write it without listing everything, the release probably has no theme,
and "maintenance release" is an honest thing to say.

## Added

- ...

## Changed

- ...

## Fixed

- ...

## Safety

- Anything affecting the memory-safety posture: unsafe policy, input
  validation, a panic path removed, a dependency added or dropped.

## Documentation

- ...

## Deprecated

- What is deprecated, what replaces it, and when it will be removed.

## Removed

- ...

## Migration notes

Only when something breaks. For each break: the old code, the new code, and
why the change was worth making.

## Known limitations

- Carry the still-true entries forward from the README's Current Limitations
  section. A release note that omits them reads as though they were fixed.

---

Install: `octomap-core = "X.Y"` · Docs: <https://docs.rs/octomap-core> ·
Full changelog: <https://github.com/Shannon-Dynamics/octo_map_rust/blob/main/CHANGELOG.md>

```
