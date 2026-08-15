## What changed

<!-- One paragraph. -->

## Why

<!-- Link the ADR if this changes a deliberate divergence from the C++
     reference, the issue otherwise. -->

## Does this change observable behavior relative to OctoMap C++?

- [ ] **No** — the differential suites still pass unchanged
- [ ] **Yes**, and it is recorded as a new ADR in `docs/decisions/`, commented at
      the point of definition, and the fixtures were regenerated from a real C++
      binary (never hand-edited)

The differential tests compare floating point with `==` on purpose. If one
fails, the port is wrong until proven otherwise — do not widen the comparison to
a tolerance.

## Checklist

- [ ] `cargo test --workspace --all-features` passes
- [ ] `cargo clippy --workspace --all-targets --all-features -- -D warnings` is clean
- [ ] `cargo fmt --all -- --check` is clean
- [ ] `cargo doc --workspace --no-deps` emits no warnings
- [ ] No `unsafe` added (it is `forbid`-ed at workspace level — see `SAFETY.md`)
- [ ] No new runtime dependency on `octomap-core`
- [ ] `octomap-ros` still builds on a machine without ROS
- [ ] Public API changes are documented, with a doc-test where an example helps
- [ ] `CHANGELOG.md` updated under `## [Unreleased]`

## Hot paths

<!-- If this touches insertion, ray traversal or queries, re-run
     `cargo bench --bench insert_point_cloud` and confirm the medians did not
     regress. Those baselines are a regression tool, not a target — see
     docs/05-regression-baselines.md. Note here what you observed. -->
