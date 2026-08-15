# Releasing

The process for cutting a release of `octomap-core` and `octomap-ros`.

**Publication is an owner action.** Nothing in this document should be run by
anyone who is not the repository owner, and steps 10 onward are irreversible: a
version published to crates.io cannot be replaced, only yanked. Read the whole
list before starting.

## Naming

Two crates share one repository, so tags name the crate as well as the version:

```text
octomap-core-v0.1.0
octomap-ros-v0.1.0
```

The crates share a version through `[workspace.package]` and are released
together, so in practice both tags point at the same commit. Naming them
separately keeps the option of releasing one alone later without renaming
anything.

## Before you start

- You are on `main`, up to date with the remote.
- CI is green on the commit you intend to release.
- You know which version you are releasing and why — see
  [Versioning](#versioning) below.
- For the first-ever publication: the crate names are still free. A search
  returns nothing for either name, so they appear unregistered — but a name can
  be taken between then and now, and it is the one thing that can invalidate
  the whole plan. Re-check immediately before publishing:

  ```bash
  cargo search octomap-core
  cargo search octomap-ros
  ```

  If either is taken, the fallback is a prefix that is unambiguous and still
  descriptive — `octomap-rs-core` / `octomap-rs-ros`. Renaming means editing
  the manifests, the `use` lines in the examples and documentation, and the
  crate READMEs; nothing about the API changes.

## The checklist

### 1. Working tree is clean

```bash
git status --porcelain      # must print nothing
```

`cargo package` refuses to run on a dirty tree without `--allow-dirty`, and
passing that flag is how a release ends up containing a file nobody meant to
ship.

### 2. Format

```bash
cargo fmt --all -- --check
```

### 3. Clippy

```bash
cargo clippy --workspace --all-targets -- -D warnings
```

### 4. Tests

```bash
cargo test --workspace
```

All of them, on the platform you are releasing from. If the change touched
`log`, `exp` or trigonometry, also run the Linux verification —
[`runbooks/linux-verify.md`](runbooks/linux-verify.md) — because the bit-exact
comparison depends on `libm` agreeing across platforms.

### 5. Documentation builds clean

```bash
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
```

Warnings denied, because a broken intra-doc link becomes a broken link on
docs.rs and there is no fixing it after publication without a new version.

### 6. Security checks

```bash
cargo audit
cargo deny check
```

`cargo audit` covers RustSec advisories across the dependency tree, including
dev-dependencies. `cargo deny check` additionally covers licences, crate
sources and duplicate versions, against [`../deny.toml`](../deny.toml).

For a release that touched parsing, geometry or anything else in the core, also
run the undefined-behaviour check:

```bash
cargo +nightly miri test -p octomap-core --lib
MIRIFLAGS="-Zmiri-strict-provenance" cargo +nightly miri test -p octomap-ros
```

Miri is slow — minutes, not seconds — which is why it is a release-time check
rather than a per-push one.

### 7. Verify what will be packaged

```bash
cargo package --list -p octomap-core
cargo package --list -p octomap-ros
```

Read the list. Every file should be one you meant to ship. Watch for:

- generated or temporary files that escaped `.gitignore`;
- large fixtures or datasets;
- local configuration;
- `LICENSE` and `NOTICE` — both **must** be present in each crate. They are
  copies of the repository root files, kept inside the crate directories
  precisely so that they end up here.

`tests/` is excluded from `octomap-core` on purpose: the differential fixtures
live outside the package, so shipping the test files would ship code that cannot
compile once unpacked.

Two expected warnings on `octomap-core`, one per excluded test file:

```text
warning: ignoring test `golden_geometry` as `tests\golden_geometry.rs` is not
         included in the published package
```

That is the `exclude` doing its job, not a problem to fix.

### 8. Build the package

```bash
cargo package -p octomap-core
cargo package -p octomap-ros    # first release: see the note below
```

This unpacks each crate into `target/package/` and builds it there, which is the
step that catches a file the crate needs but does not include.

### 9. Dry run

```bash
cargo publish --dry-run -p octomap-core
cargo publish --dry-run -p octomap-ros    # first release: see the note below
```

**On a first release, both commands fail for `octomap-ros`, and that is
expected.** It depends on `octomap-core` by `path` *and* `version`; packaging
resolves the `version` against the registry, and before `octomap-core` 0.1.0
exists there the resolution cannot succeed:

```text
error: failed to prepare local package for uploading
Caused by: no matching package named `octomap-core` found
```

It clears itself once step 10 has published `octomap-core`. Until then, verify
`octomap-ros` with `cargo package --list -p octomap-ros`, which does not
resolve.

### 10. Update the changelog and the version

- Move everything under `## [Unreleased]` into a new section headed with the
  version and the release date, in all three changelogs: the root
  [`../CHANGELOG.md`](../CHANGELOG.md) and the per-crate
  [`../crates/octomap-core/CHANGELOG.md`](../crates/octomap-core/CHANGELOG.md)
  and [`../crates/octomap-ros/CHANGELOG.md`](../crates/octomap-ros/CHANGELOG.md).
  A release date *is* worth keeping; development dates are not.
- Leave an empty `## [Unreleased]` behind in each.
- Bump `version` in the root `Cargo.toml` under `[workspace.package]`. Both
  crates inherit it.
- Bump the `version` in `octomap-ros`'s dependency on `octomap-core` to match.
- Commit: `git commit -am "Release 0.1.0"`.

### 11. Publish

Order matters — `octomap-ros` cannot resolve until `octomap-core` is on the
registry:

```bash
cargo publish -p octomap-core
# wait for the index to update, usually under a minute
cargo publish -p octomap-ros
```

### 12. Tag

```bash
git tag -a octomap-core-v0.1.0 -m "octomap-core 0.1.0"
git tag -a octomap-ros-v0.1.0  -m "octomap-ros 0.1.0"
git push origin main --follow-tags
```

Tag after publishing, not before: if the publish fails you want to fix it and
try again without a tag already claiming it happened.

### 13. GitHub release

Create a release against the tag. Use the template in
[`RELEASE_NOTES_TEMPLATE.md`](RELEASE_NOTES_TEMPLATE.md), filled in from the
changelog section you just wrote.

### 14. Verify docs.rs

docs.rs builds within a few minutes of publication. Check
<https://docs.rs/octomap-core> for a successful build, and specifically:

- the front page renders the crate README;
- the doc-tests in it are the ones CI ran;
- no intra-doc link is broken.

A failed docs.rs build cannot be retried by pushing — it needs a new version.

### 15. Afterwards

- Add the crates.io and docs.rs badges to the README. They are deliberately
  absent until they point at something real.
- Update the [`../ROADMAP.md`](../ROADMAP.md) phase checkboxes.
- Enable private vulnerability reporting in the repository settings if it is
  not on yet, so the route [`../SECURITY.md`](../SECURITY.md) describes exists.
- Update the installation instructions: the "current development usage" git
  dependency becomes a fallback, and `octomap-core = "0.1"` becomes the default.

## Versioning

[Semantic versioning](https://semver.org/), with the 0.x convention: while the
major version is zero, **the minor version is the breaking one**. `0.1.x` → `0.2.0`
may break; `0.1.0` → `0.1.1` may not.

Both crates share a version through `[workspace.package]`. They are released
together, which keeps `octomap-ros`'s dependency on `octomap-core` trivial to
reason about at the cost of occasionally publishing a crate that did not change.

Breaking, for this library, includes:

- removing or renaming a public item;
- changing a function signature, including argument order or a `&self` → `&mut
  self` change;
- adding a variant to an enum a caller can match exhaustively;
- changing the output of file writing, since files are an interface too;
- raising the MSRV.

Not breaking:

- adding a new public item;
- adding a `#[non_exhaustive]` variant;
- changing internal representation, timing, or memory use;
- documentation.

## If a release goes wrong

- **Published something broken:** `cargo yank --vers 0.1.0 -p octomap-core`. A
  yank stops new dependants from selecting that version but does not remove it —
  anything with it in a lockfile keeps building. Then fix and publish a patch.
- **Published with a missing file:** same thing. You cannot re-upload a version.
- **Wrong tag:** delete and re-tag before anyone fetches. After that, add a new
  tag rather than moving an old one.
