# Runbook — verifying on Linux

**When to use it:** after touching anything that calls a transcendental function
— `logodds`/`probability` in `occupancy.rs`, or the trigonometry in `pose.rs` —
and before a release.

**Prerequisites:** Rust on Linux (WSL2 is enough). **No C++ toolchain needed** —
the fixtures are committed.

---

## Why this exists

This project's method compares **raw IEEE-754 bit patterns**
([ADR-0001](../decisions/0001-differential-bit-exact.md)). Basic arithmetic
(`+ − × ÷`, `floor`, comparison) is guaranteed identical across platforms by
IEEE-754. **Transcendental functions are not.**

`log`/`exp` in `logodds`/`probability` and the trigonometry in `Quaternion`
depend on the `libm` implementation, and MinGW (Windows) is not obliged to
produce the same last bit as glibc (Linux).

If they differed, the failure would have a specific, recognizable shape:
`golden_occupancy.rs` and `golden_pose.rs` crack while `golden_geometry.rs` and
`golden_tree.rs` stay green.

## Steps

```bash
bash scripts/linux_verify.sh
```

The script copies the source to `$HOME` first — building under `/mnt` is slow
and collides with the Windows `target/` — then runs the tests and clippy.

Manually, to see each step:

```bash
cp -r /mnt/<path to the checkout>/octo_map_rust ~/octomap-verify
cd ~/octomap-verify
cargo test --workspace
cargo clippy --workspace --all-targets
```

## Verification

| Platform | Toolchain | Correct result |
|---|---|---|
| Windows 11 x86-64 | `x86_64-pc-windows-gnu`, MinGW libm | Full suite passing, clippy clean |
| Ubuntu 24.04 x86-64 (WSL2) | `x86_64-unknown-linux-gnu`, glibc 2.39 | Full suite passing, clippy clean |

**The `libm` hypothesis did not materialise.** The two riskiest suites pass
bit-exact on both — `log`, `exp`, `sin`, `cos` and `atan2` return identical
results for every input the fixtures use.

That is **not guaranteed by any standard**; it simply holds for these inputs on
these two libm implementations.

## If `golden_occupancy` or `golden_pose` fails here

Do not assume the port is wrong. First check whether the failure matches the
`libm` signature:

1. `golden_geometry.rs` and `golden_tree.rs` are **green** — they touch no
   transcendental function.
2. The failures differ by **one or two ULP**, not by a large margin.

If both hold, this is a `libm` divergence, not a port bug. The action: move
those two suites to a small ULP tolerance, **record it as a deliberate
divergence with a new ADR**, and name the platforms affected.

If either does not hold — `golden_tree.rs` failing too, or a large difference —
it is an ordinary bug and is treated as one.

## If it fails for another reason

| Symptom | Cause | What to do |
|---|---|---|
| The build is very slow | Run directly under `/mnt/...` | Copy to `$HOME` first — that is what the script does |
| `target/` collides with the Windows build | The target directory is shared | Copy to `$HOME`, or set a separate `CARGO_TARGET_DIR` |
| The same test fails on both platforms | Not a `libm` issue | An ordinary bug; treat it as one |
