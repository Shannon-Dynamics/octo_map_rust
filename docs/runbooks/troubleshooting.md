# Runbook — troubleshooting

Grouped by symptom. Most of what fails in this repository fails in a way that
does not point at its cause.

---

## A differential test fails

**Do not widen the comparison to a tolerance.** That erases the reason the test
exists ([ADR-0001](../decisions/0001-differential-bit-exact.md)).

Diagnose by which suite failed:

| What failed | Likely cause |
|---|---|
| `golden_geometry.rs` | coord ↔ key conversion. Check whether the reciprocal multiply became a division — [ADR-0005](../decisions/0005-reciprocal-multiply.md) |
| `golden_tree.rs` | Tree structure. Check `prune()` — [ADR-0003](../decisions/0003-prune-stops-early.md) |
| `golden_ray.rs`, only some rays | The `compute_ray_keys` vs `cast_ray` narrowing — [ADR-0004](../decisions/0004-raykeys-castray-narrowing.md) |
| `golden_pose.rs`, off by one ULP | `Quaternion::norm` — [ADR-0006](../decisions/0006-quaternion-norm-f32.md) |
| `golden_occupancy.rs` **and** `golden_pose.rs`, with `geometry`/`tree` green | A `libm` divergence. See [`linux-verify.md`](linux-verify.md) |
| `interop_io.rs`, bytes differ in the header | Resolution precision — [ADR-0007](../decisions/0007-resolution-six-digits.md) |
| `interop_io.rs`, whole files differ | Fixtures checked out with CRLF. `.gitattributes` forces `eol=lf` |

The failure pattern is informative: a `libm` divergence touches exactly two
suites and leaves the other two green. If `golden_tree.rs` fails too, it is not
`libm`.

## Nobody can decode the ROS 2 message

Almost always one cause: **the wrong payload**.

`octomap_msgs/Octomap` carries the **headerless** node payload in its `data`
field — not the contents of a `.bt`/`.ot` file.

| Wrong | Right |
|---|---|
| `write_binary_file` / `write_binary` | `write_binary_data` |
| `write_full_file` / `write_full` | `write_full_data` |

The symptom never mentions a header: RViz stays silent, or a consumer throws a
parse error pointing at an arbitrary offset.

## The ROS 2 build fails

| Symptom | Cause | What to do |
|---|---|---|
| `r2r` cannot generate bindings | `AMENT_PREFIX_PATH` is empty | `source /opt/ros/$ROS_DISTRO/setup.bash` **before** `cargo build` |
| `libclang not found` | Incomplete bindgen toolchain | `apt install clang libclang-dev` |
| `cargo test --workspace` at the root tries to build the node | The node ended up in `members` | It must stay in `exclude` — [ADR-0009](../decisions/0009-ros-split.md) |
| `ros2 topic pub` fails to load the typesupport | Mismatched `octomap_msgs` build | Reinstall the message package; `ros2 interface show` will not catch this |

## The benchmark numbers make no sense

| Symptom | Cause |
|---|---|
| Rust roughly 2× slower on insertion | LTO is not active. Use `cargo bench`, not `cargo run --release` — [ADR-0010](../decisions/0010-lto-in-bench-profile.md) |
| Numbers do not match the recorded ones | Reading criterion's **mean** off the screen; the documents use the **median** from `estimates.json` |
| The node counts differ between sides | **This is a correctness problem, not a timing one.** Stop measuring and run `cargo test --workspace` |
| The checksums differ between sides | The scene was regenerated between the two runs |

Details in [`benchmark.md`](benchmark.md).

## Fixture regeneration

| Symptom | Cause |
|---|---|
| Fixtures change on many rows after a fresh clone | The reference is not 1.10.0 — `--depth 1` fetches HEAD, not the `f012f5f` tag |
| `undefined reference to octomath::...` | Library order: `-loctomap` must come before `-loctomath` |
| Reference debug output lands in the CSV | The `io` and `pose` generators take an **output path**, not a `>` redirect |

## Everything else

| Symptom | Cause | What to do |
|---|---|---|
| The Linux build is very slow | Run under `/mnt/...` | Copy to `$HOME` first — `linux_verify.sh` does this |
| `target/` collides between Windows and WSL | A shared target directory | Set a separate `CARGO_TARGET_DIR`, or copy the source |
| `cargo test` seems to want a C++ toolchain | A misunderstanding | It does not need one. The fixtures are committed — [ADR-0002](../decisions/0002-commit-fixtures-not-reference.md) |
| `tests/bench/scene.txt` is missing | Gitignored, ~1.9 MB, deterministic | `cargo run --release --example dump_bench_fixture` |
| A query returns `None` where the space is clearly empty | `None` means **unknown**, not free | That is correct behaviour. Space no ray ever crossed is unknown |
