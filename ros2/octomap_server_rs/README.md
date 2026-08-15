# octomap_server_rs

A ROS 2 occupancy mapping node backed by
[octomap-core](../../crates/octomap-core/README.md), the pure-Rust port of
OctoMap. Same topics, services and parameter names as the C++ `octomap_server`,
with no C++ OctoMap in the build.

This binary is the one place in the repository with an FFI boundary: `r2r`
generates bindings against a C++ ROS 2 installation. The workspace-wide
`unsafe_code = "forbid"` covers the code here, not what `r2r` links — see
[`SAFETY.md`](../../SAFETY.md).

```bash
source /opt/ros/$ROS_DISTRO/setup.bash    # required before cargo, see below
cargo build --release
./target/release/octomap_server_rs --ros-args -r cloud_in:=/camera/depth/points
```

| | |
|---|---|
| In | `sensor_msgs/PointCloud2` on `cloud_in`, `/tf` |
| Out | `octomap_msgs/Octomap` on `octomap_binary` and `octomap_full`, plus markers and cell centers |
| Services | `octomap_binary`, `octomap_full` (`GetOctomap`), `~/reset` (`Empty`) |

**[docs/07-ros2.md](../../docs/07-ros2.md) is the full reference** — every parameter,
the colcon build, tuning, and troubleshooting. What follows is only what is
specific to this directory.

## ROS must be sourced before cargo

`r2r` generates Rust bindings for ROS message types in its build script, from
whatever it finds on `AMENT_PREFIX_PATH`. Building without sourcing produces a
binary with no message types; installing a new message package afterwards needs
`cargo clean -p r2r_msg_gen` before it is visible.

## Its own workspace

`Cargo.toml` declares an empty `[workspace]`, and the repository root excludes
this directory. That is because the node only builds where ROS is installed —
keeping it in the parent workspace would break `cargo test --workspace` on
every machine that has Rust but not ROS. The library it wraps has no such
constraint and stays in `crates/`.

## Layout

| File | |
|---|---|
| `src/main.rs` | Node setup, the `select!` loop that owns the map |
| `src/params.rs` | Parameters, defaults, and reporting the effective values |
| `src/publish.rs` | Building outgoing messages |
| `src/tf.rs` | A minimal `/tf` graph and lookup. **Read its module docs before trusting it on a fast robot** — it does not interpolate over time |

The conversions themselves are not here. `PointCloud2` decoding, scan
filtering, message payloads and voxel extraction live in
[`crates/octomap-ros`](../../crates/octomap-ros), which has no ROS dependency
and is unit tested. This binary moves fields between `r2r` structs and those
functions.
