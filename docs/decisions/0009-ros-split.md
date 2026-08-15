# ADR-0009 — ROS integration split: conversions without ROS, node separate

- **Status:** Accepted

## Context

The map needs to be buildable from live sensor data and publishable to a ROS 2
graph. The most direct route: one crate that depends on a ROS client library,
uses the generated message types, and does everything.

The consequence: that crate only builds on a machine with ROS sourced —
including for testing `PointCloud2` decoding logic, which needs no middleware at
all.

## Decision

Split in two, and **the boundary is the dependency, not the concept**:

| | Contents | Needs ROS |
|---|---|---|
| `crates/octomap-ros` | Conversions: `PointCloud2` decoding, scan filtering, `octomap_msgs` payloads, marker extraction | **No** |
| `ros2/octomap_server_rs` | The mapping node on top of it, using `r2r` | Yes |

`octomap-ros` **never touches a generated message type**. It takes the raw data
a message carries — a byte blob, a list of field descriptors, a resolution — and
returns raw data too.

`ros2/octomap_server_rs` is `exclude`d from the root workspace.

## Evidence

- `octomap-ros` runs under `cargo test --workspace` on a machine **without
  ROS**, Windows and CI included. **47 tests.**
- Which ROS client library is used stays the caller's decision: anyone who wants
  an octree in their own node depends on `octomap-ros` and skips
  `octomap_server_rs` entirely.
- If the node were in the root workspace, `cargo test --workspace` would fail on
  every machine with Rust but no ROS — including the one this port was developed
  on. `r2r` builds its message bindings from `AMENT_PREFIX_PATH` **at compile
  time**, so this is not something a feature flag can work around.

What is left in the node really is thin: subscriptions, a timer, parameters, TF,
publishing. That is what `scripts/ros2/smoke_test.sh` tests, decoding the node's
output with the **C++ OctoMap library** — 637 nodes / 454 leaves, five queries
agreeing, passing on both Fast-RTPS and CycloneDDS.

## Consequences

- There are two `Cargo.lock` files. That is the ordinary consequence of two
  workspaces.
- CI does not build the node ([`ci.yml`](../../.github/workflows/ci.yml)
  explains why in a comment): installing ROS 2 in CI to test a thin transport
  layer whose logic `octomap-ros` already covers is not worth it.
- `octomap-core` gained `write_binary_data` / `read_binary_data` /
  `write_full_data` / `read_full_data` because of this split — the
  **headerless** payloads `octomap_msgs` carries, which are not the contents of
  a `.bt`/`.ot` file.
- **`octomap-ros` must never gain a ROS dependency.** The moment it has one it
  stops being testable on a machine without ROS, and the whole reason for the
  split is gone.
