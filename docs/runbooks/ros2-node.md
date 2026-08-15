# Runbook — building and running the ROS 2 node

**When to use it:** building a map from live sensor data, or feeding one to RViz
and other ROS consumers.

**Prerequisites:** ROS 2 (developed against **Jazzy**), Rust, and the toolchain
`r2r` needs to generate its bindings.

The full reference — every parameter, RViz, tuning — is in
[`../07-ros2.md`](../07-ros2.md). This page is the operational procedure only.

---

## 1. Prerequisites

```bash
sudo apt install -y clang libclang-dev python3-pip
sudo apt install -y ros-$ROS_DISTRO-octomap-msgs   # the message package — REQUIRED
sudo apt install -y ros-$ROS_DISTRO-octomap        # for the smoke test only
```

The C++ `octomap` package is **not** needed to run anything here. It is used by
the interoperability test, which decodes the node's output with the reference
library **on purpose**.

Check that the messages are installed **and loadable**:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 interface show octomap_msgs/msg/Octomap
ros2 topic pub -1 /probe octomap_msgs/msg/Octomap "{binary: true, id: OcTree}"
```

The second command matters more than the first. `ros2 interface show` only reads
a file; publishing **loads the typesupport library**, and that is where a
mismatched `octomap_msgs` build fails.

## 2. Build and run

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2/octomap_server_rs
cargo build --release
./target/release/octomap_server_rs --ros-args -r cloud_in:=/camera/depth/points
```

For a colcon deployment, see [`../07-ros2.md`](../07-ros2.md).

## 3. Test without hardware

```bash
bash scripts/ros2/smoke_test.sh
```

The script runs the node against a synthetic sensor, captures
`/octomap_binary`, then **decodes the payload with the C++ OctoMap library** and
asks it about voxels whose answers are already known (occupied / free /
unknown).

---

## Verification

The correct result, from the recorded run on ROS 2 Jazzy (WSL2, Ubuntu 24.04):

| | |
|---|---|
| The map | **637 nodes / 454 leaves** |
| Bounds | `[1, −0.5, 0] .. [3.1, 0.6, 1.1]` |
| Probe queries | **5 of 5 agreeing**, on both the binary and the full payload |
| RMW | Passing on **both Fast-RTPS and CycloneDDS** |

Those bounds match the synthetic wall geometry the script publishes. If the
numbers differ, do not move on to a real sensor before knowing why.

## If insertion cannot keep up with the sensor

The node **does not silently drop scans**. It integrates inline, and the
best-effort subscription lets the middleware drop what does not fit — the right
failure mode for a sensor, but it means an unconfigured node maps at a fraction
of the frame rate.

Four knobs, cheapest first:

1. **`point_stride`** — the most direct. `4` cuts the work to a quarter and
   costs almost nothing in sparse parts of the map.
2. **`max_range`** — ray length dominates the cost. Capping at 5 m on a sensor
   that reports 30 m is usually a large win for nothing.
3. **`resolution`** — halving it is an eight-fold change in node count.
4. **`publish_period`** — if the map is large, serialization is not free.

For context on the size of the problem: a 50k-point frame at 0.1 m does not fit
a 30 Hz budget in this port **or** in single-threaded C++. What the reference has
and this port does not is an OpenMP path. See
[`../05-regression-baselines.md`](../05-regression-baselines.md).

## If it fails

| Symptom | Cause | What to do |
|---|---|---|
| `ros2 topic pub` fails to load the typesupport | Mismatched `octomap_msgs` build | Reinstall `ros-$ROS_DISTRO-octomap-msgs`; see [`../07-ros2.md`](../07-ros2.md) |
| `r2r` fails to build | ROS not sourced, or libclang missing | `source /opt/ros/$ROS_DISTRO/setup.bash`; `apt install libclang-dev` |
| RViz shows nothing although the node is running | No Octomap display added, or the wrong frame | Add an Octomap display on `octomap_binary`; its QoS is `transient_local` |
| Free space below the sensor is wrong | `use_tf: false` without `sensor_transform` | Set `sensor_transform` to `[x, y, z, roll, pitch, yaw]` |
| Scans smear along the trajectory | TF does not interpolate in time | A known limitation — `tf.rs` keeps the latest transform per edge and ignores the cloud stamp |
| `ros2 param set` has no effect | Parameters are read once at start | Restart the node. Changing resolution or the sensor model midway corrupts the map |
| A `ColorOcTree` message is rejected | Not supported, in either direction | Rejected by its type name — deliberate, rather than silently decoded as a plain occupancy tree |
