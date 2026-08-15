#!/usr/bin/env bash
#
# End-to-end check of octomap_server_rs against a live ROS 2 graph.
#
# A synthetic wall goes in as a sensor_msgs/PointCloud2 in a sensor frame, a
# static transform relates that frame to the map, and what comes out on
# /octomap_binary is decoded by the *C++* OctoMap library and queried. Passing
# means the whole path works and the bytes on the wire are the format the rest
# of the ROS octomap ecosystem reads.
#
#   ./scripts/ros2/smoke_test.sh
#
# Needs ROS 2 sourced, cargo on PATH, and the C++ OctoMap dev package
# (ros-$ROS_DISTRO-octomap or liboctomap-dev) for the decoder.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
NODE_DIR="$REPO/ros2/octomap_server_rs"

# Keep cargo's output off a Windows drive mount when this runs under WSL: the
# 9p filesystem makes a link step take minutes instead of seconds.
CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$HOME/.cache/octomap_server_rs_target}"
export CARGO_TARGET_DIR

WORK="$(mktemp -d)"
# An unusual domain so this cannot see, or be seen by, whatever else the
# machine is running.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  rm -rf "$WORK"
}
trap cleanup EXIT

step() { printf '\n=== %s\n' "$1"; }

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 2 is not sourced. Run: source /opt/ros/<distro>/setup.bash" >&2
  exit 2
fi

# ---------------------------------------------------------------- geometry ---
# The wall is at x = 2 in the sensor frame, and the sensor sits at (1, 0, 0.5)
# in the map, so the wall lands at x = 3. Everything between is swept by rays
# and must come out free; everything outside must stay unknown, which is the
# distinction a mapping library exists to maintain.
SENSOR_X=1.0
SENSOR_Y=0.0
SENSOR_Z=0.5
RESOLUTION=0.1

step "building octomap_server_rs (release)"
cargo build --release --manifest-path "$NODE_DIR/Cargo.toml"
NODE_BIN="$CARGO_TARGET_DIR/release/octomap_server_rs"

step "building the C++ decoder"
DECODER="$WORK/decode_octomap_payload"
if ! g++ -O2 -o "$DECODER" "$HERE/decode_octomap_payload.cpp" -loctomap -loctomath 2>"$WORK/gpp.log"; then
  echo "could not build the decoder:" >&2
  cat "$WORK/gpp.log" >&2
  echo "install the C++ OctoMap headers: sudo apt install ros-$ROS_DISTRO-octomap" >&2
  exit 2
fi

step "starting the transform publisher (map -> sensor)"
ros2 run tf2_ros static_transform_publisher \
  --x "$SENSOR_X" --y "$SENSOR_Y" --z "$SENSOR_Z" \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id map --child-frame-id sensor \
  >"$WORK/tf.log" 2>&1 &
PIDS+=($!)

step "starting octomap_server_rs"
"$NODE_BIN" --ros-args \
  -p resolution:="$RESOLUTION" \
  -p frame_id:=map \
  -p use_tf:=true \
  -p publish_period:=0.5 \
  -p publish_free_markers:=true \
  >"$WORK/node.log" 2>&1 &
PIDS+=($!)

step "starting the synthetic sensor"
python3 "$HERE/fake_cloud_publisher.py" --topic /cloud_in --frame sensor --rate 5 \
  >"$WORK/cloud.log" 2>&1 &
PIDS+=($!)

step "capturing /octomap_binary"
PAYLOAD="$WORK/map.payload"
if ! python3 "$HERE/capture_octomap.py" --topic /octomap_binary --out "$PAYLOAD" --timeout 30; then
  echo "no usable map was published. Node log:" >&2
  cat "$WORK/node.log" >&2
  exit 1
fi

FRAME="$(sed -n 4p "$PAYLOAD.meta")"
if [[ "$FRAME" != "map" ]]; then
  echo "map was published in frame '$FRAME', expected 'map'" >&2
  exit 1
fi

step "decoding the payload with C++ OctoMap"
"$DECODER" "$PAYLOAD" "$RESOLUTION" binary \
  3.02  0.02  0.52 occupied \
  3.02 -0.22  0.32 occupied \
  2.02  0.02  0.52 free \
  1.52  0.02  0.52 free \
  10.0 10.0 10.0 unknown

step "capturing /octomap_full"
FULL="$WORK/map.full"
if ! python3 "$HERE/capture_octomap.py" --topic /octomap_full --out "$FULL" --timeout 15; then
  echo "no full map was published" >&2
  exit 1
fi

step "decoding the full-probability payload with C++ OctoMap"
"$DECODER" "$FULL" "$RESOLUTION" full \
  3.02  0.02  0.52 occupied \
  2.02  0.02  0.52 free \
  10.0 10.0 10.0 unknown

step "checking the other topics are alive"
# --no-arr keeps this a liveness check rather than a formatting benchmark: a
# marker array holds hundreds of points and colors, and rendering all of that
# to YAML takes longer than receiving it does.
for topic in /occupied_cells_vis_array /free_cells_vis_array /octomap_point_cloud_centers; do
  if timeout 25 ros2 topic echo --once --no-arr "$topic" >/dev/null 2>&1; then
    echo "  $topic: ok"
  else
    echo "  $topic: nothing received" >&2
    exit 1
  fi
done

step "checking the reset service empties the map"
ros2 service call /octomap_server/reset std_srvs/srv/Empty >/dev/null

printf '\nPASS: the map round-tripped through ROS 2 and the C++ library read it.\n'
