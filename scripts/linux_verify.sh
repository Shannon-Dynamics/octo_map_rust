#!/usr/bin/env bash
#
# Runs the workspace test suite on x86_64-unknown-linux-gnu.
#
# The port is developed on Windows, and its verification methodology compares
# raw IEEE-754 bit patterns rather than tolerances. Basic arithmetic is
# guaranteed identical across platforms by IEEE-754, but the transcendental
# functions behind `logodds`/`probability` (log, exp) and the quaternion
# trigonometry are *not* — libm implementations may differ in the last bit.
# This script exists to turn that from an assumption into a measurement.
#
# Usage, from WSL or a Linux box:
#     bash scripts/linux_verify.sh [source-dir]
#
# From Windows, with <path> the checkout as WSL sees it:
#     MSYS_NO_PATHCONV=1 wsl -d <distro> -- \
#         bash /mnt/<path>/octo_map_rust/scripts/linux_verify.sh
#
# The source tree is copied to $HOME first: building in place under /mnt is
# slow, and it would collide with the Windows `target/` directory.
#
# No C++ toolchain is needed — the golden fixtures are committed. CMake and g++
# are only required to regenerate them (see scripts/README.md).

set -uo pipefail

export RUSTUP_HOME="${RUSTUP_HOME:-$HOME/.rustup}"
export CARGO_HOME="${CARGO_HOME:-$HOME/.cargo}"
export PATH="$CARGO_HOME/bin:$PATH"

SRC="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DST="$HOME/octomap-rs-linux-verify"

echo "=== environment ==="
uname -srm
ldd --version 2>/dev/null | head -1
rustc -vV | sed -n '1p;/^host/p'
echo

echo "=== syncing source from $SRC ==="
rm -rf "$DST"
mkdir -p "$DST"
tar cf - -C "$SRC" \
  --exclude=./target \
  --exclude=./build-cpp \
  --exclude=./reference-cpp \
  --exclude=./.git \
  . | tar xf - -C "$DST"

cd "$DST" || exit 1
echo "rust source files: $(find . -name '*.rs' | wc -l)"
echo "golden fixtures:   $(ls tests/golden | wc -l)"
echo

echo "=== cargo test --workspace ==="
cargo test --workspace 2>&1
test_status=${PIPESTATUS[0]}
echo "test exit status: $test_status"
echo

echo "=== cargo clippy --workspace --all-targets ==="
cargo clippy --workspace --all-targets 2>&1
clippy_status=${PIPESTATUS[0]}
echo "clippy exit status: $clippy_status"

if [ "$test_status" -eq 0 ] && [ "$clippy_status" -eq 0 ]; then
  echo
  echo "RESULT: clean on $(uname -m)"
  exit 0
fi
echo
echo "RESULT: failures present"
exit 1
