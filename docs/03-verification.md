# 3. Verification

This is the single decision that shaped the whole project.

Behavioural compatibility with C++ is the **point** of a port, so it is
measured, not assumed. The C++ reference is built, driven through the same
inputs, its answers are captured as fixtures, and Rust is compared against them.

---

## 3.1 Bit-exact comparison, not a tolerance

Floating point is compared as **raw IEEE-754 bit patterns**, with `==`.

The reason is not bravado: both implementations perform the same sequence of
operations on the same data, so the results are **bit-identical if the port is
correct**. Anything short of bit equality is a real divergence.

A tolerance would hide precisely the class of bug these tests exist to catch.
That is not a theoretical worry — all three bugs in
[§3.4](#34-three-bugs-that-only-this-caught) would have passed any epsilon you
care to pick.

Every value is printed with `%.17g`, which round-trips a `double` exactly.

## 3.2 What is pinned against C++

**284 tests** across the workspace: 206 in-module unit tests, 63 differential,
12 parser-robustness, 3 doc-tests.

| Suite | Count | What it pins |
|---|---:|---|
| `golden_geometry.rs` | 7 | 938 rows: coord ↔ key conversions, depth variants, bounds checks, node sizes, sensor defaults |
| `golden_tree.rs` | 9 | Node counts, leaf and tree iteration order with keys and depths, prune, delete, depth-limited views |
| `golden_occupancy.rs` | 12 | 43 sequential updates bit-for-bit, clamping, auto-prune, block reopen, change detection |
| `golden_ray.rs` | 15 | DDA key sequences for 12 ray shapes (one spanning 653 voxels), 8 ray-cast cases, point-cloud integration |
| `golden_pose.rs` | 11 | Euler ↔ quaternion, rotation, axis-angle, pose transform / inverse / composition |
| `interop_io.rs` | 9 | Byte-identical `.ot` and `.bt` output, plus decoding files the reference wrote |
| In-module unit tests | 206 | Per-module behaviour, edge cases, error handling, parser fuzzing |
| Doc-tests | 3 | The examples in each crate's documentation |

## 3.3 File interoperability — both directions

1. Rust **reads** C++-written `.bt` and `.ot` → the contents match the CSV
   fixtures.
2. Rust's output is **byte-for-byte identical** to C++'s output, for both
   formats.
3. `scripts/verify_rust_io.cpp` hands Rust-written files to C++ — they load,
   occupancy matches on **99 of 99 leaves**, and the C++ rewrite is
   byte-identical.

Point 2 is stronger than "C++ can parse it": identical files cannot decode
differently. The practical consequence is that `cargo test` demonstrates interop
**with no C++ toolchain at all**.

## 3.4 Three bugs that only this caught

All of them differ by one ULP or less. All of them would have passed any
reasonable tolerance.

### 1. The reciprocal trap

`coordToKey` multiplies by a cached `1.0 / resolution` — it does not divide by
`resolution`. In IEEE-754 those are not the same operation:

```
1.2 / 0.1        == 11.999999999999998   → floor 11
1.2 * (1.0/0.1)  == 12.0                 → floor 12
```

Dividing puts the point on the **wrong side of a voxel boundary**. Found through
a failing test — where the author's expectation was wrong, not the code.
[ADR-0005](decisions/0005-reciprocal-multiply.md).

### 2. One ULP in `Quaternion::norm`

C++ squares the components in `float` and accumulates into `double`. The first
implementation promoted to `f64` before squaring. The result shifts by **exactly
one ULP**, and that propagates through `normalized()` into every composed pose.
Three differential tests failed on it.
[ADR-0006](decisions/0006-quaternion-norm-f32.md).

### 3. `updateNode` silently auto-prunes

It calls `pruneNode` on the way back up the recursion. Used to generate
**structural** fixtures, the result reflects occupancy behaviour rather than
generic insertion — so the generator has to use
`setNodeValue(..., lazy_eval = true)` instead.

The C++ data also exposed two mistakes in the hand-written test set: a duplicate
key (42 keys → 41 leaves) and a block assumed to be siblings that was not. Both
were kept as edge cases.

## 3.5 Cross-platform verification

The bit-exact method puts this port at the mercy of `libm`. Basic arithmetic
(`+ − × ÷`, `floor`, comparison) is guaranteed identical across platforms by
IEEE-754 — **but transcendental functions are not**. `log`/`exp` in
`logodds`/`probability` and the trigonometry in `Quaternion` depend on the
`libm` implementation, and MinGW is not obliged to produce the same last bit as
glibc.

That is not a theoretical risk: if they differed, `golden_occupancy.rs` and
`golden_pose.rs` would crack while `golden_geometry.rs` and `golden_tree.rs`
stayed green. So it was measured.

| Platform | Toolchain | Result |
|---|---|---|
| Windows 11 x86-64 | `x86_64-pc-windows-gnu`, MinGW libm | Full suite passing, clippy clean |
| Ubuntu 24.04 x86-64 (WSL2) | `x86_64-unknown-linux-gnu`, glibc 2.39 | Full suite passing, clippy clean |

**The `libm` hypothesis did not materialise.** The two riskiest suites pass
bit-exact on both — `log`, `exp`, `sin`, `cos` and `atan2` returned identical
results for every input the fixtures use.

This is **not guaranteed by any standard**; it simply holds for these inputs on
these two libm implementations. If a platform ever cracks, the answer is not to
declare the port wrong, but to move those two suites to a small ULP tolerance
and record it as a deliberate divergence.

To reproduce: [`runbooks/linux-verify.md`](runbooks/linux-verify.md).

## 3.6 ROS 2 verification

Interop here is measured too. `scripts/ros2/smoke_test.sh` runs the node against
a synthetic sensor, captures `/octomap_binary`, then decodes the payload **with
the C++ OctoMap library** and asks it about voxels whose answers are already
known (occupied / free / unknown).

It has been run on ROS 2 Jazzy (WSL2, Ubuntu 24.04), passing on both Fast-RTPS
and CycloneDDS. The resulting map: **637 nodes / 454 leaves**, bounds
`[1, −0.5, 0] .. [3.1, 0.6, 1.1]` — matching the synthetic wall geometry, with
all five probe queries agreeing on both the binary and the full payload.

## 3.7 Why the fixtures are committed but the reference is not

A contributor has to be able to run the differential tests **without a C++
toolchain**. Committing the CSVs makes the reference answers available
everywhere; committing the C++ source would fork the reference.

174 KB is a cheap price for that. The details are in
[ADR-0002](decisions/0002-commit-fixtures-not-reference.md); regeneration is in
[`runbooks/regenerate-fixtures.md`](runbooks/regenerate-fixtures.md).
