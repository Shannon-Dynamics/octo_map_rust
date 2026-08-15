# C++ source audit

Reference: **OctoMap 1.10.0**, commit `f012f5f0a4f58cad19501833f9c0ea9d864427b6`
(2026-02-08), cloned to `reference-cpp/` (gitignored — re-clone with
`git clone --depth 1 https://github.com/OctoMap/octomap.git reference-cpp`).

Every constant and formula below was read out of that tree, not from
documentation. File references are relative to `reference-cpp/octomap/`.

## Tree constants

| Constant | Value | Source |
|---|---|---|
| `tree_depth` | 16 | `include/octomap/OcTreeBaseImpl.hxx:47` |
| `tree_max_val` | 32768 | `include/octomap/OcTreeBaseImpl.hxx:47` |
| `key_type` | `uint16_t` | `include/octomap/OcTreeKey.h:63` |
| `point3d` scalar | `float` | `include/octomap/octomap_types.h:49` |
| `KeyRay` max size | 100000 | `include/octomap/OcTreeKey.h:181` |

`tree_max_val` offsets the world origin into the middle of the `u16` range, so
the addressable volume is `[-tree_max_val * resolution, +tree_max_val *
resolution)` per axis — ±3276.8 m at 0.1 m resolution.

`point3d` is **float**, not double, while the conversion arithmetic below runs
in **double**. Widening `Point3` to `f64` would diverge from the reference at
voxel boundaries, so `Point3` stays `f32`.

## Coordinate ↔ key conversion

Derived state, set in `setResolution` (`OcTreeBaseImpl.hxx`):

```
resolution_factor = 1.0 / resolution
tree_center       = tree_max_val / resolution_factor
node_size(d)      = resolution * (1 << (tree_depth - d))
```

### The reciprocal trap

`coordToKey` multiplies by the cached `resolution_factor`; it does **not**
divide by `resolution`. These are not interchangeable in IEEE-754:

```
1.2 / 0.1        == 11.999999999999998   -> floor 11
1.2 * (1.0/0.1)  == 12.0                 -> floor 12
```

Dividing would place points on the wrong side of a voxel boundary relative to
C++. Covered by `geometry::tests::scaling_multiplies_by_the_reciprocal_rather_than_dividing`.

### Formulas

`OcTreeBaseImpl.h:367`, `.h:494`, `.hxx` — verbatim semantics:

```
coordToKey(c)              = (int)floor(resolution_factor * c) + tree_max_val
keyToCoord(k)              = ((int)k - (int)tree_max_val + 0.5) * resolution

coordToKey(c, depth):
    keyval = (int)floor(resolution_factor * c)
    diff   = tree_depth - depth
    diff == 0 -> keyval + tree_max_val
    else      -> ((keyval >> diff) << diff) + (1 << (diff-1)) + tree_max_val

adjustKeyAtDepth(k, depth):
    diff = tree_depth - depth
    diff == 0 -> k
    else      -> (((k - tree_max_val) >> diff) << diff) + (1 << (diff-1)) + tree_max_val

keyToCoord(k, depth):
    depth == 0          -> 0.0                     // root is centered on origin
    depth == tree_depth -> keyToCoord(k)
    else                -> (floor((k - tree_max_val) / (1 << (tree_depth-depth))) + 0.5)
                           * node_size(depth)

coordToKeyChecked(c, depth):
    scaled = (int)floor(resolution_factor * c) + tree_max_val
    valid iff scaled >= 0 && (unsigned)scaled < 2*tree_max_val
```

### Signed vs unsigned shift

In `adjustKeyAtDepth`, `key - tree_max_val` mixes `key_type` (promoted to `int`)
with `unsigned int`, so C++ evaluates it as **unsigned** and `>>` is a logical
shift. The port uses `i64` with an arithmetic shift. The two differ only in bits
above bit 15, which the truncation back to `u16` discards, so they agree for
every input. Verified for the boundary cases in
`geometry::tests::adjust_key_at_depth_is_idempotent`.

## Key helpers (`OcTreeKey.h`)

```
KeyHash(k)             = k0 + 1447*k1 + 345637*k2
computeChildIdx(k, d)  = bit d of k0 | (bit d of k1) << 1 | (bit d of k2) << 2
computeIndexKey(l, k)  = l == 0 ? k : k & (65535 << l)   // per axis

computeChildKey(pos, off, parent) per axis:
    pos & bit -> parent + off
    else      -> parent - off - (off ? 0 : 1)
```

The `- (off ? 0 : 1)` is not a typo. At the deepest level the half-extent
rounds to zero, and without the extra subtraction both children would alias the
parent key.

## Occupancy model (`AbstractOccupancyOcTree.cpp:40`)

Defaults, given as probabilities and converted to log-odds on assignment:

| Parameter | Probability | Log-odds |
|---|---|---|
| `occupancyThres` | 0.5 | 0.0 |
| `probHit` | 0.7 | ≈ 0.8473 |
| `probMiss` | 0.4 | ≈ −0.4055 |
| `clampingThresMin` | 0.1192 | ≈ −2.0 |
| `clampingThresMax` | 0.971 | ≈ 3.5 |

Conversions (`octomap_utils.h`):

```
logodds(p)  = (float) log(p / (1-p))     // computed in double, stored as float
probability(l) = 1 - (1 / (1 + exp(l)))  // returns double
```

Note the storage asymmetry: thresholds are `float` fields, but the arithmetic
that produces them runs in `double`. The port must round the same way.

Occupancy test is `logOdds >= occ_prob_thres_log` — **inclusive**, so a node at
exactly 0.5 counts as occupied.

Setters assert sign: `probHit` must give log-odds ≥ 0, `probMiss` ≤ 0.

## Tree structure (`OcTreeBaseImpl.hxx`)

### Descent convention

A child at depth `d` is selected by bit `tree_depth - 1 - d` of the key, so the
root branches on the most significant bit. `search` walks `i` from
`tree_depth - 1` down to `tree_depth - depth`.

### Search through a pruned node

When `search` wants a child that does not exist, the result depends on whether
the current node has *any* children:

```
child missing, node is childless -> return the node   (pruned leaf covers the key)
child missing, node has children -> return NULL       (genuine miss)
```

### Prune and expand

```
isNodeCollapsible: all 8 children exist, none has children of its own,
                   and all compare equal to child 0
pruneNode:         copy child 0's value up, then delete all 8 children
expandNode:        create 8 children, each a copy of the node's own value
```

`prune()` sweeps `depth` from `tree_depth - 1` down to 0 and **breaks at the
first level that merges nothing**. The reference carries a `FIXME` about this:
a partly pruned tree whose deepest level has nothing left to merge is left
alone even when shallower levels could still collapse. Reproduced deliberately
in `OctreeCore::prune`, and documented there.

`expandRecurs` descends to `max_depth` expanding every leaf it meets, so its
cost is exponential in the gap between the shallowest leaf and `max_depth`.

### Delete

`deleteNodeRecurs` expands a childless non-root node it meets mid-descent
rather than failing, which is what makes deleting one voxel out of a pruned
block leave the other seven behind. Confirmed against the reference: deleting
from a pruned block moves the tree from 159 to 166 nodes (+8 expanded, −1
deleted).

### Iterators (`OcTreeIterator.hxx`)

```
root key   = (tree_max_val, tree_max_val, tree_max_val), depth 0
child key  = computeChildKey(i, tree_max_val >> child_depth, parent_key)
```

Children are pushed onto the stack in reverse so they pop in index order 0..7.
A node counts as a leaf when it has no children **or** sits at `maxDepth`,
which is what makes depth-limited iteration report a coarse view.

### Insertion caveat for golden data

`OcTree::updateNode` auto-prunes on the way back up the recursion. Generating
purely structural fixtures requires `setNodeValue(key, value, lazy_eval = true)`
instead, which skips both the prune and the inner-node update.

## Occupancy update (`OccupancyOcTreeBase.hxx`)

```
updateNodeLogOdds:  add, then clamp — the value may overshoot before clamping
integrateHit/Miss:  updateNodeLogOdds(prob_hit_log / prob_miss_log)
updateOccupancyChildren: setLogOdds(getMaxChildLogOdds())   // conservative
getMaxChildLogOdds: -FLT_MAX when childless, not -infinity
```

### Early abort in `updateNode`

Before descending, `updateNode` looks the leaf up and returns immediately if it
is already pinned at the clamp the update pushes toward:

```
update >= 0 and logOdds >= clamping_max  -> return unchanged
update <= 0 and logOdds <= clamping_min  -> return unchanged
```

This is observable beyond the value: it also skips node creation.

### Prune-on-the-way-up

With `lazy_eval` false, `updateNodeRecurs` tries `pruneNode` on each node as the
recursion unwinds, and only refreshes the inner node from its children when the
prune fails. So ordinary insertion auto-prunes. With `lazy_eval` true neither
happens, and `updateInnerOccupancy` must be called afterwards.

Descending into a childless non-root node expands it first, which is what makes
a single update inside a pruned block reopen it.

### `getMeanChildLogOdds`

Averages the children's **probabilities** and converts back, rather than
averaging log-odds. Not the same operation. Unused by `updateOccupancyChildren`,
which takes the maximum.

## Ray traversal

Amanatides–Woo 3D DDA. `computeRayKeys` includes the origin's voxel and excludes
the endpoint's. Two details:

- The tie-breaking order when picking the axis with the smallest `tMax` is
  written out explicitly and must be reproduced, or rays that graze a voxel
  corner step through different neighbours.
- `computeRayKeys` narrows the voxel-border offset to `float`; `castRay` keeps
  it `double`. The two functions are otherwise identical setups.

A DDA walk is **not** symmetric under reversal — `tMax` is seeded from where the
origin sits inside its voxel. Confirmed in the reference, pinned in
`tests/golden/ray.csv`.

`insertPointCloud` applies free cells before occupied ones, and makes the two
sets disjoint with occupied winning.

## File formats

Both formats are an ASCII header followed by a binary payload:

```
<marker>\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n
id OcTree\n
size <n>\n
res <resolution>\n
data\n
```

Markers are `# Octomap OcTree file` (`.ot`) and `# Octomap OcTree binary file`
(`.bt`). `res` is written with an unconfigured `ostream`, so **six significant
digits** — `0.1` becomes `0.1`, not `0.100000`. Byte-identical output requires
reproducing that formatting.

`.ot` payload, pre-order per node: 4-byte `f32` value, then one byte of child
bits, then each existing child.

`.bt` payload, pre-order per node: two descriptor bytes, four children each, two
bits per child, low index in the low bits (`std::bitset<8>` indexes from the
LSB):

```
00 unknown   01 free leaf   10 occupied leaf   11 has children
```

Only children marked `11` are recursed into. On read, every node starts at
`clamping_thres_max` and inner nodes are corrected to `getMaxChildLogOdds()`
once their children are decoded.

`writeBinary` calls `toMaxLikelihood()` and `prune()` first — it **mutates the
tree**. `writeBinaryConst` does not.

Node values are written with a raw `memcpy`, so the on-disk byte order is the
writing machine's.

## Status of this phase

Audit complete for everything this port implements. Verified against a running
C++ binary (built into `build-cpp/`, fixtures in `tests/golden/`), comparing
floating point as raw IEEE-754 bit patterns:

- Conversion formulas, key adjustment, bounds checking, node sizes — 938 rows.
- Tree structure, iteration order, prune, delete, depth-limited views.
- Occupancy: 43 sequential updates, clamping, auto-prune, block reopen,
  max likelihood, change detection.
- Rays: DDA sequences for 12 shapes, 8 ray-cast cases, point-cloud integration.
- Pose: Euler ↔ quaternion, rotation, axis-angle, transform, inverse,
  composition.
- File I/O: byte-identical `.ot` and `.bt` output, and decoding of files the
  reference wrote.

Not audited, because the port does not implement them:

- `ColorOcTree`, `OcTreeStamped`, `CountingOcTree`, `ScanGraph`,
  `MapCollection`.
- Bounding-box-limited insertion (`setBBXMin` / `setBBXMax`, the `inBBX` branch
  of `computeUpdate`).
- The legacy headerless `.bt` format (`readBinaryLegacyHeader`).
