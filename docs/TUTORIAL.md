# Tutorial

A working introduction to `octomap-core`, from an empty project to a saved map.
No prior knowledge of OctoMap is assumed — the concepts are introduced where
they are first needed.

Every snippet below compiles against the current API. Where a snippet builds on
the previous one, it says so.

| Part | |
|---|---|
| 1 | [Installation](#part-1--installation) |
| 2 | [Creating a map](#part-2--creating-a-map) |
| 3 | [Resolution and coordinates](#part-3--resolution-and-coordinates) |
| 4 | [Inserting data](#part-4--inserting-data) |
| 5 | [Querying occupancy](#part-5--querying-occupancy) |
| 6 | [Iterating through the map](#part-6--iterating-through-the-map) |
| 7 | [Saving and loading](#part-7--saving-and-loading) |
| 8 | [Working with point clouds](#part-8--working-with-point-clouds) |
| 9 | [The Candi scanning example](#part-9--the-candi-scanning-example) |

---

## Part 1 — Installation

You need Rust 1.75 or newer and nothing else. The library has no runtime
dependencies and no C++ toolchain in the build.

The crate is **not published to crates.io yet**, so depend on the repository:

```bash
cargo new my-mapper
cd my-mapper
```

```toml
# Cargo.toml
[dependencies]
octomap-core = { git = "https://github.com/Shannon-Dynamics/octo_map_rust" }
```

Check it resolves:

```bash
cargo build
```

If you would rather read the code as you go, clone the repository instead and
run the examples in place:

```bash
git clone https://github.com/Shannon-Dynamics/octo_map_rust
cd octo_map_rust
cargo run --example insert_and_query
```

---

## Part 2 — Creating a map

An occupancy map answers one question: **is there something at this place?**
Not with a yes or a no, but with a confidence that strengthens as observations
accumulate.

```rust
use octomap_core::{OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;   // 10 cm voxels

    println!("resolution {} m", map.resolution());
    println!("{} nodes, empty: {}", map.len(), map.is_empty());

    // The first observation — Part 4 covers this properly.
    map.update_node_at(Point3::new(1.05, 0.05, 0.05), true);
    println!("after one observation: {} nodes", map.len());
    Ok(())
}
```

Three things are worth noticing there.

**`new` returns a `Result`.** The only way it fails is a resolution that is not
finite and strictly positive — `0.0`, `-1.0`, `f64::NAN`. Rejecting that at
construction means every method afterwards can rely on it, so nothing downstream
needs to re-check or silently clamp.

**The map starts genuinely empty.** Not "all free" — empty. Nothing has been
observed, and the map will say so.

**`mut` matters.** Reading a map borrows it; changing it takes `&mut`. That is
not ceremony: it is what stops one part of your program mutating the map while
another walks it, checked at compile time rather than discovered in a debugger.

---

## Part 3 — Resolution and coordinates

The map is an octree. The world is divided in half along each axis, then in half
again, sixteen times over — and the smallest box that produces is one **voxel**,
with an edge length equal to the resolution.

Two coordinate systems come out of that, and both are worth understanding before
you insert anything:

| | What it is | Type |
|---|---|---|
| **World coordinate** | Metres, as your sensor reports them | `Point3` |
| **Key** | The integer address of a voxel | `OcTreeKey` |

A key is what the tree actually stores. Converting a coordinate to a key is a
lossy step by design: every point inside a voxel maps to the same key.

```rust
use octomap_core::{OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let map = OcTree::new(0.1)?;
    let geometry = map.geometry();

    // Two points 1 cm apart in a 10 cm map land in the same voxel.
    let a = geometry.coord_to_key(Point3::new(1.21, 0.0, 0.0));
    let b = geometry.coord_to_key(Point3::new(1.22, 0.0, 0.0));
    assert_eq!(a, b);

    // Converting back gives the voxel's centre, not the original point.
    let centre = geometry.key_to_coord(a);
    println!("both points live in the voxel centred at {:.2}", centre.x);

    // A coordinate outside the addressable volume has no key at all.
    let half_extent = geometry.half_extent();
    println!("the tree addresses +/- {half_extent:.1} m per axis");
    assert!(geometry.coord_to_key_checked(Point3::new(1e9, 0.0, 0.0)).is_none());
    Ok(())
}
```

**Choosing a resolution** is the first real design decision. It is fixed for the
life of a map — you cannot change it later, only build a new map — and it trades
detail against memory and time:

| Resolution | Reasonable for |
|---|---|
| 0.01–0.05 m | Tabletop manipulation, small objects, close-range scanning |
| 0.05–0.20 m | Indoor mobile robots, drone scans of a structure |
| 0.20–1.00 m | Outdoor navigation, large-area survey |

There are two ways to hold a key, and it is worth picking deliberately. If you
query the same place repeatedly — a planner sampling a path, a control loop
watching one region — convert once and keep the key: `map.is_occupied(key)`
skips the coordinate conversion that `map.is_occupied_at(point)` repeats on
every call, and the conversion is the part that can reject an out-of-range
coordinate.

`coord_to_key_checked` returns `Option`; `coord_to_key` does not check bounds
and is for coordinates you have already validated. Prefer the checked one for
anything coming from a sensor.

---

## Part 4 — Inserting data

There are three ways in, from most direct to most useful.

### One voxel at a time

```rust
use octomap_core::{OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;

    // "I observed something here." Returns the voxel's new log-odds.
    let after_one = map.update_node_at(Point3::new(1.05, 0.05, 0.05), true);
    println!("after one hit:  {after_one:?}");

    // Observing it again strengthens the belief rather than replacing it.
    let after_two = map.update_node_at(Point3::new(1.05, 0.05, 0.05), true);
    println!("after two hits: {after_two:?}");
    Ok(())
}
```

This is *probabilistic* mapping: each observation moves a confidence value
rather than setting a flag. Contradicting observations pull it back, so a
transient obstacle fades once it stops being seen. Confidence is clamped at both
ends, which keeps a voxel revisable — a voxel observed a thousand times is not
so certain that it can never be cleared.

`update_node_at` returns `Option<f32>`: `None` means the coordinate was outside
the addressable volume.

### One ray

A range sensor tells you more than where the surface is. It also tells you that
everything between the sensor and that surface is **empty**, and that is half of
what makes a map useful:

```rust
use octomap_core::{OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;

    let sensor = Point3::new(0.05, 0.05, 0.05);
    let hit = Point3::new(1.05, 0.05, 0.05);

    // Frees every voxel the ray crosses, marks the endpoint occupied.
    map.insert_ray(sensor, hit, -1.0, false);

    assert_eq!(map.is_occupied_at(hit), Some(true));
    assert_eq!(map.is_occupied_at(Point3::new(0.55, 0.05, 0.05)), Some(false));
    Ok(())
}
```

### A whole scan

In practice you insert a scan, not a ray — see
[Part 8](#part-8--working-with-point-clouds).

---

## Part 5 — Querying occupancy

The API distinguishes **three** states, and the third is the one that makes a
map safe to plan against:

```rust
use octomap_core::{OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;
    map.insert_ray(Point3::new(0.05, 0.05, 0.05), Point3::new(1.05, 0.05, 0.05), -1.0, false);

    for (label, p) in [
        ("the endpoint", Point3::new(1.05, 0.05, 0.05)),
        ("halfway there", Point3::new(0.55, 0.05, 0.05)),
        ("behind it", Point3::new(2.05, 0.05, 0.05)),
    ] {
        let state = match map.is_occupied_at(p) {
            Some(true) => "OCCUPIED",
            Some(false) => "FREE",
            None => "UNKNOWN — never observed",
        };
        println!("{label:<15} {state}");
    }
    Ok(())
}
```

`None` is not an error and not a failure. It means no ray has ever reached that
place, so the map has nothing to say about it. A ray-cast map physically cannot
know what is behind a wall, and reporting that as free is how a planner drives
through one. `Option<bool>` makes that unignorable: you have to decide what
unknown means for your application, and the compiler will not let you forget.

Three levels of detail are available for an observed voxel:

```rust
use octomap_core::{OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;
    let p = Point3::new(1.05, 0.05, 0.05);
    map.update_node_at(p, true);

    println!("occupied?    {:?}", map.is_occupied_at(p));   // Option<bool>
    println!("probability  {:?}", map.get_occupancy_at(p)); // Option<f64>, 0.0..1.0
    println!("log-odds     {:?}", map.get_log_odds_at(p));  // Option<f32>, as stored
    Ok(())
}
```

Use `is_occupied_at` for a decision, `get_occupancy_at` when you want the
confidence, and `get_log_odds_at` when you are comparing against the sensor
model's own thresholds.

**Where does the occupied/free line fall?** In the `SensorModel`, and it is
adjustable:

```rust
use octomap_core::OcTree;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;

    println!("hit           {}", map.sensor().prob_hit());
    println!("miss          {}", map.sensor().prob_miss());
    println!("threshold     {}", map.sensor().occupancy_thres());
    println!("clamp         {} .. {}",
             map.sensor().clamping_thres_min_prob(),
             map.sensor().clamping_thres_max_prob());

    // Setters validate: each probability has a range its role allows, and a
    // value outside it is an error rather than a silently clamped map.
    map.sensor_mut().set_prob_hit(0.8)?;
    assert!(map.sensor_mut().set_prob_hit(1.5).is_err());
    Ok(())
}
```

The defaults are the reference's, and they are a reasonable starting point for a
depth camera or a lidar. Raise `prob_hit` if your sensor is precise and you want
the map to reach a conclusion from fewer observations; lower it if you expect
noise.

---

## Part 6 — Iterating through the map

To export a map, draw it, or feed it to something else, you walk its leaves:

```rust
use octomap_core::{OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;
    map.insert_ray(Point3::new(0.05, 0.05, 0.05), Point3::new(1.05, 0.05, 0.05), -1.0, false);

    let threshold = map.sensor().occupancy_thres_log();
    let mut occupied = Vec::new();

    for visit in map.iter_leaves() {
        if visit.value().log_odds >= threshold {
            let centre = map.geometry().key_to_coord(visit.key());
            let size = map.geometry().node_size(visit.depth());
            occupied.push((centre, size));
        }
    }

    println!("{} occupied leaves", occupied.len());
    for (centre, size) in occupied.iter().take(3) {
        println!("  ({:5.2}, {:5.2}, {:5.2})  {size:.2} m across", centre.x, centre.y, centre.z);
    }
    Ok(())
}
```

Two things about that loop are easy to get wrong.

**A leaf is not always one voxel.** When all eight children of a node agree, the
tree **prunes** them into their parent, and that parent becomes a leaf covering
a larger box. This is why the octree is compact, and it is why you must ask for
`node_size(visit.depth())` rather than assuming the resolution. A leaf at depth
14 in a 16-deep tree is 4× the resolution across.

**Iterate leaves, not nodes.** `iter_nodes` walks inner nodes too, so the same
region appears at several depths. `iter_leaves` is what corresponds to cells.

`iter_leaves_to_depth(d)` stops the walk early and gives you a coarser view of
the same map — useful for a fast overview or a low-detail visualization.

---

## Part 7 — Saving and loading

Two file formats, and choosing between them matters:

| | `.bt` — binary tree | `.ot` — full tree |
|---|---|---|
| Stores | one bit per leaf: occupied or free | the exact log-odds of every node |
| Round trips | the *thresholded* map | the map, exactly |
| For | viewers, `octomap_server`, anything that needs geometry | checkpointing a map you will keep updating |

```rust
use octomap_core::{io, OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;
    map.insert_ray(Point3::new(0.05, 0.05, 0.05), Point3::new(1.05, 0.05, 0.05), -1.0, false);

    let dir = std::env::temp_dir();

    // Write .ot first: writing .bt thresholds the tree in memory as well as on
    // disk, and an .ot saved afterwards would have lost the confidences.
    io::write_full_file(&map, dir.join("scan.ot"))?;
    io::write_binary_file(&mut map, dir.join("scan.bt"))?;

    let reloaded = io::read_binary_file(dir.join("scan.bt"))?;
    assert_eq!(reloaded.resolution(), 0.1);
    assert_eq!(reloaded.is_occupied_at(Point3::new(1.05, 0.05, 0.05)), Some(true));

    std::fs::remove_file(dir.join("scan.ot")).ok();
    std::fs::remove_file(dir.join("scan.bt")).ok();
    Ok(())
}
```

Both formats are byte-identical to what the C++ library writes, so a `.bt` from
here opens in `octovis` and a `.bt` from a C++ tool loads here. That is checked
by the differential tests in both directions, not assumed —
[`03-verification.md`](03-verification.md).

Reading is the one place the library consumes bytes it did not produce, so
malformed input is an `IoError`, never a panic:

```rust
use octomap_core::io;

fn main() {
    let truncated: &[u8] = b"# Octomap OcTree binary file\nid OcTree\n";
    match io::read_binary(&mut { truncated }) {
        Ok(_) => println!("parsed"),
        Err(e) => println!("rejected: {e}"),
    }
}
```

`write_binary_data` / `read_binary_data` are the **headerless** variants that an
`octomap_msgs/Octomap` message carries. They are not the contents of a `.bt`
file: the resolution and tree id travel in the message's own fields. Sending one
where the other is expected produces a payload nobody can decode, which is why
they are named differently rather than being a flag.

`cargo run --example save_load` runs all of this with printed output.

---

## Part 8 — Working with point clouds

A real sensor produces thousands of points per frame. `PointCloud` is the batch
of endpoints from one reading, and `insert_point_cloud` integrates the whole
frame from one sensor position:

```rust
use octomap_core::{OcTree, Point3, PointCloud};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;
    let sensor = Point3::new(0.05, 0.05, 0.05);

    // A flat wall, as a depth camera would see it.
    let mut scan = PointCloud::new();
    for dy in -10..=10 {
        for dz in -10..=10 {
            scan.push(Point3::new(2.05, 0.05 + 0.1 * dy as f32, 0.05 + 0.1 * dz as f32));
        }
    }

    map.insert_point_cloud(&scan, sensor, -1.0, false, true);

    println!("{} points -> {} leaves", scan.len(), map.count_leaf_nodes());
    assert_eq!(map.is_occupied_at(Point3::new(2.05, 0.05, 0.05)), Some(true));
    Ok(())
}
```

The signature mirrors the C++ reference, so the three trailing arguments deserve
naming:

```rust,ignore
map.insert_point_cloud(&scan, sensor_origin, max_range, lazy_eval, discretize);
```

| Argument | Meaning |
|---|---|
| `max_range` | Metres. Points beyond it contribute free space up to that range and no endpoint. `-1.0` means unlimited |
| `lazy_eval` | Skip inner-node maintenance during insertion. The map is then not internally consistent until you call `update_inner_occupancy()` |
| `discretize` | Collapse duplicate endpoints to one ray per voxel first. Much cheaper on dense scans, and slightly different: rays go to voxel centres rather than to the original points |

**Set `max_range` for any real sensor.** A depth camera looking at the horizon
reports points hundreds of metres away, and each of those traces a ray through
every voxel on the way. Limiting the range is usually the difference between a
map that keeps up and one that does not.

**`lazy_eval` has a follow-up call.** If you use it, the tree's inner nodes are
stale until you say otherwise:

```rust
use octomap_core::{OcTree, Point3, PointCloud};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;
    let scan: PointCloud = vec![Point3::new(1.05, 0.05, 0.05)].into_iter().collect();

    map.insert_point_cloud(&scan, Point3::new(0.05, 0.05, 0.05), 10.0, true, false);

    // Required after a lazy insert, before querying inner nodes or writing out.
    map.update_inner_occupancy();
    let merged = map.prune();
    println!("{merged} nodes merged away, {} left", map.len());
    Ok(())
}
```

**Change detection** is how a consumer of the map avoids re-reading all of it
every frame:

```rust
use octomap_core::{OcTree, Point3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;
    map.enable_change_detection(true);

    map.update_node_at(Point3::new(1.05, 0.05, 0.05), true);
    map.update_node_at(Point3::new(1.15, 0.05, 0.05), true);

    println!("{} voxels changed since the last reset", map.changed_key_count());
    for (key, _) in map.changed_keys().iter().take(3) {
        println!("  changed: {:?}", map.geometry().key_to_coord(*key));
    }

    map.reset_change_detection();
    assert_eq!(map.changed_key_count(), 0);
    Ok(())
}
```

If your points arrive in a sensor frame rather than the world frame, transform
them first with `Pose6` — and if they arrive as a ROS 2 `PointCloud2` message,
`octomap-ros` decodes and transforms them for you without depending on ROS.

---

## Part 9 — The Candi scanning example

[`scan_candi_with_octomap_rust`](https://github.com/Shannon-Dynamics/scan_candi_with_octomap_rust)
is a complete application built on this library: a simulated drone flies an
orbit around a scanned model of Borobudur temple, renders a depth image at every
waypoint, and folds those frames into an occupancy map.

It is worth reading for the parts this tutorial does not cover, because they are
the parts a real deployment spends its time on:

- turning a depth image into a world-frame point cloud;
- deciding a sensor path that actually covers the structure;
- filtering the ground plane out before insertion — without it the map is mostly
  floor;
- choosing `max_range` and resolution against a per-frame time budget;
- comparing this library's octree against an independently written occupancy map
  on the same input, which is how that project validates both.

Start with its README, then `docs/SCANNING_TUTORIAL.md`.

---

## Where to go next

| | |
|---|---|
| The full API | `cargo doc --open`, or docs.rs after the first release |
| How correctness is measured | [`03-verification.md`](03-verification.md) |
| The safety model | [`../SAFETY.md`](../SAFETY.md) |
| What is missing, and why | [`../ROADMAP.md`](../ROADMAP.md) |
| Using it from ROS 2 | [`07-ros2.md`](07-ros2.md) |
| Contributing | [`../CONTRIBUTING.md`](../CONTRIBUTING.md) |
