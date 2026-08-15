//! The runtime pattern an application actually uses: integrate a scan taken
//! from a known sensor pose, then ask the map about specific places in the
//! world.
//!
//! ```text
//! cargo run --example insert_and_query
//! ```
//!
//! The scene is a sensor looking down +x at a flat wall two metres away. What
//! matters is that it produces all three occupancy states, including the one
//! that is easy to get wrong:
//!
//! ```text
//!   sensor                      wall              behind the wall
//!     o - - - - - - - - - - - - ####                   ?
//!     ^          ^               ^                     ^
//!  origin      FREE           OCCUPIED              UNKNOWN
//!             (rays passed     (rays ended        (no ray ever
//!              through)         here)              reached)
//! ```
//!
//! A ray-cast map cannot say anything about space its rays never reached.
//! Reporting that as *free* would let a planner drive straight through a
//! wall, so the API keeps it separate: `None`, not `Some(false)`.

use octomap_core::{OcTree, Point3, PointCloud};

/// Sensor position. Offset to a voxel centre so the printed coordinates line
/// up with the voxels they name.
const SENSOR: Point3 = Point3::new(0.05, 0.05, 0.05);

/// Distance from the sensor to the wall, in metres.
const WALL_X: f32 = 2.05;

/// Half-width of the wall, in voxels.
const WALL_HALF_EXTENT: i32 = 6;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;

    // A flat wall of returns, as a depth sensor would see it.
    let mut scan = PointCloud::new();
    for dy in -WALL_HALF_EXTENT..=WALL_HALF_EXTENT {
        for dz in -WALL_HALF_EXTENT..=WALL_HALF_EXTENT {
            scan.push(Point3::new(
                WALL_X,
                0.05 + 0.1 * dy as f32,
                0.05 + 0.1 * dz as f32,
            ));
        }
    }

    println!("resolution     {} m", map.resolution());
    println!("sensor origin  {:?}", SENSOR);
    println!("scan           {} returns\n", scan.len());

    // Each ray frees the space it crosses and marks its endpoint occupied.
    map.insert_point_cloud(&scan, SENSOR, -1.0, false, false);

    let before = map.len();
    let removed = map.prune();
    println!("nodes after insertion  {before}");
    println!(
        "nodes after prune      {} ({removed} merged away)",
        map.len()
    );
    println!("leaves                 {}", map.count_leaf_nodes());
    if removed == 0 {
        // Not a bug: eager insertion collapses uniform blocks as the recursion
        // unwinds, so an explicit prune afterwards usually finds nothing left
        // to do. It matters after `insert_point_cloud(.., lazy_eval = true)`,
        // which defers that work rather than doing it during insertion.
        println!("                       (insertion already pruned as it went)");
    }
    println!();

    // Three probes, one per occupancy state.
    let on_wall = Point3::new(WALL_X, 0.05, 0.05);
    let in_front = Point3::new(1.05, 0.05, 0.05);
    let behind = Point3::new(WALL_X + 1.0, 0.05, 0.05);

    report(&map, "on the wall", on_wall);
    report(&map, "between sensor and wall", in_front);
    report(&map, "behind the wall", behind);

    // The example is also a smoke test: a regression here should fail the run,
    // not just print something odd.
    assert_eq!(
        map.is_occupied_at(on_wall),
        Some(true),
        "the wall should read as occupied"
    );
    assert_eq!(
        map.is_occupied_at(in_front),
        Some(false),
        "space the rays crossed should read as free"
    );
    assert_eq!(
        map.is_occupied_at(behind),
        None,
        "occluded space must stay unknown — this is the assertion that matters"
    );

    println!("\nAll three states behaved as expected.");
    println!("Note the third: occluded space is None, not Some(false).");
    Ok(())
}

fn report(map: &OcTree, label: &str, p: Point3) {
    let state = match map.is_occupied_at(p) {
        Some(true) => "OCCUPIED",
        Some(false) => "FREE",
        None => "UNKNOWN",
    };
    // `{:?}` on the Option keeps the three-state shape visible rather than
    // flattening it into a bool.
    match (map.get_log_odds_at(p), map.get_occupancy_at(p)) {
        (Some(log_odds), Some(probability)) => println!(
            "{label:<24} ({:5.2}, {:5.2}, {:5.2})  {state:<8}  p={probability:.3}  log-odds={log_odds:+.3}",
            p.x, p.y, p.z
        ),
        _ => println!(
            "{label:<24} ({:5.2}, {:5.2}, {:5.2})  {state:<8}  never observed",
            p.x, p.y, p.z
        ),
    }
}
