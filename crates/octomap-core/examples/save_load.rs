//! Writing a map to disk in both OctoMap file formats and reading it back.
//!
//! ```text
//! cargo run --example save_load
//! ```
//!
//! The two formats answer different questions:
//!
//! | | `.bt` — binary tree | `.ot` — full tree |
//! |---|---|---|
//! | Stores | one bit per leaf: occupied or free | the exact log-odds of every node |
//! | Round trips | the *thresholded* map | the map, exactly |
//! | Used by | viewers, `octomap_server`, anything that only needs geometry | checkpointing a map you intend to keep updating |
//!
//! `.bt` is lossy on purpose: it is the interchange format, and it is what
//! `octovis` and most ROS tooling expect. `.ot` is what you save when the map
//! will be loaded and updated again, because a thresholded map has forgotten
//! how confident it was.
//!
//! Both are written byte-identically to what OctoMap C++ produces, which is
//! pinned by the differential tests in `tests/interop_io.rs`.

use octomap_core::{io, OcTree, Point3, PointCloud};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = OcTree::new(0.1)?;

    // A short wall, seen from a sensor one voxel off the origin.
    let sensor = Point3::new(0.05, 0.05, 0.05);
    let mut scan = PointCloud::new();
    for dz in -3..=3 {
        scan.push(Point3::new(2.05, 0.05, 0.05 + 0.1 * dz as f32));
    }
    map.insert_point_cloud(&scan, sensor, -1.0, false, false);

    let probe = Point3::new(2.05, 0.05, 0.05);
    let free = Point3::new(1.05, 0.05, 0.05);
    let log_odds_before = map.get_log_odds_at(probe).expect("the wall was observed");

    println!(
        "original       {} leaves, log-odds at the wall {log_odds_before:+.3}",
        map.count_leaf_nodes()
    );

    // Examples write to the system temp directory rather than the repository,
    // so running one leaves nothing behind to clean up or accidentally commit.
    let dir = std::env::temp_dir();
    let bt_path = dir.join("octomap-core-example.bt");
    let ot_path = dir.join("octomap-core-example.ot");

    // Order matters here, and the signatures say why: `write_full_file` takes
    // `&OcTree`, `write_binary_file` takes `&mut OcTree`. Writing `.bt`
    // thresholds the tree to max likelihood, and that is a mutation of the map
    // in memory, not only of the bytes on disk. Writing `.ot` afterwards would
    // save the thresholded map and lose exactly the confidence values the
    // format exists to keep. `write_binary_const` is the non-mutating variant
    // when that matters more than matching the reference's behaviour.
    io::write_full_file(&map, &ot_path)?;
    io::write_binary_file(&mut map, &bt_path)?;

    let bt_bytes = std::fs::metadata(&bt_path)?.len();
    let ot_bytes = std::fs::metadata(&ot_path)?.len();
    println!("wrote          {} ({bt_bytes} bytes)", bt_path.display());
    println!("wrote          {} ({ot_bytes} bytes)", ot_path.display());

    let from_bt = io::read_binary_file(&bt_path)?;
    let from_ot = io::read_full_file(&ot_path)?;

    println!();
    report(&from_bt, ".bt", probe, free);
    report(&from_ot, ".ot", probe, free);

    // Both formats preserve geometry: the wall is occupied and the space the
    // rays crossed is free in each of them.
    assert_eq!(from_bt.is_occupied_at(probe), Some(true));
    assert_eq!(from_ot.is_occupied_at(probe), Some(true));
    assert_eq!(from_bt.is_occupied_at(free), Some(false));
    assert_eq!(from_ot.is_occupied_at(free), Some(false));

    // The resolution travels in the file header, so a reloaded map addresses
    // the same voxels as the one that was written.
    assert_eq!(from_bt.resolution(), 0.1);
    assert_eq!(from_ot.resolution(), 0.1);

    std::fs::remove_file(&bt_path).ok();
    std::fs::remove_file(&ot_path).ok();

    println!();
    println!("Both round trips kept the geometry. Only .ot kept the confidence:");
    println!("a .bt leaf is one bit, so its log-odds comes back as the format's");
    println!("occupied or free value rather than as what was measured.");
    Ok(())
}

fn report(map: &OcTree, label: &str, probe: Point3, free: Point3) {
    println!(
        "{label:<15}{} leaves, wall {:>+7.3}, free space {:>+7.3}",
        map.count_leaf_nodes(),
        map.get_log_odds_at(probe).unwrap_or(f32::NAN),
        map.get_log_odds_at(free).unwrap_or(f32::NAN),
    );
}
