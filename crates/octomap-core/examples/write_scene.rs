//! Writes a map in both formats, for the cross-language interoperability check.
//!
//! ```text
//! cargo run --example write_scene -- <output-dir>
//! ```
//!
//! Produces `rust_scene.ot` and `rust_scene.bt`. `scripts/verify_rust_io.cpp`
//! then reads them with the C++ reference — see `scripts/README.md`.

use std::path::PathBuf;

use octomap_core::io::{write_binary_file, write_full_file};
use octomap_core::{OcTree, OcTreeKey, Point3, PointCloud};

const C: u16 = 32768;

fn offset(dx: i32, dy: i32, dz: i32) -> OcTreeKey {
    OcTreeKey::new(
        (C as i32 + dx) as u16,
        (C as i32 + dy) as u16,
        (C as i32 + dz) as u16,
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dir: PathBuf = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "tests/golden".to_string())
        .into();

    let mut tree = OcTree::new(0.1)?;

    for i in 0..24u16 {
        tree.update_node(OcTreeKey::new(C + i, C, C), i % 3 != 0);
    }
    for dx in 0..2u16 {
        for dy in 0..2u16 {
            for dz in 0..2u16 {
                tree.update_node(
                    OcTreeKey::new(C + 100 + dx, C + 100 + dy, C + 100 + dz),
                    true,
                );
            }
        }
    }
    for &(dx, dy, dz) in &[
        (-40, 12, 7),
        (300, -250, 90),
        (1, -1, 1),
        (-5000, 2000, -300),
        (77, 77, 77),
    ] {
        tree.update_node(offset(dx, dy, dz), true);
    }

    let scan: PointCloud = [
        Point3::new(1.05, 0.05, 0.05),
        Point3::new(0.05, 1.05, 0.05),
        Point3::new(-1.05, -0.35, 0.45),
        Point3::new(2.05, 1.05, -1.05),
    ]
    .into_iter()
    .collect();
    tree.insert_point_cloud(&scan, Point3::new(0.05, 0.05, 0.05), -1.0, false, false);

    println!(
        "scene: {} nodes, {} leaves",
        tree.len(),
        tree.count_leaf_nodes()
    );

    // .ot first — write_binary collapses the map onto its clamps.
    write_full_file(&tree, dir.join("rust_scene.ot"))?;
    write_binary_file(&mut tree, dir.join("rust_scene.bt"))?;

    println!(
        "after binary collapse: {} nodes, {} leaves",
        tree.len(),
        tree.count_leaf_nodes()
    );
    println!("wrote rust_scene.ot and rust_scene.bt to {}", dir.display());
    Ok(())
}
