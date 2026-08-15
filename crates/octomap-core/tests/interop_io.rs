//! Interoperability test: map files, both directions.
//!
//! The fixtures `cpp_scene.ot` and `cpp_scene.bt` were written by OctoMap C++
//! 1.10.0 (`scripts/gen_golden_io.cpp`) and are committed as binary.
//!
//! **C++ → Rust** is checked by decoding those files and comparing every leaf
//! against `io.csv`.
//!
//! **Rust → C++** is checked by building the same scene here, serializing it,
//! and asserting the bytes are *identical* to what the reference produced. That
//! is a stronger claim than "the reference can parse it" and needs no C++
//! toolchain at test time: identical bytes cannot decode differently.

use octomap_core::io::{read_binary, read_full, write_binary, write_binary_const, write_full};
use octomap_core::{OcTree, OcTreeKey, Point3, PointCloud};

const CPP_OT: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../tests/golden/cpp_scene.ot"
));
const CPP_BT: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../tests/golden/cpp_scene.bt"
));
const GOLDEN: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../tests/golden/io.csv"
));

const C: u16 = 32768;

fn rows(section: &str) -> Vec<Vec<&'static str>> {
    GOLDEN
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| l.split(',').collect::<Vec<_>>())
        .filter(|f| f[0] == section)
        .collect()
}

fn num(s: &str) -> i64 {
    s.parse()
        .unwrap_or_else(|e| panic!("{s:?} is not a number: {e}"))
}

fn golden_counts(stage: &str) -> (usize, usize) {
    let row = rows("counts")
        .into_iter()
        .find(|f| f[1] == stage)
        .unwrap_or_else(|| panic!("no counts row for stage {stage:?}"));
    (num(row[2]) as usize, num(row[3]) as usize)
}

fn golden_leaves(stage: &str) -> Vec<(OcTreeKey, u32, u32)> {
    let mut rows: Vec<_> = rows("leaf").into_iter().filter(|f| f[1] == stage).collect();
    rows.sort_by_key(|f| num(f[2]));
    rows.iter()
        .map(|f| {
            (
                OcTreeKey::new(num(f[3]) as u16, num(f[4]) as u16, num(f[5]) as u16),
                num(f[6]) as u32,
                num(f[7]) as u32,
            )
        })
        .collect()
}

fn actual_leaves(tree: &OcTree) -> Vec<(OcTreeKey, u32, u32)> {
    tree.iter_leaves()
        .map(|v| (v.key(), v.depth(), v.value().log_odds.to_bits()))
        .collect()
}

fn offset(dx: i32, dy: i32, dz: i32) -> OcTreeKey {
    OcTreeKey::new(
        (C as i32 + dx) as u16,
        (C as i32 + dy) as u16,
        (C as i32 + dz) as u16,
    )
}

/// The same scene `scripts/gen_golden_io.cpp` builds.
fn build_scene() -> OcTree {
    let mut tree = OcTree::new(0.1).unwrap();

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

    tree
}

/// Reports the first differing byte, with context. A bare `assert_eq!` on two
/// 1.5 KB blobs is unreadable when it fails.
fn assert_bytes_equal(actual: &[u8], expected: &[u8], what: &str) {
    if actual == expected {
        return;
    }
    let at = actual
        .iter()
        .zip(expected)
        .position(|(a, b)| a != b)
        .unwrap_or(actual.len().min(expected.len()));
    let lo = at.saturating_sub(16);
    let hi_a = (at + 16).min(actual.len());
    let hi_e = (at + 16).min(expected.len());
    panic!(
        "{what} differs at byte {at} (rust {} bytes, reference {} bytes)\n\
         rust      [{lo}..{hi_a}]: {:02x?}\n\
         reference [{lo}..{hi_e}]: {:02x?}",
        actual.len(),
        expected.len(),
        &actual[lo..hi_a],
        &expected[lo..hi_e],
    );
}

#[test]
fn the_scene_matches_the_reference_before_serialization() {
    // If this fails, any byte comparison below is meaningless — the trees are
    // not the same tree.
    let tree = build_scene();
    let (size, leaves) = golden_counts("scene");
    assert_eq!(tree.len(), size, "node count");
    assert_eq!(tree.count_leaf_nodes(), leaves, "leaf count");
    assert_eq!(actual_leaves(&tree), golden_leaves("scene"));
}

#[test]
fn point_cloud_insertion_is_order_independent() {
    // The reference iterates an unordered_set when applying an update; this
    // port iterates a HashSet with a different order. The scene above goes
    // through that path, so if the resulting trees agree, the order genuinely
    // does not matter. Repeated builds also exercise Rust's per-process hash
    // seed randomization.
    let first = actual_leaves(&build_scene());
    for _ in 0..8 {
        assert_eq!(actual_leaves(&build_scene()), first);
    }
}

#[test]
fn rust_writes_a_byte_identical_ot_file() {
    let tree = build_scene();
    let mut out = Vec::new();
    write_full(&tree, &mut out).unwrap();
    assert_bytes_equal(&out, CPP_OT, ".ot output");
}

#[test]
fn rust_writes_a_byte_identical_bt_file() {
    let mut tree = build_scene();
    let mut out = Vec::new();
    write_binary(&mut tree, &mut out).unwrap();
    assert_bytes_equal(&out, CPP_BT, ".bt output");

    // writeBinary collapses and prunes; confirm we land where C++ landed.
    let (size, leaves) = golden_counts("binary");
    assert_eq!(tree.len(), size);
    assert_eq!(tree.count_leaf_nodes(), leaves);
    assert_eq!(actual_leaves(&tree), golden_leaves("binary"));
}

#[test]
fn rust_reads_the_cpp_ot_file() {
    let tree = read_full(&mut &CPP_OT[..]).unwrap();
    let (size, leaves) = golden_counts("ot_reloaded");
    assert_eq!(tree.len(), size, "node count");
    assert_eq!(tree.count_leaf_nodes(), leaves, "leaf count");
    assert_eq!(tree.resolution(), 0.1);
    assert_eq!(actual_leaves(&tree), golden_leaves("ot_reloaded"));
}

#[test]
fn rust_reads_the_cpp_bt_file() {
    let tree = read_binary(&mut &CPP_BT[..]).unwrap();
    let (size, leaves) = golden_counts("bt_reloaded");
    assert_eq!(tree.len(), size, "node count");
    assert_eq!(tree.count_leaf_nodes(), leaves, "leaf count");
    assert_eq!(tree.resolution(), 0.1);
    assert_eq!(actual_leaves(&tree), golden_leaves("bt_reloaded"));
}

#[test]
fn reading_a_cpp_file_and_writing_it_back_reproduces_it() {
    // Covers the reader and writer together: decode the reference's bytes,
    // re-encode, and land on the same bytes.
    let tree = read_full(&mut &CPP_OT[..]).unwrap();
    let mut out = Vec::new();
    write_full(&tree, &mut out).unwrap();
    assert_bytes_equal(&out, CPP_OT, ".ot re-encode");

    let tree = read_binary(&mut &CPP_BT[..]).unwrap();
    let mut out = Vec::new();
    // The file is already collapsed and pruned, so the const writer is the
    // right one — write_binary would collapse it a second time.
    write_binary_const(&tree, &mut out).unwrap();
    assert_bytes_equal(&out, CPP_BT, ".bt re-encode");
}

#[test]
fn the_ot_file_preserves_more_than_the_bt_file() {
    // Sanity check on what the two formats are for: .ot keeps the graded
    // log-odds, .bt keeps one bit per voxel.
    let ot = read_full(&mut &CPP_OT[..]).unwrap();
    let bt = read_binary(&mut &CPP_BT[..]).unwrap();

    let distinct = |t: &OcTree| -> usize {
        let mut v: Vec<u32> = t
            .iter_leaves()
            .map(|l| l.value().log_odds.to_bits())
            .collect();
        v.sort_unstable();
        v.dedup();
        v.len()
    };

    assert!(
        distinct(&ot) > distinct(&bt),
        ".ot kept {} distinct values, .bt kept {}",
        distinct(&ot),
        distinct(&bt)
    );
    assert_eq!(distinct(&bt), 2, ".bt should hold only the two clamps");
}

#[test]
fn occupancy_survives_the_binary_round_trip() {
    let mut original = build_scene();
    let mut out = Vec::new();
    write_binary(&mut original, &mut out).unwrap();
    let restored = read_binary(&mut out.as_slice()).unwrap();

    for leaf in original.iter_leaves() {
        assert_eq!(
            restored.is_occupied(leaf.key()),
            Some(original.sensor().is_occupied(*leaf.value())),
            "occupancy lost for {:?}",
            leaf.key()
        );
    }
}
