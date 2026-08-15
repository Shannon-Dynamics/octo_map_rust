//! Occupied cells as boxes, for `visualization_msgs/MarkerArray`.
//!
//! RViz has a dedicated octomap display, but it is a separate package and it
//! only speaks `octomap_msgs/Octomap`. A `MarkerArray` of cubes needs nothing
//! installed, works in rqt and foxglove, and is the fastest way to see whether
//! a map is being built at all. Both are worth publishing.
//!
//! # Why the boxes come in sizes
//!
//! A pruned octree stores a uniform region as one large node instead of eight
//! small ones, and that is most of why it is compact — a mapped room is mostly
//! large free nodes. So iterating leaves yields nodes at different depths, and
//! each needs a cube of its own edge length.
//!
//! A `Marker` of type `CUBE_LIST` carries a single `scale` for every cube in
//! it, so one marker cannot draw two sizes. The standard answer, and the one
//! [`voxels_by_depth`] is built for, is one marker per depth.

use std::collections::BTreeMap;

use octomap_core::{OcTree, Point3};

/// One octree node, as a cube.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Voxel {
    /// Center of the node, in the map frame.
    pub center: Point3,
    /// Edge length, in meters. A node at the tree's full depth measures one
    /// resolution; each level up doubles it.
    pub size: f64,
    /// Depth of the node, with the root at 0.
    pub depth: u32,
}

/// Iterates the map's occupied leaves as cubes.
///
/// Order follows the tree's own depth-first traversal, so it is stable between
/// calls on an unchanged map — which keeps marker ids stable and stops RViz
/// from flickering.
pub fn occupied_voxels(tree: &OcTree) -> impl Iterator<Item = Voxel> + '_ {
    voxels(tree, true)
}

/// Iterates the map's free leaves as cubes.
///
/// There are usually far more of these than occupied ones — a mapped room is
/// mostly free space — so publishing them is a debugging tool, not something to
/// leave running.
pub fn free_voxels(tree: &OcTree) -> impl Iterator<Item = Voxel> + '_ {
    voxels(tree, false)
}

fn voxels(tree: &OcTree, occupied: bool) -> impl Iterator<Item = Voxel> + '_ {
    let geometry = tree.geometry();
    let sensor = tree.sensor();

    tree.iter_leaves()
        .filter(move |leaf| sensor.is_occupied(*leaf.value()) == occupied)
        .map(move |leaf| {
            let depth = leaf.depth();
            Voxel {
                // A leaf's key addresses its center at its own depth, and the
                // depth-aware conversion is what re-quantizes it — the
                // full-depth conversion would land half a node off for
                // anything the pruner merged.
                center: geometry
                    .key_to_coord_at_depth(leaf.key(), depth)
                    .expect("a depth the tree produced is a depth the tree accepts"),
                size: geometry.node_size(depth),
                depth,
            }
        })
}

/// Groups occupied cells by depth, which is how they become markers.
///
/// Every cube in one entry shares an edge length, so each entry maps onto
/// exactly one `CUBE_LIST` marker with that `scale`. The map is ordered by
/// depth, so marker ids assigned from the iteration order stay put across
/// publishes.
///
/// Depths with no occupied cells are absent rather than empty. A node
/// republishing markers has to delete the ids it used last time and no longer
/// does, or RViz keeps drawing the stale ones.
pub fn voxels_by_depth(tree: &OcTree) -> BTreeMap<u32, Vec<Voxel>> {
    let mut by_depth: BTreeMap<u32, Vec<Voxel>> = BTreeMap::new();
    for voxel in occupied_voxels(tree) {
        by_depth.entry(voxel.depth).or_default().push(voxel);
    }
    by_depth
}

/// The rainbow `octomap_server` colors a map by height with.
///
/// Returns RGBA in 0..=1, running blue at `min_z` up to red at `max_z` — the
/// normalized height is inverted before the sweep, which is why a floor comes
/// out blue. Heights outside the range clamp to its ends.
/// `color_factor` compresses the spectrum — the C++ node defaults to `0.8`,
/// which stops the top of the range from wrapping back around to the red it
/// started at.
///
/// This is a direct transcription of the reference's `heightMapColor`, an
/// open-coded HSV sweep at full saturation and value. Matching it means a map
/// published by this node looks like one published by the C++ node, which
/// matters when the two are being compared side by side.
pub fn height_color(z: f64, min_z: f64, max_z: f64, color_factor: f64) -> [f32; 4] {
    let span = max_z - min_z;
    let normalized = if span > 0.0 {
        ((z - min_z) / span).clamp(0.0, 1.0)
    } else {
        // A degenerate range would divide by zero. Everything gets the same
        // color, which is the honest rendering of "there is no height range".
        0.0
    };

    let mut h = (1.0 - normalized) * color_factor;
    h -= h.floor();
    h *= 6.0;

    let i = h.floor() as i32;
    let mut f = h - f64::from(i);
    if i % 2 == 0 {
        f = 1.0 - f;
    }

    // Saturation and value are both 1, so `m` is 0 and `n` is `1 - f`.
    let m = 0.0f32;
    let n = (1.0 - f) as f32;

    let (r, g, b) = match i {
        0 | 6 => (1.0, n, m),
        1 => (n, 1.0, m),
        2 => (m, 1.0, n),
        3 => (m, n, 1.0),
        4 => (n, m, 1.0),
        5 => (1.0, m, n),
        _ => (1.0, 0.5, 0.5),
    };

    [r, g, b, 1.0]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map() -> OcTree {
        let mut tree = OcTree::new(0.1).unwrap();
        tree.update_node_at(Point3::new(1.05, 0.05, 0.05), true);
        tree.update_node_at(Point3::new(1.15, 0.05, 0.05), true);
        tree.update_node_at(Point3::new(0.05, 0.05, 0.05), false);
        tree
    }

    #[test]
    fn only_occupied_leaves_come_out_of_occupied_voxels() {
        let tree = map();
        let occupied: Vec<_> = occupied_voxels(&tree).collect();
        assert_eq!(occupied.len(), 2);
        for v in &occupied {
            assert_eq!(tree.is_occupied_at(v.center), Some(true));
        }

        let free: Vec<_> = free_voxels(&tree).collect();
        assert_eq!(free.len(), 1);
        assert_eq!(tree.is_occupied_at(free[0].center), Some(false));
    }

    #[test]
    fn a_full_depth_leaf_is_one_resolution_across_and_centered_on_its_voxel() {
        let tree = map();
        let v = occupied_voxels(&tree)
            .find(|v| (v.center.x - 1.05).abs() < 1e-6)
            .expect("the voxel that was inserted");

        assert_eq!(v.size, 0.1);
        assert_eq!(v.depth, tree.geometry().tree_depth());
        assert!((v.center.y - 0.05).abs() < 1e-6);
    }

    #[test]
    fn a_pruned_node_reports_its_own_size_and_center() {
        // Fill one whole depth-15 octant so the pruner can merge it, then check
        // the merged node describes itself rather than one of its children.
        let mut tree = OcTree::new(0.1).unwrap();
        for x in 0..2 {
            for y in 0..2 {
                for z in 0..2 {
                    let p = Point3::new(
                        0.05 + x as f32 * 0.1,
                        0.05 + y as f32 * 0.1,
                        0.05 + z as f32 * 0.1,
                    );
                    tree.update_node_at(p, true);
                }
            }
        }
        tree.prune();

        let voxels: Vec<_> = occupied_voxels(&tree).collect();
        assert_eq!(voxels.len(), 1, "the eight children should have merged");
        assert_eq!(voxels[0].size, 0.2);
        assert_eq!(voxels[0].depth, tree.geometry().tree_depth() - 1);

        // The merged node spans [0.0, 0.2) on each axis, so its center is 0.1 —
        // not the 0.15 the full-depth conversion of its key would give.
        for c in [voxels[0].center.x, voxels[0].center.y, voxels[0].center.z] {
            assert!((c - 0.1).abs() < 1e-6, "center landed at {c}");
        }
    }

    #[test]
    fn grouping_by_depth_partitions_the_occupied_cells() {
        let mut tree = OcTree::new(0.1).unwrap();
        for x in 0..2 {
            for y in 0..2 {
                for z in 0..2 {
                    tree.update_node_at(
                        Point3::new(
                            0.05 + x as f32 * 0.1,
                            0.05 + y as f32 * 0.1,
                            0.05 + z as f32 * 0.1,
                        ),
                        true,
                    );
                }
            }
        }
        tree.update_node_at(Point3::new(5.05, 5.05, 5.05), true);
        tree.prune();

        let grouped = voxels_by_depth(&tree);
        assert_eq!(grouped.len(), 2, "one merged node and one lone voxel");
        let total: usize = grouped.values().map(Vec::len).sum();
        assert_eq!(total, occupied_voxels(&tree).count());
        for (depth, voxels) in &grouped {
            assert!(voxels.iter().all(|v| v.depth == *depth));
            assert!(!voxels.is_empty(), "empty depths must not be emitted");
        }
    }

    #[test]
    fn an_empty_map_produces_nothing() {
        let tree = OcTree::new(0.1).unwrap();
        assert_eq!(occupied_voxels(&tree).count(), 0);
        assert!(voxels_by_depth(&tree).is_empty());
    }

    #[test]
    fn iteration_order_is_stable_so_marker_ids_are_too() {
        let tree = map();
        let a: Vec<_> = occupied_voxels(&tree).collect();
        let b: Vec<_> = occupied_voxels(&tree).collect();
        assert_eq!(a, b);
    }

    #[test]
    fn the_height_ramp_runs_blue_at_the_bottom_to_red_at_the_top() {
        // The reference inverts the normalized height before the sweep, so the
        // floor of a map comes out blue and the ceiling red — which is what an
        // octomap in RViz looks like.
        let low = height_color(0.0, 0.0, 10.0, 0.8);
        assert_eq!(low[2], 1.0, "the bottom of the range is blue");
        assert_eq!(low[3], 1.0, "opaque");

        let high = height_color(10.0, 0.0, 10.0, 0.8);
        assert_eq!(
            [high[0], high[1], high[2]],
            [1.0, 0.0, 0.0],
            "the top is red"
        );

        for z in [-5.0, 0.0, 2.5, 5.0, 7.5, 10.0, 15.0] {
            let c = height_color(z, 0.0, 10.0, 0.8);
            for channel in c {
                assert!(
                    (0.0..=1.0).contains(&channel),
                    "channel {channel} out of range at z={z}"
                );
            }
        }
    }

    #[test]
    fn heights_outside_the_range_clamp_to_its_ends() {
        assert_eq!(
            height_color(-100.0, 0.0, 10.0, 0.8),
            height_color(0.0, 0.0, 10.0, 0.8)
        );
        assert_eq!(
            height_color(100.0, 0.0, 10.0, 0.8),
            height_color(10.0, 0.0, 10.0, 0.8)
        );
    }

    #[test]
    fn a_degenerate_height_range_does_not_divide_by_zero() {
        let c = height_color(3.0, 3.0, 3.0, 0.8);
        assert!(c.iter().all(|v| v.is_finite()));
    }
}
