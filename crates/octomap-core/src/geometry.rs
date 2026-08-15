//! Coordinate ↔ key conversion.
//!
//! Ported from the conversion half of `OcTreeBaseImpl`. Everything here is the
//! part of the tree that depends only on resolution and depth, kept separate
//! from node storage so the octree can compose it rather than inherit it.

use crate::error::{OctomapError, Result};
use crate::key::{KeyScalar, OcTreeKey};
use crate::point::Point3;

/// Maximum tree depth used by every stock OctoMap tree.
pub const DEFAULT_TREE_DEPTH: u32 = 16;

/// Key-space offset of the world origin, `2^(DEFAULT_TREE_DEPTH - 1)`.
pub const DEFAULT_TREE_MAX_VAL: u32 = 32768;

/// Resolution and depth of a tree, plus the conversions they induce.
///
/// Conversions are deliberately written to match the reference bit-for-bit
/// rather than to be the most natural Rust formulation — the arithmetic below
/// mirrors `OcTreeBaseImpl::coordToKey` and friends, including their
/// truncation behavior.
#[derive(Debug, Clone, PartialEq)]
pub struct TreeGeometry {
    resolution: f64,
    resolution_factor: f64,
    tree_depth: u32,
    tree_max_val: u32,
    node_size_table: Vec<f64>,
}

impl TreeGeometry {
    /// Builds the geometry for a tree of the given resolution, in meters.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidResolution`] unless `resolution` is
    /// finite and strictly positive.
    pub fn new(resolution: f64) -> Result<Self> {
        Self::with_depth(resolution, DEFAULT_TREE_DEPTH, DEFAULT_TREE_MAX_VAL)
    }

    /// Builds the geometry with non-default tree constants.
    ///
    /// The reference exposes this as a protected constructor for derived trees
    /// that change the depth; stock trees should use [`TreeGeometry::new`].
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidResolution`] unless `resolution` is
    /// finite and strictly positive.
    pub fn with_depth(resolution: f64, tree_depth: u32, tree_max_val: u32) -> Result<Self> {
        if !resolution.is_finite() || resolution <= 0.0 {
            return Err(OctomapError::InvalidResolution { got: resolution });
        }

        let node_size_table = (0..=tree_depth)
            .map(|d| resolution * f64::from(1u32 << (tree_depth - d)))
            .collect();

        Ok(Self {
            resolution,
            resolution_factor: 1.0 / resolution,
            tree_depth,
            tree_max_val,
            node_size_table,
        })
    }

    /// Edge length of the smallest voxel, in meters.
    #[inline]
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// Maximum depth of the tree.
    #[inline]
    pub fn tree_depth(&self) -> u32 {
        self.tree_depth
    }

    /// Key-space offset of the world origin.
    #[inline]
    pub fn tree_max_val(&self) -> u32 {
        self.tree_max_val
    }

    /// Edge length of a node at `depth`, in meters.
    ///
    /// # Panics
    ///
    /// Panics if `depth > tree_depth`.
    #[inline]
    pub fn node_size(&self, depth: u32) -> f64 {
        self.node_size_table[depth as usize]
    }

    /// Half the edge length of the whole addressable volume, in meters.
    ///
    /// Coordinates must lie in `[-half_extent, half_extent)` to be addressable.
    #[inline]
    pub fn half_extent(&self) -> f64 {
        f64::from(self.tree_max_val) * self.resolution
    }

    // ---- coordinate -> key -------------------------------------------------

    /// Converts one axis of a world coordinate into a key, without bounds
    /// checking.
    ///
    /// Out-of-range coordinates wrap rather than report an error, matching the
    /// reference's unchecked `coordToKey`. Prefer
    /// [`TreeGeometry::coord_to_key_checked`] on any input that came from a
    /// sensor.
    #[inline]
    pub fn coord_to_key_axis(&self, coordinate: f64) -> KeyScalar {
        let scaled = (self.resolution_factor * coordinate).floor() as i64;
        (scaled + i64::from(self.tree_max_val)) as KeyScalar
    }

    /// Converts a world coordinate into a key, without bounds checking.
    #[inline]
    pub fn coord_to_key(&self, point: Point3) -> OcTreeKey {
        OcTreeKey::new(
            self.coord_to_key_axis(f64::from(point.x)),
            self.coord_to_key_axis(f64::from(point.y)),
            self.coord_to_key_axis(f64::from(point.z)),
        )
    }

    /// Converts one axis of a world coordinate into a key at `depth`, without
    /// bounds checking.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn coord_to_key_axis_at_depth(&self, coordinate: f64, depth: u32) -> Result<KeyScalar> {
        self.check_depth(depth)?;

        let keyval = (self.resolution_factor * coordinate).floor() as i64;
        let diff = self.tree_depth - depth;
        if diff == 0 {
            return Ok((keyval + i64::from(self.tree_max_val)) as KeyScalar);
        }
        let snapped = (keyval >> diff) << diff;
        Ok((snapped + (1i64 << (diff - 1)) + i64::from(self.tree_max_val)) as KeyScalar)
    }

    /// Converts a world coordinate into a key at `depth`, without bounds
    /// checking.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn coord_to_key_at_depth(&self, point: Point3, depth: u32) -> Result<OcTreeKey> {
        if depth == self.tree_depth {
            return Ok(self.coord_to_key(point));
        }
        Ok(OcTreeKey::new(
            self.coord_to_key_axis_at_depth(f64::from(point.x), depth)?,
            self.coord_to_key_axis_at_depth(f64::from(point.y), depth)?,
            self.coord_to_key_axis_at_depth(f64::from(point.z), depth)?,
        ))
    }

    /// Converts one axis of a world coordinate into a key, rejecting
    /// coordinates outside the addressable volume.
    #[inline]
    pub fn coord_to_key_axis_checked(&self, coordinate: f64) -> Option<KeyScalar> {
        if !coordinate.is_finite() {
            return None;
        }
        let scaled =
            (self.resolution_factor * coordinate).floor() as i64 + i64::from(self.tree_max_val);
        if scaled >= 0 && scaled < i64::from(2 * self.tree_max_val) {
            Some(scaled as KeyScalar)
        } else {
            None
        }
    }

    /// Converts a world coordinate into a key, rejecting coordinates outside
    /// the addressable volume.
    #[inline]
    pub fn coord_to_key_checked(&self, point: Point3) -> Option<OcTreeKey> {
        Some(OcTreeKey::new(
            self.coord_to_key_axis_checked(f64::from(point.x))?,
            self.coord_to_key_axis_checked(f64::from(point.y))?,
            self.coord_to_key_axis_checked(f64::from(point.z))?,
        ))
    }

    /// Converts a world coordinate into a key at `depth`, rejecting
    /// coordinates outside the addressable volume.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn coord_to_key_checked_at_depth(
        &self,
        point: Point3,
        depth: u32,
    ) -> Result<Option<OcTreeKey>> {
        self.check_depth(depth)?;

        let Some(key) = self.coord_to_key_checked(point) else {
            return Ok(None);
        };
        Ok(Some(self.adjust_key_at_depth(key, depth)?))
    }

    // ---- key -> key --------------------------------------------------------

    /// Snaps one axis of a bottom-level key to the voxel that contains it at
    /// `depth`.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn adjust_key_axis_at_depth(&self, key: KeyScalar, depth: u32) -> Result<KeyScalar> {
        self.check_depth(depth)?;

        let diff = self.tree_depth - depth;
        if diff == 0 {
            return Ok(key);
        }
        // The reference performs this in `unsigned int` and truncates to 16
        // bits. Signed arithmetic here differs only in bits that truncation
        // discards, so the two agree for every input.
        let centered = i64::from(key) - i64::from(self.tree_max_val);
        let snapped = (centered >> diff) << diff;
        Ok((snapped + (1i64 << (diff - 1)) + i64::from(self.tree_max_val)) as KeyScalar)
    }

    /// Snaps a bottom-level key to the voxel that contains it at `depth`.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn adjust_key_at_depth(&self, key: OcTreeKey, depth: u32) -> Result<OcTreeKey> {
        if depth == self.tree_depth {
            return Ok(key);
        }
        Ok(OcTreeKey::new(
            self.adjust_key_axis_at_depth(key.k[0], depth)?,
            self.adjust_key_axis_at_depth(key.k[1], depth)?,
            self.adjust_key_axis_at_depth(key.k[2], depth)?,
        ))
    }

    // ---- key -> coordinate -------------------------------------------------

    /// Center of the voxel addressed by one axis of a bottom-level key.
    #[inline]
    pub fn key_to_coord_axis(&self, key: KeyScalar) -> f64 {
        ((f64::from(key) - f64::from(self.tree_max_val)) + 0.5) * self.resolution
    }

    /// Center of the voxel addressed by a bottom-level key.
    #[inline]
    pub fn key_to_coord(&self, key: OcTreeKey) -> Point3 {
        Point3::new(
            self.key_to_coord_axis(key.k[0]) as f32,
            self.key_to_coord_axis(key.k[1]) as f32,
            self.key_to_coord_axis(key.k[2]) as f32,
        )
    }

    /// Center of the voxel addressed by one axis of a key at `depth`.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn key_to_coord_axis_at_depth(&self, key: KeyScalar, depth: u32) -> Result<f64> {
        self.check_depth(depth)?;

        // The root spans the whole volume, so it is centered on the origin.
        if depth == 0 {
            return Ok(0.0);
        }
        if depth == self.tree_depth {
            return Ok(self.key_to_coord_axis(key));
        }
        let divisor = f64::from(1u32 << (self.tree_depth - depth));
        let cell = ((f64::from(key) - f64::from(self.tree_max_val)) / divisor).floor();
        Ok((cell + 0.5) * self.node_size(depth))
    }

    /// Center of the voxel addressed by a key at `depth`.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn key_to_coord_at_depth(&self, key: OcTreeKey, depth: u32) -> Result<Point3> {
        Ok(Point3::new(
            self.key_to_coord_axis_at_depth(key.k[0], depth)? as f32,
            self.key_to_coord_axis_at_depth(key.k[1], depth)? as f32,
            self.key_to_coord_axis_at_depth(key.k[2], depth)? as f32,
        ))
    }

    /// Confirms `depth` is addressable in this tree.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidDepth`] if `depth > tree_depth`.
    #[inline]
    pub fn validate_depth(&self, depth: u32) -> Result<()> {
        self.check_depth(depth)
    }

    #[inline]
    fn check_depth(&self, depth: u32) -> Result<()> {
        if depth > self.tree_depth {
            return Err(OctomapError::InvalidDepth {
                got: depth,
                tree_depth: self.tree_depth,
            });
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geom() -> TreeGeometry {
        TreeGeometry::new(0.1).unwrap()
    }

    #[test]
    fn rejects_non_positive_or_non_finite_resolution() {
        assert_eq!(
            TreeGeometry::new(-0.1).unwrap_err(),
            OctomapError::InvalidResolution { got: -0.1 }
        );
        assert!(matches!(
            TreeGeometry::new(0.0),
            Err(OctomapError::InvalidResolution { .. })
        ));
        assert!(matches!(
            TreeGeometry::new(f64::NAN),
            Err(OctomapError::InvalidResolution { .. })
        ));
        assert!(matches!(
            TreeGeometry::new(f64::INFINITY),
            Err(OctomapError::InvalidResolution { .. })
        ));
    }

    #[test]
    fn origin_maps_to_tree_max_val() {
        let g = geom();
        assert_eq!(
            g.coord_to_key(Point3::ORIGIN),
            OcTreeKey::new(32768, 32768, 32768)
        );
    }

    #[test]
    fn coord_to_key_offsets_by_tree_max_val() {
        let g = geom();
        assert_eq!(g.coord_to_key_axis(1.2), 32768 + 12);
        assert_eq!(g.coord_to_key_axis(1.35), 32768 + 13);
        assert_eq!(g.coord_to_key_axis(-0.05), 32768 - 1);
    }

    #[test]
    fn scaling_multiplies_by_the_reciprocal_rather_than_dividing() {
        // Not interchangeable in IEEE754, and the difference is observable:
        // 1.2 / 0.1 == 11.999999999999998 (floor 11), but the reference caches
        // resolution_factor = 1.0 / resolution and multiplies, which gives
        // exactly 12.0 (floor 12). Dividing here would put points on the wrong
        // side of a voxel boundary relative to C++.
        let g = geom();
        assert_eq!((1.2f64 / 0.1).floor(), 11.0);
        assert_eq!(g.coord_to_key_axis(1.2) as i64 - 32768, 12);
    }

    #[test]
    fn negative_coordinates_land_below_the_origin_key() {
        let g = geom();
        assert_eq!(g.coord_to_key_axis(-1.0), 32768 - 10);
        assert_eq!(g.coord_to_key_axis(-0.001), 32768 - 1);
    }

    #[test]
    fn key_to_coord_returns_the_voxel_center() {
        let g = geom();
        // Key 32768 covers [0.0, 0.1), so its center is 0.05.
        assert!((g.key_to_coord_axis(32768) - 0.05).abs() < 1e-12);
        assert!((g.key_to_coord_axis(32767) + 0.05).abs() < 1e-12);
    }

    #[test]
    fn key_to_coord_then_coord_to_key_is_the_identity() {
        let g = geom();
        for raw in [0u16, 1, 12345, 32767, 32768, 32769, 65534, 65535] {
            let center = g.key_to_coord_axis(raw);
            assert_eq!(
                g.coord_to_key_axis(center),
                raw,
                "round trip failed for key {raw} (center {center})"
            );
        }
    }

    #[test]
    fn coord_to_key_then_key_to_coord_stays_inside_the_voxel() {
        let g = geom();
        let res = g.resolution();
        for &c in &[0.0f64, 0.049, 0.051, 1.2, -1.2, -0.001, 123.456, -98.7] {
            let key = g.coord_to_key_axis(c);
            let center = g.key_to_coord_axis(key);
            assert!(
                (c - center).abs() <= res / 2.0 + 1e-9,
                "coord {c} mapped to center {center}, which is more than half a voxel away"
            );
        }
    }

    #[test]
    fn checked_conversion_rejects_coordinates_outside_the_volume() {
        let g = geom();
        let half = g.half_extent(); // 3276.8 m at 0.1 m resolution

        assert!(g.coord_to_key_axis_checked(0.0).is_some());
        assert!(g.coord_to_key_axis_checked(half - 1.0).is_some());
        assert!(g.coord_to_key_axis_checked(-half).is_some());

        assert_eq!(g.coord_to_key_axis_checked(half), None);
        assert_eq!(g.coord_to_key_axis_checked(-half - 1.0), None);
        assert_eq!(g.coord_to_key_axis_checked(f64::NAN), None);
        assert_eq!(g.coord_to_key_axis_checked(f64::INFINITY), None);
    }

    #[test]
    fn node_size_doubles_on_the_way_up() {
        let g = geom();
        assert!((g.node_size(16) - 0.1).abs() < 1e-12);
        assert!((g.node_size(15) - 0.2).abs() < 1e-12);
        assert!((g.node_size(14) - 0.4).abs() < 1e-12);
        // The root spans the whole addressable volume.
        assert!((g.node_size(0) - 2.0 * g.half_extent()).abs() < 1e-9);
    }

    #[test]
    fn adjust_key_at_depth_snaps_to_the_containing_voxel() {
        let g = geom();
        // At depth 15 voxels are two bottom-level cells wide, and the reference
        // addresses them by their upper cell.
        let a = g.adjust_key_axis_at_depth(32768, 15).unwrap();
        let b = g.adjust_key_axis_at_depth(32769, 15).unwrap();
        assert_eq!(a, b, "neighbouring keys must share a parent at depth 15");

        // Depth == tree_depth is the identity.
        assert_eq!(g.adjust_key_axis_at_depth(12345, 16).unwrap(), 12345);
    }

    #[test]
    fn adjust_key_at_depth_is_idempotent() {
        let g = geom();
        for depth in 1..=16u32 {
            for raw in [0u16, 1, 32767, 32768, 40000, 65535] {
                let once = g.adjust_key_axis_at_depth(raw, depth).unwrap();
                let twice = g.adjust_key_axis_at_depth(once, depth).unwrap();
                assert_eq!(once, twice, "depth {depth}, key {raw}");
            }
        }
    }

    #[test]
    fn coord_to_key_at_depth_agrees_with_adjust_key_at_depth() {
        let g = geom();
        for depth in 1..=16u32 {
            for &c in &[0.0f64, 1.2, -1.2, 55.5, -300.25] {
                let direct = g.coord_to_key_axis_at_depth(c, depth).unwrap();
                let via_adjust = g
                    .adjust_key_axis_at_depth(g.coord_to_key_axis(c), depth)
                    .unwrap();
                assert_eq!(direct, via_adjust, "depth {depth}, coord {c}");
            }
        }
    }

    #[test]
    fn root_is_centered_on_the_origin() {
        let g = geom();
        assert_eq!(g.key_to_coord_axis_at_depth(32768, 0).unwrap(), 0.0);
        assert_eq!(g.key_to_coord_axis_at_depth(0, 0).unwrap(), 0.0);
    }

    #[test]
    fn depth_beyond_tree_depth_is_an_error() {
        let g = geom();
        assert_eq!(
            g.adjust_key_axis_at_depth(0, 17).unwrap_err(),
            OctomapError::InvalidDepth {
                got: 17,
                tree_depth: 16
            }
        );
        assert!(g.coord_to_key_axis_at_depth(0.0, 17).is_err());
        assert!(g.key_to_coord_axis_at_depth(0, 17).is_err());
    }

    #[test]
    fn different_resolutions_scale_the_addressable_volume() {
        let fine = TreeGeometry::new(0.01).unwrap();
        let coarse = TreeGeometry::new(1.0).unwrap();
        assert!((fine.half_extent() - 327.68).abs() < 1e-9);
        assert!((coarse.half_extent() - 32768.0).abs() < 1e-9);
    }
}
