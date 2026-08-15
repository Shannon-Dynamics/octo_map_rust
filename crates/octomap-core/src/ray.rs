//! Ray traversal and point clouds.
//!
//! Ported from `OcTreeBaseImpl::computeRayKeys` and the ray half of
//! `OccupancyOcTreeBase`. The traversal is the Amanatides–Woo 3D DDA the
//! reference uses.
//!
//! # A deliberate inconsistency, preserved
//!
//! `computeRayKeys` and `castRay` set up identical DDA state except for one
//! line. Computing the voxel border, `computeRayKeys` narrows the offset to
//! `float`:
//!
//! ```text
//! voxelBorder += (float) (step[i] * resolution * 0.5);   // computeRayKeys
//! voxelBorder += double(step[i] * resolution * 0.5);     // castRay
//! ```
//!
//! That narrowing shifts `tMax` slightly, which can move where the ray crosses
//! a voxel boundary. Both are reproduced exactly as written; unifying them
//! would be a behavior change.

use std::collections::HashSet;

use crate::geometry::TreeGeometry;
use crate::key::{KeyRay, OcTreeKey};
use crate::occupancy::OcTree;
use crate::point::Point3;

/// A set of sensor endpoints in world coordinates.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct PointCloud {
    points: Vec<Point3>,
}

impl PointCloud {
    /// An empty cloud.
    #[inline]
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// An empty cloud with room for `capacity` points.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
        }
    }

    /// Appends a point.
    #[inline]
    pub fn push(&mut self, point: Point3) {
        self.points.push(point);
    }

    /// Number of points.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// True when the cloud holds no points.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Removes every point.
    #[inline]
    pub fn clear(&mut self) {
        self.points.clear();
    }

    /// The points, in insertion order.
    #[inline]
    pub fn as_slice(&self) -> &[Point3] {
        &self.points
    }

    /// The points, mutably.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [Point3] {
        &mut self.points
    }

    /// Iterates the points.
    #[inline]
    pub fn iter(&self) -> core::slice::Iter<'_, Point3> {
        self.points.iter()
    }

    /// Iterates the points mutably.
    #[inline]
    pub fn iter_mut(&mut self) -> core::slice::IterMut<'_, Point3> {
        self.points.iter_mut()
    }

    /// Translates every point by `offset`.
    pub fn translate(&mut self, offset: Point3) {
        for p in &mut self.points {
            *p += offset;
        }
    }
}

impl From<Vec<Point3>> for PointCloud {
    #[inline]
    fn from(points: Vec<Point3>) -> Self {
        Self { points }
    }
}

impl FromIterator<Point3> for PointCloud {
    fn from_iter<I: IntoIterator<Item = Point3>>(iter: I) -> Self {
        Self {
            points: iter.into_iter().collect(),
        }
    }
}

impl<'a> IntoIterator for &'a PointCloud {
    type Item = &'a Point3;
    type IntoIter = core::slice::Iter<'a, Point3>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.points.iter()
    }
}

impl core::ops::Index<usize> for PointCloud {
    type Output = Point3;

    #[inline]
    fn index(&self, i: usize) -> &Point3 {
        &self.points[i]
    }
}

/// Why a ray cast ended without finding an occupied voxel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RayCastMiss {
    /// The origin lies outside the addressable volume.
    OriginOutOfBounds,
    /// The direction vector was zero, so there is nothing to follow.
    ZeroDirection,
    /// The ray entered a voxel the map has never observed, and
    /// `ignore_unknown` was not set.
    UnknownVoxel,
    /// The ray travelled past `max_range`.
    MaxRange,
    /// The ray walked off the edge of the addressable volume.
    OutOfBounds,
}

/// The result of a ray cast.
///
/// The reference returns a `bool` and writes the endpoint through an out
/// parameter, leaving that parameter untouched in some failure paths. This
/// enum keeps the same information without the "is `end` meaningful?" question.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RayCast {
    /// The ray terminated on an occupied voxel. The payload is its center.
    Hit(Point3),
    /// The ray ended without hitting anything.
    Miss {
        /// Center of the last voxel reached, when the ray got that far.
        last: Option<Point3>,
        /// What stopped the ray.
        reason: RayCastMiss,
    },
}

impl RayCast {
    /// True when the ray terminated on an occupied voxel.
    #[inline]
    pub fn is_hit(&self) -> bool {
        matches!(self, Self::Hit(_))
    }

    /// The occupied voxel's center, if the ray hit one.
    #[inline]
    pub fn hit_point(&self) -> Option<Point3> {
        match self {
            Self::Hit(p) => Some(*p),
            Self::Miss { .. } => None,
        }
    }

    /// The last voxel center the ray reached, hit or not.
    ///
    /// Corresponds to the reference's `end` out-parameter.
    #[inline]
    pub fn end(&self) -> Option<Point3> {
        match self {
            Self::Hit(p) => Some(*p),
            Self::Miss { last, .. } => *last,
        }
    }
}

/// Per-axis DDA state.
struct Dda {
    step: [i32; 3],
    t_max: [f64; 3],
    t_delta: [f64; 3],
}

impl Dda {
    /// Sets up the traversal. `narrow_border_to_f32` selects between the
    /// `computeRayKeys` and `castRay` spellings of the voxel-border offset —
    /// see the module docs.
    fn new(
        geometry: &TreeGeometry,
        origin: Point3,
        direction: Point3,
        start: OcTreeKey,
        narrow_border_to_f32: bool,
    ) -> Self {
        let resolution = geometry.resolution();
        let mut step = [0i32; 3];
        let mut t_max = [f64::MAX; 3];
        let mut t_delta = [f64::MAX; 3];

        for i in 0..3 {
            let d = direction[i];
            step[i] = if d > 0.0 {
                1
            } else if d < 0.0 {
                -1
            } else {
                0
            };

            if step[i] != 0 {
                let mut voxel_border = geometry.key_to_coord_axis(start.k[i]);
                let offset = f64::from(step[i]) * resolution * 0.5;
                voxel_border += if narrow_border_to_f32 {
                    f64::from(offset as f32)
                } else {
                    offset
                };

                let d = f64::from(d);
                t_max[i] = (voxel_border - f64::from(origin[i])) / d;
                t_delta[i] = resolution / d.abs();
            }
        }

        Self {
            step,
            t_max,
            t_delta,
        }
    }

    /// The axis with the smallest `t_max`, using the reference's exact
    /// comparison order so ties break identically.
    #[inline]
    fn next_axis(&self) -> usize {
        if self.t_max[0] < self.t_max[1] {
            if self.t_max[0] < self.t_max[2] {
                0
            } else {
                2
            }
        } else if self.t_max[1] < self.t_max[2] {
            1
        } else {
            2
        }
    }

    /// Advances `key` along `axis` and bumps that axis's `t_max`.
    #[inline]
    fn advance(&mut self, key: &mut OcTreeKey, axis: usize) {
        key.k[axis] = (i32::from(key.k[axis]) + self.step[axis]) as u16;
        self.t_max[axis] += self.t_delta[axis];
    }

    #[inline]
    fn min_t_max(&self) -> f64 {
        self.t_max[0].min(self.t_max[1]).min(self.t_max[2])
    }

    #[inline]
    fn is_stationary(&self) -> bool {
        self.step == [0, 0, 0]
    }
}

/// Collects the voxels a ray passes through, from `origin` up to but **not
/// including** the voxel containing `end`.
///
/// Returns `false` when either endpoint lies outside the addressable volume,
/// leaving `ray` empty. When both endpoints fall in the same voxel the call
/// succeeds with an empty ray — there is no free space between them.
///
/// # Divergence from the reference
///
/// `KeyRay` in C++ asserts when it reaches its 100000-key ceiling, which is a
/// no-op in release builds and writes out of bounds. Here the traversal stops
/// at the ceiling instead. Only reachable for a ray spanning most of the map.
pub fn compute_ray_keys(
    geometry: &TreeGeometry,
    origin: Point3,
    end: Point3,
    ray: &mut KeyRay,
) -> bool {
    ray.clear();

    let (Some(key_origin), Some(key_end)) = (
        geometry.coord_to_key_checked(origin),
        geometry.coord_to_key_checked(end),
    ) else {
        return false;
    };

    if key_origin == key_end {
        return true;
    }

    ray.push(key_origin);

    let delta = end - origin;
    let length = delta.norm();
    let direction = delta / length;

    let mut dda = Dda::new(geometry, origin, direction, key_origin, true);
    let mut current = key_origin;

    loop {
        let axis = dda.next_axis();
        dda.advance(&mut current, axis);

        if current == key_end {
            break;
        }

        // Discretization error can carry the walk past the endpoint without
        // ever landing on its key; this is the reference's guard against
        // running away when that happens.
        if dda.min_t_max() > f64::from(length) {
            break;
        }

        if !ray.push(current) {
            break;
        }
    }

    true
}

impl OcTree {
    /// Collects the voxels a ray passes through, excluding the endpoint's.
    ///
    /// See [`compute_ray_keys`].
    pub fn compute_ray_keys(&self, origin: Point3, end: Point3, ray: &mut KeyRay) -> bool {
        compute_ray_keys(self.geometry(), origin, end, ray)
    }

    /// The world coordinates of the voxels a ray passes through.
    ///
    /// Returns `None` when either endpoint is outside the addressable volume.
    pub fn compute_ray(&self, origin: Point3, end: Point3) -> Option<Vec<Point3>> {
        let mut ray = KeyRay::new();
        if !self.compute_ray_keys(origin, end, &mut ray) {
            return None;
        }
        Some(
            ray.iter()
                .map(|k| self.geometry().key_to_coord(*k))
                .collect(),
        )
    }

    /// Marks every voxel along the ray free, without touching the endpoint.
    ///
    /// Returns `false` when either endpoint is outside the addressable volume.
    pub fn integrate_miss_on_ray(&mut self, origin: Point3, end: Point3, lazy_eval: bool) -> bool {
        let mut ray = KeyRay::new();
        if !self.compute_ray_keys(origin, end, &mut ray) {
            return false;
        }
        let miss = self.sensor().prob_miss_log();
        for key in ray.as_slice() {
            self.update_node_log_odds(*key, miss, lazy_eval);
        }
        true
    }

    /// Inserts a single sensor ray: free along the way, occupied at the end.
    ///
    /// When `max_range` is positive and the ray is longer, the ray is truncated
    /// and **no** occupied endpoint is recorded — a reading past the sensor's
    /// range says nothing about what is there.
    ///
    /// Returns `false` when either endpoint is outside the addressable volume.
    pub fn insert_ray(
        &mut self,
        origin: Point3,
        end: Point3,
        max_range: f64,
        lazy_eval: bool,
    ) -> bool {
        if max_range > 0.0 && f64::from((end - origin).norm()) > max_range {
            let Some(direction) = (end - origin).normalized() else {
                return false;
            };
            let new_end = origin + direction * (max_range as f32);
            return self.integrate_miss_on_ray(origin, new_end, lazy_eval);
        }

        if !self.integrate_miss_on_ray(origin, end, lazy_eval) {
            return false;
        }
        let Some(key) = self.geometry().coord_to_key_checked(end) else {
            return false;
        };
        self.update_node(key, true);
        true
    }

    /// Follows `direction` from `origin` until it meets an occupied voxel.
    ///
    /// `ignore_unknown` lets the ray pass through voxels the map has never
    /// observed; otherwise the first such voxel stops it. `max_range` of zero
    /// or less means unlimited.
    pub fn cast_ray(
        &self,
        origin: Point3,
        direction: Point3,
        ignore_unknown: bool,
        max_range: f64,
    ) -> RayCast {
        let geometry = self.geometry();
        let Some(mut current) = geometry.coord_to_key_checked(origin) else {
            return RayCast::Miss {
                last: None,
                reason: RayCastMiss::OriginOutOfBounds,
            };
        };

        match self.search(current) {
            Some(node) => {
                if self.sensor().is_occupied(*node.value()) {
                    // The origin itself is inside something solid. Report the
                    // voxel center, not the origin, since the origin need not
                    // sit at a center.
                    return RayCast::Hit(geometry.key_to_coord(current));
                }
            }
            None => {
                if !ignore_unknown {
                    return RayCast::Miss {
                        last: Some(geometry.key_to_coord(current)),
                        reason: RayCastMiss::UnknownVoxel,
                    };
                }
            }
        }

        let Some(direction) = direction.normalized() else {
            return RayCast::Miss {
                last: None,
                reason: RayCastMiss::ZeroDirection,
            };
        };

        // castRay keeps the border offset in double, unlike computeRayKeys.
        let mut dda = Dda::new(geometry, origin, direction, current, false);
        if dda.is_stationary() {
            return RayCast::Miss {
                last: None,
                reason: RayCastMiss::ZeroDirection,
            };
        }

        let max_range_set = max_range > 0.0;
        let max_range_sq = max_range * max_range;
        let limit = 2 * geometry.tree_max_val() - 1;

        loop {
            let axis = dda.next_axis();

            let at_edge = (dda.step[axis] < 0 && current.k[axis] == 0)
                || (dda.step[axis] > 0 && u32::from(current.k[axis]) == limit);
            if at_edge {
                return RayCast::Miss {
                    last: Some(geometry.key_to_coord(current)),
                    reason: RayCastMiss::OutOfBounds,
                };
            }

            dda.advance(&mut current, axis);
            let end = geometry.key_to_coord(current);

            if max_range_set {
                let d = end - origin;
                let dist_sq = f64::from(d.x) * f64::from(d.x)
                    + f64::from(d.y) * f64::from(d.y)
                    + f64::from(d.z) * f64::from(d.z);
                if dist_sq > max_range_sq {
                    return RayCast::Miss {
                        last: Some(end),
                        reason: RayCastMiss::MaxRange,
                    };
                }
            }

            match self.search(current) {
                Some(node) => {
                    if self.sensor().is_occupied(*node.value()) {
                        return RayCast::Hit(end);
                    }
                }
                None => {
                    if !ignore_unknown {
                        return RayCast::Miss {
                            last: Some(end),
                            reason: RayCastMiss::UnknownVoxel,
                        };
                    }
                }
            }
        }
    }

    /// Works out which voxels a scan frees and which it occupies, without
    /// touching the map.
    ///
    /// The two sets are disjoint: a voxel that is both an endpoint and on
    /// another ray's path counts as **occupied**, matching the reference's
    /// "prefer occupied" rule.
    ///
    /// `max_range` of less than zero means unlimited. A point beyond
    /// `max_range` contributes free space up to that range and no endpoint.
    pub fn compute_update(
        &self,
        scan: &PointCloud,
        origin: Point3,
        max_range: f64,
    ) -> (HashSet<OcTreeKey>, HashSet<OcTreeKey>) {
        let mut free = HashSet::new();
        let mut occupied = HashSet::new();
        let mut ray = KeyRay::new();
        let geometry = self.geometry();

        for &p in scan.iter() {
            let in_range = max_range < 0.0 || f64::from((p - origin).norm()) <= max_range;

            if in_range {
                if compute_ray_keys(geometry, origin, p, &mut ray) {
                    free.extend(ray.as_slice().iter().copied());
                }
                if let Some(key) = geometry.coord_to_key_checked(p) {
                    occupied.insert(key);
                }
            } else if let Some(direction) = (p - origin).normalized() {
                let new_end = origin + direction * (max_range as f32);
                if compute_ray_keys(geometry, origin, new_end, &mut ray) {
                    free.extend(ray.as_slice().iter().copied());
                }
            }
        }

        free.retain(|k| !occupied.contains(k));
        (free, occupied)
    }

    /// Like [`OcTree::compute_update`], but collapses duplicate endpoints to
    /// one ray per voxel first.
    ///
    /// Much cheaper for dense scans, and slightly different: rays are cast to
    /// voxel *centers* rather than to the original points.
    pub fn compute_discrete_update(
        &self,
        scan: &PointCloud,
        origin: Point3,
        max_range: f64,
    ) -> (HashSet<OcTreeKey>, HashSet<OcTreeKey>) {
        let geometry = self.geometry();
        let mut seen = HashSet::new();
        let mut discrete = PointCloud::with_capacity(scan.len());

        for &p in scan.iter() {
            let key = geometry.coord_to_key(p);
            if seen.insert(key) {
                discrete.push(geometry.key_to_coord(key));
            }
        }
        self.compute_update(&discrete, origin, max_range)
    }

    /// Integrates a full scan taken from `sensor_origin`.
    ///
    /// Free cells are applied before occupied ones, matching the reference.
    /// With `discretize` set, duplicate endpoints collapse to one ray per voxel
    /// first.
    pub fn insert_point_cloud(
        &mut self,
        scan: &PointCloud,
        sensor_origin: Point3,
        max_range: f64,
        lazy_eval: bool,
        discretize: bool,
    ) {
        let (free, occupied) = if discretize {
            self.compute_discrete_update(scan, sensor_origin, max_range)
        } else {
            self.compute_update(scan, sensor_origin, max_range)
        };

        let miss = self.sensor().prob_miss_log();
        let hit = self.sensor().prob_hit_log();
        for key in free {
            self.update_node_log_odds(key, miss, lazy_eval);
        }
        for key in occupied {
            self.update_node_log_odds(key, hit, lazy_eval);
        }
    }

    /// Integrates a scan by casting each ray separately.
    ///
    /// Slower than [`OcTree::insert_point_cloud`] and it does **not** make the
    /// free and occupied sets disjoint, so a voxel can be freed by one ray and
    /// occupied by another within the same call. The reference keeps both
    /// entry points for the same reason.
    pub fn insert_point_cloud_rays(
        &mut self,
        scan: &PointCloud,
        sensor_origin: Point3,
        lazy_eval: bool,
    ) {
        let mut ray = KeyRay::new();
        for &p in scan.iter() {
            if !compute_ray_keys(self.geometry(), sensor_origin, p, &mut ray) {
                continue;
            }
            let miss = self.sensor().prob_miss_log();
            let keys: Vec<OcTreeKey> = ray.as_slice().to_vec();
            for key in keys {
                self.update_node_log_odds(key, miss, lazy_eval);
            }
            if let Some(key) = self.geometry().coord_to_key_checked(p) {
                let hit = self.sensor().prob_hit_log();
                self.update_node_log_odds(key, hit, lazy_eval);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree() -> OcTree {
        OcTree::new(0.1).unwrap()
    }

    #[test]
    fn point_cloud_basics() {
        let mut pc = PointCloud::new();
        assert!(pc.is_empty());
        pc.push(Point3::new(1.0, 2.0, 3.0));
        pc.push(Point3::new(4.0, 5.0, 6.0));
        assert_eq!(pc.len(), 2);
        assert_eq!(pc[0], Point3::new(1.0, 2.0, 3.0));

        pc.translate(Point3::new(1.0, 0.0, 0.0));
        assert_eq!(pc[0], Point3::new(2.0, 2.0, 3.0));

        pc.clear();
        assert!(pc.is_empty());
    }

    #[test]
    fn a_ray_within_one_voxel_yields_no_free_cells() {
        let t = tree();
        let mut ray = KeyRay::new();
        // Both ends inside the same 0.1 m voxel.
        assert!(t.compute_ray_keys(
            Point3::new(0.01, 0.01, 0.01),
            Point3::new(0.02, 0.02, 0.02),
            &mut ray
        ));
        assert!(
            ray.is_empty(),
            "no free space between two points in one voxel"
        );
    }

    #[test]
    fn an_axis_aligned_ray_visits_every_voxel_along_it() {
        let t = tree();
        let mut ray = KeyRay::new();
        assert!(t.compute_ray_keys(
            Point3::new(0.05, 0.05, 0.05),
            Point3::new(1.05, 0.05, 0.05),
            &mut ray
        ));

        // 0.05 -> 1.05 at 0.1 m spans ten voxel boundaries; the endpoint's own
        // voxel is excluded, so ten keys come back.
        assert_eq!(ray.len(), 10);

        let keys = ray.as_slice();
        // Only x should change, and monotonically.
        for w in keys.windows(2) {
            assert_eq!(w[0].y(), w[1].y());
            assert_eq!(w[0].z(), w[1].z());
            assert_eq!(w[1].x(), w[0].x() + 1);
        }
        assert_eq!(
            keys[0],
            t.geometry().coord_to_key(Point3::new(0.05, 0.05, 0.05))
        );
    }

    #[test]
    fn the_endpoint_voxel_is_never_in_the_ray() {
        let t = tree();
        let mut ray = KeyRay::new();
        let end = Point3::new(1.05, 0.05, 0.05);
        t.compute_ray_keys(Point3::new(0.05, 0.05, 0.05), end, &mut ray);

        let end_key = t.geometry().coord_to_key(end);
        assert!(
            !ray.as_slice().contains(&end_key),
            "the occupied endpoint must not be marked free"
        );
    }

    #[test]
    fn a_ray_visits_each_voxel_at_most_once() {
        let t = tree();
        let mut ray = KeyRay::new();
        t.compute_ray_keys(
            Point3::new(0.05, 0.05, 0.05),
            Point3::new(1.05, 0.35, -0.25),
            &mut ray,
        );

        let unique: std::collections::HashSet<_> = ray.as_slice().iter().collect();
        assert_eq!(unique.len(), ray.len(), "the walk revisited a voxel");
        assert_eq!(
            ray.as_slice()[0],
            t.geometry().coord_to_key(Point3::new(0.05, 0.05, 0.05)),
            "the ray must start at the origin's voxel"
        );
    }

    #[test]
    fn a_reversed_ray_is_not_required_to_match() {
        // A DDA walk is not symmetric under reversal: tMax is seeded from the
        // origin's position within its voxel, so a ray that grazes a corner can
        // step through a different pair of neighbours each way. The reference
        // behaves the same, and tests/golden/ray.csv pins both directions
        // against it. This test exists to record that the asymmetry is expected
        // rather than a defect waiting to be "fixed".
        let t = tree();
        let a = Point3::new(0.05, 0.05, 0.05);
        let b = Point3::new(1.05, 0.35, -0.25);

        let mut forward = KeyRay::new();
        let mut backward = KeyRay::new();
        t.compute_ray_keys(a, b, &mut forward);
        t.compute_ray_keys(b, a, &mut backward);

        // Both must at least connect the same two voxels.
        assert_eq!(forward.as_slice()[0], t.geometry().coord_to_key(a));
        assert_eq!(backward.as_slice()[0], t.geometry().coord_to_key(b));
    }

    #[test]
    fn rays_out_of_bounds_are_rejected() {
        let t = tree();
        let mut ray = KeyRay::new();
        assert!(!t.compute_ray_keys(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0e9, 0.0, 0.0),
            &mut ray
        ));
        assert!(ray.is_empty());
        assert!(!t.compute_ray_keys(
            Point3::new(1.0e9, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            &mut ray
        ));
    }

    #[test]
    fn insert_ray_frees_the_path_and_occupies_the_end() {
        let mut t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        let end = Point3::new(1.05, 0.05, 0.05);
        assert!(t.insert_ray(origin, end, -1.0, false));

        let end_key = t.geometry().coord_to_key(end);
        assert_eq!(t.is_occupied(end_key), Some(true));

        let origin_key = t.geometry().coord_to_key(origin);
        assert_eq!(t.is_occupied(origin_key), Some(false));
    }

    #[test]
    fn a_truncated_ray_records_no_endpoint() {
        let mut t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        let end = Point3::new(5.05, 0.05, 0.05);
        // The reading is past the sensor's range, so it says nothing about the
        // endpoint — only that the first metre is clear.
        assert!(t.insert_ray(origin, end, 1.0, false));

        let end_key = t.geometry().coord_to_key(end);
        assert_eq!(t.is_occupied(end_key), None, "endpoint must stay unknown");

        let near = t.geometry().coord_to_key(Point3::new(0.55, 0.05, 0.05));
        assert_eq!(t.is_occupied(near), Some(false), "path should be freed");
    }

    #[test]
    fn cast_ray_finds_an_occupied_voxel() {
        let mut t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        let obstacle = Point3::new(1.05, 0.05, 0.05);
        t.insert_ray(origin, obstacle, -1.0, false);

        let result = t.cast_ray(origin, Point3::new(1.0, 0.0, 0.0), true, -1.0);
        assert!(result.is_hit());
        let hit = result.hit_point().unwrap();
        assert!((hit.x - 1.05).abs() < 0.06, "hit at {hit:?}");
    }

    #[test]
    fn cast_ray_stops_at_unknown_space_unless_told_otherwise() {
        let mut t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        // Free the path without marking an endpoint — insert_ray would occupy
        // the far end, and the cast would stop there instead of on unknown space.
        t.integrate_miss_on_ray(origin, Point3::new(0.55, 0.05, 0.05), false);

        // Beyond the short freed stretch everything is unknown.
        let strict = t.cast_ray(origin, Point3::new(1.0, 0.0, 0.0), false, -1.0);
        assert!(!strict.is_hit());
        assert!(matches!(
            strict,
            RayCast::Miss {
                reason: RayCastMiss::UnknownVoxel,
                ..
            }
        ));
    }

    #[test]
    fn cast_ray_from_inside_an_obstacle_reports_that_voxel() {
        let mut t = tree();
        let p = Point3::new(1.05, 0.05, 0.05);
        t.update_node_at(p, true);

        let result = t.cast_ray(p, Point3::new(1.0, 0.0, 0.0), true, -1.0);
        assert!(result.is_hit());
        // Reports the voxel center, not the query point.
        let center = t.geometry().key_to_coord(t.geometry().coord_to_key(p));
        assert_eq!(result.hit_point(), Some(center));
    }

    #[test]
    fn cast_ray_rejects_a_zero_direction() {
        let mut t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        t.update_node_at(origin, false);

        let result = t.cast_ray(origin, Point3::new(0.0, 0.0, 0.0), true, -1.0);
        assert!(matches!(
            result,
            RayCast::Miss {
                reason: RayCastMiss::ZeroDirection,
                ..
            }
        ));
    }

    #[test]
    fn cast_ray_respects_max_range() {
        let mut t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        t.insert_ray(origin, Point3::new(5.05, 0.05, 0.05), -1.0, false);

        // The obstacle sits at 5.05 m, well past a 1 m limit.
        let result = t.cast_ray(origin, Point3::new(1.0, 0.0, 0.0), true, 1.0);
        assert!(matches!(
            result,
            RayCast::Miss {
                reason: RayCastMiss::MaxRange,
                ..
            }
        ));
    }

    #[test]
    fn cast_ray_from_outside_the_volume_is_rejected() {
        let t = tree();
        let result = t.cast_ray(
            Point3::new(1.0e9, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            true,
            -1.0,
        );
        assert!(matches!(
            result,
            RayCast::Miss {
                last: None,
                reason: RayCastMiss::OriginOutOfBounds,
            }
        ));
    }

    #[test]
    fn compute_update_keeps_the_two_sets_disjoint() {
        let t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        let mut scan = PointCloud::new();
        scan.push(Point3::new(1.05, 0.05, 0.05));
        scan.push(Point3::new(2.05, 0.05, 0.05));

        let (free, occupied) = t.compute_update(&scan, origin, -1.0);
        assert!(!free.is_empty() && !occupied.is_empty());

        // The first endpoint also lies on the second ray's path; occupied wins.
        for key in &occupied {
            assert!(!free.contains(key), "{key:?} is in both sets");
        }
        assert_eq!(occupied.len(), 2);
    }

    #[test]
    fn point_cloud_insertion_frees_the_path_and_occupies_endpoints() {
        let mut t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        let mut scan = PointCloud::new();
        scan.push(Point3::new(1.05, 0.05, 0.05));
        scan.push(Point3::new(0.05, 1.05, 0.05));
        t.insert_point_cloud(&scan, origin, -1.0, false, false);

        for p in scan.iter() {
            assert_eq!(t.is_occupied(t.geometry().coord_to_key(*p)), Some(true));
        }
        let midway = t.geometry().coord_to_key(Point3::new(0.55, 0.05, 0.05));
        assert_eq!(t.is_occupied(midway), Some(false));
    }

    #[test]
    fn discretizing_collapses_duplicate_endpoints() {
        let t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        let mut scan = PointCloud::new();
        // Three points inside one voxel.
        scan.push(Point3::new(1.01, 0.05, 0.05));
        scan.push(Point3::new(1.05, 0.05, 0.05));
        scan.push(Point3::new(1.09, 0.05, 0.05));

        let (_, occupied) = t.compute_discrete_update(&scan, origin, -1.0);
        assert_eq!(occupied.len(), 1, "one voxel, one endpoint");
    }

    #[test]
    fn an_empty_scan_changes_nothing() {
        let mut t = tree();
        t.insert_point_cloud(&PointCloud::new(), Point3::ORIGIN, -1.0, false, false);
        assert!(t.is_empty());
    }

    #[test]
    fn max_range_drops_endpoints_beyond_the_limit() {
        let t = tree();
        let origin = Point3::new(0.05, 0.05, 0.05);
        let mut scan = PointCloud::new();
        scan.push(Point3::new(0.55, 0.05, 0.05)); // within 1 m
        scan.push(Point3::new(5.05, 0.05, 0.05)); // beyond

        let (free, occupied) = t.compute_update(&scan, origin, 1.0);
        assert_eq!(occupied.len(), 1, "only the in-range endpoint counts");
        assert!(!free.is_empty(), "the far ray still frees its first metre");
    }

    #[test]
    fn a_diagonal_ray_is_connected() {
        let t = tree();
        let mut ray = KeyRay::new();
        t.compute_ray_keys(
            Point3::new(0.05, 0.05, 0.05),
            Point3::new(1.05, 1.05, 1.05),
            &mut ray,
        );

        // Consecutive keys must be face neighbours: exactly one axis moves, by
        // exactly one step.
        for w in ray.as_slice().windows(2) {
            let d = [
                i32::from(w[1].x()) - i32::from(w[0].x()),
                i32::from(w[1].y()) - i32::from(w[0].y()),
                i32::from(w[1].z()) - i32::from(w[0].z()),
            ];
            let moved: Vec<_> = d.iter().filter(|v| **v != 0).collect();
            assert_eq!(moved.len(), 1, "step {d:?} is not a face move");
            assert_eq!(moved[0].abs(), 1, "step {d:?} skips a voxel");
        }
    }
}
