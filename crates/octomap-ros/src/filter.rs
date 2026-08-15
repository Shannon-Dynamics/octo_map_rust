//! Turning a decoded cloud into a scan the map can absorb.
//!
//! Between "the message decoded" and "the octree updated" sit a handful of
//! decisions that every mapping node makes and that the octree itself has no
//! opinion about: which points are real, which frame they belong in, and how
//! many of them are worth the insertion cost. [`ScanFilter`] is those decisions
//! in one place.
//!
//! # What is not here: `max_range`
//!
//! There is deliberately no maximum-range filter. Dropping a far point and
//! capping a far ray are different operations: [`OcTree::insert_point_cloud`]
//! given a `max_range` still walks the ray out to that distance and marks it
//! free, it just does not mark an endpoint occupied. Filtering the point away
//! first loses that free space, which is how a map ends up with unknown voids
//! where the sensor plainly saw through.
//!
//! So range capping belongs on the insert call, and this filter only removes
//! points that are not real measurements at all.
//!
//! [`OcTree::insert_point_cloud`]: octomap_core::OcTree::insert_point_cloud

use octomap_core::PointCloud;

use crate::pointcloud2::Cloud;
use crate::transform::Transform3;

/// The limits applied to a scan on its way into the map.
///
/// [`ScanFilter::default`] keeps everything: no minimum range, unbounded
/// height, every point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanFilter {
    /// Points closer than this to the sensor are dropped, in meters.
    ///
    /// Measured in the cloud's own frame, where the sensor sits at the origin,
    /// and applied before the transform. This is the knob for the robot's own
    /// chassis showing up in a 360-degree LiDAR sweep: those returns are real
    /// but they are not the world, and integrating them walls the robot in.
    pub min_range: f64,

    /// Lowest height kept, in meters, in the **output** frame.
    ///
    /// Applied after the transform, so for a `map`-frame output this is a
    /// height above the map origin, not above the sensor. The usual use is
    /// cutting the ground plane out of a map a planner will read.
    pub min_z: f64,

    /// Highest height kept, in meters, in the output frame.
    ///
    /// The counterpart to [`min_z`](Self::min_z), for ceilings and overhead
    /// structure a ground robot cannot collide with.
    pub max_z: f64,

    /// Keep one point in every `stride`.
    ///
    /// Subsampling by index, which is uniform over a LiDAR sweep and close
    /// enough to uniform over a depth image. A cheaper knob than lowering the
    /// resolution when insertion cannot keep up with the frame rate, and unlike
    /// a coarser map it costs nothing in accuracy where the map is sparse. Zero
    /// is treated as one.
    pub stride: usize,
}

impl Default for ScanFilter {
    fn default() -> Self {
        Self {
            min_range: 0.0,
            min_z: f64::NEG_INFINITY,
            max_z: f64::INFINITY,
            stride: 1,
        }
    }
}

/// What a [`ScanFilter`] did to one scan.
///
/// Worth logging when a map comes out empty: the counts say whether the cloud
/// was empty, the sensor returned nothing, the transform was broken, or the
/// height limits are wrong — four failures that look identical from the outside.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ScanStats {
    /// Points the message declared.
    pub total: usize,
    /// Points that reached the map.
    pub kept: usize,
    /// Points skipped by [`ScanFilter::stride`].
    pub skipped_stride: usize,
    /// Points holding `NaN` or an infinity, before or after the transform.
    pub non_finite: usize,
    /// Points inside [`ScanFilter::min_range`].
    pub below_min_range: usize,
    /// Points outside the height limits.
    pub outside_height: usize,
}

impl ScanFilter {
    /// Filters and transforms a cloud into a scan in the output frame.
    ///
    /// `transform` maps the cloud's frame onto the map's; pass
    /// [`Transform3::IDENTITY`] when the cloud already arrives in the map frame.
    pub fn apply(&self, cloud: &Cloud<'_>, transform: &Transform3) -> PointCloud {
        self.apply_with_stats(cloud, transform).0
    }

    /// [`apply`](Self::apply), also reporting what was dropped and why.
    pub fn apply_with_stats(
        &self,
        cloud: &Cloud<'_>,
        transform: &Transform3,
    ) -> (PointCloud, ScanStats) {
        let stride = self.stride.max(1);
        let min_range_sq = if self.min_range > 0.0 {
            self.min_range * self.min_range
        } else {
            0.0
        };

        let mut stats = ScanStats {
            total: cloud.len(),
            ..ScanStats::default()
        };
        let mut scan = PointCloud::with_capacity(stats.total.div_ceil(stride));

        for (index, point) in cloud.iter().enumerate() {
            if index % stride != 0 {
                stats.skipped_stride += 1;
                continue;
            }

            // A depth camera reports "no return" as NaN on every pixel that
            // failed, so on an organized cloud this is the common case rather
            // than the error case.
            if !point.is_finite() {
                stats.non_finite += 1;
                continue;
            }

            if min_range_sq > 0.0 {
                let d = point.x as f64 * point.x as f64
                    + point.y as f64 * point.y as f64
                    + point.z as f64 * point.z as f64;
                if d < min_range_sq {
                    stats.below_min_range += 1;
                    continue;
                }
            }

            let mapped = transform.apply(point);

            // A broken transform turns finite points into NaN ones. Counting
            // them here rather than at the top is what makes "every point went
            // non-finite" distinguishable from "the sensor saw nothing".
            if !mapped.is_finite() {
                stats.non_finite += 1;
                continue;
            }

            let z = mapped.z as f64;
            if z < self.min_z || z > self.max_z {
                stats.outside_height += 1;
                continue;
            }

            scan.push(mapped);
            stats.kept += 1;
        }

        (scan, stats)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pointcloud2::{datatype, FieldRef};

    fn fields() -> [FieldRef<'static>; 3] {
        [
            FieldRef::new("x", 0, datatype::FLOAT32, 1),
            FieldRef::new("y", 4, datatype::FLOAT32, 1),
            FieldRef::new("z", 8, datatype::FLOAT32, 1),
        ]
    }

    fn blob(points: &[[f32; 3]]) -> Vec<u8> {
        let mut data = Vec::new();
        for p in points {
            for v in p {
                data.extend_from_slice(&v.to_le_bytes());
            }
        }
        data
    }

    fn cloud<'a>(fields: &'a [FieldRef<'a>], data: &'a [u8]) -> Cloud<'a> {
        let n = (data.len() / 12) as u32;
        Cloud::new(fields, data, n, 1, 12, n * 12, false).unwrap()
    }

    #[test]
    fn the_default_filter_keeps_every_finite_point() {
        let f = fields();
        let data = blob(&[[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, -3.0]]);
        let (scan, stats) =
            ScanFilter::default().apply_with_stats(&cloud(&f, &data), &Transform3::IDENTITY);

        assert_eq!(scan.len(), 3);
        assert_eq!(stats.kept, 3);
        assert_eq!(stats.total, 3);
    }

    #[test]
    fn non_finite_points_are_dropped_and_counted() {
        let f = fields();
        let data = blob(&[
            [f32::NAN, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [0.0, f32::INFINITY, 0.0],
        ]);
        let (scan, stats) =
            ScanFilter::default().apply_with_stats(&cloud(&f, &data), &Transform3::IDENTITY);

        assert_eq!(scan.len(), 1);
        assert_eq!(stats.non_finite, 2);
    }

    #[test]
    fn min_range_is_measured_in_the_sensor_frame_not_the_map_frame() {
        let f = fields();
        // Half a meter out, which the filter should drop — even though the
        // transform puts it 100 m from the map origin.
        let data = blob(&[[0.5, 0.0, 0.0], [3.0, 0.0, 0.0]]);
        let filter = ScanFilter {
            min_range: 1.0,
            ..ScanFilter::default()
        };
        let far_away = Transform3::from_translation(100.0, 0.0, 0.0);

        let (scan, stats) = filter.apply_with_stats(&cloud(&f, &data), &far_away);
        assert_eq!(stats.below_min_range, 1);
        assert_eq!(scan.len(), 1);
        assert_eq!(scan.as_slice()[0].x, 103.0);
    }

    #[test]
    fn height_limits_are_measured_after_the_transform() {
        let f = fields();
        let data = blob(&[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 5.0]]);
        let filter = ScanFilter {
            min_z: 1.5,
            max_z: 3.5,
            ..ScanFilter::default()
        };

        // Lift everything by 2 m: the first two points land in range, the
        // third does not. Without the transform the answer would be reversed.
        let lift = Transform3::from_translation(0.0, 0.0, 2.0);
        let (scan, stats) = filter.apply_with_stats(&cloud(&f, &data), &lift);

        assert_eq!(stats.outside_height, 1);
        let heights: Vec<_> = scan.iter().map(|p| p.z).collect();
        assert_eq!(heights, [2.0, 3.0]);
    }

    #[test]
    fn stride_keeps_the_first_of_every_group() {
        let f = fields();
        let points: Vec<[f32; 3]> = (0..10).map(|i| [i as f32, 0.0, 0.0]).collect();
        let data = blob(&points);
        let filter = ScanFilter {
            stride: 3,
            ..ScanFilter::default()
        };

        let (scan, stats) = filter.apply_with_stats(&cloud(&f, &data), &Transform3::IDENTITY);
        let kept: Vec<_> = scan.iter().map(|p| p.x).collect();
        assert_eq!(kept, [0.0, 3.0, 6.0, 9.0]);
        assert_eq!(stats.skipped_stride, 6);
        assert_eq!(stats.kept, 4);
    }

    #[test]
    fn a_zero_stride_does_not_divide_by_zero() {
        let f = fields();
        let data = blob(&[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
        let filter = ScanFilter {
            stride: 0,
            ..ScanFilter::default()
        };
        assert_eq!(
            filter.apply(&cloud(&f, &data), &Transform3::IDENTITY).len(),
            2
        );
    }

    #[test]
    fn a_broken_transform_empties_the_scan_and_says_so() {
        let f = fields();
        let data = blob(&[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
        let broken = Transform3::new([f64::NAN, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

        let (scan, stats) = ScanFilter::default().apply_with_stats(&cloud(&f, &data), &broken);
        assert!(scan.is_empty());
        assert_eq!(stats.non_finite, 2, "the count is what tells them apart");
        assert_eq!(stats.kept, 0);
    }

    #[test]
    fn stride_is_applied_before_the_finite_check_so_the_counts_add_up() {
        let f = fields();
        let data = blob(&[[1.0, 0.0, 0.0], [f32::NAN, 0.0, 0.0], [3.0, 0.0, 0.0]]);
        let filter = ScanFilter {
            stride: 2,
            ..ScanFilter::default()
        };
        let (_, stats) = filter.apply_with_stats(&cloud(&f, &data), &Transform3::IDENTITY);

        assert_eq!(stats.total, 3);
        assert_eq!(
            stats.kept + stats.skipped_stride + stats.non_finite,
            stats.total
        );
        assert_eq!(stats.non_finite, 0, "the NaN point was never examined");
    }
}
