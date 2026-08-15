//! A rigid transform in the shape ROS hands one over.
//!
//! [`octomap_core::Pose6`] already models a rigid transform, but it stores its
//! components as `f32` because the reference C++ library does. Everything
//! arriving from ROS — `geometry_msgs/Transform`, a TF lookup, a pose topic —
//! is `f64`, and a transform chain is composed before it is ever applied. So
//! this type keeps `f64` from the message all the way through composition and
//! inversion, and narrows only at the moment a point becomes a
//! [`octomap_core::Point3`].
//!
//! The difference shows up in the third or fourth link of a
//! `map → odom → base_link → sensor` chain, where rounding each intermediate
//! product to `f32` accumulates into a visible offset between scans.

use octomap_core::{Point3, Pose6, Quaternion};

/// A translation and a rotation, both in `f64`.
///
/// The quaternion is stored `(x, y, z, w)`, the order `geometry_msgs/Quaternion`
/// uses — not the `(u, x, y, z)` of [`octomap_core::Quaternion`]. Getting these
/// two conventions confused silently produces a rotated map, so the field names
/// are explicit and the conversion is a method rather than a `From` impl that
/// could fire by accident.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform3 {
    /// Translation `(x, y, z)`, in meters.
    pub translation: [f64; 3],
    /// Rotation as `(x, y, z, w)`.
    pub rotation: [f64; 4],
}

impl Default for Transform3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform3 {
    /// No translation, no rotation.
    pub const IDENTITY: Self = Self {
        translation: [0.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    /// Builds a transform from a translation and a `(x, y, z, w)` quaternion.
    pub const fn new(translation: [f64; 3], rotation: [f64; 4]) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// Builds a pure translation.
    pub const fn from_translation(x: f64, y: f64, z: f64) -> Self {
        Self {
            translation: [x, y, z],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    /// Builds a rotation from roll, pitch and yaw, in radians.
    ///
    /// The intrinsic Z-Y-X convention every ROS tool means by "rpy".
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = (roll * 0.5).sin_cos();
        let (sp, cp) = (pitch * 0.5).sin_cos();
        let (sy, cy) = (yaw * 0.5).sin_cos();

        Self {
            translation: [0.0, 0.0, 0.0],
            rotation: [
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            ],
        }
    }

    /// The transform's translation, as a point.
    ///
    /// For a `map → sensor` transform this is the sensor origin in the map
    /// frame, which is exactly what ray insertion needs.
    pub fn origin(&self) -> Point3 {
        Point3::new(
            self.translation[0] as f32,
            self.translation[1] as f32,
            self.translation[2] as f32,
        )
    }

    /// Rotates a vector by this transform's rotation, ignoring its translation.
    fn rotate(&self, v: [f64; 3]) -> [f64; 3] {
        let [qx, qy, qz, qw] = self.rotation;

        // v + 2w(q × v) + 2(q × (q × v)). Equivalent to building the rotation
        // matrix, with fewer multiplies and no intermediate storage.
        let t = [
            2.0 * (qy * v[2] - qz * v[1]),
            2.0 * (qz * v[0] - qx * v[2]),
            2.0 * (qx * v[1] - qy * v[0]),
        ];

        [
            v[0] + qw * t[0] + qy * t[2] - qz * t[1],
            v[1] + qw * t[1] + qz * t[0] - qx * t[2],
            v[2] + qw * t[2] + qx * t[1] - qy * t[0],
        ]
    }

    /// Applies the transform to a point: rotate, then translate.
    ///
    /// The point widens to `f64` on the way in and narrows on the way out, so
    /// the rotation is not evaluated in `f32`.
    pub fn apply(&self, p: Point3) -> Point3 {
        let r = self.rotate([p.x as f64, p.y as f64, p.z as f64]);
        Point3::new(
            (r[0] + self.translation[0]) as f32,
            (r[1] + self.translation[1]) as f32,
            (r[2] + self.translation[2]) as f32,
        )
    }

    /// Composes two transforms: `self * other`.
    ///
    /// With `self` as `a_T_b` and `other` as `b_T_c`, the result is `a_T_c` —
    /// the order a TF chain is walked in, from the target frame down to the
    /// source.
    pub fn compose(&self, other: &Self) -> Self {
        let t = self.rotate(other.translation);
        let [ax, ay, az, aw] = self.rotation;
        let [bx, by, bz, bw] = other.rotation;

        Self {
            translation: [
                t[0] + self.translation[0],
                t[1] + self.translation[1],
                t[2] + self.translation[2],
            ],
            rotation: [
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ],
        }
    }

    /// The inverse transform.
    ///
    /// Assumes a unit quaternion, as every transform on `/tf` is meant to be;
    /// the conjugate is the inverse rotation only under that assumption.
    pub fn inverse(&self) -> Self {
        let inv = Self {
            translation: [0.0, 0.0, 0.0],
            rotation: [
                -self.rotation[0],
                -self.rotation[1],
                -self.rotation[2],
                self.rotation[3],
            ],
        };
        let t = inv.rotate(self.translation);

        Self {
            translation: [-t[0], -t[1], -t[2]],
            rotation: inv.rotation,
        }
    }

    /// Whether every component is finite.
    ///
    /// A transform that is not is a producer bug, and applying it turns a whole
    /// scan into `NaN` points that then vanish in the filter — a silent
    /// failure worth catching at the source.
    pub fn is_finite(&self) -> bool {
        self.translation.iter().all(|v| v.is_finite())
            && self.rotation.iter().all(|v| v.is_finite())
    }

    /// Converts to the core library's pose type, narrowing to `f32`.
    ///
    /// Note the component reordering: [`octomap_core::Quaternion`] is
    /// `(u, x, y, z)` with the real part first.
    pub fn to_pose6(&self) -> Pose6 {
        Pose6::new(
            self.origin(),
            Quaternion::new(
                self.rotation[3] as f32,
                self.rotation[0] as f32,
                self.rotation[1] as f32,
                self.rotation[2] as f32,
            ),
        )
    }

    /// Builds one from the core library's pose type, widening to `f64`.
    pub fn from_pose6(pose: &Pose6) -> Self {
        Self {
            translation: [
                pose.translation.x as f64,
                pose.translation.y as f64,
                pose.translation.z as f64,
            ],
            rotation: [
                pose.rotation.x as f64,
                pose.rotation.y as f64,
                pose.rotation.z as f64,
                pose.rotation.u as f64,
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Point3, b: [f32; 3]) {
        for (got, want) in [a.x, a.y, a.z].iter().zip(b.iter()) {
            assert!(
                (got - want).abs() < 1e-5,
                "got {:?}, want {b:?}",
                [a.x, a.y, a.z]
            );
        }
    }

    #[test]
    fn the_identity_leaves_a_point_alone() {
        let p = Point3::new(1.0, -2.0, 3.5);
        assert_eq!(Transform3::IDENTITY.apply(p), p);
    }

    #[test]
    fn a_quarter_turn_about_z_maps_x_onto_y() {
        let t = Transform3::from_rpy(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        close(t.apply(Point3::new(1.0, 0.0, 0.0)), [0.0, 1.0, 0.0]);
        close(t.apply(Point3::new(0.0, 1.0, 0.0)), [-1.0, 0.0, 0.0]);
        close(t.apply(Point3::new(0.0, 0.0, 1.0)), [0.0, 0.0, 1.0]);
    }

    #[test]
    fn rotation_is_applied_before_translation() {
        let mut t = Transform3::from_rpy(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        t.translation = [10.0, 0.0, 0.0];
        close(t.apply(Point3::new(1.0, 0.0, 0.0)), [10.0, 1.0, 0.0]);
    }

    #[test]
    fn composing_matches_applying_in_sequence() {
        let mut a = Transform3::from_rpy(0.3, -0.2, 1.1);
        a.translation = [1.0, 2.0, -0.5];
        let mut b = Transform3::from_rpy(-0.7, 0.4, 0.2);
        b.translation = [-3.0, 0.25, 4.0];

        let p = Point3::new(0.5, -1.25, 2.0);
        let composed = a.compose(&b).apply(p);
        let stepwise = a.apply(b.apply(p));
        close(composed, [stepwise.x, stepwise.y, stepwise.z]);
    }

    #[test]
    fn a_transform_composed_with_its_inverse_is_the_identity() {
        let mut t = Transform3::from_rpy(0.9, 0.1, -2.2);
        t.translation = [4.0, -1.5, 0.75];

        let round_trip = t.compose(&t.inverse());
        for v in round_trip.translation {
            assert!(v.abs() < 1e-12, "translation left over: {v}");
        }
        assert!((round_trip.rotation[3].abs() - 1.0).abs() < 1e-12);

        let p = Point3::new(1.0, 2.0, 3.0);
        close(t.inverse().apply(t.apply(p)), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn the_origin_is_the_translation() {
        let t = Transform3::from_translation(1.5, -2.5, 0.25);
        assert_eq!(t.origin(), Point3::new(1.5, -2.5, 0.25));
    }

    #[test]
    fn pose6_conversion_keeps_the_real_part_in_the_right_slot() {
        let t = Transform3::from_rpy(0.4, -0.3, 0.8);
        let pose = t.to_pose6();
        // Narrowing to f32 is lossy; the point of the test is which component
        // lands where, not the last few bits.
        assert!((pose.rotation.u as f64 - t.rotation[3]).abs() < 1e-6);
        assert!((pose.rotation.x as f64 - t.rotation[0]).abs() < 1e-6);
        assert!((pose.rotation.y as f64 - t.rotation[1]).abs() < 1e-6);
        assert!((pose.rotation.z as f64 - t.rotation[2]).abs() < 1e-6);

        let back = Transform3::from_pose6(&pose);
        for (a, b) in back.rotation.iter().zip(t.rotation.iter()) {
            assert!((a - b).abs() < 1e-6);
        }
    }

    #[test]
    fn rpy_matches_a_hand_built_rotation() {
        // Roll 90 degrees: y goes to z.
        let t = Transform3::from_rpy(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
        close(t.apply(Point3::new(0.0, 1.0, 0.0)), [0.0, 0.0, 1.0]);

        // Pitch 90 degrees: z goes to x.
        let t = Transform3::from_rpy(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        close(t.apply(Point3::new(0.0, 0.0, 1.0)), [1.0, 0.0, 0.0]);
    }

    #[test]
    fn a_non_finite_transform_is_flagged() {
        assert!(Transform3::IDENTITY.is_finite());
        let bad = Transform3::new([f64::NAN, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        assert!(!bad.is_finite());
    }
}
