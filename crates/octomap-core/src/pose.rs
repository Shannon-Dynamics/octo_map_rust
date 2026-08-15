//! Rigid-body orientation and pose.
//!
//! Ported from `octomath::Quaternion` and `octomath::Pose6D`.
//!
//! # Not the Euler triple the learning guide describes
//!
//! `Pose6D` stores a **quaternion**, not roll/pitch/yaw. The Euler accessors
//! are derived on each call by converting the quaternion to a rotation matrix
//! and back out with `atan2`. Storing Euler angles instead would change what
//! `pose * pose` and `pose.inv()` produce, and would introduce gimbal lock the
//! reference does not have — so this port keeps the quaternion.
//!
//! # Precision
//!
//! Components are `f32`, matching the reference. Intermediate trigonometry runs
//! in `f64` and narrows on assignment, again matching. Quaternion products are
//! computed in `f32` because that is what the reference does; widening them
//! would drift away from C++ results.

use crate::point::Point3;

/// A rotation, stored as a unit-ish quaternion `(u, x, y, z)`.
///
/// Nothing forces the quaternion to stay normalized — the reference does not
/// either. [`Quaternion::normalized`] is applied explicitly where composition
/// would otherwise let the norm drift.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// Real part.
    pub u: f32,
    /// First imaginary part.
    pub x: f32,
    /// Second imaginary part.
    pub y: f32,
    /// Third imaginary part.
    pub z: f32,
}

impl Default for Quaternion {
    /// The identity rotation.
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quaternion {
    /// No rotation.
    pub const IDENTITY: Self = Self {
        u: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Builds a quaternion from its four components.
    #[inline]
    pub const fn new(u: f32, x: f32, y: f32, z: f32) -> Self {
        Self { u, x, y, z }
    }

    /// Builds a rotation from Tait–Bryan angles, in radians.
    ///
    /// The reference goes via a rotation matrix and takes square roots of the
    /// diagonal, recovering each component's sign from the off-diagonal terms.
    /// A direct half-angle formula would be shorter but would not round
    /// identically.
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = (roll.sin(), roll.cos());
        let (sp, cp) = (pitch.sin(), pitch.cos());
        let (sy, cy) = (yaw.sin(), yaw.cos());

        let m = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ];

        let half_sqrt = |v: f64| (v.max(0.0).sqrt() / 2.0) as f32;
        let u = half_sqrt(1.0 + m[0][0] + m[1][1] + m[2][2]);
        let x = half_sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
        let y = half_sqrt(1.0 - m[0][0] + m[1][1] - m[2][2]);
        let z = half_sqrt(1.0 - m[0][0] - m[1][1] + m[2][2]);

        let signed = |magnitude: f32, indicator: f64| {
            if indicator >= 0.0 {
                magnitude.abs()
            } else {
                -magnitude.abs()
            }
        };

        Self {
            u,
            x: signed(x, m[2][1] - m[1][2]),
            y: signed(y, m[0][2] - m[2][0]),
            z: signed(z, m[1][0] - m[0][1]),
        }
    }

    /// Builds a rotation of `angle` radians about `axis`.
    ///
    /// `axis` is used as given; the reference does not normalize it either.
    pub fn from_axis_angle(axis: Point3, angle: f64) -> Self {
        let sa = (angle / 2.0).sin();
        let ca = (angle / 2.0).cos();
        Self {
            u: ca as f32,
            x: (f64::from(axis.x) * sa) as f32,
            y: (f64::from(axis.y) * sa) as f32,
            z: (f64::from(axis.z) * sa) as f32,
        }
    }

    /// Length of the quaternion.
    ///
    /// Each **square is computed in `f32`** and only then widened for the
    /// accumulation — that is what the reference does, and squaring in `f64`
    /// instead shifts the result by one ULP, which propagates through
    /// [`Quaternion::normalized`] into every composed pose.
    pub fn norm(&self) -> f32 {
        let n = f64::from(self.u * self.u)
            + f64::from(self.x * self.x)
            + f64::from(self.y * self.y)
            + f64::from(self.z * self.z);
        n.sqrt() as f32
    }

    /// A unit-length copy. A zero quaternion is returned unchanged.
    pub fn normalized(&self) -> Self {
        let len = f64::from(self.norm());
        if len > 0.0 {
            let len = len as f32;
            Self::new(self.u / len, self.x / len, self.y / len, self.z / len)
        } else {
            *self
        }
    }

    /// The conjugate, which inverts the rotation for a unit quaternion.
    #[inline]
    pub fn inv(&self) -> Self {
        Self::new(self.u, -self.x, -self.y, -self.z)
    }

    /// Tait–Bryan angles `(roll, pitch, yaw)`, in radians.
    ///
    /// Derived through the rotation matrix, as the reference does.
    pub fn to_euler(&self) -> (f32, f32, f32) {
        let m = self.to_rotation_matrix();
        let roll = m[2][1].atan2(m[2][2]) as f32;
        let pitch = (-m[2][0]).atan2((m[2][1] * m[2][1] + m[2][2] * m[2][2]).sqrt()) as f32;
        let yaw = m[1][0].atan2(m[0][0]) as f32;
        (roll, pitch, yaw)
    }

    /// The equivalent 3×3 rotation matrix, row-major.
    ///
    /// Scales by `2 / norm²`, so it stays correct for a quaternion that has
    /// drifted off unit length.
    pub fn to_rotation_matrix(&self) -> [[f64; 3]; 3] {
        let n = f64::from(self.norm());
        let s = if n > 0.0 { 2.0 / (n * n) } else { 0.0 };

        let (qu, qx, qy, qz) = (
            f64::from(self.u),
            f64::from(self.x),
            f64::from(self.y),
            f64::from(self.z),
        );
        let (xs, ys, zs) = (qx * s, qy * s, qz * s);
        let (ux, uy, uz) = (qu * xs, qu * ys, qu * zs);
        let (xx, xy, xz) = (qx * xs, qx * ys, qx * zs);
        let (yy, yz) = (qy * ys, qy * zs);
        let zz = qz * zs;

        [
            [1.0 - (yy + zz), xy - uz, xz + uy],
            [xy + uz, 1.0 - (xx + zz), yz - ux],
            [xz - uy, yz + ux, 1.0 - (xx + yy)],
        ]
    }

    /// Rotates `v` by this quaternion.
    ///
    /// Evaluated as `q * v * q⁻¹` in `f32`, like the reference — not via the
    /// rotation matrix, which would round differently.
    pub fn rotate(&self, v: Point3) -> Point3 {
        let q = *self * Self::new(0.0, v.x, v.y, v.z) * self.inv();
        Point3::new(q.x, q.y, q.z)
    }
}

impl core::ops::Mul for Quaternion {
    type Output = Self;

    /// Hamilton product, in the reference's term order.
    fn mul(self, o: Self) -> Self {
        Self::new(
            self.u * o.u - self.x * o.x - self.y * o.y - self.z * o.z,
            self.y * o.z - o.y * self.z + self.u * o.x + o.u * self.x,
            self.z * o.x - o.z * self.x + self.u * o.y + o.u * self.y,
            self.x * o.y - o.x * self.y + self.u * o.z + o.u * self.z,
        )
    }
}

/// A rigid-body transform: a translation and a rotation.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Pose6 {
    /// Position, in meters.
    pub translation: Point3,
    /// Orientation.
    pub rotation: Quaternion,
}

impl Pose6 {
    /// Builds a pose from a translation and a rotation.
    #[inline]
    pub const fn new(translation: Point3, rotation: Quaternion) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// Builds a pose from a position and Tait–Bryan angles, in radians.
    pub fn from_euler(x: f32, y: f32, z: f32, roll: f64, pitch: f64, yaw: f64) -> Self {
        Self {
            translation: Point3::new(x, y, z),
            rotation: Quaternion::from_euler(roll, pitch, yaw),
        }
    }

    /// Roll, in radians.
    #[inline]
    pub fn roll(&self) -> f32 {
        self.rotation.to_euler().0
    }

    /// Pitch, in radians.
    #[inline]
    pub fn pitch(&self) -> f32 {
        self.rotation.to_euler().1
    }

    /// Yaw, in radians.
    #[inline]
    pub fn yaw(&self) -> f32 {
        self.rotation.to_euler().2
    }

    /// Applies this transform to `v`: rotate, then translate.
    #[inline]
    pub fn transform(&self, v: Point3) -> Point3 {
        self.rotation.rotate(v) + self.translation
    }

    /// The inverse transform.
    pub fn inv(&self) -> Self {
        let rotation = self.rotation.inv().normalized();
        Self {
            translation: rotation.rotate(-self.translation),
            rotation,
        }
    }

    /// Distance between the two poses' positions, ignoring orientation.
    pub fn distance(&self, other: &Self) -> f64 {
        let d = self.translation - other.translation;
        (f64::from(d.x) * f64::from(d.x)
            + f64::from(d.y) * f64::from(d.y)
            + f64::from(d.z) * f64::from(d.z))
        .sqrt()
    }

    /// Length of the translation.
    ///
    /// Computed entirely in `f32` and widened at the end. `distance` below
    /// widens *first*, because the reference declares its intermediates
    /// `double` there and does not here.
    pub fn translation_length(&self) -> f64 {
        let t = self.translation;
        f64::from((t.x * t.x + t.y * t.y + t.z * t.z).sqrt())
    }
}

impl core::ops::Mul for Pose6 {
    type Output = Self;

    /// Composition: `self * other` applies `other` first.
    ///
    /// The reference normalizes the resulting rotation, which keeps repeated
    /// composition from drifting off unit length.
    fn mul(self, other: Self) -> Self {
        Self {
            translation: self.rotation.rotate(other.translation) + self.translation,
            rotation: (self.rotation * other.rotation).normalized(),
        }
    }
}

impl crate::ray::PointCloud {
    /// Applies `pose` to every point.
    pub fn transform(&mut self, pose: &Pose6) {
        for p in self.iter_mut() {
            *p = pose.transform(*p);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, PI};

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    fn points_close(a: Point3, b: Point3, tol: f32) -> bool {
        close(a.x, b.x, tol) && close(a.y, b.y, tol) && close(a.z, b.z, tol)
    }

    #[test]
    fn identity_rotates_nothing() {
        let q = Quaternion::IDENTITY;
        let v = Point3::new(1.0, 2.0, 3.0);
        assert!(points_close(q.rotate(v), v, 1e-6));
    }

    #[test]
    fn zero_euler_angles_give_the_identity() {
        let q = Quaternion::from_euler(0.0, 0.0, 0.0);
        assert!(close(q.u, 1.0, 1e-6));
        assert!(close(q.x, 0.0, 1e-6));
        assert!(close(q.y, 0.0, 1e-6));
        assert!(close(q.z, 0.0, 1e-6));
    }

    #[test]
    fn a_quarter_turn_about_z_maps_x_onto_y() {
        let q = Quaternion::from_euler(0.0, 0.0, FRAC_PI_2);
        let rotated = q.rotate(Point3::new(1.0, 0.0, 0.0));
        assert!(
            points_close(rotated, Point3::new(0.0, 1.0, 0.0), 1e-6),
            "got {rotated:?}"
        );
    }

    #[test]
    fn a_quarter_turn_about_y_maps_z_onto_x() {
        let q = Quaternion::from_euler(0.0, FRAC_PI_2, 0.0);
        let rotated = q.rotate(Point3::new(0.0, 0.0, 1.0));
        assert!(
            points_close(rotated, Point3::new(1.0, 0.0, 0.0), 1e-6),
            "got {rotated:?}"
        );
    }

    #[test]
    fn a_quarter_turn_about_x_maps_y_onto_z() {
        let q = Quaternion::from_euler(FRAC_PI_2, 0.0, 0.0);
        let rotated = q.rotate(Point3::new(0.0, 1.0, 0.0));
        assert!(
            points_close(rotated, Point3::new(0.0, 0.0, 1.0), 1e-6),
            "got {rotated:?}"
        );
    }

    #[test]
    fn euler_angles_survive_a_round_trip() {
        for &(r, p, y) in &[
            (0.0, 0.0, 0.0),
            (0.3, 0.0, 0.0),
            (0.0, 0.4, 0.0),
            (0.0, 0.0, 0.5),
            (0.3, -0.4, 0.5),
            (-1.0, 0.2, 2.0),
        ] {
            let q = Quaternion::from_euler(r, p, y);
            let (rr, pp, yy) = q.to_euler();
            assert!(close(rr, r as f32, 1e-5), "roll {r} -> {rr}");
            assert!(close(pp, p as f32, 1e-5), "pitch {p} -> {pp}");
            assert!(close(yy, y as f32, 1e-5), "yaw {y} -> {yy}");
        }
    }

    #[test]
    fn from_euler_produces_a_unit_quaternion() {
        for &(r, p, y) in &[(0.3, -0.4, 0.5), (1.0, 1.0, 1.0), (-2.0, 0.5, 3.0)] {
            let q = Quaternion::from_euler(r, p, y);
            assert!(
                close(q.norm(), 1.0, 1e-5),
                "norm {} for {r},{p},{y}",
                q.norm()
            );
        }
    }

    #[test]
    fn axis_angle_matches_the_equivalent_euler_rotation() {
        let axis = Quaternion::from_axis_angle(Point3::new(0.0, 0.0, 1.0), FRAC_PI_2);
        let euler = Quaternion::from_euler(0.0, 0.0, FRAC_PI_2);
        let v = Point3::new(1.0, 2.0, 3.0);
        assert!(points_close(axis.rotate(v), euler.rotate(v), 1e-5));
    }

    #[test]
    fn rotating_preserves_length() {
        let q = Quaternion::from_euler(0.3, -0.4, 0.5);
        let v = Point3::new(1.0, 2.0, 3.0);
        assert!(close(q.rotate(v).norm(), v.norm(), 1e-5));
    }

    #[test]
    fn the_inverse_undoes_the_rotation() {
        let q = Quaternion::from_euler(0.3, -0.4, 0.5);
        let v = Point3::new(1.0, 2.0, 3.0);
        assert!(points_close(q.inv().rotate(q.rotate(v)), v, 1e-5));
    }

    #[test]
    fn normalizing_a_zero_quaternion_leaves_it_alone() {
        let zero = Quaternion::new(0.0, 0.0, 0.0, 0.0);
        assert_eq!(zero.normalized(), zero);
    }

    #[test]
    fn a_pose_rotates_then_translates() {
        let pose = Pose6::from_euler(10.0, 0.0, 0.0, 0.0, 0.0, FRAC_PI_2);
        let moved = pose.transform(Point3::new(1.0, 0.0, 0.0));
        // Rotated onto +y first, then shifted along +x.
        assert!(
            points_close(moved, Point3::new(10.0, 1.0, 0.0), 1e-5),
            "got {moved:?}"
        );
    }

    #[test]
    fn a_pose_and_its_inverse_cancel() {
        let pose = Pose6::from_euler(1.0, -2.0, 0.5, 0.3, -0.4, 0.5);
        let v = Point3::new(3.0, 1.0, -2.0);
        let round_trip = pose.inv().transform(pose.transform(v));
        assert!(points_close(round_trip, v, 1e-4), "got {round_trip:?}");
    }

    #[test]
    fn composition_applies_the_right_hand_pose_first() {
        let a = Pose6::from_euler(0.0, 0.0, 0.0, 0.0, 0.0, FRAC_PI_2);
        let b = Pose6::from_euler(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let v = Point3::new(0.0, 0.0, 0.0);

        // b shifts to (1,0,0), then a rotates that onto (0,1,0).
        let composed = (a * b).transform(v);
        let stepwise = a.transform(b.transform(v));
        assert!(points_close(composed, stepwise, 1e-5));
        assert!(points_close(composed, Point3::new(0.0, 1.0, 0.0), 1e-5));
    }

    #[test]
    fn composition_keeps_the_rotation_normalized() {
        let mut pose = Pose6::from_euler(0.1, 0.2, 0.3, 0.2, 0.3, 0.4);
        for _ in 0..200 {
            pose = pose * Pose6::from_euler(0.0, 0.0, 0.0, 0.01, 0.02, 0.03);
        }
        assert!(
            close(pose.rotation.norm(), 1.0, 1e-4),
            "norm drifted to {}",
            pose.rotation.norm()
        );
    }

    #[test]
    fn distance_and_translation_length_ignore_orientation() {
        let a = Pose6::from_euler(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);
        let b = Pose6::from_euler(3.0, 4.0, 0.0, 0.0, 0.0, 0.0);
        assert!((a.distance(&b) - 5.0).abs() < 1e-6);
        assert!((b.translation_length() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn transforming_a_point_cloud_moves_every_point() {
        let mut cloud: crate::ray::PointCloud = [
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ]
        .into_iter()
        .collect();

        let pose = Pose6::from_euler(5.0, 0.0, 0.0, 0.0, 0.0, FRAC_PI_2);
        cloud.transform(&pose);

        assert!(points_close(cloud[0], Point3::new(5.0, 1.0, 0.0), 1e-5));
        assert!(points_close(cloud[1], Point3::new(4.0, 0.0, 0.0), 1e-5));
        assert!(points_close(cloud[2], Point3::new(5.0, 0.0, 1.0), 1e-5));
    }

    #[test]
    fn a_half_turn_is_its_own_inverse() {
        let q = Quaternion::from_euler(0.0, 0.0, PI);
        let v = Point3::new(1.0, 2.0, 0.0);
        assert!(points_close(q.rotate(q.rotate(v)), v, 1e-5));
    }
}
