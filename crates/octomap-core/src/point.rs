//! 3D points.
//!
//! Mirrors `octomath::Vector3` from OctoMap C++, which is `float`-backed
//! (`typedef octomath::Vector3 point3d`). The element type is `f32` on purpose:
//! widening to `f64` here would silently diverge from the reference whenever a
//! coordinate is rounded on its way into a key.

use core::ops::{Add, AddAssign, Div, Index, IndexMut, Mul, Neg, Sub, SubAssign};

/// A position in 3D space, in meters.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Point3 {
    /// Position along the x axis.
    pub x: f32,
    /// Position along the y axis.
    pub y: f32,
    /// Position along the z axis.
    pub z: f32,
}

impl Point3 {
    /// The origin, `(0, 0, 0)`.
    pub const ORIGIN: Self = Self::new(0.0, 0.0, 0.0);

    /// Builds a point from its components.
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Euclidean length.
    #[inline]
    pub fn norm(&self) -> f32 {
        self.norm_squared().sqrt()
    }

    /// Squared euclidean length. Avoids the square root when only comparing.
    #[inline]
    pub fn norm_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Distance to `other`.
    #[inline]
    pub fn distance(&self, other: Self) -> f32 {
        (*self - other).norm()
    }

    /// Returns a unit-length copy, or `None` for the zero vector.
    #[inline]
    pub fn normalized(&self) -> Option<Self> {
        let n = self.norm();
        if n > 0.0 {
            Some(Self::new(self.x / n, self.y / n, self.z / n))
        } else {
            None
        }
    }

    /// Dot product.
    #[inline]
    pub fn dot(&self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product.
    #[inline]
    pub fn cross(&self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// True when every component is finite.
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

/// Axis access by index, for code that iterates over the three axes.
///
/// # Panics
///
/// Panics on an index above 2, the way indexing a slice out of range does.
/// A `Point3` has exactly three components and an out-of-range axis is a bug at
/// the call site, not a runtime condition worth a `Result`.
impl Index<usize> for Point3 {
    type Output = f32;

    #[inline]
    fn index(&self, i: usize) -> &f32 {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Point3 index out of range: {i}"),
        }
    }
}

/// Mutable axis access by index.
///
/// # Panics
///
/// Panics on an index above 2, like [`Index`].
impl IndexMut<usize> for Point3 {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut f32 {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Point3 index out of range: {i}"),
        }
    }
}

impl Add for Point3 {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Point3 {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f32> for Point3 {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Div<f32> for Point3 {
    type Output = Self;

    #[inline]
    fn div(self, rhs: f32) -> Self {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl Neg for Point3 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl AddAssign for Point3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl SubAssign for Point3 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl From<[f32; 3]> for Point3 {
    #[inline]
    fn from(v: [f32; 3]) -> Self {
        Self::new(v[0], v[1], v[2])
    }
}

impl From<Point3> for [f32; 3] {
    #[inline]
    fn from(p: Point3) -> Self {
        [p.x, p.y, p.z]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arithmetic_matches_component_wise_expectation() {
        let a = Point3::new(1.0, 2.0, 3.0);
        let b = Point3::new(0.5, -1.0, 2.0);

        assert_eq!(a + b, Point3::new(1.5, 1.0, 5.0));
        assert_eq!(a - b, Point3::new(0.5, 3.0, 1.0));
        assert_eq!(a * 2.0, Point3::new(2.0, 4.0, 6.0));
        assert_eq!(-a, Point3::new(-1.0, -2.0, -3.0));
    }

    #[test]
    fn norm_of_three_four_five_triangle() {
        assert_eq!(Point3::new(3.0, 4.0, 0.0).norm(), 5.0);
        assert_eq!(Point3::new(3.0, 4.0, 0.0).norm_squared(), 25.0);
    }

    #[test]
    fn zero_vector_has_no_normalization() {
        assert_eq!(Point3::ORIGIN.normalized(), None);
        assert_eq!(
            Point3::new(0.0, 5.0, 0.0).normalized(),
            Some(Point3::new(0.0, 1.0, 0.0))
        );
    }

    #[test]
    fn cross_product_follows_right_hand_rule() {
        let x = Point3::new(1.0, 0.0, 0.0);
        let y = Point3::new(0.0, 1.0, 0.0);
        assert_eq!(x.cross(y), Point3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn indexing_reads_and_writes_components() {
        let mut p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(p[0], 1.0);
        assert_eq!(p[2], 3.0);
        p[1] = 9.0;
        assert_eq!(p.y, 9.0);
    }
}
