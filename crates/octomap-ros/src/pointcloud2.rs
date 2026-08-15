//! Decoding the `sensor_msgs/PointCloud2` binary blob.
//!
//! A `PointCloud2` is a flat byte buffer plus a description of how to read it:
//! a list of named fields with offsets and types, a stride between points, a
//! stride between rows, and a byte order. Nothing about the layout is fixed —
//! an organized depth image, an unorganized LiDAR sweep with intensity and
//! ring, and a bare XYZ cloud all arrive in the same message type — so the
//! offsets have to be resolved at runtime, per message.
//!
//! [`Cloud`] does that resolution once and then yields points. It borrows the
//! blob rather than copying it, so decoding a 300k-point frame allocates
//! nothing.
//!
//! # What is rejected
//!
//! A cloud with no `x`, `y` or `z` field cannot be a geometric point cloud, and
//! a `point_step` that does not leave room for the fields it declares means the
//! producer and the message disagree. Both are errors rather than best-effort
//! guesses: silently reading garbage coordinates into a map is worse than a
//! log line saying the cloud is unusable.
//!
//! Individual points that are not finite are skipped, not rejected. Depth
//! cameras signal "no return here" with `NaN` on every pixel that failed, and
//! `is_dense` being false is exactly the producer saying so.

use octomap_core::Point3;

/// The `sensor_msgs/PointField` type constants.
///
/// Named rather than an enum so a field's `datatype` can be compared against
/// them without a fallible conversion at the call site.
pub mod datatype {
    /// Signed 8-bit integer.
    pub const INT8: u8 = 1;
    /// Unsigned 8-bit integer.
    pub const UINT8: u8 = 2;
    /// Signed 16-bit integer.
    pub const INT16: u8 = 3;
    /// Unsigned 16-bit integer.
    pub const UINT16: u8 = 4;
    /// Signed 32-bit integer.
    pub const INT32: u8 = 5;
    /// Unsigned 32-bit integer.
    pub const UINT32: u8 = 6;
    /// IEEE-754 single precision.
    pub const FLOAT32: u8 = 7;
    /// IEEE-754 double precision.
    pub const FLOAT64: u8 = 8;
}

/// The width in bytes of a `PointField` datatype, or `None` if unrecognized.
fn datatype_size(datatype: u8) -> Option<usize> {
    match datatype {
        datatype::INT8 | datatype::UINT8 => Some(1),
        datatype::INT16 | datatype::UINT16 => Some(2),
        datatype::INT32 | datatype::UINT32 | datatype::FLOAT32 => Some(4),
        datatype::FLOAT64 => Some(8),
        _ => None,
    }
}

/// One entry of a `PointCloud2`'s `fields` array, borrowing its name.
///
/// The name is a `&str` so that building this from a generated message type is
/// a borrow rather than a `String` clone per field per message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FieldRef<'a> {
    /// Field name, `"x"` / `"y"` / `"z"` for the coordinates.
    pub name: &'a str,
    /// Byte offset of the field within a point.
    pub offset: u32,
    /// One of the [`datatype`] constants.
    pub datatype: u8,
    /// Number of elements; only the first is read.
    pub count: u32,
}

impl<'a> FieldRef<'a> {
    /// Builds a field descriptor.
    pub const fn new(name: &'a str, offset: u32, datatype: u8, count: u32) -> Self {
        Self {
            name,
            offset,
            datatype,
            count,
        }
    }
}

/// A `PointCloud2` that could not be decoded.
///
/// Marked `#[non_exhaustive]`: a message format this decoder does not yet
/// reject can become a new variant without that being a breaking change.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum CloudError {
    /// One of the coordinate fields is absent.
    MissingField(&'static str),
    /// A coordinate field has a type this decoder does not recognize.
    UnsupportedDatatype {
        /// The coordinate field that carries it.
        field: &'static str,
        /// The unrecognized `datatype` value.
        datatype: u8,
    },
    /// A field is declared with `count` 0, so it holds nothing.
    EmptyField(&'static str),
    /// A field extends past the end of a point.
    FieldOutOfBounds {
        /// The coordinate field.
        field: &'static str,
        /// Byte one past the field's last.
        end: usize,
        /// The declared stride between points.
        point_step: usize,
    },
    /// The blob is shorter than `height` rows of `row_step` bytes.
    ShortData {
        /// Bytes the declared geometry needs.
        expected: usize,
        /// Bytes actually present.
        actual: usize,
    },
    /// `point_step` is zero, so points do not advance.
    ZeroPointStep,
    /// The declared geometry does not fit in a `usize` on this platform.
    ///
    /// `width`, `height`, `point_step` and `row_step` arrive as `u32` and are
    /// multiplied together to size the blob. On a 64-bit target that product
    /// always fits; on a 32-bit one a hostile message can wrap it, which would
    /// turn a length check into an out-of-range index later. Rejected here
    /// instead.
    GeometryOverflow,
}

impl std::fmt::Display for CloudError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingField(name) => {
                write!(f, "point cloud has no {name:?} field")
            }
            Self::UnsupportedDatatype { field, datatype } => write!(
                f,
                "field {field:?} has unrecognized PointField datatype {datatype}"
            ),
            Self::EmptyField(name) => write!(f, "field {name:?} declares count 0"),
            Self::FieldOutOfBounds {
                field,
                end,
                point_step,
            } => write!(
                f,
                "field {field:?} ends at byte {end} but point_step is only {point_step}"
            ),
            Self::ShortData { expected, actual } => write!(
                f,
                "point cloud data is {actual} bytes, the declared geometry needs {expected}"
            ),
            Self::ZeroPointStep => write!(f, "point_step is zero"),
            Self::GeometryOverflow => write!(
                f,
                "the declared width/height/point_step geometry overflows this platform's usize"
            ),
        }
    }
}

impl std::error::Error for CloudError {}

/// Where one coordinate lives inside a point, and how to read it.
#[derive(Debug, Clone, Copy)]
struct Axis {
    offset: usize,
    datatype: u8,
    size: usize,
}

impl Axis {
    /// Resolves one coordinate field out of the `fields` array.
    fn resolve(
        fields: &[FieldRef<'_>],
        name: &'static str,
        point_step: usize,
    ) -> Result<Self, CloudError> {
        let field = fields
            .iter()
            .find(|f| f.name == name)
            .ok_or(CloudError::MissingField(name))?;

        if field.count == 0 {
            return Err(CloudError::EmptyField(name));
        }

        let size = datatype_size(field.datatype).ok_or(CloudError::UnsupportedDatatype {
            field: name,
            datatype: field.datatype,
        })?;

        let offset = field.offset as usize;
        // `offset` is whatever the message said, up to u32::MAX. Checked so
        // that a near-maximum offset cannot wrap past the bounds test below on
        // a 32-bit target.
        let end = offset
            .checked_add(size)
            .ok_or(CloudError::GeometryOverflow)?;
        if end > point_step {
            return Err(CloudError::FieldOutOfBounds {
                field: name,
                end,
                point_step,
            });
        }

        Ok(Self {
            offset,
            datatype: field.datatype,
            size,
        })
    }

    /// Reads this coordinate out of one point's bytes.
    ///
    /// Every type widens to `f64` before anything else happens: the arithmetic
    /// downstream (range checks, the transform) is `f64`, and narrowing to the
    /// `f32` an [`octomap_core::Point3`] holds is left to the very end.
    fn read(&self, point: &[u8], big_endian: bool) -> f64 {
        let bytes = &point[self.offset..self.offset + self.size];
        macro_rules! scalar {
            ($ty:ty, $n:literal) => {{
                let mut buf = [0u8; $n];
                buf.copy_from_slice(bytes);
                if big_endian {
                    <$ty>::from_be_bytes(buf)
                } else {
                    <$ty>::from_le_bytes(buf)
                }
            }};
        }

        match self.datatype {
            datatype::INT8 => scalar!(i8, 1) as f64,
            datatype::UINT8 => scalar!(u8, 1) as f64,
            datatype::INT16 => scalar!(i16, 2) as f64,
            datatype::UINT16 => scalar!(u16, 2) as f64,
            datatype::INT32 => scalar!(i32, 4) as f64,
            datatype::UINT32 => scalar!(u32, 4) as f64,
            datatype::FLOAT32 => scalar!(f32, 4) as f64,
            datatype::FLOAT64 => scalar!(f64, 8),
            // Unreachable: `resolve` rejects anything `datatype_size` does not
            // know, and that is the same match.
            _ => f64::NAN,
        }
    }
}

/// A borrowed `sensor_msgs/PointCloud2` with its layout already resolved.
#[derive(Debug, Clone, Copy)]
pub struct Cloud<'a> {
    data: &'a [u8],
    x: Axis,
    y: Axis,
    z: Axis,
    width: usize,
    height: usize,
    point_step: usize,
    row_step: usize,
    big_endian: bool,
}

impl<'a> Cloud<'a> {
    /// Resolves a cloud's layout, borrowing its data.
    ///
    /// The arguments are the message's fields verbatim. `row_step` is honored
    /// as the stride between rows, which is how an organized cloud with padded
    /// rows is addressed correctly; a producer that leaves it too small to hold
    /// `width` points is treated as meaning the unpadded `width * point_step`,
    /// since that is the only reading under which its own data is consistent.
    pub fn new(
        fields: &[FieldRef<'_>],
        data: &'a [u8],
        width: u32,
        height: u32,
        point_step: u32,
        row_step: u32,
        is_bigendian: bool,
    ) -> Result<Self, CloudError> {
        let point_step = point_step as usize;
        if point_step == 0 {
            return Err(CloudError::ZeroPointStep);
        }

        let x = Axis::resolve(fields, "x", point_step)?;
        let y = Axis::resolve(fields, "y", point_step)?;
        let z = Axis::resolve(fields, "z", point_step)?;

        let width = width as usize;
        let height = height as usize;

        // Checked, not plain, arithmetic: every operand here comes from the
        // message, and on a 32-bit target the product of two u32 values can
        // wrap a usize. A wrapped `expected` would pass the length check below
        // and leave the out-of-range read to be caught by a slice index — a
        // panic, from a message a stranger sent.
        let packed = width
            .checked_mul(point_step)
            .ok_or(CloudError::GeometryOverflow)?;
        let row_step = (row_step as usize).max(packed);

        // The last row only needs to hold its points, not a full stride of
        // padding — a producer that trims the trailing padding is still
        // self-consistent, and rejecting it would drop otherwise good clouds.
        let expected = match height {
            0 => 0,
            _ => (height - 1)
                .checked_mul(row_step)
                .and_then(|rows| rows.checked_add(packed))
                .ok_or(CloudError::GeometryOverflow)?,
        };
        if data.len() < expected {
            return Err(CloudError::ShortData {
                expected,
                actual: data.len(),
            });
        }

        Ok(Self {
            data,
            x,
            y,
            z,
            width,
            height,
            point_step,
            row_step,
            big_endian: is_bigendian,
        })
    }

    /// Number of points the cloud declares, finite or not.
    pub fn len(&self) -> usize {
        self.width * self.height
    }

    /// Whether the cloud declares no points at all.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Iterates over every point, in message order, including non-finite ones.
    ///
    /// Coordinates are read in `f64` and narrowed on the way out, matching what
    /// the reference C++ stack does with a `float`-typed `pcl::PointXYZ`.
    pub fn iter(&self) -> PointIter<'_, 'a> {
        PointIter {
            cloud: self,
            index: 0,
        }
    }
}

impl<'a, 'b> IntoIterator for &'b Cloud<'a> {
    type Item = Point3;
    type IntoIter = PointIter<'b, 'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

/// Iterator over a [`Cloud`]'s points, in message order.
#[derive(Debug)]
pub struct PointIter<'c, 'a> {
    cloud: &'c Cloud<'a>,
    index: usize,
}

impl Iterator for PointIter<'_, '_> {
    type Item = Point3;

    fn next(&mut self) -> Option<Point3> {
        let cloud = self.cloud;
        if self.index >= cloud.len() {
            return None;
        }

        let row = self.index / cloud.width;
        let col = self.index % cloud.width;
        self.index += 1;

        let base = row * cloud.row_step + col * cloud.point_step;
        let point = &cloud.data[base..base + cloud.point_step];

        Some(Point3::new(
            cloud.x.read(point, cloud.big_endian) as f32,
            cloud.y.read(point, cloud.big_endian) as f32,
            cloud.z.read(point, cloud.big_endian) as f32,
        ))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let left = self.cloud.len() - self.index;
        (left, Some(left))
    }
}

impl ExactSizeIterator for PointIter<'_, '_> {}

#[cfg(test)]
mod tests {
    use super::*;

    /// `x`/`y`/`z` as `float32` at offsets 0/4/8, the layout of every depth
    /// camera and LiDAR driver in practice.
    fn xyz_fields() -> [FieldRef<'static>; 3] {
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

    #[test]
    fn a_packed_xyz_cloud_decodes_in_order() {
        let points = [[1.0, 2.0, 3.0], [-4.0, 5.5, 6.25]];
        let data = blob(&points);
        let cloud = Cloud::new(&xyz_fields(), &data, 2, 1, 12, 24, false).unwrap();

        assert_eq!(cloud.len(), 2);
        let decoded: Vec<_> = cloud.iter().map(|p| [p.x, p.y, p.z]).collect();
        assert_eq!(decoded, points);
    }

    #[test]
    fn extra_fields_and_padding_are_stepped_over() {
        // A realistic LiDAR point: xyz, then intensity, ring, and padding out
        // to a 32-byte stride.
        let fields = [
            FieldRef::new("x", 0, datatype::FLOAT32, 1),
            FieldRef::new("y", 4, datatype::FLOAT32, 1),
            FieldRef::new("z", 8, datatype::FLOAT32, 1),
            FieldRef::new("intensity", 12, datatype::FLOAT32, 1),
            FieldRef::new("ring", 16, datatype::UINT16, 1),
        ];

        let mut data = vec![0u8; 64];
        for (i, p) in [[1.0f32, 2.0, 3.0], [4.0, 5.0, 6.0]].iter().enumerate() {
            let base = i * 32;
            for (a, v) in p.iter().enumerate() {
                data[base + a * 4..base + a * 4 + 4].copy_from_slice(&v.to_le_bytes());
            }
            data[base + 12..base + 16].copy_from_slice(&0.5f32.to_le_bytes());
        }

        let cloud = Cloud::new(&fields, &data, 2, 1, 32, 64, false).unwrap();
        let decoded: Vec<_> = cloud.iter().map(|p| [p.x, p.y, p.z]).collect();
        assert_eq!(decoded, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    }

    #[test]
    fn organized_clouds_honor_row_step_padding() {
        // Two rows of two points, with 8 bytes of padding after each row.
        let point_step = 12usize;
        let row_step = 2 * point_step + 8;
        let mut data = vec![0u8; 2 * row_step];
        let points = [
            [0.0f32, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ];
        for (i, p) in points.iter().enumerate() {
            let base = (i / 2) * row_step + (i % 2) * point_step;
            for (a, v) in p.iter().enumerate() {
                data[base + a * 4..base + a * 4 + 4].copy_from_slice(&v.to_le_bytes());
            }
        }

        let cloud = Cloud::new(
            &xyz_fields(),
            &data,
            2,
            2,
            point_step as u32,
            row_step as u32,
            false,
        )
        .unwrap();
        let decoded: Vec<_> = cloud.iter().map(|p| [p.x, p.y, p.z]).collect();
        assert_eq!(decoded, points, "padding between rows must be skipped");
    }

    #[test]
    fn big_endian_data_is_byte_swapped() {
        let mut data = Vec::new();
        for v in [1.5f32, -2.5, 3.5] {
            data.extend_from_slice(&v.to_be_bytes());
        }

        let cloud = Cloud::new(&xyz_fields(), &data, 1, 1, 12, 12, true).unwrap();
        let p = cloud.iter().next().unwrap();
        assert_eq!([p.x, p.y, p.z], [1.5, -2.5, 3.5]);
    }

    #[test]
    fn every_pointfield_datatype_reads_back() {
        let fields = [
            FieldRef::new("x", 0, datatype::INT16, 1),
            FieldRef::new("y", 2, datatype::UINT8, 1),
            FieldRef::new("z", 3, datatype::FLOAT64, 1),
        ];
        let mut data = Vec::new();
        data.extend_from_slice(&(-7i16).to_le_bytes());
        data.push(200u8);
        data.extend_from_slice(&0.125f64.to_le_bytes());

        let cloud = Cloud::new(&fields, &data, 1, 1, 11, 11, false).unwrap();
        let p = cloud.iter().next().unwrap();
        assert_eq!([p.x, p.y, p.z], [-7.0, 200.0, 0.125]);
    }

    #[test]
    fn non_finite_points_survive_decoding_and_are_left_to_the_filter() {
        let data = blob(&[[f32::NAN, 0.0, 0.0], [1.0, 1.0, 1.0]]);
        let cloud = Cloud::new(&xyz_fields(), &data, 2, 1, 12, 24, false).unwrap();
        let decoded: Vec<_> = cloud.iter().collect();
        assert_eq!(decoded.len(), 2, "decoding does not drop points");
        assert!(decoded[0].x.is_nan());
    }

    #[test]
    fn a_cloud_without_coordinates_is_rejected() {
        let fields = [
            FieldRef::new("intensity", 0, datatype::FLOAT32, 1),
            FieldRef::new("x", 4, datatype::FLOAT32, 1),
            FieldRef::new("y", 8, datatype::FLOAT32, 1),
        ];
        assert_eq!(
            Cloud::new(&fields, &[0u8; 12], 1, 1, 12, 12, false).unwrap_err(),
            CloudError::MissingField("z")
        );
    }

    #[test]
    fn a_field_reaching_past_the_stride_is_rejected() {
        let fields = [
            FieldRef::new("x", 0, datatype::FLOAT32, 1),
            FieldRef::new("y", 4, datatype::FLOAT32, 1),
            FieldRef::new("z", 8, datatype::FLOAT32, 1),
        ];
        assert_eq!(
            Cloud::new(&fields, &[0u8; 32], 1, 1, 10, 10, false).unwrap_err(),
            CloudError::FieldOutOfBounds {
                field: "z",
                end: 12,
                point_step: 10,
            }
        );
    }

    #[test]
    fn a_blob_shorter_than_the_declared_geometry_is_rejected() {
        let data = blob(&[[1.0, 2.0, 3.0]]);
        assert_eq!(
            Cloud::new(&xyz_fields(), &data, 4, 1, 12, 48, false).unwrap_err(),
            CloudError::ShortData {
                expected: 48,
                actual: 12,
            }
        );
    }

    #[test]
    fn a_trimmed_last_row_is_accepted() {
        // row_step says 32, but the producer stopped writing after the last
        // point's 12 bytes instead of padding the final row.
        let point_step = 12usize;
        let row_step = 32usize;
        let mut data = vec![0u8; row_step + point_step];
        data[row_step..row_step + 4].copy_from_slice(&9.0f32.to_le_bytes());

        let cloud = Cloud::new(
            &xyz_fields(),
            &data,
            1,
            2,
            point_step as u32,
            row_step as u32,
            false,
        )
        .unwrap();
        assert_eq!(cloud.iter().nth(1).unwrap().x, 9.0);
    }

    #[test]
    fn an_unknown_datatype_is_reported_with_its_value() {
        let fields = [
            FieldRef::new("x", 0, datatype::FLOAT32, 1),
            FieldRef::new("y", 4, datatype::FLOAT32, 1),
            FieldRef::new("z", 8, 99, 1),
        ];
        assert_eq!(
            Cloud::new(&fields, &[0u8; 12], 1, 1, 12, 12, false).unwrap_err(),
            CloudError::UnsupportedDatatype {
                field: "z",
                datatype: 99,
            }
        );
    }

    #[test]
    fn an_empty_cloud_yields_no_points() {
        let cloud = Cloud::new(&xyz_fields(), &[], 0, 1, 12, 0, false).unwrap();
        assert!(cloud.is_empty());
        assert_eq!(cloud.iter().count(), 0);
    }
}
