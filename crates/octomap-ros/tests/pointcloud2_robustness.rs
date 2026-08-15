//! Robustness tests for `PointCloud2` decoding.
//!
//! A mapping node accepts point clouds from whatever is on the ROS graph, so
//! this decoder is the crate's most exposed surface. These tests assert the
//! property that matters: for *any* combination of field descriptors,
//! dimensions and blob length, `Cloud::new` either rejects the message or
//! produces a cloud that can be iterated to the end — never a panic, and never
//! a read outside the buffer it was given.
//!
//! The generator is a small xorshift rather than a dependency, so any failure
//! replays from its seed.

use octomap_ros::pointcloud2::{datatype, Cloud, FieldRef};
use octomap_ros::{ScanFilter, Transform3};

struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    fn below(&mut self, limit: u64) -> u64 {
        self.next_u64() % limit.max(1)
    }
}

/// Every datatype code the decoder might meet, valid or not.
const DATATYPES: [u8; 10] = [
    datatype::INT8,
    datatype::UINT8,
    datatype::INT16,
    datatype::UINT16,
    datatype::INT32,
    datatype::UINT32,
    datatype::FLOAT32,
    datatype::FLOAT64,
    0,   // not a PointField datatype
    200, // nor is this
];

#[test]
fn hostile_geometry_is_rejected_or_survivable() {
    let mut rng = Rng(0xBADC0DE);

    for _ in 0..4000 {
        // Deliberately unconstrained: offsets and steps that make no sense
        // together are the whole point.
        let point_step = rng.below(40) as u32;
        let width = rng.below(64) as u32;
        let height = rng.below(8) as u32;
        let row_step = rng.below(256) as u32;
        let blob_len = rng.below(512) as usize;

        let names = ["x", "y", "z"];
        let fields: Vec<FieldRef<'_>> = names
            .iter()
            .map(|name| {
                FieldRef::new(
                    name,
                    rng.below(48) as u32,
                    DATATYPES[rng.below(DATATYPES.len() as u64) as usize],
                    rng.below(3) as u32,
                )
            })
            .collect();

        let blob: Vec<u8> = (0..blob_len)
            .map(|_| (rng.next_u64() & 0xFF) as u8)
            .collect();

        let Ok(cloud) = Cloud::new(
            &fields,
            &blob,
            width,
            height,
            point_step,
            row_step,
            rng.below(2) == 0,
        ) else {
            continue;
        };

        // Accepted: then every point it claims must be readable. Walking the
        // whole iterator is what would fault if a length check had been
        // computed wrongly.
        let counted = cloud.iter().count();
        assert_eq!(counted, cloud.len(), "iterator disagreed with len()");

        // And the filter — the other consumer of a decoded cloud — must cope
        // with whatever came out, including non-finite coordinates.
        let scan = ScanFilter::default().apply(&cloud, &Transform3::IDENTITY);
        assert!(scan.len() <= counted, "filter invented points");
    }
}

#[test]
fn a_field_offset_past_the_point_is_rejected() {
    let fields = [
        FieldRef::new("x", 0, datatype::FLOAT32, 1),
        FieldRef::new("y", 4, datatype::FLOAT32, 1),
        // Ends at byte 16, past a 12-byte point.
        FieldRef::new("z", 12, datatype::FLOAT32, 1),
    ];
    let blob = vec![0u8; 120];
    assert!(Cloud::new(&fields, &blob, 10, 1, 12, 120, false).is_err());
}

#[test]
fn an_extreme_offset_does_not_wrap() {
    // The wrap this guards against only bites on a 32-bit target, where
    // `offset + size` can exceed usize. The check has to reject it on every
    // target, which is what makes it testable here.
    let fields = [
        FieldRef::new("x", u32::MAX, datatype::FLOAT64, 1),
        FieldRef::new("y", 4, datatype::FLOAT32, 1),
        FieldRef::new("z", 8, datatype::FLOAT32, 1),
    ];
    let blob = vec![0u8; 120];
    assert!(Cloud::new(&fields, &blob, 10, 1, 12, 120, false).is_err());
}

#[test]
fn a_blob_shorter_than_the_declared_geometry_is_rejected() {
    let fields = [
        FieldRef::new("x", 0, datatype::FLOAT32, 1),
        FieldRef::new("y", 4, datatype::FLOAT32, 1),
        FieldRef::new("z", 8, datatype::FLOAT32, 1),
    ];
    // 1000 points declared, 12 bytes supplied.
    let blob = vec![0u8; 12];
    assert!(Cloud::new(&fields, &blob, 1000, 1, 12, 12000, false).is_err());
}

#[test]
fn a_cloud_of_pure_noise_yields_no_impossible_points() {
    // Random bytes decode to whatever they decode to, including NaN and
    // infinities. The filter's job is to drop those, and the map must never
    // see one.
    let mut rng = Rng(0x5EED_5EED);
    let fields = [
        FieldRef::new("x", 0, datatype::FLOAT32, 1),
        FieldRef::new("y", 4, datatype::FLOAT32, 1),
        FieldRef::new("z", 8, datatype::FLOAT32, 1),
    ];

    for _ in 0..200 {
        let points = rng.below(64) as usize;
        let blob: Vec<u8> = (0..points * 12)
            .map(|_| (rng.next_u64() & 0xFF) as u8)
            .collect();
        let cloud = Cloud::new(
            &fields,
            &blob,
            points as u32,
            1,
            12,
            (points * 12) as u32,
            false,
        )
        .expect("the geometry is self-consistent");

        let scan = ScanFilter::default().apply(&cloud, &Transform3::IDENTITY);
        for p in scan.iter() {
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "a non-finite point reached the scan: {p:?}"
            );
        }
    }
}
