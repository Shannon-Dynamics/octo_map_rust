//! ROS 2 conversions for `octomap-rs`.
//!
//! This crate is the bridge between what a ROS 2 message carries and what
//! [`octomap_core`] works with, and it deliberately has **no ROS dependency**.
//! Every entry point takes the plain data a message holds — a byte blob, a list
//! of field descriptors, a resolution — rather than a generated message type.
//! Three things follow from that:
//!
//! - it builds and tests on any platform, including one with no ROS installed;
//! - the client library is the caller's choice (`r2r`, `rclrs`, a rosbag
//!   reader, a recorded fixture in a test);
//! - the conversions are unit-testable without a running middleware.
//!
//! `ros2/octomap_server_rs` in this repository is the `r2r` side of the
//! boundary: it does nothing but move fields between `r2r` structs and the
//! functions here.
//!
//! # The two directions
//!
//! **Sensor data in.** [`pointcloud2`] decodes a `sensor_msgs/PointCloud2`
//! blob into points, and [`ScanFilter`] turns those into an
//! [`octomap_core::PointCloud`] in the map frame, applying the range, height
//! and subsampling limits a mapping node wants.
//!
//! ```
//! use octomap_core::{OcTree, Point3};
//! use octomap_ros::{pointcloud2::{Cloud, FieldRef, datatype}, ScanFilter, Transform3};
//!
//! # let mut blob = Vec::new();
//! # for p in [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0]] {
//! #     for v in p { blob.extend_from_slice(&v.to_le_bytes()); }
//! # }
//! let fields = [
//!     FieldRef::new("x", 0, datatype::FLOAT32, 1),
//!     FieldRef::new("y", 4, datatype::FLOAT32, 1),
//!     FieldRef::new("z", 8, datatype::FLOAT32, 1),
//! ];
//! let cloud = Cloud::new(&fields, &blob, 2, 1, 12, 12, false)?;
//!
//! let mut map = OcTree::new(0.1)?;
//! let scan = ScanFilter::default().apply(&cloud, &Transform3::IDENTITY);
//! map.insert_point_cloud(&scan, Point3::ORIGIN, -1.0, false, true);
//! assert_eq!(map.is_occupied_at(Point3::new(1.0, 0.0, 0.0)), Some(true));
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! **Map out.** [`msg`] produces the `data` payload of an
//! `octomap_msgs/Octomap` message, and reads one back. The payload is
//! headerless — the resolution and tree id travel in their own message fields —
//! which is why it is not the same bytes as a `.bt` or `.ot` file. See
//! [`msg`] for the details, they matter for interoperability.
//!
//! [`voxels`] covers the third thing a mapping node publishes: the occupied
//! cells, as centers and edge lengths ready to become a
//! `visualization_msgs/MarkerArray`.

pub mod msg;
pub mod pointcloud2;
pub mod voxels;

mod filter;
mod transform;

pub use filter::{ScanFilter, ScanStats};
pub use msg::{OctomapPayload, PayloadError};
pub use pointcloud2::{Cloud, CloudError, FieldRef};
pub use transform::Transform3;
pub use voxels::{height_color, Voxel};
