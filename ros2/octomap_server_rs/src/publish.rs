//! Building the outgoing messages.
//!
//! Everything here is mechanical: take a map, produce a message. The
//! interesting decisions — what a payload contains, where a voxel's center is,
//! what color a height gets — live in `octomap-ros`, which has no ROS
//! dependency and is unit-tested. This module is the part that cannot be
//! tested without a middleware, so it is kept as thin as it can be.

use octomap_core::OcTree;
use octomap_ros::voxels::{self, Voxel};
use octomap_ros::{height_color, OctomapPayload};

use r2r::builtin_interfaces::msg::{Duration, Time};
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion, Vector3};
use r2r::octomap_msgs::msg::Octomap as OctomapMsg;
use r2r::sensor_msgs::msg::{PointCloud2, PointField};
use r2r::std_msgs::msg::{ColorRGBA, Header};
use r2r::visualization_msgs::msg::{Marker, MarkerArray};

// The message constants `r2r` generates come from bindgen, so each one has its
// own anonymous enum type rather than the field's type. The casts below are
// that, and nothing more.

/// Wraps a payload in an `octomap_msgs/Octomap`.
pub fn octomap_message(payload: OctomapPayload, frame_id: &str, stamp: Time) -> OctomapMsg {
    OctomapMsg {
        header: Header {
            stamp,
            frame_id: frame_id.to_string(),
        },
        binary: payload.binary,
        id: payload.id.to_string(),
        resolution: payload.resolution,
        data: payload.into_i8(),
    }
}

/// Header shared by every message published in one round.
fn header(frame_id: &str, stamp: &Time) -> Header {
    Header {
        stamp: stamp.clone(),
        frame_id: frame_id.to_string(),
    }
}

/// A marker's `pose` when its points already carry map-frame coordinates.
///
/// A `CUBE_LIST` positions each cube by its entry in `points`, relative to this
/// pose — so it has to be the identity, and the orientation has to be a real
/// unit quaternion. Leaving it zeroed is the classic way to get an empty RViz
/// display and a "uninitialized quaternion" warning.
fn identity_pose() -> Pose {
    Pose {
        position: Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    }
}

/// The vertical extent of a set of voxels, for the height ramp.
///
/// The reference node rescales its colors to the map's own bounds on every
/// publish, so this follows the map as it grows rather than needing a
/// hand-tuned range.
fn height_range(voxels: &[Voxel]) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for v in voxels {
        min = min.min(v.center.z as f64);
        max = max.max(v.center.z as f64);
    }
    if min > max {
        (0.0, 1.0)
    } else {
        (min, max)
    }
}

/// Turns the map's cells into one `CUBE_LIST` marker per octree depth.
///
/// One marker cannot draw two cube sizes — `scale` is per marker, not per
/// point — and a pruned tree holds nodes at many depths, so the split by depth
/// is forced rather than chosen.
///
/// Every depth the tree could use gets a marker, including the empty ones,
/// which are published with `DELETE`. Without that, a region that was occupied
/// and has since been cleared keeps being drawn: RViz holds the last marker it
/// saw for an id forever.
pub fn marker_array(
    tree: &OcTree,
    frame_id: &str,
    stamp: &Time,
    namespace: &str,
    color_factor: f64,
    occupied: bool,
) -> MarkerArray {
    let mut by_depth: Vec<Vec<Voxel>> = vec![Vec::new(); tree.geometry().tree_depth() as usize + 1];
    let cells: Box<dyn Iterator<Item = Voxel> + '_> = if occupied {
        Box::new(voxels::occupied_voxels(tree))
    } else {
        Box::new(voxels::free_voxels(tree))
    };
    for voxel in cells {
        by_depth[voxel.depth as usize].push(voxel);
    }

    let all: Vec<Voxel> = by_depth.iter().flatten().copied().collect();
    let (min_z, max_z) = height_range(&all);

    let markers = by_depth
        .into_iter()
        .enumerate()
        .map(|(depth, cells)| {
            if cells.is_empty() {
                return Marker {
                    header: header(frame_id, stamp),
                    ns: namespace.to_string(),
                    id: depth as i32,
                    action: Marker::DELETE as i32,
                    ..Marker::default()
                };
            }

            let size = cells[0].size;
            let points = cells
                .iter()
                .map(|v| Point {
                    x: v.center.x as f64,
                    y: v.center.y as f64,
                    z: v.center.z as f64,
                })
                .collect();
            let colors = cells
                .iter()
                .map(|v| {
                    let [r, g, b, a] = height_color(v.center.z as f64, min_z, max_z, color_factor);
                    ColorRGBA { r, g, b, a }
                })
                .collect();

            Marker {
                header: header(frame_id, stamp),
                ns: namespace.to_string(),
                id: depth as i32,
                type_: Marker::CUBE_LIST as i32,
                action: Marker::ADD as i32,
                pose: identity_pose(),
                scale: Vector3 {
                    x: size,
                    y: size,
                    z: size,
                },
                // Per-point colors override this, but a marker with an alpha of
                // zero is invisible in some clients regardless, so it is set.
                color: ColorRGBA {
                    r: 0.5,
                    g: 0.5,
                    b: 0.5,
                    a: 1.0,
                },
                points,
                colors,
                lifetime: Duration { sec: 0, nanosec: 0 },
                frame_locked: false,
                ..Marker::default()
            }
        })
        .collect();

    MarkerArray { markers }
}

/// A `MarkerArray` that removes everything this node has drawn.
pub fn clear_markers(namespace: &str, frame_id: &str, stamp: &Time) -> MarkerArray {
    MarkerArray {
        markers: vec![Marker {
            header: header(frame_id, stamp),
            ns: namespace.to_string(),
            action: Marker::DELETEALL as i32,
            ..Marker::default()
        }],
    }
}

/// The occupied cell centers as an unorganized XYZ `PointCloud2`.
///
/// Cheaper for RViz to draw than the marker array and easier to feed into
/// anything expecting a cloud, at the cost of losing each cell's size.
pub fn centers_cloud(tree: &OcTree, frame_id: &str, stamp: &Time) -> PointCloud2 {
    const POINT_STEP: u32 = 12;

    let centers: Vec<_> = voxels::occupied_voxels(tree).map(|v| v.center).collect();
    let mut data = Vec::with_capacity(centers.len() * POINT_STEP as usize);
    for c in &centers {
        data.extend_from_slice(&c.x.to_le_bytes());
        data.extend_from_slice(&c.y.to_le_bytes());
        data.extend_from_slice(&c.z.to_le_bytes());
    }

    let field = |name: &str, offset: u32| PointField {
        name: name.to_string(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };

    PointCloud2 {
        header: header(frame_id, stamp),
        height: 1,
        width: centers.len() as u32,
        fields: vec![field("x", 0), field("y", 4), field("z", 8)],
        // `to_le_bytes` above, unconditionally — the same choice the map file
        // format makes, and every platform this runs on is little-endian.
        is_bigendian: false,
        point_step: POINT_STEP,
        row_step: POINT_STEP * centers.len() as u32,
        data,
        is_dense: true,
    }
}
