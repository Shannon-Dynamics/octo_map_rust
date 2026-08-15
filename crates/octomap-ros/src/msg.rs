//! The payload of an `octomap_msgs/Octomap` message.
//!
//! ```text
//! std_msgs/Header header
//! bool            binary
//! string          id
//! float64         resolution
//! int8[]          data
//! ```
//!
//! # `data` is not a file
//!
//! The single most common way to get this wrong is to put the contents of a
//! `.bt` or `.ot` file into `data`. It will not decode. The message carries the
//! resolution and the tree id in their own fields, so `data` holds **only the
//! node payload**, with the text header stripped — that is what the C++
//! `octomap_msgs::binaryMapToMsg` writes (`writeBinaryData`, not `writeBinary`)
//! and what `binaryMsgToMap` reads back (`readBinaryData`).
//!
//! [`binary_payload`] and [`full_payload`] produce that, and [`decode`] parses
//! it. A tree that goes out through one and comes back through the other is the
//! same tree; a tree written by the C++ node decodes here and vice versa.
//!
//! # Binary or full
//!
//! `binary` selects the format, and the two are not interchangeable:
//!
//! - **binary** ([`binary_payload`]) reduces every voxel to one bit. Small,
//!   fast, and what RViz's octomap display and most consumers expect. The
//!   confidence behind each voxel is gone.
//! - **full** ([`full_payload`]) keeps every node's `f32` log-odds, so the
//!   receiver can keep updating the map. Several times larger.
//!
//! A mapping node normally publishes both, on separate topics, and lets
//! subscribers pick.

use octomap_core::io::{self, IoError};
use octomap_core::OcTree;

/// The tree type id this crate reads and writes.
///
/// The C++ side dispatches on this string to decide which class to construct.
/// `"ColorOcTree"`, `"OcTreeStamped"` and the rest are not implemented by this
/// port, so a message carrying one of those is refused rather than silently
/// decoded as a plain occupancy tree.
pub const TREE_ID: &str = "OcTree";

/// An `octomap_msgs/Octomap` without its header.
///
/// The stamp and frame belong to the publishing node, not to the map, so they
/// are left for the caller to fill in.
#[derive(Debug, Clone, PartialEq)]
pub struct OctomapPayload {
    /// The message's `binary` field.
    pub binary: bool,
    /// The message's `id` field; always [`TREE_ID`] here.
    pub id: &'static str,
    /// The message's `resolution` field, in meters.
    pub resolution: f64,
    /// The message's `data` field, as unsigned bytes.
    ///
    /// ROS types this `int8[]`, which a Rust binding surfaces as `Vec<i8>`.
    /// [`into_i8`](Self::into_i8) does that reinterpretation; the bits are
    /// unchanged either way.
    pub data: Vec<u8>,
}

impl OctomapPayload {
    /// Reinterprets the payload as the `Vec<i8>` the generated message wants.
    ///
    /// A pure sign reinterpretation — `0xFF` becomes `-1`, no bits move.
    pub fn into_i8(self) -> Vec<i8> {
        to_i8(&self.data)
    }
}

/// Reinterprets message bytes as signed, for an `int8[]` field.
pub fn to_i8(data: &[u8]) -> Vec<i8> {
    data.iter().map(|&b| b as i8).collect()
}

/// Reinterprets an `int8[]` field's contents as bytes.
pub fn from_i8(data: &[i8]) -> Vec<u8> {
    data.iter().map(|&b| b as u8).collect()
}

/// A payload that could not be built or decoded.
#[derive(Debug)]
pub enum PayloadError {
    /// The node payload is malformed.
    Io(IoError),
    /// The message declares a tree class this crate does not implement.
    UnsupportedTreeId(String),
    /// The message declares a resolution no tree can have.
    InvalidResolution(f64),
}

impl std::fmt::Display for PayloadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "{e}"),
            Self::UnsupportedTreeId(id) => write!(
                f,
                "message carries tree type {id:?}, this crate only implements {TREE_ID:?}"
            ),
            Self::InvalidResolution(r) => {
                write!(f, "message declares resolution {r}, which is not usable")
            }
        }
    }
}

impl std::error::Error for PayloadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<IoError> for PayloadError {
    fn from(e: IoError) -> Self {
        Self::Io(e)
    }
}

/// Builds a `binary: true` payload, leaving the map untouched.
///
/// Every voxel collapses to one bit, decided by the tree's occupancy
/// threshold. The map itself keeps its log-odds, so this is the call a live
/// mapping node makes on every publish — it can publish a binary map at 1 Hz
/// forever without degrading what it is building.
///
/// [`binary_payload_collapsed`] is the smaller, destructive alternative.
pub fn binary_payload(tree: &OcTree) -> Result<OctomapPayload, PayloadError> {
    let mut data = Vec::new();
    io::write_binary_data(tree, &mut data)?;
    Ok(OctomapPayload {
        binary: true,
        id: TREE_ID,
        resolution: tree.resolution(),
        data,
    })
}

/// Builds a `binary: true` payload after collapsing and pruning the map.
///
/// **This modifies the map**: every voxel is pushed to whichever clamp it is
/// nearer, which then lets whole uniform regions prune into single nodes. The
/// payload gets much smaller, and the map loses the confidence values it would
/// have needed to keep refining itself.
///
/// Right for a node that has finished mapping and is about to publish or save
/// the result once. Wrong for a node still integrating scans.
pub fn binary_payload_collapsed(tree: &mut OcTree) -> Result<OctomapPayload, PayloadError> {
    tree.to_max_likelihood();
    tree.prune();
    binary_payload(tree)
}

/// Builds a `binary: false` payload, preserving every node's log-odds.
pub fn full_payload(tree: &OcTree) -> Result<OctomapPayload, PayloadError> {
    let mut data = Vec::new();
    io::write_full_data(tree, &mut data)?;
    Ok(OctomapPayload {
        binary: false,
        id: TREE_ID,
        resolution: tree.resolution(),
        data,
    })
}

/// Decodes a received `octomap_msgs/Octomap` into a map.
///
/// The arguments are the message's fields. `binary` selects which reader is
/// used and has to be honored: the two encodings share no framing, and reading
/// one as the other yields nonsense rather than an error.
///
/// An empty `data` decodes to an empty map, which is what a publisher that has
/// not seen a scan yet sends.
pub fn decode(
    binary: bool,
    id: &str,
    resolution: f64,
    data: &[u8],
) -> Result<OcTree, PayloadError> {
    if id != TREE_ID {
        return Err(PayloadError::UnsupportedTreeId(id.to_string()));
    }
    if !(resolution.is_finite() && resolution > 0.0) {
        return Err(PayloadError::InvalidResolution(resolution));
    }

    let mut reader = data;
    let tree = if binary {
        io::read_binary_data(&mut reader, resolution)?
    } else {
        io::read_full_data(&mut reader, resolution)?
    };
    Ok(tree)
}

/// [`decode`], taking the `int8[]` field directly.
pub fn decode_i8(
    binary: bool,
    id: &str,
    resolution: f64,
    data: &[i8],
) -> Result<OcTree, PayloadError> {
    decode(binary, id, resolution, &from_i8(data))
}

#[cfg(test)]
mod tests {
    use super::*;
    use octomap_core::Point3;

    fn scene() -> OcTree {
        let mut tree = OcTree::new(0.1).unwrap();
        for i in 0..20 {
            let x = 1.0 + i as f32 * 0.1;
            tree.update_node_at(Point3::new(x, 0.05, 0.05), true);
        }
        tree.update_node_at(Point3::new(0.55, 0.05, 0.05), false);
        tree.update_node_at(Point3::new(-2.05, 0.35, 0.45), false);
        tree
    }

    #[test]
    fn a_binary_payload_round_trips_through_decode() {
        let tree = scene();
        let payload = binary_payload(&tree).unwrap();
        assert!(payload.binary);
        assert_eq!(payload.id, "OcTree");
        assert_eq!(payload.resolution, 0.1);

        let back = decode(
            payload.binary,
            payload.id,
            payload.resolution,
            &payload.data,
        )
        .unwrap();

        for leaf in tree.iter_leaves() {
            assert_eq!(
                back.is_occupied(leaf.key()),
                Some(tree.sensor().is_occupied(*leaf.value())),
            );
        }
    }

    #[test]
    fn a_full_payload_round_trips_every_log_odds_value() {
        let tree = scene();
        let payload = full_payload(&tree).unwrap();
        assert!(!payload.binary);

        let back = decode(false, payload.id, payload.resolution, &payload.data).unwrap();
        let original: Vec<_> = tree
            .iter_leaves()
            .map(|v| (v.key(), v.value().log_odds.to_bits()))
            .collect();
        let restored: Vec<_> = back
            .iter_leaves()
            .map(|v| (v.key(), v.value().log_odds.to_bits()))
            .collect();
        assert_eq!(restored, original);
    }

    #[test]
    fn the_payload_is_not_a_file_and_carries_no_header() {
        let tree = scene();
        let binary = binary_payload(&tree).unwrap();
        let full = full_payload(&tree).unwrap();

        // The mistake this guards against is `write_binary` reaching a message
        // instead of `write_binary_data`. The header is what tells them apart.
        assert!(!binary.data.starts_with(b"# Octomap"));
        assert!(!full.data.starts_with(b"# Octomap"));

        let mut file = Vec::new();
        octomap_core::io::write_full(&tree, &mut file).unwrap();
        assert!(file.starts_with(b"# Octomap"));
        assert!(file.ends_with(&full.data), "the payload is the file's tail");
    }

    #[test]
    fn collapsing_shrinks_the_payload_and_costs_the_log_odds() {
        let mut tree = scene();
        let before = binary_payload(&tree).unwrap().data.len();
        let nodes_before = tree.len();

        let after = binary_payload_collapsed(&mut tree).unwrap().data.len();
        assert!(
            after <= before,
            "collapsing then pruning should not grow the payload"
        );
        assert!(tree.len() <= nodes_before);
        for leaf in tree.iter_leaves() {
            let l = leaf.value().log_odds;
            assert!(
                l == tree.sensor().clamping_thres_max() || l == tree.sensor().clamping_thres_min(),
                "every node should sit on a clamp after collapsing"
            );
        }
    }

    #[test]
    fn an_empty_map_produces_an_empty_payload_that_decodes_back() {
        let tree = OcTree::new(0.05).unwrap();
        let payload = binary_payload(&tree).unwrap();
        assert!(payload.data.is_empty());

        let back = decode(true, TREE_ID, 0.05, &payload.data).unwrap();
        assert!(back.is_empty());
        assert_eq!(back.resolution(), 0.05);
    }

    #[test]
    fn an_unimplemented_tree_class_is_refused_by_name() {
        let err = decode(true, "ColorOcTree", 0.1, &[]).unwrap_err();
        assert!(matches!(err, PayloadError::UnsupportedTreeId(id) if id == "ColorOcTree"));
    }

    #[test]
    fn a_resolution_the_message_should_never_carry_is_refused() {
        for bad in [0.0, -0.1, f64::NAN, f64::INFINITY] {
            assert!(matches!(
                decode(true, TREE_ID, bad, &[]),
                Err(PayloadError::InvalidResolution(_))
            ));
        }
    }

    #[test]
    fn signed_and_unsigned_views_of_the_payload_hold_the_same_bits() {
        let tree = scene();
        let payload = binary_payload(&tree).unwrap();
        let bytes = payload.data.clone();

        let signed = payload.into_i8();
        assert_eq!(from_i8(&signed), bytes);
        assert_eq!(signed.len(), bytes.len());
        assert!(bytes.iter().any(|&b| b > 127), "test needs a high byte");
    }

    #[test]
    fn a_truncated_payload_is_an_error_not_a_partial_map() {
        let tree = scene();
        let mut payload = full_payload(&tree).unwrap();
        payload.data.truncate(payload.data.len() / 2);

        assert!(matches!(
            decode(false, TREE_ID, 0.1, &payload.data),
            Err(PayloadError::Io(_))
        ));
    }
}
