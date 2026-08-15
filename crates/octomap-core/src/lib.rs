#![doc = include_str!("../README.md")]
//!
//! # Module map
//!
//! | Module | What lives there |
//! |---|---|
//! | [`point`] | [`Point3`], the world coordinate type |
//! | [`pose`] | [`Pose6`] and [`Quaternion`] — sensor poses and rotations |
//! | [`key`] | [`OcTreeKey`], [`KeyRay`] — integer voxel addresses |
//! | [`geometry`] | [`TreeGeometry`] — every coordinate ↔ key conversion |
//! | [`node`] | [`Node<T>`](Node) — one octree node with lazily allocated children |
//! | [`tree`] | [`OctreeCore<T>`](OctreeCore) — the generic octree and its iterators |
//! | [`occupancy`] | [`OcTree`], [`SensorModel`] — the probabilistic occupancy map |
//! | [`ray`] | [`PointCloud`], ray traversal, ray casting, scan integration |
//! | [`io`] | `.bt` / `.ot` files and the headerless message payloads |
//! | [`error`] | [`OctomapError`], the crate's error type |
//!
//! # Errors
//!
//! Two error types, split by what can go wrong:
//!
//! - [`OctomapError`] — invalid arguments: a resolution that is not finite and
//!   positive, a depth beyond the tree, a coordinate outside the addressable
//!   volume, a sensor-model probability outside its allowed range.
//! - [`IoError`] — everything reading or writing can hit, including a wrapped
//!   [`std::io::Error`] and malformed file contents.
//!
//! Queries that can legitimately have no answer return [`Option`] rather than an
//! error: an unobserved voxel is not a failure.
//!
//! # Panics
//!
//! The library does not panic on any input it accepts. Arithmetic on world
//! coordinates saturates or is bounds-checked before use, and every fallible
//! entry point returns [`Result`] or [`Option`]. The one exception is
//! [`Index`](core::ops::Index) on [`Point3`], which panics on an index above 2
//! the same way indexing a slice out of range does — documented on the impl.
//!
//! # Safety
//!
//! `unsafe_code` is `forbid`-ed at the workspace level, so this crate contains
//! no `unsafe` blocks, no raw pointers and no FFI. See `SAFETY.md` in the
//! repository for the full policy.

pub mod error;
pub mod geometry;
pub mod io;
pub mod key;
pub mod node;
pub mod occupancy;
pub mod point;
pub mod pose;
pub mod ray;
pub mod tree;

pub use error::{OctomapError, Result};
pub use geometry::{TreeGeometry, DEFAULT_TREE_DEPTH, DEFAULT_TREE_MAX_VAL};
pub use io::{
    read_binary, read_binary_data, read_binary_file, read_full, read_full_data, read_full_file,
    write_binary, write_binary_const, write_binary_data, write_binary_file, write_full,
    write_full_data, write_full_file, IoError,
};
pub use key::{
    compute_child_index, compute_child_key, compute_index_key, KeyRay, KeyScalar, OcTreeKey,
    KEY_RAY_MAX_SIZE,
};
pub use node::{Node, CHILD_COUNT};
pub use occupancy::{log_odds, probability, OcTree, OccupancyValue, SensorModel};
pub use point::Point3;
pub use pose::{Pose6, Quaternion};
pub use ray::{compute_ray_keys, PointCloud, RayCast, RayCastMiss};
pub use tree::{LeafIter, OctreeCore, TreeIter, Visit};
