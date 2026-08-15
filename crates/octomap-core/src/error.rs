//! Error types for the core crate.

use core::fmt;

/// Errors produced by `octomap-core`.
#[derive(Debug, Clone, PartialEq)]
pub enum OctomapError {
    /// The resolution was not a finite, strictly positive number.
    InvalidResolution {
        /// The rejected value.
        got: f64,
    },
    /// A depth argument exceeded the tree depth.
    InvalidDepth {
        /// The rejected depth.
        got: u32,
        /// The maximum depth the tree accepts.
        tree_depth: u32,
    },
    /// A world coordinate fell outside the addressable volume of the tree.
    CoordinateOutOfBounds {
        /// The rejected coordinate, in meters.
        coordinate: f64,
        /// Half-extent of the addressable volume, in meters.
        half_extent: f64,
    },
    /// A sensor-model probability was outside the range its role allows.
    InvalidProbability {
        /// Which parameter was rejected.
        parameter: &'static str,
        /// The rejected value.
        got: f64,
        /// The range the parameter must lie in.
        expected: &'static str,
    },
}

impl fmt::Display for OctomapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidResolution { got } => {
                write!(f, "resolution must be finite and > 0, got {got}")
            }
            Self::InvalidDepth { got, tree_depth } => {
                write!(f, "depth {got} exceeds tree depth {tree_depth}")
            }
            Self::CoordinateOutOfBounds {
                coordinate,
                half_extent,
            } => write!(
                f,
                "coordinate {coordinate} lies outside the addressable range \
                 [-{half_extent}, {half_extent})"
            ),
            Self::InvalidProbability {
                parameter,
                got,
                expected,
            } => write!(f, "{parameter} must be {expected}, got {got}"),
        }
    }
}

impl std::error::Error for OctomapError {}

/// Convenience alias for results in this crate.
pub type Result<T> = core::result::Result<T, OctomapError>;
