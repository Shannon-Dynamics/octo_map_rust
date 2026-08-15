//! A minimal transform buffer, built from `/tf` and `/tf_static`.
//!
//! ROS 2 ships `tf2_ros`, and this is not it. `tf2_ros` is a C++ library with
//! no Rust binding, and the part of it a mapping node actually uses is small:
//! resolve one frame into another, once per scan. So this module keeps the
//! frame graph and walks it.
//!
//! # What it does not do
//!
//! **It does not interpolate over time.** Each edge keeps only the latest
//! transform received for it, and a lookup uses whatever is current — the
//! `stamp` on the cloud being processed is ignored. For a robot moving slowly
//! relative to the TF publication rate the error is small; for a fast one, or
//! for a bag replayed with clumped timestamps, it shows up as scans smeared
//! along the path. That is the trade for not binding to `tf2`.
//!
//! `use_tf: false` sidesteps the whole thing when clouds already arrive in the
//! map frame, which is the common case for a static sensor or a simulator.
//!
//! **It has no timeout.** A transform published once and never again stays
//! usable forever. `tf2` would declare it expired.

use std::collections::HashMap;

use octomap_ros::Transform3;

/// How deep a frame chain may go before it is treated as a cycle.
///
/// A real robot's TF tree is a handful of links deep. Anything past this is a
/// misconfiguration — most often two publishers claiming the same child frame,
/// which turns the graph into a loop and would otherwise spin forever.
const MAX_CHAIN: usize = 64;

/// A lookup that could not be answered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TfError {
    /// The frame has never appeared, as a parent or a child.
    UnknownFrame(String),
    /// Both frames are known but sit in disconnected parts of the graph.
    Disconnected {
        /// Frame being resolved into.
        target: String,
        /// Frame being resolved from.
        source: String,
    },
    /// Following parents from this frame never reached a root.
    Cycle(String),
}

impl std::fmt::Display for TfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownFrame(frame) => {
                write!(f, "frame {frame:?} has not appeared on /tf or /tf_static")
            }
            Self::Disconnected { target, source } => write!(
                f,
                "frames {source:?} and {target:?} are not connected; \
                 something between them is not being published"
            ),
            Self::Cycle(frame) => write!(
                f,
                "the parents of frame {frame:?} form a cycle, so it has no root"
            ),
        }
    }
}

impl std::error::Error for TfError {}

/// One edge of the frame graph: a child and the transform to its parent.
#[derive(Debug, Clone)]
struct Edge {
    parent: String,
    /// `parent_T_child`, exactly as `geometry_msgs/TransformStamped` carries it.
    transform: Transform3,
    stamp_ns: i64,
    is_static: bool,
}

/// The frame graph, keyed by child frame.
///
/// Every frame has at most one parent — that is the invariant `tf2` enforces
/// too — so a child name identifies an edge.
#[derive(Debug, Default)]
pub struct TfBuffer {
    edges: HashMap<String, Edge>,
}

/// Drops the leading slash a ROS 1 era publisher may still emit.
///
/// `tf2` normalizes the same way. Without it `/base_link` and `base_link` are
/// two frames and the lookup between them fails for no visible reason.
fn normalize(frame: &str) -> &str {
    frame.strip_prefix('/').unwrap_or(frame)
}

impl TfBuffer {
    /// An empty graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Records `parent_T_child`.
    ///
    /// A dynamic transform older than the one already held for the same edge is
    /// dropped: DDS does not guarantee ordering, and letting a late arrival win
    /// makes the map jitter backwards. Static transforms always win, since they
    /// are republished verbatim to every late subscriber.
    pub fn insert(
        &mut self,
        parent: &str,
        child: &str,
        transform: Transform3,
        stamp_ns: i64,
        is_static: bool,
    ) {
        let parent = normalize(parent);
        let child = normalize(child);

        if let Some(existing) = self.edges.get(child) {
            let stale = !is_static
                && !existing.is_static
                && existing.parent == parent
                && stamp_ns < existing.stamp_ns;
            if stale {
                return;
            }
        }

        self.edges.insert(
            child.to_string(),
            Edge {
                parent: parent.to_string(),
                transform,
                stamp_ns,
                is_static,
            },
        );
    }

    /// Number of edges held.
    pub fn len(&self) -> usize {
        self.edges.len()
    }

    /// Whether the graph mentions this frame at all, as parent or child.
    fn knows(&self, frame: &str) -> bool {
        self.edges.contains_key(frame) || self.edges.values().any(|e| e.parent == frame)
    }

    /// Walks from `frame` up to its root.
    ///
    /// Returns the frames encountered, starting with `frame` itself, and the
    /// transform of each step — `steps[i]` is `frames[i + 1]_T_frames[i]`.
    fn chain<'a>(&'a self, frame: &'a str) -> Result<(Vec<&'a str>, Vec<Transform3>), TfError> {
        let mut frames = vec![frame];
        let mut steps = Vec::new();
        let mut current = frame;

        while let Some(edge) = self.edges.get(current) {
            if frames.len() > MAX_CHAIN || frames.contains(&edge.parent.as_str()) {
                return Err(TfError::Cycle(frame.to_string()));
            }
            steps.push(edge.transform);
            frames.push(&edge.parent);
            current = &edge.parent;
        }

        Ok((frames, steps))
    }

    /// Resolves `target_T_source`: the transform that maps a point given in
    /// `source` into `target`.
    ///
    /// For a cloud in `sensor` being mapped into `map`, that is
    /// `lookup("map", "sensor")`, and applying it to each point puts the scan
    /// in the map frame.
    pub fn lookup(&self, target: &str, source: &str) -> Result<Transform3, TfError> {
        let target = normalize(target);
        let source = normalize(source);

        if target == source {
            return Ok(Transform3::IDENTITY);
        }
        if !self.knows(target) {
            return Err(TfError::UnknownFrame(target.to_string()));
        }
        if !self.knows(source) {
            return Err(TfError::UnknownFrame(source.to_string()));
        }

        let (source_frames, source_steps) = self.chain(source)?;
        let (target_frames, target_steps) = self.chain(target)?;

        // The first frame on the source's path that also lies on the target's
        // is their nearest common ancestor. Walking the source path outward
        // rather than either path inward is what makes it nearest: the first
        // hit is the shallowest join.
        let ancestor_in_target: HashMap<&str, usize> = target_frames
            .iter()
            .enumerate()
            .map(|(i, f)| (*f, i))
            .collect();

        let (source_up, target_up) = source_frames
            .iter()
            .enumerate()
            .find_map(|(i, f)| ancestor_in_target.get(f).map(|j| (i, *j)))
            .ok_or_else(|| TfError::Disconnected {
                target: target.to_string(),
                source: source.to_string(),
            })?;

        let ancestor_t_source = compose_chain(&source_steps[..source_up]);
        let ancestor_t_target = compose_chain(&target_steps[..target_up]);

        Ok(ancestor_t_target.inverse().compose(&ancestor_t_source))
    }
}

/// Folds a chain of `parent_T_child` steps into one `ancestor_T_frame`.
fn compose_chain(steps: &[Transform3]) -> Transform3 {
    steps
        .iter()
        .fold(Transform3::IDENTITY, |acc, step| step.compose(&acc))
}

#[cfg(test)]
mod tests {
    use super::*;
    use octomap_core::Point3;

    fn close(a: Point3, b: [f32; 3]) {
        for (got, want) in [a.x, a.y, a.z].iter().zip(b.iter()) {
            assert!(
                (got - want).abs() < 1e-4,
                "got {:?}, want {b:?}",
                [a.x, a.y, a.z]
            );
        }
    }

    /// `map → odom → base_link → sensor`, the shape of a real robot's tree.
    fn robot() -> TfBuffer {
        let mut tf = TfBuffer::new();
        tf.insert(
            "map",
            "odom",
            Transform3::from_translation(1.0, 0.0, 0.0),
            10,
            false,
        );
        tf.insert(
            "odom",
            "base_link",
            Transform3::from_translation(0.0, 2.0, 0.0),
            10,
            false,
        );
        tf.insert(
            "base_link",
            "sensor",
            Transform3::from_translation(0.0, 0.0, 0.5),
            0,
            true,
        );
        tf
    }

    #[test]
    fn a_frame_resolves_to_itself_as_the_identity() {
        let tf = TfBuffer::new();
        assert_eq!(tf.lookup("map", "map"), Ok(Transform3::IDENTITY));
    }

    #[test]
    fn a_direct_parent_child_lookup_is_the_stored_transform() {
        let tf = robot();
        let t = tf.lookup("map", "odom").unwrap();
        close(t.apply(Point3::ORIGIN), [1.0, 0.0, 0.0]);
    }

    #[test]
    fn a_chain_composes_from_the_leaf_up() {
        let tf = robot();
        let t = tf.lookup("map", "sensor").unwrap();
        // 1 along x, 2 along y, 0.5 up, accumulated down the chain.
        close(t.apply(Point3::ORIGIN), [1.0, 2.0, 0.5]);
        close(t.apply(Point3::new(1.0, 0.0, 0.0)), [2.0, 2.0, 0.5]);
    }

    #[test]
    fn rotation_in_the_middle_of_a_chain_is_applied_to_what_hangs_below_it() {
        let mut tf = TfBuffer::new();
        tf.insert("map", "base", Transform3::from_translation(0.0, 0.0, 0.0), 0, true);

        let mut turned = Transform3::from_rpy(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        turned.translation = [0.0, 0.0, 0.0];
        tf.insert("base", "sensor", turned, 0, true);

        // A point 1 m in front of the sensor is 1 m to the left in the map.
        let t = tf.lookup("map", "sensor").unwrap();
        close(t.apply(Point3::new(1.0, 0.0, 0.0)), [0.0, 1.0, 0.0]);
    }

    #[test]
    fn looking_up_the_other_way_gives_the_inverse() {
        let tf = robot();
        let forward = tf.lookup("map", "sensor").unwrap();
        let backward = tf.lookup("sensor", "map").unwrap();

        let p = Point3::new(3.0, -1.0, 2.0);
        close(backward.apply(forward.apply(p)), [p.x, p.y, p.z]);
    }

    #[test]
    fn siblings_resolve_through_their_common_ancestor() {
        let mut tf = TfBuffer::new();
        tf.insert(
            "base",
            "left",
            Transform3::from_translation(0.0, 1.0, 0.0),
            0,
            true,
        );
        tf.insert(
            "base",
            "right",
            Transform3::from_translation(0.0, -1.0, 0.0),
            0,
            true,
        );

        // The right sensor's origin, expressed in the left sensor's frame.
        let t = tf.lookup("left", "right").unwrap();
        close(t.apply(Point3::ORIGIN), [0.0, -2.0, 0.0]);
    }

    #[test]
    fn a_later_transform_replaces_an_earlier_one() {
        let mut tf = robot();
        tf.insert(
            "map",
            "odom",
            Transform3::from_translation(5.0, 0.0, 0.0),
            20,
            false,
        );
        close(
            tf.lookup("map", "odom").unwrap().apply(Point3::ORIGIN),
            [5.0, 0.0, 0.0],
        );
    }

    #[test]
    fn an_out_of_order_arrival_does_not_rewind_the_pose() {
        let mut tf = robot();
        tf.insert(
            "map",
            "odom",
            Transform3::from_translation(99.0, 0.0, 0.0),
            5, // older than the stamp 10 already held
            false,
        );
        close(
            tf.lookup("map", "odom").unwrap().apply(Point3::ORIGIN),
            [1.0, 0.0, 0.0],
        );
    }

    #[test]
    fn a_static_transform_wins_regardless_of_stamp() {
        let mut tf = robot();
        tf.insert(
            "base_link",
            "sensor",
            Transform3::from_translation(0.0, 0.0, 9.0),
            -1,
            true,
        );
        close(
            tf.lookup("base_link", "sensor").unwrap().apply(Point3::ORIGIN),
            [0.0, 0.0, 9.0],
        );
    }

    #[test]
    fn an_unheard_of_frame_is_named_in_the_error() {
        let tf = robot();
        assert_eq!(
            tf.lookup("map", "camera_optical"),
            Err(TfError::UnknownFrame("camera_optical".to_string()))
        );
    }

    #[test]
    fn two_disconnected_trees_report_the_gap_rather_than_a_wrong_answer() {
        let mut tf = robot();
        tf.insert(
            "other_root",
            "floating",
            Transform3::IDENTITY,
            0,
            true,
        );
        assert_eq!(
            tf.lookup("map", "floating"),
            Err(TfError::Disconnected {
                target: "map".to_string(),
                source: "floating".to_string(),
            })
        );
    }

    #[test]
    fn a_cycle_is_refused_instead_of_looping_forever() {
        let mut tf = TfBuffer::new();
        tf.insert("a", "b", Transform3::IDENTITY, 0, true);
        tf.insert("b", "c", Transform3::IDENTITY, 0, true);
        tf.insert("c", "a", Transform3::IDENTITY, 0, true);

        assert!(matches!(tf.lookup("a", "c"), Err(TfError::Cycle(_))));
    }

    #[test]
    fn leading_slashes_are_normalized_away_on_both_sides() {
        let mut tf = TfBuffer::new();
        tf.insert(
            "/map",
            "/sensor",
            Transform3::from_translation(1.0, 0.0, 0.0),
            0,
            true,
        );

        close(
            tf.lookup("map", "sensor").unwrap().apply(Point3::ORIGIN),
            [1.0, 0.0, 0.0],
        );
        close(
            tf.lookup("/map", "sensor").unwrap().apply(Point3::ORIGIN),
            [1.0, 0.0, 0.0],
        );
    }

    #[test]
    fn the_nearest_common_ancestor_is_used_not_the_root() {
        // map -> odom -> base -> {left, right}. Resolving left into right must
        // not travel up to map and back.
        let mut tf = robot();
        tf.insert(
            "base_link",
            "left",
            Transform3::from_translation(0.0, 1.0, 0.0),
            0,
            true,
        );
        tf.insert(
            "base_link",
            "right",
            Transform3::from_translation(0.0, -1.0, 0.0),
            0,
            true,
        );

        let t = tf.lookup("left", "right").unwrap();
        close(t.apply(Point3::ORIGIN), [0.0, -2.0, 0.0]);
    }
}
