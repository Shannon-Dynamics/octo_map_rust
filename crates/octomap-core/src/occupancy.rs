//! The probabilistic occupancy model.
//!
//! Ported from `OcTreeNode`, `AbstractOccupancyOcTree` and
//! `OccupancyOcTreeBase`. A voxel stores confidence as log-odds rather than a
//! boolean, which turns the Bayesian update into an addition.
//!
//! # Precision
//!
//! The reference computes log-odds in `double` and stores them in `float`
//! fields, so the stored thresholds are `f32`-rounded versions of the textbook
//! values — `clampingThresMax` of 0.971 lands on 3.5110307, not 3.5. That
//! rounding is reproduced here; skipping it would put nodes on the wrong side
//! of a clamp.

use std::collections::HashMap;

use crate::error::{OctomapError, Result};
use crate::geometry::TreeGeometry;
use crate::key::{compute_child_index, OcTreeKey};
use crate::node::{Node, CHILD_COUNT};
use crate::point::Point3;
use crate::tree::{LeafIter, OctreeCore, TreeIter};

/// Converts a probability into log-odds.
///
/// Computed in `f64` and narrowed to `f32`, matching `octomap_utils.h`.
#[inline]
pub fn log_odds(probability: f64) -> f32 {
    (probability / (1.0 - probability)).ln() as f32
}

/// Converts log-odds back into a probability.
#[inline]
pub fn probability(log_odds: f64) -> f64 {
    1.0 - (1.0 / (1.0 + log_odds.exp()))
}

/// The value stored in an occupancy node: confidence, as log-odds.
///
/// Defaults to zero — probability 0.5, "unknown" — matching
/// `OcTreeNode::OcTreeNode()`.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct OccupancyValue {
    /// Occupancy confidence in log-odds.
    pub log_odds: f32,
}

impl OccupancyValue {
    /// Wraps a log-odds value.
    #[inline]
    pub const fn new(log_odds: f32) -> Self {
        Self { log_odds }
    }

    /// The occupancy probability this value represents.
    #[inline]
    pub fn probability(&self) -> f64 {
        probability(f64::from(self.log_odds))
    }
}

/// The sensor model: how much a hit or a miss moves a voxel's confidence, and
/// how far that confidence may travel.
///
/// Stored as log-odds, like the reference, so that no conversion happens on the
/// hot path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorModel {
    prob_hit_log: f32,
    prob_miss_log: f32,
    occupancy_thres_log: f32,
    clamping_thres_min: f32,
    clamping_thres_max: f32,
}

impl Default for SensorModel {
    /// The reference's defaults from `AbstractOccupancyOcTree`:
    /// hit 0.7, miss 0.4, threshold 0.5, clamping 0.1192 to 0.971.
    fn default() -> Self {
        Self {
            prob_hit_log: log_odds(0.7),
            prob_miss_log: log_odds(0.4),
            occupancy_thres_log: log_odds(0.5),
            clamping_thres_min: log_odds(0.1192),
            clamping_thres_max: log_odds(0.971),
        }
    }
}

fn check_unit_interval(parameter: &'static str, p: f64) -> Result<()> {
    if !p.is_finite() || p <= 0.0 || p >= 1.0 {
        return Err(OctomapError::InvalidProbability {
            parameter,
            got: p,
            expected: "a probability strictly between 0 and 1",
        });
    }
    Ok(())
}

impl SensorModel {
    /// Log-odds added to a voxel the sensor reports as occupied.
    #[inline]
    pub fn prob_hit_log(&self) -> f32 {
        self.prob_hit_log
    }

    /// Log-odds added to a voxel a ray passed through.
    #[inline]
    pub fn prob_miss_log(&self) -> f32 {
        self.prob_miss_log
    }

    /// Log-odds at or above which a voxel counts as occupied.
    #[inline]
    pub fn occupancy_thres_log(&self) -> f32 {
        self.occupancy_thres_log
    }

    /// Lower clamp on a voxel's confidence.
    #[inline]
    pub fn clamping_thres_min(&self) -> f32 {
        self.clamping_thres_min
    }

    /// Upper clamp on a voxel's confidence.
    #[inline]
    pub fn clamping_thres_max(&self) -> f32 {
        self.clamping_thres_max
    }

    /// Hit probability.
    #[inline]
    pub fn prob_hit(&self) -> f64 {
        probability(f64::from(self.prob_hit_log))
    }

    /// Miss probability.
    #[inline]
    pub fn prob_miss(&self) -> f64 {
        probability(f64::from(self.prob_miss_log))
    }

    /// Occupancy threshold, as a probability.
    #[inline]
    pub fn occupancy_thres(&self) -> f64 {
        probability(f64::from(self.occupancy_thres_log))
    }

    /// Lower clamp, as a probability.
    #[inline]
    pub fn clamping_thres_min_prob(&self) -> f64 {
        probability(f64::from(self.clamping_thres_min))
    }

    /// Upper clamp, as a probability.
    #[inline]
    pub fn clamping_thres_max_prob(&self) -> f64 {
        probability(f64::from(self.clamping_thres_max))
    }

    /// Sets the hit probability.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidProbability`] unless `p` is in `(0, 1)`
    /// and at least 0.5 — a "hit" that lowered confidence would invert the
    /// sensor model. The reference asserts on the same condition.
    pub fn set_prob_hit(&mut self, p: f64) -> Result<()> {
        check_unit_interval("prob_hit", p)?;
        let l = log_odds(p);
        if l < 0.0 {
            return Err(OctomapError::InvalidProbability {
                parameter: "prob_hit",
                got: p,
                expected: "at least 0.5, so that a hit raises occupancy",
            });
        }
        self.prob_hit_log = l;
        Ok(())
    }

    /// Sets the miss probability.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidProbability`] unless `p` is in `(0, 1)`
    /// and at most 0.5.
    pub fn set_prob_miss(&mut self, p: f64) -> Result<()> {
        check_unit_interval("prob_miss", p)?;
        let l = log_odds(p);
        if l > 0.0 {
            return Err(OctomapError::InvalidProbability {
                parameter: "prob_miss",
                got: p,
                expected: "at most 0.5, so that a miss lowers occupancy",
            });
        }
        self.prob_miss_log = l;
        Ok(())
    }

    /// Sets the occupancy threshold.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidProbability`] unless `p` is in `(0, 1)`.
    pub fn set_occupancy_thres(&mut self, p: f64) -> Result<()> {
        check_unit_interval("occupancy_thres", p)?;
        self.occupancy_thres_log = log_odds(p);
        Ok(())
    }

    /// Sets the lower clamp.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidProbability`] unless `p` is in `(0, 1)`.
    pub fn set_clamping_thres_min(&mut self, p: f64) -> Result<()> {
        check_unit_interval("clamping_thres_min", p)?;
        self.clamping_thres_min = log_odds(p);
        Ok(())
    }

    /// Sets the upper clamp.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidProbability`] unless `p` is in `(0, 1)`.
    pub fn set_clamping_thres_max(&mut self, p: f64) -> Result<()> {
        check_unit_interval("clamping_thres_max", p)?;
        self.clamping_thres_max = log_odds(p);
        Ok(())
    }

    /// True when `value` is at or above the occupancy threshold.
    ///
    /// The comparison is inclusive, so a voxel sitting exactly on the threshold
    /// counts as occupied — same as `isNodeOccupied`.
    #[inline]
    pub fn is_occupied(&self, value: OccupancyValue) -> bool {
        value.log_odds >= self.occupancy_thres_log
    }

    /// True when `value` has reached either clamp and can no longer move.
    #[inline]
    pub fn is_at_clamping_threshold(&self, value: OccupancyValue) -> bool {
        value.log_odds >= self.clamping_thres_max || value.log_odds <= self.clamping_thres_min
    }

    /// Clamps `log_odds` into the model's range.
    #[inline]
    pub fn clamp(&self, log_odds: f32) -> f32 {
        log_odds
            .max(self.clamping_thres_min)
            .min(self.clamping_thres_max)
    }
}

/// What a descent does once it reaches the target leaf.
#[derive(Debug, Clone, Copy)]
enum LeafOp {
    /// Add to the existing confidence, then clamp.
    Add(f32),
    /// Overwrite the confidence with an already-clamped value.
    Set(f32),
}

/// Bookkeeping and settings threaded through the recursive update.
struct UpdateCtx<'a> {
    tree_depth: u32,
    op: LeafOp,
    lazy_eval: bool,
    sensor: SensorModel,
    created: usize,
    removed: usize,
    change_detection: bool,
    changed_keys: &'a mut HashMap<OcTreeKey, bool>,
}

/// A probabilistic 3D occupancy map.
///
/// Composes [`OctreeCore`] rather than inheriting from it, which is where this
/// port diverges structurally from the C++ class hierarchy. Observable behavior
/// is the same.
#[derive(Debug, Clone, PartialEq)]
pub struct OcTree {
    core: OctreeCore<OccupancyValue>,
    sensor: SensorModel,
    use_change_detection: bool,
    changed_keys: HashMap<OcTreeKey, bool>,
}

impl OcTree {
    /// Creates an empty map with the given resolution, in meters.
    ///
    /// # Errors
    ///
    /// Returns [`OctomapError::InvalidResolution`] unless `resolution` is
    /// finite and strictly positive.
    pub fn new(resolution: f64) -> Result<Self> {
        Ok(Self {
            core: OctreeCore::new(resolution)?,
            sensor: SensorModel::default(),
            use_change_detection: false,
            changed_keys: HashMap::new(),
        })
    }

    /// The sensor model in use.
    #[inline]
    pub fn sensor(&self) -> &SensorModel {
        &self.sensor
    }

    /// Mutable access to the sensor model.
    #[inline]
    pub fn sensor_mut(&mut self) -> &mut SensorModel {
        &mut self.sensor
    }

    /// The tree's geometry.
    #[inline]
    pub fn geometry(&self) -> &TreeGeometry {
        self.core.geometry()
    }

    /// Edge length of the smallest voxel, in meters.
    #[inline]
    pub fn resolution(&self) -> f64 {
        self.core.geometry().resolution()
    }

    /// The underlying generic tree.
    #[inline]
    pub fn core(&self) -> &OctreeCore<OccupancyValue> {
        &self.core
    }

    /// Mutable access to the underlying tree, for deserialization.
    #[inline]
    pub(crate) fn core_mut(&mut self) -> &mut OctreeCore<OccupancyValue> {
        &mut self.core
    }

    /// Number of nodes, inner nodes included.
    #[inline]
    pub fn len(&self) -> usize {
        self.core.len()
    }

    /// True when the map holds no nodes.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.core.is_empty()
    }

    /// Recomputes the node count by walking the tree.
    #[inline]
    pub fn count_nodes(&self) -> usize {
        self.core.count_nodes()
    }

    /// Number of leaves.
    #[inline]
    pub fn count_leaf_nodes(&self) -> usize {
        self.core.count_leaf_nodes()
    }

    /// Removes every node.
    pub fn clear(&mut self) {
        self.core.clear();
        self.changed_keys.clear();
    }

    /// Finds the node addressed by `key`.
    #[inline]
    pub fn search(&self, key: OcTreeKey) -> Option<&Node<OccupancyValue>> {
        self.core.search(key)
    }

    /// Finds the node containing `point`.
    #[inline]
    pub fn search_point(&self, point: Point3) -> Option<&Node<OccupancyValue>> {
        self.core.search_point(point)
    }

    /// The confidence stored at `key`, if the map covers it.
    #[inline]
    pub fn get_log_odds(&self, key: OcTreeKey) -> Option<f32> {
        self.core.get(key).map(|v| v.log_odds)
    }

    /// The occupancy probability at `key`, if the map covers it.
    #[inline]
    pub fn get_occupancy(&self, key: OcTreeKey) -> Option<f64> {
        self.core.get(key).map(OccupancyValue::probability)
    }

    /// Whether `key` is occupied.
    ///
    /// `None` means unknown — the map has never observed that voxel. Callers
    /// that treat unknown as free should say so explicitly rather than
    /// `unwrap_or(false)` by reflex.
    #[inline]
    pub fn is_occupied(&self, key: OcTreeKey) -> Option<bool> {
        self.core.get(key).map(|v| self.sensor.is_occupied(*v))
    }

    // The `_at` variants below take world coordinates instead of a key. They
    // are the entry points an application actually calls — a sensor produces
    // meters, not voxel indices. Each is a wrapper over the key-based traversal
    // above, so the pruned-node semantics are the same ones documented in
    // `docs/reference-audit.md`: a key falling inside a merged block resolves to
    // that block's leaf rather than reporting a miss.

    /// The confidence at `point`, if the map covers it.
    ///
    /// `None` covers two distinct situations that callers rarely need to
    /// separate: the point lies outside the addressable volume, or the map has
    /// never observed it.
    #[inline]
    pub fn get_log_odds_at(&self, point: Point3) -> Option<f32> {
        self.search_point(point).map(|n| n.value().log_odds)
    }

    /// The occupancy probability at `point`, if the map covers it.
    #[inline]
    pub fn get_occupancy_at(&self, point: Point3) -> Option<f64> {
        self.search_point(point).map(|n| n.value().probability())
    }

    /// Whether the voxel containing `point` is occupied.
    ///
    /// Three-state on purpose:
    ///
    /// - `Some(true)` — observed, and at or above the occupancy threshold.
    /// - `Some(false)` — observed, and below it.
    /// - `None` — **unknown**: never observed, or outside the map.
    ///
    /// Unknown is not free. A planner that collapses the two with
    /// `unwrap_or(false)` will fly through unobserved space; if that is the
    /// intent, write it out so the next reader can see it was a decision.
    #[inline]
    pub fn is_occupied_at(&self, point: Point3) -> Option<bool> {
        self.search_point(point)
            .map(|n| self.sensor.is_occupied(*n.value()))
    }

    /// Iterates every node, inner nodes included.
    #[inline]
    pub fn iter_nodes(&self) -> TreeIter<'_, OccupancyValue> {
        self.core.iter_nodes()
    }

    /// Iterates the leaves.
    #[inline]
    pub fn iter_leaves(&self) -> LeafIter<'_, OccupancyValue> {
        self.core.iter_leaves()
    }

    /// Iterates the map as if pruned to `max_depth`.
    #[inline]
    pub fn iter_leaves_to_depth(&self, max_depth: u32) -> LeafIter<'_, OccupancyValue> {
        self.core.iter_leaves_to_depth(max_depth)
    }

    /// Merges every group of eight equal childless children into its parent.
    ///
    /// Returns the number of nodes removed.
    #[inline]
    pub fn prune(&mut self) -> usize {
        self.core.prune()
    }

    /// Splits every leaf into eight children, down to `max_depth`.
    #[inline]
    pub fn expand_to_depth(&mut self, max_depth: u32) -> usize {
        self.core.expand_to_depth(max_depth)
    }

    /// Removes the voxel at `key`.
    #[inline]
    pub fn delete(&mut self, key: OcTreeKey) -> bool {
        self.core.delete(key)
    }

    // ---- updates -----------------------------------------------------------

    /// Applies a hit or a miss to the voxel at `key`.
    ///
    /// Returns the voxel's confidence afterwards.
    #[inline]
    pub fn update_node(&mut self, key: OcTreeKey, occupied: bool) -> f32 {
        let delta = if occupied {
            self.sensor.prob_hit_log
        } else {
            self.sensor.prob_miss_log
        };
        self.update_node_log_odds(key, delta, false)
    }

    /// Applies a hit or a miss to the voxel containing `point`.
    ///
    /// Returns `None` when the point lies outside the addressable volume.
    pub fn update_node_at(&mut self, point: Point3, occupied: bool) -> Option<f32> {
        let key = self.core.geometry().coord_to_key_checked(point)?;
        Some(self.update_node(key, occupied))
    }

    /// Adds `log_odds_update` to the voxel at `key`, then clamps.
    ///
    /// With `lazy_eval` set, inner nodes are left stale and no pruning happens
    /// during the descent — call [`OcTree::update_inner_occupancy`] afterwards.
    /// This is the reference's batching mode and is what makes bulk insertion
    /// fast.
    ///
    /// Returns the voxel's confidence afterwards.
    pub fn update_node_log_odds(
        &mut self,
        key: OcTreeKey,
        log_odds_update: f32,
        lazy_eval: bool,
    ) -> f32 {
        // Early abort, straight from the reference: a voxel already pinned at
        // the clamp it is being pushed toward cannot move, so the whole descent
        // is skipped. This is observable — it also skips node creation.
        if let Some(node) = self.core.search(key) {
            let l = node.value().log_odds;
            if (log_odds_update >= 0.0 && l >= self.sensor.clamping_thres_max)
                || (log_odds_update <= 0.0 && l <= self.sensor.clamping_thres_min)
            {
                return l;
            }
        }
        self.descend(key, LeafOp::Add(log_odds_update), lazy_eval)
    }

    /// Overwrites the confidence at `key`, clamping the value first.
    ///
    /// Returns the voxel's confidence afterwards.
    pub fn set_node_value(&mut self, key: OcTreeKey, log_odds_value: f32, lazy_eval: bool) -> f32 {
        let clamped = self.sensor.clamp(log_odds_value);
        self.descend(key, LeafOp::Set(clamped), lazy_eval)
    }

    fn descend(&mut self, key: OcTreeKey, op: LeafOp, lazy_eval: bool) -> f32 {
        let Self {
            core,
            sensor,
            use_change_detection,
            changed_keys,
        } = self;

        let created_root = core.ensure_root(OccupancyValue::default());
        let (geometry, root, tree_size) = core.parts_mut();
        let tree_depth = geometry.tree_depth();
        let root_node = root.as_mut().expect("root was just ensured");

        let mut ctx = UpdateCtx {
            tree_depth,
            op,
            lazy_eval,
            sensor: *sensor,
            created: 0,
            removed: 0,
            change_detection: *use_change_detection,
            changed_keys,
        };

        let result = update_recurs(root_node, created_root, &key, 0, &mut ctx);

        *tree_size = tree_size
            .saturating_add(ctx.created)
            .saturating_sub(ctx.removed);
        result
    }

    /// Recomputes every inner node from its children, bottom up.
    ///
    /// Needed after any `lazy_eval` insertion. Each inner node takes the
    /// **maximum** of its children's confidence, which is the conservative
    /// choice the reference makes.
    pub fn update_inner_occupancy(&mut self) {
        let (geometry, root, _) = self.core.parts_mut();
        let tree_depth = geometry.tree_depth();
        if let Some(root_node) = root.as_mut() {
            update_inner_recurs(root_node, 0, tree_depth);
        }
    }

    /// Collapses every voxel onto whichever clamp its occupancy implies.
    ///
    /// Turns a probabilistic map into a binary one, which is what `.bt`
    /// serialization stores.
    pub fn to_max_likelihood(&mut self) {
        let sensor = self.sensor;
        let (geometry, root, _) = self.core.parts_mut();
        let tree_depth = geometry.tree_depth();
        let Some(root_node) = root.as_mut() else {
            return;
        };

        // Bottom up, one sweep per level, exactly like the reference.
        for max_depth in (1..=tree_depth).rev() {
            to_max_likelihood_recurs(root_node, 0, max_depth, &sensor);
        }
        node_to_max_likelihood(root_node, &sensor);
    }

    // ---- change detection --------------------------------------------------

    /// Turns change tracking on or off.
    ///
    /// Switching it off leaves any already-recorded keys in place, matching
    /// `enableChangeDetection`.
    #[inline]
    pub fn enable_change_detection(&mut self, enabled: bool) {
        self.use_change_detection = enabled;
    }

    /// Whether change tracking is on.
    #[inline]
    pub fn change_detection_enabled(&self) -> bool {
        self.use_change_detection
    }

    /// Keys whose occupancy state changed since the last reset.
    ///
    /// The flag is `true` for voxels that did not exist before.
    #[inline]
    pub fn changed_keys(&self) -> &HashMap<OcTreeKey, bool> {
        &self.changed_keys
    }

    /// Number of tracked changes.
    #[inline]
    pub fn changed_key_count(&self) -> usize {
        self.changed_keys.len()
    }

    /// Forgets every tracked change.
    #[inline]
    pub fn reset_change_detection(&mut self) {
        self.changed_keys.clear();
    }
}

/// Highest confidence among a node's children, or `-f32::MAX` when it has none.
///
/// The sentinel is the reference's `-std::numeric_limits<float>::max()`, not
/// negative infinity.
pub fn max_child_log_odds(node: &Node<OccupancyValue>) -> f32 {
    let mut max = -f32::MAX;
    for (_, child) in node.children_iter() {
        if child.value().log_odds > max {
            max = child.value().log_odds;
        }
    }
    max
}

/// Log-odds of the mean of the children's *probabilities*.
///
/// Note the reference averages probabilities and converts back, rather than
/// averaging log-odds — the two are not the same. Returns `NaN` for a childless
/// node, since the reference computes `log(0/1)` in that case.
pub fn mean_child_log_odds(node: &Node<OccupancyValue>) -> f64 {
    let mut sum = 0.0f64;
    let mut count = 0u32;
    for (_, child) in node.children_iter() {
        sum += child.value().probability();
        count += 1;
    }
    let mean = if count > 0 {
        sum / f64::from(count)
    } else {
        0.0
    };
    (mean / (1.0 - mean)).ln()
}

fn update_recurs(
    node: &mut Node<OccupancyValue>,
    node_just_created: bool,
    key: &OcTreeKey,
    depth: u32,
    ctx: &mut UpdateCtx<'_>,
) -> f32 {
    if depth >= ctx.tree_depth {
        return apply_leaf_op(node, node_just_created, key, ctx);
    }

    let pos = compute_child_index(key, ctx.tree_depth - 1 - depth);
    let mut created_child = false;

    if !node.child_exists(pos) {
        if !node.has_children() && !node_just_created {
            // A childless node that was already there is a pruned block; open
            // it up rather than grafting a lone child onto it.
            ctx.created += node.expand();
        } else if node.create_child(pos, OccupancyValue::default()) {
            ctx.created += 1;
            created_child = true;
        }
    }

    let child = node
        .child_mut(pos)
        .expect("child exists or was just created");
    let result = update_recurs(child, created_child, key, depth + 1, ctx);

    if !ctx.lazy_eval {
        // Try to collapse on the way back up; only if that fails does the inner
        // node need refreshing from its children.
        let pruned = node.prune();
        if pruned > 0 {
            ctx.removed += pruned;
        } else {
            node.set_value(OccupancyValue::new(max_child_log_odds(node)));
        }
    }

    result
}

fn apply_leaf_op(
    node: &mut Node<OccupancyValue>,
    node_just_created: bool,
    key: &OcTreeKey,
    ctx: &mut UpdateCtx<'_>,
) -> f32 {
    let occupied_before = ctx.sensor.is_occupied(*node.value());

    let new_value = match ctx.op {
        LeafOp::Set(v) => v,
        LeafOp::Add(delta) => {
            // The reference adds first and clamps after, so an update can
            // overshoot momentarily; the clamp is what makes it observable.
            ctx.sensor.clamp(node.value().log_odds + delta)
        }
    };
    node.set_value(OccupancyValue::new(new_value));

    if ctx.change_detection {
        let occupied_after = ctx.sensor.is_occupied(*node.value());
        if node_just_created {
            ctx.changed_keys.insert(*key, true);
        } else if occupied_before != occupied_after {
            // A voxel that flips back to where it started stops being a change.
            match ctx.changed_keys.get(key) {
                None => {
                    ctx.changed_keys.insert(*key, false);
                }
                Some(false) => {
                    ctx.changed_keys.remove(key);
                }
                Some(true) => {}
            }
        }
    }

    new_value
}

fn update_inner_recurs(node: &mut Node<OccupancyValue>, depth: u32, tree_depth: u32) {
    if !node.has_children() {
        return;
    }
    if depth < tree_depth {
        for i in 0..CHILD_COUNT as u8 {
            if let Some(child) = node.child_mut(i) {
                update_inner_recurs(child, depth + 1, tree_depth);
            }
        }
    }
    node.set_value(OccupancyValue::new(max_child_log_odds(node)));
}

fn node_to_max_likelihood(node: &mut Node<OccupancyValue>, sensor: &SensorModel) {
    let value = if sensor.is_occupied(*node.value()) {
        sensor.clamping_thres_max()
    } else {
        sensor.clamping_thres_min()
    };
    node.set_value(OccupancyValue::new(value));
}

fn to_max_likelihood_recurs(
    node: &mut Node<OccupancyValue>,
    depth: u32,
    max_depth: u32,
    sensor: &SensorModel,
) {
    if depth < max_depth {
        for i in 0..CHILD_COUNT as u8 {
            if let Some(child) = node.child_mut(i) {
                to_max_likelihood_recurs(child, depth + 1, max_depth, sensor);
            }
        }
    } else {
        node_to_max_likelihood(node, sensor);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(x: u16, y: u16, z: u16) -> OcTreeKey {
        OcTreeKey::new(x, y, z)
    }

    const CENTER: u16 = 32768;

    #[test]
    fn probability_of_one_half_is_zero_log_odds() {
        assert_eq!(log_odds(0.5), 0.0);
        assert_eq!(probability(0.0), 0.5);
    }

    #[test]
    fn log_odds_and_probability_round_trip() {
        for p in [0.1, 0.25, 0.4, 0.5, 0.7, 0.9, 0.971] {
            let back = probability(f64::from(log_odds(p)));
            assert!((back - p).abs() < 1e-6, "{p} round-tripped to {back}");
        }
    }

    #[test]
    fn log_odds_sign_follows_the_half_way_point() {
        assert!(log_odds(0.3) < 0.0);
        assert!(log_odds(0.7) > 0.0);
    }

    #[test]
    fn default_sensor_model_matches_the_reference() {
        let s = SensorModel::default();
        // Exact f32-rounded values captured from the C++ binary.
        assert_eq!(s.prob_hit_log(), 0.847_297_85);
        assert_eq!(s.prob_miss_log(), -0.405_465_1);
        assert_eq!(s.occupancy_thres_log(), 0.0);
        assert_eq!(s.clamping_thres_min(), -2.000_028);
        assert_eq!(s.clamping_thres_max(), 3.511_030_7);
    }

    #[test]
    fn occupancy_test_is_inclusive_at_the_threshold() {
        let s = SensorModel::default();
        assert!(
            s.is_occupied(OccupancyValue::new(0.0)),
            "0.5 counts as occupied"
        );
        assert!(s.is_occupied(OccupancyValue::new(0.1)));
        assert!(!s.is_occupied(OccupancyValue::new(-0.001)));
    }

    #[test]
    fn sensor_setters_reject_an_inverted_model() {
        let mut s = SensorModel::default();
        assert!(s.set_prob_hit(0.9).is_ok());
        assert!(
            s.set_prob_hit(0.3).is_err(),
            "a hit must not lower occupancy"
        );
        assert!(s.set_prob_miss(0.1).is_ok());
        assert!(s.set_prob_miss(0.8).is_err(), "a miss must not raise it");
    }

    #[test]
    fn sensor_setters_reject_values_outside_the_unit_interval() {
        let mut s = SensorModel::default();
        for bad in [0.0, 1.0, -0.5, 1.5, f64::NAN] {
            assert!(s.set_occupancy_thres(bad).is_err(), "accepted {bad}");
        }
    }

    #[test]
    fn a_single_hit_raises_confidence_by_prob_hit_log() {
        let mut t = OcTree::new(0.1).unwrap();
        let l = t.update_node(key(CENTER, CENTER, CENTER), true);
        assert_eq!(l, t.sensor().prob_hit_log());
        assert_eq!(t.is_occupied(key(CENTER, CENTER, CENTER)), Some(true));
    }

    #[test]
    fn a_single_miss_lowers_confidence_by_prob_miss_log() {
        let mut t = OcTree::new(0.1).unwrap();
        let l = t.update_node(key(CENTER, CENTER, CENTER), false);
        assert_eq!(l, t.sensor().prob_miss_log());
        assert_eq!(t.is_occupied(key(CENTER, CENTER, CENTER)), Some(false));
    }

    #[test]
    fn repeated_hits_saturate_at_the_upper_clamp() {
        let mut t = OcTree::new(0.1).unwrap();
        let k = key(CENTER, CENTER, CENTER);
        let mut last = 0.0;
        for _ in 0..50 {
            last = t.update_node(k, true);
        }
        assert_eq!(last, t.sensor().clamping_thres_max());
    }

    #[test]
    fn repeated_misses_saturate_at_the_lower_clamp() {
        let mut t = OcTree::new(0.1).unwrap();
        let k = key(CENTER, CENTER, CENTER);
        let mut last = 0.0;
        for _ in 0..50 {
            last = t.update_node(k, false);
        }
        assert_eq!(last, t.sensor().clamping_thres_min());
    }

    #[test]
    fn hits_and_misses_cancel_out() {
        let mut t = OcTree::new(0.1).unwrap();
        let k = key(CENTER, CENTER, CENTER);
        t.update_node(k, true);
        let after_hit = t.get_log_odds(k).unwrap();
        t.update_node(k, false);
        let after_miss = t.get_log_odds(k).unwrap();

        assert!(after_miss < after_hit);
        let expected = t.sensor().prob_hit_log() + t.sensor().prob_miss_log();
        assert!((after_miss - expected).abs() < 1e-6);
    }

    #[test]
    fn a_saturated_voxel_is_not_touched_again() {
        let mut t = OcTree::new(0.1).unwrap();
        let k = key(CENTER, CENTER, CENTER);
        for _ in 0..50 {
            t.update_node(k, true);
        }
        let nodes_before = t.len();

        // The early abort should make this a no-op, node count included.
        let l = t.update_node(k, true);
        assert_eq!(l, t.sensor().clamping_thres_max());
        assert_eq!(t.len(), nodes_before);
    }

    #[test]
    fn the_early_abort_does_not_block_the_opposite_direction() {
        let mut t = OcTree::new(0.1).unwrap();
        let k = key(CENTER, CENTER, CENTER);
        for _ in 0..50 {
            t.update_node(k, true);
        }
        let saturated = t.get_log_odds(k).unwrap();

        // A miss pushes away from the clamp, so it must still apply.
        let after = t.update_node(k, false);
        assert!(after < saturated);
    }

    #[test]
    fn unknown_voxels_report_none_rather_than_free() {
        let t = OcTree::new(0.1).unwrap();
        assert_eq!(t.is_occupied(key(1, 2, 3)), None);
        assert_eq!(t.get_occupancy(key(1, 2, 3)), None);
    }

    #[test]
    fn set_node_value_clamps_its_input() {
        let mut t = OcTree::new(0.1).unwrap();
        let k = key(CENTER, CENTER, CENTER);
        let l = t.set_node_value(k, 1000.0, false);
        assert_eq!(l, t.sensor().clamping_thres_max());

        let l = t.set_node_value(k, -1000.0, false);
        assert_eq!(l, t.sensor().clamping_thres_min());
    }

    #[test]
    fn inner_nodes_take_the_maximum_of_their_children() {
        let mut t = OcTree::new(0.1).unwrap();
        // Two siblings with different confidence.
        t.update_node(key(CENTER, CENTER, CENTER), false);
        t.update_node(key(CENTER + 1, CENTER, CENTER), true);

        // Walk up from the leaves: every ancestor should carry the hit value.
        let hit = t.sensor().prob_hit_log();
        let root = t.core().root().unwrap();
        assert_eq!(root.value().log_odds, hit, "root must carry the maximum");
    }

    #[test]
    fn lazy_eval_leaves_inner_nodes_stale_until_asked() {
        let mut t = OcTree::new(0.1).unwrap();
        let k = key(CENTER, CENTER, CENTER);
        t.update_node_log_odds(k, t.sensor().prob_hit_log(), true);

        assert_eq!(
            t.core().root().unwrap().value().log_odds,
            0.0,
            "lazy insertion must not refresh ancestors"
        );

        t.update_inner_occupancy();
        assert_eq!(
            t.core().root().unwrap().value().log_odds,
            t.sensor().prob_hit_log()
        );
    }

    #[test]
    fn eager_insertion_prunes_a_uniform_block_on_the_way_up() {
        let mut t = OcTree::new(0.1).unwrap();
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    t.update_node(key(CENTER + dx, CENTER + dy, CENTER + dz), true);
                }
            }
        }
        // All eight agree, so the block should already be collapsed.
        assert_eq!(t.count_leaf_nodes(), 1);
        assert_eq!(t.count_nodes(), t.len(), "node count drifted");
    }

    #[test]
    fn lazy_insertion_skips_the_prune() {
        let mut t = OcTree::new(0.1).unwrap();
        let hit = t.sensor().prob_hit_log();
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    t.update_node_log_odds(key(CENTER + dx, CENTER + dy, CENTER + dz), hit, true);
                }
            }
        }
        assert_eq!(t.count_leaf_nodes(), 8, "lazy mode must not prune");

        t.update_inner_occupancy();
        assert_eq!(t.prune(), 8);
        assert_eq!(t.count_leaf_nodes(), 1);
    }

    #[test]
    fn to_max_likelihood_collapses_onto_the_clamps() {
        let mut t = OcTree::new(0.1).unwrap();
        t.update_node(key(CENTER, CENTER, CENTER), true);
        t.update_node(key(CENTER + 1, CENTER, CENTER), false);
        t.to_max_likelihood();

        assert_eq!(
            t.get_log_odds(key(CENTER, CENTER, CENTER)),
            Some(t.sensor().clamping_thres_max())
        );
        assert_eq!(
            t.get_log_odds(key(CENTER + 1, CENTER, CENTER)),
            Some(t.sensor().clamping_thres_min())
        );
    }

    #[test]
    fn change_detection_is_off_by_default() {
        let mut t = OcTree::new(0.1).unwrap();
        t.update_node(key(CENTER, CENTER, CENTER), true);
        assert_eq!(t.changed_key_count(), 0);
    }

    #[test]
    fn change_detection_records_new_voxels() {
        let mut t = OcTree::new(0.1).unwrap();
        t.enable_change_detection(true);
        let k = key(CENTER, CENTER, CENTER);
        t.update_node(k, true);

        assert_eq!(t.changed_key_count(), 1);
        assert_eq!(t.changed_keys().get(&k), Some(&true), "flagged as new");
    }

    #[test]
    fn change_detection_forgets_a_voxel_that_flips_back() {
        let mut t = OcTree::new(0.1).unwrap();
        let k = key(CENTER, CENTER, CENTER);

        // Establish the voxel first, then start watching.
        t.update_node(k, false);
        t.enable_change_detection(true);
        t.reset_change_detection();

        // free -> occupied is a change...
        t.update_node(k, true);
        assert_eq!(t.changed_keys().get(&k), Some(&false));

        // ...and going back to free cancels it.
        t.update_node(k, false);
        t.update_node(k, false);
        assert_eq!(t.changed_keys().get(&k), None, "the flip should cancel out");
    }

    #[test]
    fn reset_change_detection_clears_the_record() {
        let mut t = OcTree::new(0.1).unwrap();
        t.enable_change_detection(true);
        t.update_node(key(CENTER, CENTER, CENTER), true);
        t.reset_change_detection();
        assert_eq!(t.changed_key_count(), 0);
    }

    #[test]
    fn world_coordinate_queries_separate_occupied_free_and_unknown() {
        let mut t = OcTree::new(0.1).unwrap();
        let occupied = Point3::new(1.05, 0.05, 0.05);
        let free = Point3::new(0.55, 0.05, 0.05);
        let never_seen = Point3::new(-7.35, 2.45, 3.15);

        t.update_node_at(occupied, true);
        t.update_node_at(free, false);

        assert_eq!(t.is_occupied_at(occupied), Some(true));
        assert_eq!(t.is_occupied_at(free), Some(false));
        assert_eq!(
            t.is_occupied_at(never_seen),
            None,
            "an unobserved point must read as unknown, not as free"
        );

        assert_eq!(t.get_log_odds_at(occupied), Some(t.sensor().prob_hit_log()));
        assert_eq!(t.get_log_odds_at(free), Some(t.sensor().prob_miss_log()));
        assert_eq!(t.get_log_odds_at(never_seen), None);

        assert!(t.get_occupancy_at(occupied).unwrap() > 0.5);
        assert!(t.get_occupancy_at(free).unwrap() < 0.5);
        assert_eq!(t.get_occupancy_at(never_seen), None);
    }

    #[test]
    fn world_coordinate_queries_agree_with_the_key_based_ones() {
        let mut t = OcTree::new(0.1).unwrap();
        let p = Point3::new(1.25, -2.5, 0.35);
        t.update_node_at(p, true);

        let key = t.geometry().coord_to_key_checked(p).unwrap();
        assert_eq!(t.is_occupied_at(p), t.is_occupied(key));
        assert_eq!(t.get_log_odds_at(p), t.get_log_odds(key));
        assert_eq!(t.get_occupancy_at(p), t.get_occupancy(key));
    }

    #[test]
    fn world_coordinate_queries_see_through_a_pruned_block() {
        // The whole point of routing through the existing search: a key inside
        // a merged block must still resolve, and it must do so by coordinate.
        let mut t = OcTree::new(0.1).unwrap();
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    t.update_node(key(CENTER + dx, CENTER + dy, CENTER + dz), true);
                }
            }
        }
        assert_eq!(t.count_leaf_nodes(), 1, "block should be pruned");

        // Each of the eight original voxel centers still answers.
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    let center =
                        t.geometry()
                            .key_to_coord(key(CENTER + dx, CENTER + dy, CENTER + dz));
                    assert_eq!(
                        t.is_occupied_at(center),
                        Some(true),
                        "pruned block stopped covering {center:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn world_coordinate_queries_reject_points_outside_the_volume() {
        let mut t = OcTree::new(0.1).unwrap();
        t.update_node_at(Point3::new(1.0, 1.0, 1.0), true);

        // 0.1 m resolution addresses +/-3276.8 m.
        for far in [
            Point3::new(1.0e9, 0.0, 0.0),
            Point3::new(0.0, -1.0e9, 0.0),
            Point3::new(f32::NAN, 0.0, 0.0),
        ] {
            assert_eq!(
                t.is_occupied_at(far),
                None,
                "{far:?} should be unaddressable"
            );
            assert_eq!(t.get_log_odds_at(far), None);
        }
    }

    #[test]
    fn updating_a_point_outside_the_volume_reports_none() {
        let mut t = OcTree::new(0.1).unwrap();
        assert!(t.update_node_at(Point3::new(1.0, 2.0, 3.0), true).is_some());
        assert!(t
            .update_node_at(Point3::new(1.0e9, 0.0, 0.0), true)
            .is_none());
    }

    #[test]
    fn max_child_log_odds_of_a_childless_node_is_the_reference_sentinel() {
        let node = Node::new(OccupancyValue::new(1.0));
        assert_eq!(max_child_log_odds(&node), -f32::MAX);
    }

    #[test]
    fn updating_inside_a_pruned_block_reopens_it() {
        let mut t = OcTree::new(0.1).unwrap();
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    t.update_node(key(CENTER + dx, CENTER + dy, CENTER + dz), true);
                }
            }
        }
        assert_eq!(t.count_leaf_nodes(), 1, "starts pruned");

        // One miss in the middle of the block must split it apart again.
        t.update_node(key(CENTER, CENTER, CENTER), false);
        assert_eq!(t.count_leaf_nodes(), 8);
        assert_eq!(t.count_nodes(), t.len(), "node count drifted");

        let hit = t.sensor().prob_hit_log();
        assert!(t.get_log_odds(key(CENTER, CENTER, CENTER)).unwrap() < hit);
        assert_eq!(t.get_log_odds(key(CENTER + 1, CENTER, CENTER)), Some(hit));
    }
}
