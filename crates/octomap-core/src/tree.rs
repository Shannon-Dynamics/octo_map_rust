//! The generic octree.
//!
//! Ported from `OcTreeBaseImpl`, minus the occupancy model — this layer knows
//! how to store, find, delete, prune and expand nodes of any value type, and
//! nothing about probabilities. The occupancy tree composes this rather than
//! inheriting from it.
//!
//! # Descent convention
//!
//! The reference selects a child at depth `d` using bit `tree_depth - 1 - d` of
//! the key, so the root (depth 0) branches on the most significant bit. Every
//! traversal below follows that same convention; changing it would silently
//! reorder the whole tree relative to C++.

use crate::error::Result;
use crate::geometry::TreeGeometry;
use crate::key::{compute_child_index, compute_child_key, OcTreeKey};
use crate::node::{Node, CHILD_COUNT};
use crate::point::Point3;

/// An octree storing a value of type `T` per node.
#[derive(Debug, Clone, PartialEq)]
pub struct OctreeCore<T> {
    geometry: TreeGeometry,
    root: Option<Node<T>>,
    tree_size: usize,
}

impl<T> OctreeCore<T> {
    /// Creates an empty tree with the given resolution, in meters.
    ///
    /// # Errors
    ///
    /// Returns [`crate::OctomapError::InvalidResolution`] unless `resolution`
    /// is finite and strictly positive.
    pub fn new(resolution: f64) -> Result<Self> {
        Ok(Self::with_geometry(TreeGeometry::new(resolution)?))
    }

    /// Creates an empty tree from a prepared geometry.
    pub fn with_geometry(geometry: TreeGeometry) -> Self {
        Self {
            geometry,
            root: None,
            tree_size: 0,
        }
    }

    /// The tree's resolution and depth, and the conversions they induce.
    #[inline]
    pub fn geometry(&self) -> &TreeGeometry {
        &self.geometry
    }

    /// The root node, or `None` for an empty tree.
    #[inline]
    pub fn root(&self) -> Option<&Node<T>> {
        self.root.as_ref()
    }

    /// Mutable access to the root node.
    #[inline]
    pub fn root_mut(&mut self) -> Option<&mut Node<T>> {
        self.root.as_mut()
    }

    /// Number of nodes in the tree, inner nodes included.
    ///
    /// Maintained incrementally, like the reference's `tree_size`. Use
    /// [`OctreeCore::count_nodes`] to recompute it from the structure.
    #[inline]
    pub fn len(&self) -> usize {
        self.tree_size
    }

    /// True when the tree holds no nodes at all.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.root.is_none()
    }

    /// Recomputes the node count by walking the tree.
    ///
    /// Should always equal [`OctreeCore::len`]; the two are compared in tests
    /// to catch bookkeeping drift.
    pub fn count_nodes(&self) -> usize {
        self.root.as_ref().map_or(0, Node::subtree_size)
    }

    /// Number of leaves in the tree.
    pub fn count_leaf_nodes(&self) -> usize {
        self.root.as_ref().map_or(0, Node::leaf_count)
    }

    /// Removes every node.
    pub fn clear(&mut self) {
        self.root = None;
        self.tree_size = 0;
    }

    /// Disjoint mutable access to the geometry, the root, and the node count.
    ///
    /// Layers built on top of this one — the occupancy tree in particular —
    /// need their own recursion because they interleave pruning and inner-node
    /// updates into the descent. They still have to keep `tree_size` honest,
    /// which needs a borrow that does not conflict with holding a node
    /// reference. Crate-internal on purpose: `tree_size` is an invariant, not a
    /// knob.
    pub(crate) fn parts_mut(&mut self) -> (&TreeGeometry, &mut Option<Node<T>>, &mut usize) {
        (&self.geometry, &mut self.root, &mut self.tree_size)
    }

    /// Installs a whole subtree as the root, recomputing the node count.
    ///
    /// Used by deserialization, which builds the structure bottom-up and has no
    /// running count to maintain.
    pub(crate) fn set_root(&mut self, root: Option<Node<T>>) {
        self.tree_size = root.as_ref().map_or(0, Node::subtree_size);
        self.root = root;
    }

    /// Sets the root to a fresh node holding `value`, if the tree is empty.
    ///
    /// Returns `true` when a root was created.
    pub(crate) fn ensure_root(&mut self, value: T) -> bool {
        if self.root.is_some() {
            return false;
        }
        self.root = Some(Node::new(value));
        self.tree_size += 1;
        true
    }

    /// Finds the node addressed by `key`, descending at most `depth` levels.
    ///
    /// `depth` of 0 means the full tree depth, matching the reference. When the
    /// descent reaches a leaf before `depth` — a pruned node covering the key —
    /// that leaf is returned rather than `None`.
    ///
    /// # Errors
    ///
    /// Returns [`crate::OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn search_at_depth(&self, key: OcTreeKey, depth: u32) -> Result<Option<&Node<T>>> {
        self.geometry.validate_depth(depth)?;
        let tree_depth = self.geometry.tree_depth();
        let depth = if depth == 0 { tree_depth } else { depth };

        let Some(root) = self.root.as_ref() else {
            return Ok(None);
        };

        let key_at_depth = if depth == tree_depth {
            key
        } else {
            self.geometry.adjust_key_at_depth(key, depth)?
        };

        let mut node = root;
        let diff = tree_depth - depth;
        for level in (diff..tree_depth).rev() {
            let pos = compute_child_index(&key_at_depth, level);
            match node.child(pos) {
                Some(child) => node = child,
                None => {
                    // A missing child on a childless node means the key falls
                    // inside a pruned leaf, which is a hit, not a miss.
                    return Ok(if node.has_children() {
                        None
                    } else {
                        Some(node)
                    });
                }
            }
        }
        Ok(Some(node))
    }

    /// Finds the leaf-level node addressed by `key`.
    #[inline]
    pub fn search(&self, key: OcTreeKey) -> Option<&Node<T>> {
        self.search_at_depth(key, 0)
            .expect("depth 0 is always valid")
    }

    /// Finds the node containing `point`, or `None` when the point lies
    /// outside the addressable volume.
    #[inline]
    pub fn search_point(&self, point: Point3) -> Option<&Node<T>> {
        self.search(self.geometry.coord_to_key_checked(point)?)
    }

    /// The value stored at `key`, if the tree covers it.
    #[inline]
    pub fn get(&self, key: OcTreeKey) -> Option<&T> {
        self.search(key).map(Node::value)
    }

    /// Iterates every node in the tree, inner nodes included, depth first.
    ///
    /// Children are visited in index order 0..8, matching the reference's
    /// `tree_iterator`.
    pub fn iter_nodes(&self) -> TreeIter<'_, T> {
        self.iter_nodes_to_depth(self.geometry.tree_depth())
    }

    /// Iterates every node down to `max_depth`.
    pub fn iter_nodes_to_depth(&self, max_depth: u32) -> TreeIter<'_, T> {
        TreeIter {
            stack: self.initial_stack(),
            max_depth: self.clamp_depth(max_depth),
            tree_max_val: self.geometry.tree_max_val(),
        }
    }

    /// Iterates the leaves of the tree, depth first.
    pub fn iter_leaves(&self) -> LeafIter<'_, T> {
        self.iter_leaves_to_depth(self.geometry.tree_depth())
    }

    /// Iterates the tree as if it were pruned to `max_depth`, so that nodes at
    /// `max_depth` are reported as leaves even when they have children.
    pub fn iter_leaves_to_depth(&self, max_depth: u32) -> LeafIter<'_, T> {
        LeafIter {
            inner: TreeIter {
                stack: self.initial_stack(),
                max_depth: self.clamp_depth(max_depth),
                tree_max_val: self.geometry.tree_max_val(),
            },
        }
    }

    fn initial_stack(&self) -> Vec<Visit<'_, T>> {
        match self.root.as_ref() {
            None => Vec::new(),
            Some(root) => {
                let center = self.geometry.tree_max_val() as u16;
                vec![Visit {
                    node: root,
                    key: OcTreeKey::new(center, center, center),
                    depth: 0,
                }]
            }
        }
    }

    #[inline]
    fn clamp_depth(&self, max_depth: u32) -> u32 {
        let tree_depth = self.geometry.tree_depth();
        if max_depth == 0 || max_depth > tree_depth {
            tree_depth
        } else {
            max_depth
        }
    }
}

impl<T: Clone + Default + PartialEq> OctreeCore<T> {
    /// Stores `value` at `key`, creating the path down to it.
    ///
    /// Intermediate nodes are created holding `T::default()`. Returns a mutable
    /// reference to the stored value.
    #[inline]
    pub fn insert(&mut self, key: OcTreeKey, value: T) -> &mut T {
        self.insert_at_depth(key, 0, value)
            .expect("depth 0 is always valid")
    }

    /// Stores `value` at `key`, stopping the descent at `depth`.
    ///
    /// `depth` of 0 means the full tree depth.
    ///
    /// # Errors
    ///
    /// Returns [`crate::OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn insert_at_depth(&mut self, key: OcTreeKey, depth: u32, value: T) -> Result<&mut T> {
        self.geometry.validate_depth(depth)?;
        let tree_depth = self.geometry.tree_depth();
        let depth = if depth == 0 { tree_depth } else { depth };

        // Split the borrow so the node count can be updated while a node
        // reference into `root` is alive.
        let Self {
            root, tree_size, ..
        } = self;

        if root.is_none() {
            *root = Some(Node::new(T::default()));
            *tree_size += 1;
        }
        let mut node = root.as_mut().expect("root was just ensured");

        for level in 0..depth {
            let pos = compute_child_index(&key, tree_depth - 1 - level);
            if !node.child_exists(pos) && node.create_child(pos, T::default()) {
                *tree_size += 1;
            }
            node = node.child_mut(pos).expect("child was just created");
        }

        node.set_value(value);
        Ok(node.value_mut())
    }

    /// Stores `value` at the voxel containing `point`.
    ///
    /// Returns `None` when the point lies outside the addressable volume.
    pub fn insert_point(&mut self, point: Point3, value: T) -> Option<&mut T> {
        let key = self.geometry.coord_to_key_checked(point)?;
        Some(self.insert(key, value))
    }

    /// Removes the node addressed by `key` and everything below it.
    ///
    /// Returns `true` when something was removed. Descending into a pruned node
    /// expands it first, so deleting one voxel out of a pruned block leaves the
    /// other seven behind — the reference behaves the same way.
    #[inline]
    pub fn delete(&mut self, key: OcTreeKey) -> bool {
        self.delete_at_depth(key, 0)
            .expect("depth 0 is always valid")
    }

    /// Removes the node addressed by `key` at `depth`, and everything below it.
    ///
    /// `depth` of 0 means the full tree depth.
    ///
    /// # Errors
    ///
    /// Returns [`crate::OctomapError::InvalidDepth`] if `depth > tree_depth`.
    pub fn delete_at_depth(&mut self, key: OcTreeKey, depth: u32) -> Result<bool> {
        self.geometry.validate_depth(depth)?;
        let tree_depth = self.geometry.tree_depth();
        let max_depth = if depth == 0 { tree_depth } else { depth };

        let Self {
            root, tree_size, ..
        } = self;
        let Some(root_node) = root.as_mut() else {
            return Ok(false);
        };

        let mut delta = Delta::default();
        let drop_root = delete_recurs(root_node, 0, max_depth, &key, tree_depth, true, &mut delta);

        if drop_root {
            delta.removed += 1;
            *root = None;
        }

        *tree_size = tree_size
            .saturating_add(delta.created)
            .saturating_sub(delta.removed);
        if root.is_none() {
            *tree_size = 0;
        }
        Ok(delta.removed > 0)
    }

    /// Merges every group of eight equal childless children into its parent.
    ///
    /// Returns the number of nodes removed.
    ///
    /// # Known limitation, inherited from the reference
    ///
    /// The sweep runs from the deepest level upward and stops early at the
    /// first level that merges nothing. A tree that is already partly pruned —
    /// so that its deepest level has nothing left to merge — is therefore left
    /// alone even when shallower levels could still collapse. The reference
    /// carries a `FIXME` about this; the behavior is reproduced deliberately so
    /// that pruned output matches C++.
    pub fn prune(&mut self) -> usize {
        let Self {
            geometry,
            root,
            tree_size,
        } = self;
        let Some(root_node) = root.as_mut() else {
            return 0;
        };

        let mut total_removed = 0usize;
        for max_depth in (0..geometry.tree_depth()).rev() {
            let mut pruned_here = 0usize;
            let mut removed_here = 0usize;
            prune_recurs(root_node, 0, max_depth, &mut pruned_here, &mut removed_here);
            total_removed += removed_here;
            if pruned_here == 0 {
                break;
            }
        }

        *tree_size = tree_size.saturating_sub(total_removed);
        total_removed
    }

    /// Splits every leaf into eight children, down to `max_depth`.
    ///
    /// Returns the number of nodes created. `max_depth` of 0 means the full
    /// tree depth.
    ///
    /// # Warning
    ///
    /// Cost is exponential in the distance between the shallowest leaf and
    /// `max_depth`: expanding a leaf at depth 1 all the way to depth 16 would
    /// create over a trillion nodes. Pass an explicit `max_depth` unless the
    /// tree is known to be populated near the bottom already.
    pub fn expand_to_depth(&mut self, max_depth: u32) -> usize {
        let max_depth = self.clamp_depth(max_depth);
        let Self {
            root, tree_size, ..
        } = self;
        let Some(root_node) = root.as_mut() else {
            return 0;
        };

        let mut created = 0usize;
        expand_recurs(root_node, 0, max_depth, &mut created);
        *tree_size += created;
        created
    }
}

/// Node-count bookkeeping threaded through the recursive helpers.
#[derive(Default)]
struct Delta {
    created: usize,
    removed: usize,
}

/// Returns `true` when the caller should delete this node.
///
/// Mirrors `deleteNodeRecurs`, including the detail that hitting a childless
/// non-root node mid-descent expands it rather than failing — that is what
/// makes deleting a single voxel out of a pruned block work.
fn delete_recurs<T: Clone + Default + PartialEq>(
    node: &mut Node<T>,
    depth: u32,
    max_depth: u32,
    key: &OcTreeKey,
    tree_depth: u32,
    is_root: bool,
    delta: &mut Delta,
) -> bool {
    if depth >= max_depth {
        return true;
    }

    let pos = compute_child_index(key, tree_depth - 1 - depth);

    if !node.child_exists(pos) {
        if !node.has_children() && !is_root {
            delta.created += node.expand();
        } else {
            return false;
        }
    }

    let child = node.child_mut(pos).expect("child exists or was expanded");
    let drop_child = delete_recurs(child, depth + 1, max_depth, key, tree_depth, false, delta);

    if drop_child {
        delta.removed += node.delete_child(pos);
        if !node.has_children() {
            return true;
        }
    }
    false
}

fn prune_recurs<T: Clone + PartialEq>(
    node: &mut Node<T>,
    depth: u32,
    max_depth: u32,
    pruned: &mut usize,
    removed: &mut usize,
) {
    if depth < max_depth {
        for i in 0..CHILD_COUNT as u8 {
            if let Some(child) = node.child_mut(i) {
                prune_recurs(child, depth + 1, max_depth, pruned, removed);
            }
        }
    } else {
        let n = node.prune();
        if n > 0 {
            *pruned += 1;
            *removed += n;
        }
    }
}

fn expand_recurs<T: Clone + PartialEq>(
    node: &mut Node<T>,
    depth: u32,
    max_depth: u32,
    created: &mut usize,
) {
    if depth >= max_depth {
        return;
    }
    if !node.has_children() {
        *created += node.expand();
    }
    for i in 0..CHILD_COUNT as u8 {
        if let Some(child) = node.child_mut(i) {
            expand_recurs(child, depth + 1, max_depth, created);
        }
    }
}

/// One node reached during traversal, with the key and depth it sits at.
#[derive(Debug, Clone, Copy)]
pub struct Visit<'a, T> {
    node: &'a Node<T>,
    key: OcTreeKey,
    depth: u32,
}

impl<'a, T> Visit<'a, T> {
    /// The node itself.
    #[inline]
    pub fn node(&self) -> &'a Node<T> {
        self.node
    }

    /// The value stored in the node.
    #[inline]
    pub fn value(&self) -> &'a T {
        self.node.value()
    }

    /// The key addressing this node's center.
    #[inline]
    pub fn key(&self) -> OcTreeKey {
        self.key
    }

    /// Depth of this node, with the root at 0.
    #[inline]
    pub fn depth(&self) -> u32 {
        self.depth
    }
}

/// Depth-first iterator over every node, inner nodes included.
pub struct TreeIter<'a, T> {
    stack: Vec<Visit<'a, T>>,
    max_depth: u32,
    tree_max_val: u32,
}

impl<'a, T> TreeIter<'a, T> {
    /// Pushes `top`'s children so they pop in index order 0..8.
    fn push_children(&mut self, top: &Visit<'a, T>) {
        if top.depth >= self.max_depth {
            return;
        }
        let child_depth = top.depth + 1;
        let center_offset_key = (self.tree_max_val >> child_depth) as u16;
        for i in (0..CHILD_COUNT as u8).rev() {
            if let Some(child) = top.node.child(i) {
                self.stack.push(Visit {
                    node: child,
                    key: compute_child_key(i, center_offset_key, &top.key),
                    depth: child_depth,
                });
            }
        }
    }
}

impl<'a, T> Iterator for TreeIter<'a, T> {
    type Item = Visit<'a, T>;

    fn next(&mut self) -> Option<Self::Item> {
        let top = self.stack.pop()?;
        self.push_children(&top);
        Some(top)
    }
}

/// Depth-first iterator over leaves only.
pub struct LeafIter<'a, T> {
    inner: TreeIter<'a, T>,
}

impl<'a, T> Iterator for LeafIter<'a, T> {
    type Item = Visit<'a, T>;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(top) = self.inner.stack.pop() {
            // A node at max_depth counts as a leaf even when it has children,
            // which is what makes depth-limited iteration report a coarse view.
            if !top.node.has_children() || top.depth == self.inner.max_depth {
                return Some(top);
            }
            self.inner.push_children(&top);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree() -> OctreeCore<u8> {
        OctreeCore::new(0.1).unwrap()
    }

    fn key(x: u16, y: u16, z: u16) -> OcTreeKey {
        OcTreeKey::new(x, y, z)
    }

    #[test]
    fn a_new_tree_is_empty() {
        let t = tree();
        assert!(t.is_empty());
        assert_eq!(t.len(), 0);
        assert_eq!(t.count_nodes(), 0);
        assert_eq!(t.count_leaf_nodes(), 0);
        assert!(t.root().is_none());
        assert_eq!(t.iter_nodes().count(), 0);
        assert_eq!(t.iter_leaves().count(), 0);
    }

    #[test]
    fn insert_creates_the_full_path_to_the_leaf() {
        let mut t = tree();
        t.insert(key(32768, 32768, 32768), 42);

        // root plus one node per level.
        assert_eq!(t.len(), 1 + t.geometry().tree_depth() as usize);
        assert_eq!(t.len(), t.count_nodes(), "incremental count drifted");
        assert_eq!(t.count_leaf_nodes(), 1);
        assert_eq!(t.get(key(32768, 32768, 32768)), Some(&42));
    }

    #[test]
    fn inserting_the_same_key_twice_reuses_the_path() {
        let mut t = tree();
        t.insert(key(1, 2, 3), 1);
        let after_first = t.len();
        t.insert(key(1, 2, 3), 2);

        assert_eq!(t.len(), after_first, "no new nodes for a repeat insert");
        assert_eq!(t.get(key(1, 2, 3)), Some(&2), "value must be overwritten");
    }

    #[test]
    fn sibling_keys_share_their_ancestors() {
        let mut t = tree();
        t.insert(key(32768, 32768, 32768), 1);
        let after_first = t.len();
        // Differs only in the lowest bit of x, so only the leaf is new.
        t.insert(key(32769, 32768, 32768), 2);

        assert_eq!(t.len(), after_first + 1);
        assert_eq!(t.count_nodes(), t.len());
    }

    #[test]
    fn search_misses_return_none() {
        let mut t = tree();
        t.insert(key(100, 100, 100), 7);
        assert_eq!(t.get(key(200, 200, 200)), None);
    }

    #[test]
    fn search_inside_a_pruned_block_returns_the_covering_leaf() {
        let mut t = tree();
        // Fill one block of eight sibling leaves with the same value.
        let base = key(32768, 32768, 32768);
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    t.insert(key(base.x() + dx, base.y() + dy, base.z() + dz), 5);
                }
            }
        }
        assert_eq!(t.prune(), 8, "the eight equal leaves should merge");

        // The leaves are gone, but the block still answers for each of them.
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    let k = key(base.x() + dx, base.y() + dy, base.z() + dz);
                    assert_eq!(t.get(k), Some(&5), "pruned block must still cover {k:?}");
                }
            }
        }
    }

    #[test]
    fn prune_then_expand_restores_the_node_count() {
        let mut t = tree();
        let base = key(32768, 32768, 32768);
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    t.insert(key(base.x() + dx, base.y() + dy, base.z() + dz), 5);
                }
            }
        }
        let before = t.len();

        let removed = t.prune();
        assert_eq!(t.len(), before - removed);
        assert_eq!(t.count_nodes(), t.len());

        let created = t.expand_to_depth(t.geometry().tree_depth());
        assert_eq!(created, removed, "expansion should undo the merge");
        assert_eq!(t.len(), before);
        assert_eq!(t.count_nodes(), t.len());
    }

    #[test]
    fn prune_leaves_unequal_siblings_alone() {
        let mut t = tree();
        let base = key(32768, 32768, 32768);
        let mut n = 0u8;
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    n += 1;
                    t.insert(key(base.x() + dx, base.y() + dy, base.z() + dz), n);
                }
            }
        }
        let before = t.len();
        assert_eq!(t.prune(), 0, "differing values must not merge");
        assert_eq!(t.len(), before);
    }

    #[test]
    fn delete_removes_the_node_and_its_now_empty_ancestors() {
        let mut t = tree();
        t.insert(key(1, 2, 3), 9);
        assert!(t.delete(key(1, 2, 3)));

        assert!(t.is_empty(), "the last leaf going away empties the tree");
        assert_eq!(t.len(), 0);
        assert_eq!(t.count_nodes(), 0);
    }

    #[test]
    fn delete_keeps_ancestors_that_still_have_children() {
        let mut t = tree();
        t.insert(key(32768, 32768, 32768), 1);
        t.insert(key(32769, 32768, 32768), 2);

        assert!(t.delete(key(32768, 32768, 32768)));
        assert!(!t.is_empty());
        assert_eq!(
            t.get(key(32769, 32768, 32768)),
            Some(&2),
            "sibling survives"
        );
        assert_eq!(t.get(key(32768, 32768, 32768)), None);
        assert_eq!(t.count_nodes(), t.len(), "count drifted after delete");
    }

    #[test]
    fn deleting_a_missing_key_reports_nothing_removed() {
        let mut t = tree();
        t.insert(key(1, 2, 3), 9);
        let before = t.len();
        assert!(!t.delete(key(500, 600, 700)));
        assert_eq!(t.len(), before);
    }

    #[test]
    fn deleting_from_an_empty_tree_is_harmless() {
        let mut t = tree();
        assert!(!t.delete(key(1, 2, 3)));
        assert!(t.is_empty());
    }

    #[test]
    fn deleting_one_voxel_from_a_pruned_block_keeps_the_other_seven() {
        let mut t = tree();
        let base = key(32768, 32768, 32768);
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    t.insert(key(base.x() + dx, base.y() + dy, base.z() + dz), 5);
                }
            }
        }
        t.prune();

        // Descending into the pruned block must re-expand it.
        assert!(t.delete(base));
        assert_eq!(t.get(base), None, "the deleted voxel is gone");

        let mut survivors = 0;
        for dx in 0..2u16 {
            for dy in 0..2u16 {
                for dz in 0..2u16 {
                    let k = key(base.x() + dx, base.y() + dy, base.z() + dz);
                    if k != base && t.get(k) == Some(&5) {
                        survivors += 1;
                    }
                }
            }
        }
        assert_eq!(survivors, 7, "the rest of the block must survive");
        assert_eq!(t.count_nodes(), t.len(), "count drifted after re-expansion");
    }

    #[test]
    fn clear_empties_the_tree() {
        let mut t = tree();
        t.insert(key(1, 2, 3), 1);
        t.clear();
        assert!(t.is_empty());
        assert_eq!(t.len(), 0);
    }

    #[test]
    fn iterating_nodes_visits_every_node_once() {
        let mut t = tree();
        t.insert(key(32768, 32768, 32768), 1);
        t.insert(key(32769, 32768, 32768), 2);

        assert_eq!(t.iter_nodes().count(), t.len());
    }

    #[test]
    fn iterating_leaves_visits_only_leaves() {
        let mut t = tree();
        t.insert(key(32768, 32768, 32768), 1);
        t.insert(key(32769, 32768, 32768), 2);

        let leaves: Vec<_> = t.iter_leaves().collect();
        assert_eq!(leaves.len(), 2);
        assert_eq!(leaves.len(), t.count_leaf_nodes());
        for leaf in &leaves {
            assert!(!leaf.node().has_children());
            assert_eq!(leaf.depth(), t.geometry().tree_depth());
        }
    }

    #[test]
    fn leaf_iteration_reports_the_key_that_was_inserted() {
        let mut t = tree();
        let inserted = key(32768, 32770, 32771);
        t.insert(inserted, 1);

        let leaves: Vec<_> = t.iter_leaves().collect();
        assert_eq!(leaves.len(), 1);
        assert_eq!(
            leaves[0].key(),
            inserted,
            "the key rebuilt during descent must match the one inserted"
        );
    }

    #[test]
    fn depth_limited_iteration_reports_inner_nodes_as_leaves() {
        let mut t = tree();
        t.insert(key(32768, 32768, 32768), 1);
        t.insert(key(32769, 32768, 32768), 2);

        // Both leaves live under one node at depth 15, so a depth-4 view
        // collapses them into a single reported leaf.
        let coarse: Vec<_> = t.iter_leaves_to_depth(4).collect();
        assert_eq!(coarse.len(), 1);
        assert_eq!(coarse[0].depth(), 4);
    }

    #[test]
    fn iteration_order_is_child_index_order() {
        let mut t = OctreeCore::<u8>::new(1.0).unwrap();
        // Two keys differing in the lowest bit of x land in children 0 and 1
        // of their shared parent, so child 0 must come out first.
        t.insert(key(32768, 32768, 32768), 10);
        t.insert(key(32769, 32768, 32768), 20);

        let values: Vec<u8> = t.iter_leaves().map(|v| *v.value()).collect();
        assert_eq!(values, vec![10, 20]);
    }

    #[test]
    fn insert_at_depth_stops_early_and_stays_searchable() {
        let mut t = tree();
        t.insert_at_depth(key(32768, 32768, 32768), 4, 7).unwrap();

        assert_eq!(t.len(), 1 + 4);
        let leaves: Vec<_> = t.iter_leaves().collect();
        assert_eq!(leaves.len(), 1);
        assert_eq!(leaves[0].depth(), 4);
        // A shallow leaf answers for every key beneath it.
        assert_eq!(t.get(key(32768, 32768, 32768)), Some(&7));
    }

    #[test]
    fn depth_beyond_tree_depth_is_rejected() {
        let mut t = tree();
        assert!(t.insert_at_depth(key(0, 0, 0), 17, 1).is_err());
        assert!(t.search_at_depth(key(0, 0, 0), 17).is_err());
        assert!(t.delete_at_depth(key(0, 0, 0), 17).is_err());
    }

    #[test]
    fn point_insertion_rejects_coordinates_outside_the_volume() {
        let mut t = tree();
        assert!(t.insert_point(Point3::new(1.0, 2.0, 3.0), 1).is_some());
        // 0.1 m resolution addresses ±3276.8 m.
        assert!(t.insert_point(Point3::new(1.0e9, 0.0, 0.0), 1).is_none());
    }

    #[test]
    fn point_round_trip_finds_the_inserted_value() {
        let mut t = tree();
        let p = Point3::new(1.25, -2.5, 0.35);
        t.insert_point(p, 99);
        assert_eq!(t.search_point(p).map(Node::value), Some(&99));
    }

    #[test]
    fn node_count_survives_a_mixed_workload() {
        let mut t = tree();
        for i in 0..50u16 {
            t.insert(key(32768 + i, 32768, 32768), (i % 7) as u8);
        }
        assert_eq!(t.count_nodes(), t.len());

        for i in (0..50u16).step_by(3) {
            t.delete(key(32768 + i, 32768, 32768));
        }
        assert_eq!(t.count_nodes(), t.len(), "count drifted after deletions");

        t.prune();
        assert_eq!(t.count_nodes(), t.len(), "count drifted after pruning");
    }
}
