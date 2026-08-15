//! Octree nodes.
//!
//! Ported from `OcTreeDataNode`. The reference stores `AbstractOcTreeNode**` —
//! a lazily allocated array of eight raw child pointers — and makes the *tree*
//! responsible for freeing it. Here the child array is an
//! `Option<Box<[Option<Node<T>>; 8]>>`, which keeps the same "no array until
//! the first child" allocation behavior while letting ownership handle the
//! teardown the reference does by hand.

/// Number of children an octree node can have.
pub const CHILD_COUNT: usize = 8;

/// A node holding a value of type `T` and up to eight children.
#[derive(Debug, Clone, PartialEq)]
pub struct Node<T> {
    value: T,
    children: Option<Box<[Option<Node<T>>; CHILD_COUNT]>>,
}

impl<T> Node<T> {
    /// Creates a childless node holding `value`.
    #[inline]
    pub fn new(value: T) -> Self {
        Self {
            value,
            children: None,
        }
    }

    /// The value stored in this node.
    #[inline]
    pub fn value(&self) -> &T {
        &self.value
    }

    /// Mutable access to the value stored in this node.
    #[inline]
    pub fn value_mut(&mut self) -> &mut T {
        &mut self.value
    }

    /// Replaces the value stored in this node.
    #[inline]
    pub fn set_value(&mut self, value: T) {
        self.value = value;
    }

    /// True when at least one child slot is occupied.
    ///
    /// Mirrors `nodeHasChildren`: an allocated-but-empty child array still
    /// counts as *no* children.
    #[inline]
    pub fn has_children(&self) -> bool {
        match &self.children {
            None => false,
            Some(children) => children.iter().any(Option::is_some),
        }
    }

    /// True when the child at `index` exists.
    ///
    /// # Panics
    ///
    /// Panics if `index >= 8`.
    #[inline]
    pub fn child_exists(&self, index: u8) -> bool {
        self.child(index).is_some()
    }

    /// The child at `index`, if it exists.
    ///
    /// # Panics
    ///
    /// Panics if `index >= 8`.
    #[inline]
    pub fn child(&self, index: u8) -> Option<&Node<T>> {
        assert!((index as usize) < CHILD_COUNT, "child index {index} >= 8");
        self.children.as_ref()?[index as usize].as_ref()
    }

    /// Mutable access to the child at `index`, if it exists.
    ///
    /// # Panics
    ///
    /// Panics if `index >= 8`.
    #[inline]
    pub fn child_mut(&mut self, index: u8) -> Option<&mut Node<T>> {
        assert!((index as usize) < CHILD_COUNT, "child index {index} >= 8");
        self.children.as_mut()?[index as usize].as_mut()
    }

    /// Number of occupied child slots.
    #[inline]
    pub fn child_count(&self) -> usize {
        match &self.children {
            None => 0,
            Some(children) => children.iter().filter(|c| c.is_some()).count(),
        }
    }

    /// Iterates the occupied children together with their indices.
    pub fn children_iter(&self) -> impl Iterator<Item = (u8, &Node<T>)> {
        self.children
            .iter()
            .flat_map(|c| c.iter())
            .enumerate()
            .filter_map(|(i, child)| child.as_ref().map(|c| (i as u8, c)))
    }

    /// Inserts a child holding `value` at `index`, replacing any existing one.
    ///
    /// Returns `true` when a new slot was filled, `false` when an existing
    /// child was overwritten — the caller uses this to keep the tree's node
    /// count correct.
    ///
    /// # Panics
    ///
    /// Panics if `index >= 8`.
    pub fn create_child(&mut self, index: u8, value: T) -> bool {
        assert!((index as usize) < CHILD_COUNT, "child index {index} >= 8");
        let children = self
            .children
            .get_or_insert_with(|| Box::new(core::array::from_fn(|_| None)));
        let was_empty = children[index as usize].is_none();
        children[index as usize] = Some(Node::new(value));
        was_empty
    }

    /// Removes the child at `index` and everything below it.
    ///
    /// Returns the number of nodes removed, which is zero when the slot was
    /// already empty.
    ///
    /// # Panics
    ///
    /// Panics if `index >= 8`.
    pub fn delete_child(&mut self, index: u8) -> usize {
        assert!((index as usize) < CHILD_COUNT, "child index {index} >= 8");
        let Some(children) = self.children.as_mut() else {
            return 0;
        };
        let Some(child) = children[index as usize].take() else {
            return 0;
        };
        let removed = child.subtree_size();

        // The reference frees the child array as soon as it empties, so that
        // `nodeHasChildren` and the pruning logic see a genuine leaf.
        if children.iter().all(Option::is_none) {
            self.children = None;
        }
        removed
    }

    /// Drops the child array, removing every child at once.
    ///
    /// Returns the number of nodes removed.
    pub fn clear_children(&mut self) -> usize {
        match self.children.take() {
            None => 0,
            Some(children) => children
                .iter()
                .filter_map(Option::as_ref)
                .map(Node::subtree_size)
                .sum(),
        }
    }

    /// Number of nodes in the subtree rooted at this node, counting itself.
    pub fn subtree_size(&self) -> usize {
        1 + self
            .children_iter()
            .map(|(_, child)| child.subtree_size())
            .sum::<usize>()
    }

    /// Number of leaves in the subtree rooted at this node.
    ///
    /// A childless node is its own leaf, so this never returns zero. Mirrors
    /// `getNumLeafNodesRecurs`.
    pub fn leaf_count(&self) -> usize {
        if !self.has_children() {
            return 1;
        }
        self.children_iter()
            .map(|(_, child)| child.leaf_count())
            .sum()
    }
}

impl<T: Clone + PartialEq> Node<T> {
    /// True when this node's eight children can be merged back into it.
    ///
    /// Mirrors `isNodeCollapsible`: every child must exist, be childless
    /// itself, and hold a value equal to the first child's.
    pub fn is_collapsible(&self) -> bool {
        let Some(first) = self.child(0) else {
            return false;
        };
        if first.has_children() {
            return false;
        }
        (1..CHILD_COUNT as u8).all(|i| match self.child(i) {
            None => false,
            Some(child) => !child.has_children() && child.value == first.value,
        })
    }

    /// Merges eight equal childless children back into this node.
    ///
    /// Returns the number of nodes removed — eight on success, zero when the
    /// node was not collapsible.
    pub fn prune(&mut self) -> usize {
        if !self.is_collapsible() {
            return 0;
        }
        // All eight are equal, so child 0 is representative.
        self.value = self
            .child(0)
            .expect("collapsible implies child 0")
            .value
            .clone();
        self.clear_children()
    }

    /// Splits this node into eight children, each a copy of its own value.
    ///
    /// Returns the number of nodes created, or zero when the node already had
    /// children. The reference asserts in that case; returning zero keeps the
    /// caller's bookkeeping honest without a panic.
    pub fn expand(&mut self) -> usize {
        if self.has_children() {
            return 0;
        }
        let value = self.value.clone();
        for i in 0..CHILD_COUNT as u8 {
            self.create_child(i, value.clone());
        }
        CHILD_COUNT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_node_has_no_children_and_no_array() {
        let node = Node::new(1u8);
        assert!(!node.has_children());
        assert_eq!(node.child_count(), 0);
        assert_eq!(node.subtree_size(), 1);
        assert_eq!(node.leaf_count(), 1);
        assert!(node.children.is_none(), "array must stay unallocated");
    }

    #[test]
    fn create_child_reports_whether_a_slot_was_filled() {
        let mut node = Node::new(0u8);
        assert!(node.create_child(3, 7));
        assert!(!node.create_child(3, 9), "overwrite is not a new node");
        assert_eq!(node.child(3).unwrap().value(), &9);
        assert_eq!(node.child_count(), 1);
    }

    #[test]
    fn delete_child_frees_the_array_once_empty() {
        let mut node = Node::new(0u8);
        node.create_child(1, 1);
        node.create_child(2, 2);

        assert_eq!(node.delete_child(1), 1);
        assert!(node.children.is_some(), "still has one child");

        assert_eq!(node.delete_child(2), 1);
        assert!(
            node.children.is_none(),
            "array must be released so the node reads as a leaf"
        );
        assert!(!node.has_children());
    }

    #[test]
    fn deleting_an_empty_slot_removes_nothing() {
        let mut node = Node::new(0u8);
        assert_eq!(node.delete_child(5), 0);
        node.create_child(1, 1);
        assert_eq!(node.delete_child(5), 0);
    }

    #[test]
    fn delete_child_counts_the_whole_subtree() {
        let mut node = Node::new(0u8);
        node.create_child(0, 1);
        let child = node.child_mut(0).unwrap();
        child.create_child(0, 2);
        child.create_child(1, 3);

        // child + its two grandchildren
        assert_eq!(node.delete_child(0), 3);
        assert_eq!(node.subtree_size(), 1);
    }

    #[test]
    fn collapsible_requires_all_eight_equal_childless_children() {
        let mut node = Node::new(0u8);
        assert!(!node.is_collapsible(), "no children at all");

        for i in 0..8 {
            node.create_child(i, 5);
        }
        assert!(node.is_collapsible());

        // One differing value blocks the merge.
        node.child_mut(4).unwrap().set_value(6);
        assert!(!node.is_collapsible());
        node.child_mut(4).unwrap().set_value(5);
        assert!(node.is_collapsible());

        // So does a grandchild.
        node.child_mut(2).unwrap().create_child(0, 5);
        assert!(!node.is_collapsible());
    }

    #[test]
    fn collapsible_is_false_when_a_child_is_missing() {
        let mut node = Node::new(0u8);
        for i in 0..7 {
            node.create_child(i, 5);
        }
        assert!(!node.is_collapsible(), "only seven children");
    }

    #[test]
    fn prune_adopts_the_child_value_and_removes_eight_nodes() {
        let mut node = Node::new(0u8);
        for i in 0..8 {
            node.create_child(i, 5);
        }
        assert_eq!(node.prune(), 8);
        assert_eq!(node.value(), &5, "parent takes the children's value");
        assert!(!node.has_children());
        assert_eq!(node.subtree_size(), 1);
    }

    #[test]
    fn prune_is_a_no_op_when_not_collapsible() {
        let mut node = Node::new(0u8);
        node.create_child(0, 5);
        assert_eq!(node.prune(), 0);
        assert_eq!(node.value(), &0, "value must be untouched");
        assert_eq!(node.child_count(), 1);
    }

    #[test]
    fn expand_copies_the_value_into_eight_children() {
        let mut node = Node::new(42u8);
        assert_eq!(node.expand(), 8);
        assert_eq!(node.child_count(), 8);
        for i in 0..8 {
            assert_eq!(node.child(i).unwrap().value(), &42);
        }
        assert_eq!(node.leaf_count(), 8);
    }

    #[test]
    fn expand_then_prune_is_the_identity() {
        let mut node = Node::new(42u8);
        let before = node.clone();
        node.expand();
        node.prune();
        assert_eq!(node, before);
    }

    #[test]
    fn expand_refuses_a_node_that_already_has_children() {
        let mut node = Node::new(1u8);
        node.create_child(0, 2);
        assert_eq!(node.expand(), 0);
        assert_eq!(node.child_count(), 1);
    }

    #[test]
    fn leaf_count_ignores_inner_nodes() {
        let mut root = Node::new(0u8);
        root.create_child(0, 1);
        root.create_child(1, 1);
        root.child_mut(0).unwrap().expand();

        // child 0 contributes its eight leaves, child 1 contributes itself.
        assert_eq!(root.leaf_count(), 9);
        assert_eq!(root.subtree_size(), 1 + 2 + 8);
    }

    #[test]
    #[should_panic(expected = "child index 8 >= 8")]
    fn child_index_above_seven_panics() {
        Node::new(0u8).child(8);
    }
}
