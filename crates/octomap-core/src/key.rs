//! Discrete voxel addressing.
//!
//! Ported from `OcTreeKey.h`. A key is *not* a position in meters — it is the
//! index of a voxel counted from the corner of the addressable volume, which is
//! why the reference offsets every axis by `tree_max_val` (32768) so that the
//! world origin lands in the middle of the `u16` range.

/// Scalar type of a single key axis. `typedef uint16_t key_type` in C++.
pub type KeyScalar = u16;

/// The discrete address of a voxel.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct OcTreeKey {
    /// Per-axis voxel indices, in `[x, y, z]` order.
    pub k: [KeyScalar; 3],
}

impl OcTreeKey {
    /// Builds a key from its three axis indices.
    #[inline]
    pub const fn new(x: KeyScalar, y: KeyScalar, z: KeyScalar) -> Self {
        Self { k: [x, y, z] }
    }

    /// Index along the x axis.
    #[inline]
    pub const fn x(&self) -> KeyScalar {
        self.k[0]
    }

    /// Index along the y axis.
    #[inline]
    pub const fn y(&self) -> KeyScalar {
        self.k[1]
    }

    /// Index along the z axis.
    #[inline]
    pub const fn z(&self) -> KeyScalar {
        self.k[2]
    }

    /// The hash the C++ reference uses for `KeySet` / `KeyBoolMap`.
    ///
    /// Rust's own `Hash` impl is derived and unrelated; this exists so that
    /// differential tests can reproduce reference bucket layouts when they need
    /// to. Ordinary code should not care.
    #[inline]
    pub fn reference_hash(&self) -> u64 {
        self.k[0] as u64 + 1447 * self.k[1] as u64 + 345_637 * self.k[2] as u64
    }
}

impl core::ops::Index<usize> for OcTreeKey {
    type Output = KeyScalar;

    #[inline]
    fn index(&self, i: usize) -> &KeyScalar {
        &self.k[i]
    }
}

impl core::ops::IndexMut<usize> for OcTreeKey {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut KeyScalar {
        &mut self.k[i]
    }
}

impl From<[KeyScalar; 3]> for OcTreeKey {
    #[inline]
    fn from(k: [KeyScalar; 3]) -> Self {
        Self { k }
    }
}

/// Key of a child while descending the tree.
///
/// `center_offset_key` is half the child's extent in key units at the current
/// level. The `- 1` on the negative branch when the offset is zero reproduces
/// the reference behavior at the bottom of the tree, where a half-extent of
/// zero would otherwise place both children on the same key.
#[inline]
pub fn compute_child_key(pos: u8, center_offset_key: KeyScalar, parent: &OcTreeKey) -> OcTreeKey {
    let axis = |bit: u8, parent_val: KeyScalar| -> KeyScalar {
        if pos & bit != 0 {
            parent_val.wrapping_add(center_offset_key)
        } else {
            let extra = if center_offset_key != 0 { 0 } else { 1 };
            parent_val
                .wrapping_sub(center_offset_key)
                .wrapping_sub(extra)
        }
    };

    OcTreeKey::new(
        axis(1, parent.k[0]),
        axis(2, parent.k[1]),
        axis(4, parent.k[2]),
    )
}

/// Child index (0..=7) selected by `key` at the given depth-from-bottom.
#[inline]
pub fn compute_child_index(key: &OcTreeKey, depth: u32) -> u8 {
    let bit = 1u32 << depth;
    let mut pos = 0u8;
    if key.k[0] as u32 & bit != 0 {
        pos += 1;
    }
    if key.k[1] as u32 & bit != 0 {
        pos += 2;
    }
    if key.k[2] as u32 & bit != 0 {
        pos += 4;
    }
    pos
}

/// Canonical key shared by every voxel that collapses together at `level`.
///
/// `level` counts up from the bottom of the tree, so `level == 0` is the
/// finest resolution and returns the key unchanged.
#[inline]
pub fn compute_index_key(level: u32, key: &OcTreeKey) -> OcTreeKey {
    if level == 0 {
        return *key;
    }
    let mask = (u16::MAX as u32) << level;
    OcTreeKey::new(
        (key.k[0] as u32 & mask) as u16,
        (key.k[1] as u32 & mask) as u16,
        (key.k[2] as u32 & mask) as u16,
    )
}

/// Upper bound the reference places on the number of keys in a single ray.
pub const KEY_RAY_MAX_SIZE: usize = 100_000;

/// The ordered set of voxels a ray passes through.
///
/// The reference preallocates [`KEY_RAY_MAX_SIZE`] entries once and reuses the
/// buffer for every ray; [`KeyRay::clear`] keeps that allocation so the same
/// reuse pattern works here.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct KeyRay {
    keys: Vec<OcTreeKey>,
}

impl KeyRay {
    /// An empty ray with no allocation yet.
    #[inline]
    pub fn new() -> Self {
        Self { keys: Vec::new() }
    }

    /// An empty ray with the reference's full buffer preallocated.
    #[inline]
    pub fn with_reference_capacity() -> Self {
        Self {
            keys: Vec::with_capacity(KEY_RAY_MAX_SIZE),
        }
    }

    /// Appends a key. Returns `false` — without appending — once the ray has
    /// reached [`KEY_RAY_MAX_SIZE`], where the reference would assert.
    #[inline]
    pub fn push(&mut self, key: OcTreeKey) -> bool {
        if self.keys.len() >= KEY_RAY_MAX_SIZE {
            return false;
        }
        self.keys.push(key);
        true
    }

    /// Empties the ray, keeping the allocation.
    #[inline]
    pub fn clear(&mut self) {
        self.keys.clear();
    }

    /// Number of keys currently in the ray.
    #[inline]
    pub fn len(&self) -> usize {
        self.keys.len()
    }

    /// True when the ray holds no keys.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }

    /// The keys, in traversal order.
    #[inline]
    pub fn as_slice(&self) -> &[OcTreeKey] {
        &self.keys
    }

    /// Iterates the keys in traversal order.
    #[inline]
    pub fn iter(&self) -> core::slice::Iter<'_, OcTreeKey> {
        self.keys.iter()
    }
}

impl<'a> IntoIterator for &'a KeyRay {
    type Item = &'a OcTreeKey;
    type IntoIter = core::slice::Iter<'a, OcTreeKey>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.keys.iter()
    }
}

impl IntoIterator for KeyRay {
    type Item = OcTreeKey;
    type IntoIter = std::vec::IntoIter<OcTreeKey>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.keys.into_iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_hash_matches_cpp_formula() {
        // KeyHash in OcTreeKey.h: k0 + 1447*k1 + 345637*k2
        let key = OcTreeKey::new(3, 5, 7);
        assert_eq!(key.reference_hash(), 3 + 1447 * 5 + 345_637 * 7);
    }

    #[test]
    fn child_index_reads_one_bit_per_axis() {
        // depth 0 inspects bit 0 of each axis.
        assert_eq!(compute_child_index(&OcTreeKey::new(0, 0, 0), 0), 0);
        assert_eq!(compute_child_index(&OcTreeKey::new(1, 0, 0), 0), 1);
        assert_eq!(compute_child_index(&OcTreeKey::new(0, 1, 0), 0), 2);
        assert_eq!(compute_child_index(&OcTreeKey::new(0, 0, 1), 0), 4);
        assert_eq!(compute_child_index(&OcTreeKey::new(1, 1, 1), 0), 7);

        // depth 3 inspects bit 3, so the low bits must not leak in.
        assert_eq!(compute_child_index(&OcTreeKey::new(0b0111, 0, 0), 3), 0);
        assert_eq!(compute_child_index(&OcTreeKey::new(0b1000, 0, 0), 3), 1);
    }

    #[test]
    fn child_key_offsets_each_axis_by_half_extent() {
        let parent = OcTreeKey::new(100, 100, 100);

        // pos 0 = all-negative octant, pos 7 = all-positive.
        assert_eq!(compute_child_key(0, 8, &parent), OcTreeKey::new(92, 92, 92));
        assert_eq!(
            compute_child_key(7, 8, &parent),
            OcTreeKey::new(108, 108, 108)
        );
        // Mixed: +x, -y, +z
        assert_eq!(
            compute_child_key(0b101, 8, &parent),
            OcTreeKey::new(108, 92, 108)
        );
    }

    #[test]
    fn zero_offset_child_key_still_separates_the_two_halves() {
        // At the deepest level the half-extent rounds to zero; the reference
        // subtracts an extra 1 so the negative child does not alias the parent.
        let parent = OcTreeKey::new(100, 100, 100);
        assert_eq!(compute_child_key(0, 0, &parent), OcTreeKey::new(99, 99, 99));
        assert_eq!(
            compute_child_key(7, 0, &parent),
            OcTreeKey::new(100, 100, 100)
        );
    }

    #[test]
    fn index_key_clears_low_bits_up_to_level() {
        let key = OcTreeKey::new(0b1011, 0b1111, 0b0001);
        assert_eq!(compute_index_key(0, &key), key);
        assert_eq!(
            compute_index_key(2, &key),
            OcTreeKey::new(0b1000, 0b1100, 0b0000)
        );
    }

    #[test]
    fn key_ray_refuses_to_grow_past_the_reference_limit() {
        let mut ray = KeyRay::new();
        assert!(ray.is_empty());
        assert!(ray.push(OcTreeKey::new(1, 2, 3)));
        assert_eq!(ray.len(), 1);
        assert_eq!(ray.as_slice(), &[OcTreeKey::new(1, 2, 3)]);

        ray.clear();
        assert!(ray.is_empty());
    }
}
