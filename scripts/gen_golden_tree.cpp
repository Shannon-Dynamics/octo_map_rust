// Dumps octree structure from the C++ reference: node counts, leaf iteration
// order, and how both change across prune and delete. Consumed by
// crates/octomap-core/tests/golden_tree.rs.
//
// Structure only — node *values* are occupancy log-odds, which is Phase 4
// material. Insertion uses setNodeValue(..., lazy_eval = true) rather than
// updateNode: updateNode auto-prunes on the way back up the recursion, which
// would make the "inserted" stage reflect occupancy behavior instead of the
// plain structural insert the generic tree performs. Every leaf gets the same
// value so the pruning predicate can still fire when prune() is called
// explicitly.
//
// Build (from the repo root, after the reference is built into build-cpp/):
//   g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
//       scripts/gen_golden_tree.cpp -o build-cpp/gen_golden_tree.exe \
//       -L build-cpp -loctomap -loctomath

#include <cstdio>
#include <octomap/OcTree.h>
#include <vector>

using octomap::OcTree;
using octomap::OcTreeKey;

namespace {

// A deterministic mix: one full prunable block of eight siblings, a scattered
// set, and a short run along one axis.
std::vector<OcTreeKey> buildKeys() {
  std::vector<OcTreeKey> keys;

  // Block A: eight siblings sharing a parent -> prunable.
  for (unsigned dx = 0; dx < 2; ++dx)
    for (unsigned dy = 0; dy < 2; ++dy)
      for (unsigned dz = 0; dz < 2; ++dz)
        keys.push_back(OcTreeKey(32768 + dx, 32768 + dy, 32768 + dz));

  // Block B: eight siblings elsewhere -> also prunable.
  for (unsigned dx = 0; dx < 2; ++dx)
    for (unsigned dy = 0; dy < 2; ++dy)
      for (unsigned dz = 0; dz < 2; ++dz)
        keys.push_back(OcTreeKey(40000 + dx, 40001 + dy, 39998 + dz));

  // Scattered singletons.
  const int offsets[][3] = {{100, 200, 300},   {-100, -200, -300}, {1, 1, 1},
                            {5000, 0, -5000},  {33000, 33000, 100}, {7, 65530, 12}};
  for (const auto& o : offsets)
    keys.push_back(OcTreeKey((uint16_t)(32768 + o[0]), (uint16_t)(32768 + o[1]),
                             (uint16_t)(32768 + o[2])));

  // A run along x, which shares most of its ancestry.
  for (unsigned i = 0; i < 20; ++i)
    keys.push_back(OcTreeKey(20000 + i, 21000, 22000));

  return keys;
}

void dumpCounts(const char* stage, OcTree& tree) {
  printf("counts,%s,%zu,%zu,%zu\n", stage, tree.size(), tree.calcNumNodes(),
         tree.getNumLeafNodes());
}

void dumpLeaves(const char* stage, OcTree& tree) {
  unsigned i = 0;
  for (OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs();
       it != end; ++it, ++i) {
    OcTreeKey k = it.getKey();
    printf("leaf,%s,%u,%u,%u,%u,%u\n", stage, i, (unsigned)k[0], (unsigned)k[1],
           (unsigned)k[2], (unsigned)it.getDepth());
  }
}

void dumpTreeNodes(const char* stage, OcTree& tree) {
  unsigned i = 0;
  for (OcTree::tree_iterator it = tree.begin_tree(), end = tree.end_tree();
       it != end; ++it, ++i) {
    OcTreeKey k = it.getKey();
    printf("node,%s,%u,%u,%u,%u,%u,%d\n", stage, i, (unsigned)k[0],
           (unsigned)k[1], (unsigned)k[2], (unsigned)it.getDepth(),
           it.isLeaf() ? 1 : 0);
  }
}

}  // namespace

int main() {
  const std::vector<OcTreeKey> keys = buildKeys();

  printf("# keys inserted structurally, identical value each\n");
  printf("# key,index,x,y,z\n");
  for (size_t i = 0; i < keys.size(); ++i)
    printf("key,%zu,%u,%u,%u\n", i, (unsigned)keys[i][0], (unsigned)keys[i][1],
           (unsigned)keys[i][2]);

  printf("# counts,stage,size,calc_num_nodes,num_leaf_nodes\n");
  printf("# leaf,stage,index,x,y,z,depth\n");
  printf("# node,stage,index,x,y,z,depth,is_leaf\n");

  OcTree tree(0.1);
  for (const OcTreeKey& k : keys) tree.setNodeValue(k, 1.0f, /*lazy_eval=*/true);

  dumpCounts("inserted", tree);
  dumpLeaves("inserted", tree);
  dumpTreeNodes("inserted", tree);

  tree.prune();
  dumpCounts("pruned", tree);
  dumpLeaves("pruned", tree);

  // Delete one voxel out of pruned block A; the reference re-expands the block
  // and keeps the other seven.
  tree.deleteNode(OcTreeKey(32768, 32768, 32768));
  dumpCounts("deleted_from_pruned_block", tree);
  dumpLeaves("deleted_from_pruned_block", tree);

  // Delete a scattered singleton, which should collapse its whole ancestry.
  tree.deleteNode(OcTreeKey((uint16_t)(32768 + 5000), 32768,
                            (uint16_t)(32768 - 5000)));
  dumpCounts("deleted_singleton", tree);

  // Depth-limited leaf views.
  for (unsigned d : {2u, 6u, 10u, 14u}) {
    unsigned i = 0;
    for (OcTree::leaf_iterator it = tree.begin_leafs(d), end = tree.end_leafs();
         it != end; ++it, ++i) {
      OcTreeKey k = it.getKey();
      printf("leaf_depth,%u,%u,%u,%u,%u,%u\n", d, i, (unsigned)k[0],
             (unsigned)k[1], (unsigned)k[2], (unsigned)it.getDepth());
    }
  }

  return 0;
}
