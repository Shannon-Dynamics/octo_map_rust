// Dumps occupancy-model behavior from the C++ reference: log-odds after each
// update, clamping, the early-abort path, lazy insertion, inner-node
// propagation, max-likelihood collapse, and change detection.
//
// Log-odds are emitted as raw IEEE-754 bit patterns, not decimal. The Rust side
// compares those bits directly, so a one-ULP divergence in the update
// arithmetic cannot hide behind decimal formatting.
//
// Build (from the repo root, after the reference is built into build-cpp/):
//   g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
//       scripts/gen_golden_occupancy.cpp -o build-cpp/gen_golden_occupancy.exe \
//       -L build-cpp -loctomap -loctomath

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <octomap/OcTree.h>
#include <vector>

using octomap::OcTree;
using octomap::OcTreeKey;

namespace {

uint32_t bits(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

const uint16_t C = 32768;

void dumpLeaves(const char* stage, OcTree& tree) {
  unsigned i = 0;
  for (OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs();
       it != end; ++it, ++i) {
    OcTreeKey k = it.getKey();
    printf("leaf,%s,%u,%u,%u,%u,%u,%u\n", stage, i, (unsigned)k[0],
           (unsigned)k[1], (unsigned)k[2], (unsigned)it.getDepth(),
           bits(it->getLogOdds()));
  }
}

}  // namespace

int main() {
  // Section 1: a long alternating sequence on one voxel, sampling the value
  // after every step. Covers accumulation, both clamps, and the early abort.
  printf("# seq,step,op,log_odds_bits,tree_size\n");
  {
    OcTree tree(0.1);
    const OcTreeKey k(C, C, C);
    // 12 hits (saturates), 3 misses, 6 hits, 20 misses (saturates), 2 hits.
    const int plan[][2] = {{1, 12}, {0, 3}, {1, 6}, {0, 20}, {1, 2}};
    unsigned step = 0;
    for (const auto& phase : plan) {
      for (int n = 0; n < phase[1]; ++n, ++step) {
        tree.updateNode(k, phase[0] != 0);
        float l = tree.search(k)->getLogOdds();
        printf("seq,%u,%d,%u,%zu\n", step, phase[0], bits(l), tree.size());
      }
    }
  }

  // Section 2: setNodeValue clamping.
  printf("# setval,input_bits,result_bits\n");
  {
    OcTree tree(0.1);
    const float inputs[] = {0.0f,   1.0f,    -1.0f,  1000.0f, -1000.0f,
                            3.5f,   3.6f,    -2.0f,  -2.1f,   0.847298f};
    for (size_t i = 0; i < sizeof(inputs) / sizeof(inputs[0]); ++i) {
      OcTreeKey k((uint16_t)(C + i), C, C);
      tree.setNodeValue(k, inputs[i]);
      printf("setval,%u,%u\n", bits(inputs[i]),
             bits(tree.search(k)->getLogOdds()));
    }
  }

  // Section 3: eager insertion of a uniform block -> auto-prune on the way up.
  printf("# counts,stage,size,num_leaf\n");
  printf("# leaf,stage,index,x,y,z,depth,log_odds_bits\n");
  {
    OcTree tree(0.1);
    for (unsigned dx = 0; dx < 2; ++dx)
      for (unsigned dy = 0; dy < 2; ++dy)
        for (unsigned dz = 0; dz < 2; ++dz)
          tree.updateNode(OcTreeKey(C + dx, C + dy, C + dz), true);
    printf("counts,eager_block,%zu,%zu\n", tree.size(), tree.getNumLeafNodes());
    dumpLeaves("eager_block", tree);

    // One miss inside the pruned block must reopen it.
    tree.updateNode(OcTreeKey(C, C, C), false);
    printf("counts,reopened,%zu,%zu\n", tree.size(), tree.getNumLeafNodes());
    dumpLeaves("reopened", tree);
  }

  // Section 4: lazy insertion, then updateInnerOccupancy, then prune.
  {
    OcTree tree(0.1);
    for (unsigned dx = 0; dx < 2; ++dx)
      for (unsigned dy = 0; dy < 2; ++dy)
        for (unsigned dz = 0; dz < 2; ++dz)
          tree.updateNode(OcTreeKey(C + dx, C + dy, C + dz), true,
                          /*lazy_eval=*/true);
    printf("counts,lazy_inserted,%zu,%zu\n", tree.size(),
           tree.getNumLeafNodes());
    // Root must still be untouched at this point.
    printf("rootval,lazy_inserted,%u\n", bits(tree.getRoot()->getLogOdds()));

    tree.updateInnerOccupancy();
    printf("rootval,after_inner_update,%u\n",
           bits(tree.getRoot()->getLogOdds()));

    tree.prune();
    printf("counts,lazy_pruned,%zu,%zu\n", tree.size(), tree.getNumLeafNodes());
  }

  // Section 5: mixed scene, inner-node propagation, then max likelihood.
  {
    OcTree tree(0.1);
    const int scene[][4] = {
        // x, y, z, occupied
        {0, 0, 0, 1},   {1, 0, 0, 0},   {0, 1, 0, 1},  {5, 5, 5, 0},
        {5, 5, 6, 0},   {-3, -3, -3, 1}, {100, 0, 0, 1}, {100, 0, 0, 1},
        {100, 0, 0, 0}, {7, 7, 7, 0},   {7, 7, 7, 0},  {7, 7, 7, 1},
    };
    for (const auto& s : scene)
      tree.updateNode(OcTreeKey((uint16_t)(C + s[0]), (uint16_t)(C + s[1]),
                                (uint16_t)(C + s[2])),
                      s[3] != 0);
    printf("counts,scene,%zu,%zu\n", tree.size(), tree.getNumLeafNodes());
    dumpLeaves("scene", tree);
    printf("rootval,scene,%u\n", bits(tree.getRoot()->getLogOdds()));

    tree.toMaxLikelihood();
    printf("counts,max_likelihood,%zu,%zu\n", tree.size(),
           tree.getNumLeafNodes());
    dumpLeaves("max_likelihood", tree);
  }

  // Section 6: change detection.
  printf("# changed,stage,x,y,z,is_new\n");
  {
    OcTree tree(0.1);
    tree.enableChangeDetection(true);

    // Fresh voxels.
    tree.updateNode(OcTreeKey(C, C, C), true);
    tree.updateNode(OcTreeKey(C + 50, C, C), false);

    std::vector<std::vector<unsigned> > rows;
    for (octomap::KeyBoolMap::const_iterator it = tree.changedKeysBegin();
         it != tree.changedKeysEnd(); ++it) {
      std::vector<unsigned> r;
      r.push_back(it->first[0]);
      r.push_back(it->first[1]);
      r.push_back(it->first[2]);
      r.push_back(it->second ? 1 : 0);
      rows.push_back(r);
    }
    std::sort(rows.begin(), rows.end());
    for (const auto& r : rows)
      printf("changed,fresh,%u,%u,%u,%u\n", r[0], r[1], r[2], r[3]);

    // Reset, then flip an existing voxel and flip it back.
    tree.resetChangeDetection();
    tree.updateNode(OcTreeKey(C + 50, C, C), true);
    printf("changed_count,after_flip,%zu\n", tree.numChangesDetected());

    tree.updateNode(OcTreeKey(C + 50, C, C), false);
    tree.updateNode(OcTreeKey(C + 50, C, C), false);
    printf("changed_count,after_flip_back,%zu\n", tree.numChangesDetected());
  }

  return 0;
}
