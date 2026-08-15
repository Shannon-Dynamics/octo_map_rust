// Builds a fixed scene with the C++ reference and writes it out as both a .ot
// and a .bt file, plus a CSV describing what those files should decode to.
//
// The Rust side then does two things with these:
//   1. reads them and checks the decoded tree against the CSV, and
//   2. writes the same scene itself and compares the bytes.
//
// Byte equality is the strong claim: if Rust emits the identical file the
// reference emits, then the reference can read Rust's output, and no C++
// toolchain is needed at `cargo test` time to know that.
//
// Build (from the repo root, after the reference is built into build-cpp/):
//   g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
//       scripts/gen_golden_io.cpp -o build-cpp/gen_golden_io.exe \
//       -L build-cpp -loctomap -loctomath

#include <cstdio>
#include <cstring>
#include <octomap/OcTree.h>

using octomap::OcTree;
using octomap::OcTreeKey;
using octomap::point3d;
using octomap::Pointcloud;

namespace {

// The reference's OCTOMAP_DEBUG writes to stdout during writeBinaryData, which
// would interleave with the CSV. Everything here goes to an explicit file
// instead, so the fixture stays clean.
FILE* g_csv = NULL;
#define EMIT(...) fprintf(g_csv, __VA_ARGS__)

uint32_t bits(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

const uint16_t C = 32768;

// Deterministic, and deliberately mixed: a run of alternating hit/miss voxels,
// a uniform block that pruning will collapse, a scattered set, and a scan whose
// rays carve free space through all of it.
void build(OcTree& tree) {
  for (unsigned i = 0; i < 24; ++i)
    tree.updateNode(OcTreeKey((uint16_t)(C + i), C, C), (i % 3) != 0);

  for (unsigned dx = 0; dx < 2; ++dx)
    for (unsigned dy = 0; dy < 2; ++dy)
      for (unsigned dz = 0; dz < 2; ++dz)
        tree.updateNode(OcTreeKey(C + 100 + dx, C + 100 + dy, C + 100 + dz), true);

  const int scattered[][3] = {{-40, 12, 7}, {300, -250, 90}, {1, -1, 1},
                              {-5000, 2000, -300}, {77, 77, 77}};
  for (const auto& s : scattered)
    tree.updateNode(OcTreeKey((uint16_t)(C + s[0]), (uint16_t)(C + s[1]),
                              (uint16_t)(C + s[2])),
                    true);

  Pointcloud scan;
  scan.push_back(point3d(1.05f, 0.05f, 0.05f));
  scan.push_back(point3d(0.05f, 1.05f, 0.05f));
  scan.push_back(point3d(-1.05f, -0.35f, 0.45f));
  scan.push_back(point3d(2.05f, 1.05f, -1.05f));
  tree.insertPointCloud(scan, point3d(0.05f, 0.05f, 0.05f), -1.0, false, false);
}

void dumpLeaves(const char* stage, OcTree& tree) {
  unsigned i = 0;
  for (OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs();
       it != end; ++it, ++i) {
    OcTreeKey k = it.getKey();
    EMIT("leaf,%s,%u,%u,%u,%u,%u,%u\n", stage, i, (unsigned)k[0],
           (unsigned)k[1], (unsigned)k[2], (unsigned)it.getDepth(),
           bits(it->getLogOdds()));
  }
}

}  // namespace

int main(int argc, char** argv) {
  const char* dir = (argc > 1) ? argv[1] : "tests/golden";
  const char* csv = (argc > 2) ? argv[2] : "tests/golden/io.csv";
  char path[512];

  g_csv = fopen(csv, "w");
  if (!g_csv) {
    fprintf(stderr, "failed to open %s for writing\n", csv);
    return 1;
  }

  OcTree tree(0.1);
  build(tree);

  EMIT("# counts,stage,size,num_leaf\n");
  EMIT("# leaf,stage,index,x,y,z,depth,log_odds_bits\n");

  // .ot captures the tree as-is, so write it before writeBinary mutates it.
  EMIT("counts,scene,%zu,%zu\n", tree.size(), tree.getNumLeafNodes());
  dumpLeaves("scene", tree);

  snprintf(path, sizeof(path), "%s/cpp_scene.ot", dir);
  if (!tree.write(path)) {
    fprintf(stderr, "failed to write %s\n", path);
    return 1;
  }

  // writeBinary collapses onto the clamps and prunes, then writes.
  snprintf(path, sizeof(path), "%s/cpp_scene.bt", dir);
  if (!tree.writeBinary(path)) {
    fprintf(stderr, "failed to write %s\n", path);
    return 1;
  }
  EMIT("counts,binary,%zu,%zu\n", tree.size(), tree.getNumLeafNodes());
  dumpLeaves("binary", tree);

  // What reading each file back yields, so the Rust reader has a target that
  // does not depend on its own writer being correct.
  {
    snprintf(path, sizeof(path), "%s/cpp_scene.ot", dir);
    octomap::AbstractOcTree* read = octomap::AbstractOcTree::read(path);
    OcTree* ot = dynamic_cast<OcTree*>(read);
    if (!ot) {
      fprintf(stderr, "failed to read back the .ot file\n");
      return 1;
    }
    EMIT("counts,ot_reloaded,%zu,%zu\n", ot->size(), ot->getNumLeafNodes());
    dumpLeaves("ot_reloaded", *ot);
    delete read;
  }
  {
    OcTree bt(0.1);
    snprintf(path, sizeof(path), "%s/cpp_scene.bt", dir);
    if (!bt.readBinary(path)) {
      fprintf(stderr, "failed to read back the .bt file\n");
      return 1;
    }
    EMIT("counts,bt_reloaded,%zu,%zu\n", bt.size(), bt.getNumLeafNodes());
    dumpLeaves("bt_reloaded", bt);
  }

  fclose(g_csv);
  return 0;
}
