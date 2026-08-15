// Reads Rust-written map files with the C++ reference and reports what it got.
// The other half of the interoperability check; see scripts/README.md.
//
// Exits non-zero if either file fails to load or the two disagree about
// occupancy where they should agree.
//
// Build (from the repo root):
//   g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
//       scripts/verify_rust_io.cpp -o build-cpp/verify_rust_io.exe \
//       -L build-cpp -loctomap -loctomath
//
// Run:
//   cargo run --example write_scene -- tests/golden
//   ./build-cpp/verify_rust_io.exe tests/golden

#include <cstdio>
#include <cstring>
#include <octomap/OcTree.h>

using octomap::AbstractOcTree;
using octomap::OcTree;
using octomap::OcTreeKey;

int main(int argc, char** argv) {
  const char* dir = (argc > 1) ? argv[1] : "tests/golden";
  char path[512];
  int failures = 0;

  // --- .ot, written by Rust -------------------------------------------------
  snprintf(path, sizeof(path), "%s/rust_scene.ot", dir);
  AbstractOcTree* read = AbstractOcTree::read(path);
  OcTree* ot = dynamic_cast<OcTree*>(read);
  if (!ot) {
    fprintf(stderr, "FAIL: C++ could not read %s\n", path);
    return 1;
  }
  printf("ok   .ot  read by C++: %zu nodes, %zu leaves, res %g\n", ot->size(),
         ot->getNumLeafNodes(), ot->getResolution());

  // --- .bt, written by Rust -------------------------------------------------
  OcTree bt(0.1);
  snprintf(path, sizeof(path), "%s/rust_scene.bt", dir);
  if (!bt.readBinary(path)) {
    fprintf(stderr, "FAIL: C++ could not read %s\n", path);
    delete read;
    return 1;
  }
  printf("ok   .bt  read by C++: %zu nodes, %zu leaves, res %g\n", bt.size(),
         bt.getNumLeafNodes(), bt.getResolution());

  // --- the two must agree about occupancy -----------------------------------
  // .bt is the max-likelihood collapse of .ot, so every leaf the .ot marks
  // occupied must still be occupied in the .bt, and likewise for free.
  size_t checked = 0, mismatched = 0;
  for (OcTree::leaf_iterator it = ot->begin_leafs(), end = ot->end_leafs();
       it != end; ++it) {
    OcTreeKey k = it.getKey();
    OcTree::NodeType* node = bt.search(k);
    if (!node) {
      fprintf(stderr, "FAIL: key (%u,%u,%u) present in .ot, absent from .bt\n",
              (unsigned)k[0], (unsigned)k[1], (unsigned)k[2]);
      ++mismatched;
      continue;
    }
    if (ot->isNodeOccupied(*it) != bt.isNodeOccupied(node)) {
      fprintf(stderr, "FAIL: occupancy differs at (%u,%u,%u)\n", (unsigned)k[0],
              (unsigned)k[1], (unsigned)k[2]);
      ++mismatched;
    }
    ++checked;
  }
  printf("%s occupancy agrees on %zu of %zu leaves\n",
         mismatched == 0 ? "ok  " : "FAIL", checked - mismatched, checked);
  if (mismatched != 0) ++failures;

  // --- round trip through C++ ----------------------------------------------
  // Re-writing what Rust wrote must reproduce it byte for byte.
  snprintf(path, sizeof(path), "%s/cpp_rewrite.ot", dir);
  ot->write(path);
  {
    char rust_path[512];
    snprintf(rust_path, sizeof(rust_path), "%s/rust_scene.ot", dir);
    FILE* a = fopen(rust_path, "rb");
    FILE* b = fopen(path, "rb");
    if (!a || !b) {
      fprintf(stderr, "FAIL: could not reopen files for comparison\n");
      ++failures;
    } else {
      int ca, cb;
      size_t at = 0;
      bool same = true;
      do {
        ca = fgetc(a);
        cb = fgetc(b);
        if (ca != cb) {
          fprintf(stderr, "FAIL: .ot rewrite differs at byte %zu\n", at);
          same = false;
          break;
        }
        ++at;
      } while (ca != EOF && cb != EOF);
      printf("%s .ot  C++ rewrite of Rust output is byte-identical (%zu bytes)\n",
             same ? "ok  " : "FAIL", at);
      if (!same) ++failures;
    }
    if (a) fclose(a);
    if (b) fclose(b);
    remove(path);
  }

  delete read;

  if (failures == 0) {
    printf("\nAll cross-language checks passed.\n");
    return 0;
  }
  printf("\n%d check(s) failed.\n", failures);
  return 1;
}
