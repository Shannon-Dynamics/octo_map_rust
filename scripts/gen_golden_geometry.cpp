// Dumps coordinate/key conversion results from the C++ reference so the Rust
// port can be diffed against them. Emits CSV on stdout; see
// scripts/README.md for how to regenerate.
//
// Build (from the repo root, after the reference is built into build-cpp/):
//   g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
//       scripts/gen_golden_geometry.cpp -o build-cpp/gen_golden_geometry \
//       -L build-cpp -loctomap -loctomath

#include <cstdio>
#include <octomap/OcTree.h>
#include <vector>

using octomap::OcTree;
using octomap::OcTreeKey;
using octomap::point3d;

int main() {
  const double resolutions[] = {0.1, 0.05, 0.02, 1.0};
  const double coords[] = {0.0,    0.05,    -0.05,  0.1,     -0.1,   1.2,
                           -1.2,   1.25,    1.35,   -0.001,  0.001,  123.456,
                           -98.7,  3276.7,  -3276.8, 0.3,    -0.3,   2.0,
                           -2.0,   99.999,  -99.999, 7.5,    -7.5};
  const unsigned depths[] = {16, 15, 14, 12, 8, 4, 1, 0};

  // Section 1: unchecked single-axis coordToKey / keyToCoord round trip.
  printf("# section,resolution,coord,key,center\n");
  for (double res : resolutions) {
    OcTree tree(res);
    for (double c : coords) {
      octomap::key_type k = tree.coordToKey(c);
      double center = tree.keyToCoord(k);
      printf("coord_to_key,%.17g,%.17g,%u,%.17g\n", res, c, (unsigned)k, center);
    }
  }

  // Section 2: depth-aware conversion and key adjustment.
  printf("# section,resolution,coord,depth,key_at_depth,adjusted,center_at_depth\n");
  for (double res : resolutions) {
    OcTree tree(res);
    for (double c : coords) {
      for (unsigned d : depths) {
        octomap::key_type kd = tree.coordToKey(c, d);
        octomap::key_type adj = tree.adjustKeyAtDepth(tree.coordToKey(c), d);
        double center = tree.keyToCoord(kd, d);
        printf("coord_to_key_depth,%.17g,%.17g,%u,%u,%u,%.17g\n", res, c, d,
               (unsigned)kd, (unsigned)adj, center);
      }
    }
  }

  // Section 3: bounds checking.
  printf("# section,resolution,coord,valid,key\n");
  for (double res : resolutions) {
    OcTree tree(res);
    const double half = 32768.0 * res;
    const double probes[] = {0.0,        half - res, half,        half + res,
                             -half,      -half - res, 1e9,        -1e9};
    for (double c : probes) {
      octomap::key_type k = 0;
      bool ok = tree.coordToKeyChecked(c, k);
      printf("checked,%.17g,%.17g,%d,%u\n", res, c, ok ? 1 : 0, (unsigned)k);
    }
  }

  // Section 4: node sizes per depth.
  printf("# section,resolution,depth,node_size\n");
  for (double res : resolutions) {
    OcTree tree(res);
    for (unsigned d = 0; d <= 16; ++d) {
      printf("node_size,%.17g,%u,%.17g\n", res, d, tree.getNodeSize(d));
    }
  }

  // Section 5: sensor model defaults, as log-odds.
  {
    OcTree tree(0.1);
    printf("# section,name,value\n");
    printf("sensor,prob_hit_log,%.17g\n", (double)tree.getProbHitLog());
    printf("sensor,prob_miss_log,%.17g\n", (double)tree.getProbMissLog());
    printf("sensor,occ_thres_log,%.17g\n", (double)tree.getOccupancyThresLog());
    printf("sensor,clamping_min_log,%.17g\n", (double)tree.getClampingThresMinLog());
    printf("sensor,clamping_max_log,%.17g\n", (double)tree.getClampingThresMaxLog());
    printf("sensor,prob_hit,%.17g\n", tree.getProbHit());
    printf("sensor,prob_miss,%.17g\n", tree.getProbMiss());
    printf("sensor,occ_thres,%.17g\n", tree.getOccupancyThres());
    printf("sensor,clamping_min,%.17g\n", tree.getClampingThresMin());
    printf("sensor,clamping_max,%.17g\n", tree.getClampingThresMax());
  }

  return 0;
}
