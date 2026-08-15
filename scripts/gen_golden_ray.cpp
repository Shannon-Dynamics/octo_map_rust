// Dumps ray behavior from the C++ reference: DDA key sequences, ray casting,
// and point-cloud integration.
//
// Coordinates are emitted as raw IEEE-754 bit patterns so the Rust side can
// compare them exactly.
//
// Build (from the repo root, after the reference is built into build-cpp/):
//   g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
//       scripts/gen_golden_ray.cpp -o build-cpp/gen_golden_ray.exe \
//       -L build-cpp -loctomap -loctomath

#include <cstdio>
#include <cstring>
#include <octomap/OcTree.h>
#include <vector>

using octomap::KeyRay;
using octomap::OcTree;
using octomap::OcTreeKey;
using octomap::point3d;
using octomap::Pointcloud;

namespace {

uint32_t bits(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

struct RayCase {
  const char* name;
  float ox, oy, oz;
  float ex, ey, ez;
};

// Axis-aligned, diagonal, oblique, reversed, degenerate, and one pair that
// grazes voxel corners — the cases where a DDA is most likely to diverge.
const RayCase kRays[] = {
    {"same_voxel", 0.01f, 0.01f, 0.01f, 0.02f, 0.02f, 0.02f},
    {"axis_x", 0.05f, 0.05f, 0.05f, 1.05f, 0.05f, 0.05f},
    {"axis_x_neg", 0.05f, 0.05f, 0.05f, -1.05f, 0.05f, 0.05f},
    {"axis_y", 0.05f, 0.05f, 0.05f, 0.05f, 2.05f, 0.05f},
    {"axis_z_neg", 0.05f, 0.05f, 0.05f, 0.05f, 0.05f, -2.05f},
    {"diagonal", 0.05f, 0.05f, 0.05f, 1.05f, 1.05f, 1.05f},
    {"oblique", 0.05f, 0.05f, 0.05f, 1.05f, 0.35f, -0.25f},
    {"oblique_reversed", 1.05f, 0.35f, -0.25f, 0.05f, 0.05f, 0.05f},
    {"corner_graze", 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f},
    {"through_origin", -0.85f, -0.35f, -0.15f, 0.95f, 0.45f, 0.25f},
    {"tiny", 0.05f, 0.05f, 0.05f, 0.16f, 0.05f, 0.05f},
    {"long", -20.05f, -5.05f, 3.05f, 25.05f, 8.05f, -4.05f},
};

struct CastCase {
  const char* name;
  float ox, oy, oz;
  float dx, dy, dz;
  int ignore_unknown;
  double max_range;
};

const CastCase kCasts[] = {
    {"hit_ahead", 0.05f, 0.05f, 0.05f, 1.0f, 0.0f, 0.0f, 1, -1.0},
    {"hit_ahead_strict", 0.05f, 0.05f, 0.05f, 1.0f, 0.0f, 0.0f, 0, -1.0},
    {"unknown_sideways", 0.05f, 0.05f, 0.05f, 0.0f, 1.0f, 0.0f, 0, -1.0},
    {"unknown_sideways_ignored", 0.05f, 0.05f, 0.05f, 0.0f, 1.0f, 0.0f, 1, 5.0},
    {"max_range_short", 0.05f, 0.05f, 0.05f, 1.0f, 0.0f, 0.0f, 1, 0.5},
    {"backwards", 0.05f, 0.05f, 0.05f, -1.0f, 0.0f, 0.0f, 1, 3.0},
    {"diagonal_cast", 0.05f, 0.05f, 0.05f, 1.0f, 1.0f, 0.0f, 1, 5.0},
    {"from_obstacle", 2.05f, 0.05f, 0.05f, 1.0f, 0.0f, 0.0f, 1, -1.0},
};

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
  OcTree geom(0.1);

  // Section 1: raw DDA key sequences.
  printf("# raymeta,name,ok,count\n");
  printf("# ray,name,index,x,y,z\n");
  for (const RayCase& c : kRays) {
    KeyRay ray;
    bool ok = geom.computeRayKeys(point3d(c.ox, c.oy, c.oz),
                                  point3d(c.ex, c.ey, c.ez), ray);
    printf("raymeta,%s,%d,%zu\n", c.name, ok ? 1 : 0, ray.size());
    unsigned i = 0;
    for (KeyRay::iterator it = ray.begin(); it != ray.end(); ++it, ++i) {
      printf("ray,%s,%u,%u,%u,%u\n", c.name, i, (unsigned)(*it)[0],
             (unsigned)(*it)[1], (unsigned)(*it)[2]);
    }
  }

  // An out-of-bounds ray must be refused outright.
  {
    KeyRay ray;
    bool ok = geom.computeRayKeys(point3d(0.0f, 0.0f, 0.0f),
                                  point3d(1.0e9f, 0.0f, 0.0f), ray);
    printf("raymeta,out_of_bounds,%d,%zu\n", ok ? 1 : 0, ray.size());
  }

  // Section 2: ray casting against a fixed scene.
  // A wall of occupied voxels at x = 2.05, and free space leading up to it.
  printf("# cast,name,returned,end_x_bits,end_y_bits,end_z_bits\n");
  {
    OcTree tree(0.1);
    for (int dy = -3; dy <= 3; ++dy)
      for (int dz = -3; dz <= 3; ++dz)
        tree.updateNode(point3d(2.05f, 0.05f + 0.1f * dy, 0.05f + 0.1f * dz),
                        true);
    // Clear the corridor from the origin up to the wall. integrateMissOnRay is
    // protected in the reference, so drive the same effect through the public
    // API: walk the ray and mark each cell free, leaving the endpoint alone.
    {
      KeyRay corridor;
      tree.computeRayKeys(point3d(0.05f, 0.05f, 0.05f),
                          point3d(2.05f, 0.05f, 0.05f), corridor);
      for (KeyRay::iterator it = corridor.begin(); it != corridor.end(); ++it)
        tree.updateNode(*it, false);
    }

    printf("counts,cast_scene,%zu,%zu\n", tree.size(), tree.getNumLeafNodes());

    for (const CastCase& c : kCasts) {
      point3d end(0.0f, 0.0f, 0.0f);
      bool ret = tree.castRay(point3d(c.ox, c.oy, c.oz),
                              point3d(c.dx, c.dy, c.dz), end,
                              c.ignore_unknown != 0, c.max_range);
      printf("cast,%s,%d,%u,%u,%u\n", c.name, ret ? 1 : 0, bits(end.x()),
             bits(end.y()), bits(end.z()));
    }
  }

  // Section 3: insertRay, including the truncated case.
  printf("# counts,stage,size,num_leaf\n");
  {
    OcTree tree(0.1);
    tree.insertRay(point3d(0.05f, 0.05f, 0.05f), point3d(1.05f, 0.05f, 0.05f),
                   -1.0);
    printf("counts,insert_ray,%zu,%zu\n", tree.size(), tree.getNumLeafNodes());
    dumpLeaves("insert_ray", tree);

    tree.insertRay(point3d(0.05f, 0.05f, 0.05f), point3d(5.05f, 0.05f, 0.05f),
                   1.0);
    printf("counts,insert_ray_truncated,%zu,%zu\n", tree.size(),
           tree.getNumLeafNodes());
  }

  // Section 4: point-cloud integration, plain and discretized.
  {
    Pointcloud scan;
    scan.push_back(point3d(1.05f, 0.05f, 0.05f));
    scan.push_back(point3d(0.05f, 1.05f, 0.05f));
    scan.push_back(point3d(0.05f, 0.05f, 1.05f));
    scan.push_back(point3d(1.05f, 1.05f, 1.05f));
    scan.push_back(point3d(-1.05f, -0.35f, 0.45f));
    // Three points inside one voxel, to exercise discretization.
    scan.push_back(point3d(2.01f, 0.05f, 0.05f));
    scan.push_back(point3d(2.05f, 0.05f, 0.05f));
    scan.push_back(point3d(2.09f, 0.05f, 0.05f));

    const point3d origin(0.05f, 0.05f, 0.05f);

    {
      OcTree tree(0.1);
      tree.insertPointCloud(scan, origin, -1.0, false, false);
      printf("counts,cloud_plain,%zu,%zu\n", tree.size(),
             tree.getNumLeafNodes());
      dumpLeaves("cloud_plain", tree);
    }
    {
      OcTree tree(0.1);
      tree.insertPointCloud(scan, origin, -1.0, false, true);
      printf("counts,cloud_discretized,%zu,%zu\n", tree.size(),
             tree.getNumLeafNodes());
      dumpLeaves("cloud_discretized", tree);
    }
    {
      OcTree tree(0.1);
      tree.insertPointCloud(scan, origin, 1.5, false, false);
      printf("counts,cloud_maxrange,%zu,%zu\n", tree.size(),
             tree.getNumLeafNodes());
      dumpLeaves("cloud_maxrange", tree);
    }
    {
      // Lazy insertion followed by the explicit inner update and prune.
      OcTree tree(0.1);
      tree.insertPointCloud(scan, origin, -1.0, true, false);
      printf("counts,cloud_lazy,%zu,%zu\n", tree.size(),
             tree.getNumLeafNodes());
      tree.updateInnerOccupancy();
      tree.prune();
      printf("counts,cloud_lazy_finished,%zu,%zu\n", tree.size(),
             tree.getNumLeafNodes());
      dumpLeaves("cloud_lazy_finished", tree);
    }

    // computeUpdate set sizes, so the disjointness rule is pinned too.
    {
      OcTree tree(0.1);
      octomap::KeySet free_cells, occupied_cells;
      tree.computeUpdate(scan, origin, free_cells, occupied_cells, -1.0);
      printf("update,plain,%zu,%zu\n", free_cells.size(),
             occupied_cells.size());
    }
    {
      OcTree tree(0.1);
      octomap::KeySet free_cells, occupied_cells;
      tree.computeDiscreteUpdate(scan, origin, free_cells, occupied_cells,
                                 -1.0);
      printf("update,discrete,%zu,%zu\n", free_cells.size(),
             occupied_cells.size());
    }
  }

  return 0;
}
