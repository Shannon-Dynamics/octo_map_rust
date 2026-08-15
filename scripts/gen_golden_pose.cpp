// Dumps quaternion and pose behavior from the C++ reference, as raw IEEE-754
// bit patterns.
//
// Build (from the repo root, after the reference is built into build-cpp/):
//   g++ -O2 -std=c++11 -I reference-cpp/octomap/include \
//       scripts/gen_golden_pose.cpp -o build-cpp/gen_golden_pose.exe \
//       -L build-cpp -loctomap -loctomath

#include <cstdio>
#include <cstring>
#include <octomap/math/Pose6D.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Vector3.h>

using octomath::Pose6D;
using octomath::Quaternion;
using octomath::Vector3;

namespace {

FILE* g_csv = NULL;
#define EMIT(...) fprintf(g_csv, __VA_ARGS__)

uint32_t bits(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

// Includes gimbal-lock-adjacent pitch values and angles past pi, where the
// matrix-and-atan2 route the reference takes is least forgiving.
const double kEuler[][3] = {
    {0.0, 0.0, 0.0},      {0.3, 0.0, 0.0},      {0.0, 0.4, 0.0},
    {0.0, 0.0, 0.5},      {0.3, -0.4, 0.5},     {-1.0, 0.2, 2.0},
    {1.5707963, 0.0, 0.0}, {0.0, 1.5707963, 0.0}, {0.0, 0.0, 3.14159265},
    {2.5, -1.4, 0.7},     {-2.9, 1.5, -3.0},    {0.1, 1.5707, 0.1},
};

const float kVectors[][3] = {
    {1.0f, 0.0f, 0.0f},  {0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, 1.0f},
    {1.0f, 2.0f, 3.0f},  {-4.5f, 0.25f, 7.5f}, {0.001f, -0.002f, 0.003f},
};

}  // namespace

int main(int argc, char** argv) {
  const char* csv = (argc > 1) ? argv[1] : "tests/golden/pose.csv";
  g_csv = fopen(csv, "w");
  if (!g_csv) {
    fprintf(stderr, "failed to open %s\n", csv);
    return 1;
  }

  // Section 1: euler -> quaternion -> euler.
  EMIT("# quat,index,u,x,y,z,roll_back,pitch_back,yaw_back\n");
  for (size_t i = 0; i < sizeof(kEuler) / sizeof(kEuler[0]); ++i) {
    Quaternion q(kEuler[i][0], kEuler[i][1], kEuler[i][2]);
    Vector3 e = q.toEuler();
    EMIT("quat,%zu,%u,%u,%u,%u,%u,%u,%u\n", i, bits(q.u()), bits(q.x()),
         bits(q.y()), bits(q.z()), bits(e.x()), bits(e.y()), bits(e.z()));
  }

  // Section 2: rotating each probe vector by each rotation.
  EMIT("# rotate,quat_index,vec_index,x,y,z\n");
  for (size_t i = 0; i < sizeof(kEuler) / sizeof(kEuler[0]); ++i) {
    Quaternion q(kEuler[i][0], kEuler[i][1], kEuler[i][2]);
    for (size_t j = 0; j < sizeof(kVectors) / sizeof(kVectors[0]); ++j) {
      Vector3 v(kVectors[j][0], kVectors[j][1], kVectors[j][2]);
      Vector3 r = q.rotate(v);
      EMIT("rotate,%zu,%zu,%u,%u,%u\n", i, j, bits(r.x()), bits(r.y()),
           bits(r.z()));
    }
  }

  // Section 3: axis-angle.
  EMIT("# axisangle,index,u,x,y,z\n");
  {
    const double angles[] = {0.0, 0.5, 1.5707963, 3.14159265, -0.9};
    const float axes[][3] = {{0, 0, 1}, {1, 0, 0}, {0, 1, 0},
                             {0.577f, 0.577f, 0.577f}, {1, 2, 3}};
    for (size_t i = 0; i < 5; ++i) {
      Quaternion q(Vector3(axes[i][0], axes[i][1], axes[i][2]), angles[i]);
      EMIT("axisangle,%zu,%u,%u,%u,%u\n", i, bits(q.u()), bits(q.x()),
           bits(q.y()), bits(q.z()));
    }
  }

  // Section 4: pose transform, inverse, and composition.
  EMIT("# pose,index,tx,ty,tz,qu,qx,qy,qz\n");
  EMIT("# transform,pose_index,vec_index,x,y,z\n");
  {
    const float trans[][3] = {{0, 0, 0}, {1, -2, 0.5f}, {10, 0, 0}, {-3.25f, 4.5f, -1.75f}};
    const double rots[][3] = {{0, 0, 0}, {0.3, -0.4, 0.5}, {0, 0, 1.5707963}, {1.0, 0.5, -2.0}};

    for (size_t i = 0; i < 4; ++i) {
      Pose6D p(trans[i][0], trans[i][1], trans[i][2], rots[i][0], rots[i][1],
               rots[i][2]);
      EMIT("pose,%zu,%u,%u,%u,%u,%u,%u,%u\n", i, bits(p.trans().x()),
           bits(p.trans().y()), bits(p.trans().z()), bits(p.rot().u()),
           bits(p.rot().x()), bits(p.rot().y()), bits(p.rot().z()));

      for (size_t j = 0; j < sizeof(kVectors) / sizeof(kVectors[0]); ++j) {
        Vector3 v(kVectors[j][0], kVectors[j][1], kVectors[j][2]);
        Vector3 r = p.transform(v);
        EMIT("transform,%zu,%zu,%u,%u,%u\n", i, j, bits(r.x()), bits(r.y()),
             bits(r.z()));
      }

      Pose6D q = p.inv();
      EMIT("inv,%zu,%u,%u,%u,%u,%u,%u,%u\n", i, bits(q.trans().x()),
           bits(q.trans().y()), bits(q.trans().z()), bits(q.rot().u()),
           bits(q.rot().x()), bits(q.rot().y()), bits(q.rot().z()));
    }

    // Composition of every ordered pair.
    EMIT("# compose,a,b,tx,ty,tz,qu,qx,qy,qz\n");
    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        Pose6D a(trans[i][0], trans[i][1], trans[i][2], rots[i][0], rots[i][1],
                 rots[i][2]);
        Pose6D b(trans[j][0], trans[j][1], trans[j][2], rots[j][0], rots[j][1],
                 rots[j][2]);
        Pose6D c = a * b;
        EMIT("compose,%zu,%zu,%u,%u,%u,%u,%u,%u,%u\n", i, j,
             bits(c.trans().x()), bits(c.trans().y()), bits(c.trans().z()),
             bits(c.rot().u()), bits(c.rot().x()), bits(c.rot().y()),
             bits(c.rot().z()));
      }
    }
  }

  fclose(g_csv);
  return 0;
}
