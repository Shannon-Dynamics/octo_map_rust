// C++ side of the reference regression comparison. Measures the same six
// operations the Rust benchmark measures, on the same points, so that the two
// runs can be checked against each other — first for building an identical
// tree, and then as regression baselines.
//
// Maintainer tooling. Nothing here is built or run by `cargo test`.
//
// The scene is read from tests/bench/scene.txt, written by the Rust generator
// (crates/octomap-core/benches/shared/fixture.rs). There is deliberately no
// second generator here — see docs/05-regression-baselines.md.
//
// Timing mirrors what criterion does on the Rust side closely enough to be
// comparable: warm-up iterations are discarded, then N timed iterations are
// collected and the MEDIAN is reported. Min and max come along so an outlier
// is visible rather than averaged away.
//
// Build (from the repo root, after the reference is built into build-cpp/):
//   g++ -O3 -DNDEBUG -std=c++11 -I reference-cpp/octomap/include \
//       scripts/bench_cpp.cpp -o build-cpp/bench_cpp.exe \
//       -L build-cpp -loctomap -loctomath
//
// Run:
//   cargo run --release --example dump_bench_fixture
//   ./build-cpp/bench_cpp.exe tests/bench/scene.txt

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

using octomap::OcTree;
using octomap::OcTreeKey;
using octomap::point3d;
using octomap::Pointcloud;

namespace {

// ---------------------------------------------------------------- fixture --

const char* kFormatTag = "octomap-bench-fixture 1";

struct Scene {
  double resolution = 0.0;
  point3d sensor;
  std::vector<point3d> scan;
  std::vector<point3d> queries;
  std::vector<point3d> directions;
};

uint32_t bits(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

// Wrapping sum over the raw f32 bits of every coordinate. The Rust side
// computes the identical value; if the two disagree, the benchmarks are not
// measuring the same scene and the comparison is void.
uint64_t checksum(const Scene& s) {
  uint64_t sum = 0;
  const std::vector<point3d>* groups[3] = {&s.scan, &s.queries, &s.directions};
  for (int g = 0; g < 3; ++g) {
    for (size_t i = 0; i < groups[g]->size(); ++i) {
      const point3d& p = (*groups[g])[i];
      sum += static_cast<uint64_t>(bits(p.x()));
      sum += static_cast<uint64_t>(bits(p.y()));
      sum += static_cast<uint64_t>(bits(p.z()));
    }
  }
  return sum;
}

// Reads the next line that is neither blank nor a comment.
bool nextLine(std::istream& in, std::string& line) {
  while (std::getline(in, line)) {
    // Tolerate CRLF, since the fixture may be written on Windows.
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
    if (!line.empty() && line[0] != '#') return true;
  }
  return false;
}

bool readGroup(std::istream& in, const char* label, std::vector<point3d>& out) {
  std::string line;
  if (!nextLine(in, line)) {
    fprintf(stderr, "fixture: missing %s\n", label);
    return false;
  }
  std::istringstream header(line);
  std::string key;
  size_t count = 0;
  header >> key >> count;
  if (key != label) {
    fprintf(stderr, "fixture: expected %s, found '%s'\n", label, key.c_str());
    return false;
  }

  out.clear();
  out.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    if (!nextLine(in, line)) {
      fprintf(stderr, "fixture: %s ran out at %zu of %zu\n", label, i, count);
      return false;
    }
    std::istringstream ss(line);
    float x, y, z;
    ss >> x >> y >> z;
    out.push_back(point3d(x, y, z));
  }
  return true;
}

bool loadScene(const char* path, Scene& scene) {
  std::ifstream in(path);
  if (!in.is_open()) {
    fprintf(stderr, "cannot open %s\n", path);
    fprintf(stderr, "run: cargo run --release --example dump_bench_fixture\n");
    return false;
  }

  std::string line;
  if (!nextLine(in, line) || line != kFormatTag) {
    fprintf(stderr, "fixture: expected '%s', found '%s'\n", kFormatTag, line.c_str());
    return false;
  }

  std::string key;
  if (!nextLine(in, line)) return false;
  { std::istringstream ss(line); ss >> key >> scene.resolution; }
  if (key != "resolution") { fprintf(stderr, "fixture: missing resolution\n"); return false; }

  if (!nextLine(in, line)) return false;
  {
    std::istringstream ss(line);
    float x, y, z;
    ss >> key >> x >> y >> z;
    if (key != "sensor") { fprintf(stderr, "fixture: missing sensor\n"); return false; }
    scene.sensor = point3d(x, y, z);
  }

  if (!readGroup(in, "scan", scene.scan)) return false;
  if (!readGroup(in, "queries", scene.queries)) return false;
  if (!readGroup(in, "directions", scene.directions)) return false;

  if (!nextLine(in, line)) { fprintf(stderr, "fixture: missing checksum\n"); return false; }
  uint64_t declared = 0;
  { std::istringstream ss(line); ss >> key >> declared; }
  if (key != "checksum") { fprintf(stderr, "fixture: missing checksum\n"); return false; }

  const uint64_t actual = checksum(scene);
  if (declared != actual) {
    fprintf(stderr, "fixture: checksum mismatch, file says %llu but points hash to %llu\n",
            (unsigned long long)declared, (unsigned long long)actual);
    return false;
  }
  return true;
}

// ----------------------------------------------------------------- timing --

typedef std::chrono::steady_clock Clock;

/// Warm-up runs are discarded, then `samples` timed runs are collected.
/// Matches the shape of what criterion does closely enough to compare.
const int kWarmup = 3;

struct Timing {
  double median_ns;
  double min_ns;
  double max_ns;
  int samples;
};

Timing summarize(std::vector<double>& ns) {
  std::sort(ns.begin(), ns.end());
  Timing t;
  t.samples = static_cast<int>(ns.size());
  t.min_ns = ns.front();
  t.max_ns = ns.back();
  const size_t mid = ns.size() / 2;
  t.median_ns = (ns.size() % 2 == 0) ? (ns[mid - 1] + ns[mid]) / 2.0 : ns[mid];
  return t;
}

// Keeps the optimizer from deleting work whose result is unused. `volatile`
// forces the store, the same job criterion's black_box does on the Rust side.
volatile uint64_t g_sink = 0;

template <typename Body>
Timing measure(int samples, Body body) {
  for (int i = 0; i < kWarmup; ++i) body();

  std::vector<double> ns;
  ns.reserve(samples);
  for (int i = 0; i < samples; ++i) {
    const Clock::time_point t0 = Clock::now();
    body();
    const Clock::time_point t1 = Clock::now();
    ns.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count());
  }
  return summarize(ns);
}

void report(const char* name, size_t elements, const Timing& t) {
  // Machine-readable, so the comparison table is transcribed rather than
  // retyped.
  printf("result,%s,%zu,%d,%.0f,%.0f,%.0f\n", name, elements, t.samples,
         t.median_ns, t.min_ns, t.max_ns);
  fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
  const char* path = (argc > 1) ? argv[1] : "tests/bench/scene.txt";

  Scene scene;
  if (!loadScene(path, scene)) return 1;

  printf("# bench_cpp\n");
  printf("# compiler        g++ %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#ifdef _OPENMP
  printf("# openmp          ENABLED (_OPENMP=%d) -- NOT single-threaded\n", _OPENMP);
#else
  printf("# openmp          disabled -- single-threaded\n");
#endif
#ifdef NDEBUG
  printf("# assertions      disabled (NDEBUG)\n");
#else
  printf("# assertions      ENABLED -- this is not a release build\n");
#endif
  printf("# fixture         %s\n", path);
  printf("# resolution      %g\n", scene.resolution);
  printf("# scan            %zu\n", scene.scan.size());
  printf("# queries         %zu\n", scene.queries.size());
  printf("# directions      %zu\n", scene.directions.size());
  printf("# checksum        %llu\n", (unsigned long long)checksum(scene));
  printf("# format          result,name,elements,samples,median_ns,min_ns,max_ns\n");
  fflush(stdout);

  Pointcloud cloud;
  cloud.reserve(scene.scan.size());
  for (size_t i = 0; i < scene.scan.size(); ++i) cloud.push_back(scene.scan[i]);

  const double res = scene.resolution;
  const point3d origin = scene.sensor;
  const int kInsertSamples = 20;
  const int kQuerySamples = 50;

  // --- insertion ------------------------------------------------------------
  // The tree is constructed and destroyed inside the timed region, matching
  // the Rust benchmark, which builds a fresh OcTree per iteration and drops it
  // before the closure returns.

  report("insert_eager", scene.scan.size(), measure(kInsertSamples, [&]() {
           OcTree tree(res);
           tree.insertPointCloud(cloud, origin, -1.0, false, false);
           g_sink += tree.size();
         }));

  report("insert_lazy_then_inner", scene.scan.size(), measure(kInsertSamples, [&]() {
           OcTree tree(res);
           tree.insertPointCloud(cloud, origin, -1.0, true, false);
           tree.updateInnerOccupancy();
           g_sink += tree.size();
         }));

  // Supplementary: the Rust "lazy_then_finish" row also calls prune(). The
  // operation-mapping table for this task does not, so both are measured and
  // the report names which one is like-for-like.
  report("insert_lazy_then_inner_and_prune", scene.scan.size(),
         measure(kInsertSamples, [&]() {
           OcTree tree(res);
           tree.insertPointCloud(cloud, origin, -1.0, true, false);
           tree.updateInnerOccupancy();
           tree.prune();
           g_sink += tree.size();
         }));

  report("insert_discretized", scene.scan.size(), measure(kInsertSamples, [&]() {
           OcTree tree(res);
           tree.insertPointCloud(cloud, origin, -1.0, false, true);
           g_sink += tree.size();
         }));

  // --- queries --------------------------------------------------------------
  // Built once, outside the timed region, exactly as the Rust benchmark does.

  OcTree map(res);
  map.insertPointCloud(cloud, origin, -1.0, false, false);
  printf("# populated map   %zu nodes, %zu leaves\n", map.size(), map.getNumLeafNodes());
  fflush(stdout);

  const std::vector<point3d>& queries = scene.queries;
  std::vector<OcTreeKey> keys;
  keys.reserve(queries.size());
  for (size_t i = 0; i < queries.size(); ++i) {
    OcTreeKey k;
    if (map.coordToKeyChecked(queries[i], k)) keys.push_back(k);
  }
  printf("# addressable keys %zu of %zu queries\n", keys.size(), queries.size());
  fflush(stdout);

  report("query_by_coordinate", queries.size(), measure(kQuerySamples, [&]() {
           size_t hits = 0;
           for (size_t i = 0; i < queries.size(); ++i) {
             OcTree::NodeType* node = map.search(queries[i]);
             if (node && map.isNodeOccupied(node)) ++hits;
           }
           g_sink += hits;
         }));

  report("query_by_key", keys.size(), measure(kQuerySamples, [&]() {
           size_t hits = 0;
           for (size_t i = 0; i < keys.size(); ++i) {
             OcTree::NodeType* node = map.search(keys[i]);
             if (node && map.isNodeOccupied(node)) ++hits;
           }
           g_sink += hits;
         }));

  // --- ray casting ----------------------------------------------------------
  // ignoreUnknown = true and maxRange = -1 to match the Rust call. Both differ
  // from castRay's C++ defaults (false, -1.0), so they are passed explicitly.

  const std::vector<point3d>& directions = scene.directions;
  report("cast_ray", directions.size(), measure(kQuerySamples, [&]() {
           size_t hits = 0;
           point3d end;
           for (size_t i = 0; i < directions.size(); ++i) {
             if (map.castRay(origin, directions[i], end, true, -1.0)) ++hits;
           }
           g_sink += hits;
         }));

  printf("# done, sink=%llu\n", (unsigned long long)g_sink);
  return 0;
}
