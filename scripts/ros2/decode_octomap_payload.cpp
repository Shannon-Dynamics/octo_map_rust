// Decode an octomap_msgs/Octomap payload with the *C++* OctoMap library.
//
// This is the interoperability check that matters. The Rust node serialized a
// map into the `data` field of a message; if the reference implementation can
// read those bytes back and answer queries about them correctly, the two are
// speaking the same format. A Rust-side round trip could not show that — it
// would only prove the port is self-consistent.
//
// Note that the payload is *not* a .bt file: it carries no header, because the
// message puts the resolution and tree id in their own fields. So this reads it
// with readBinaryData / readData rather than readBinary / read.
//
//   g++ -O2 -o decode_octomap_payload decode_octomap_payload.cpp -loctomap -loctomath
//   ./decode_octomap_payload map.payload 0.1 binary  3.02 0.02 0.52 occupied ...

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <octomap/octomap.h>

namespace {

struct Query {
  double x, y, z;
  std::string expected;  // "occupied", "free" or "unknown"
};

std::string stateAt(const octomap::OcTree& tree, double x, double y, double z) {
  const octomap::OcTreeNode* node = tree.search(octomap::point3d(x, y, z));
  if (node == nullptr) {
    return "unknown";
  }
  return tree.isNodeOccupied(node) ? "occupied" : "free";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: " << argv[0]
              << " <payload-file> <resolution> <binary|full>"
                 " [x y z expected]...\n";
    return 2;
  }

  const std::string path = argv[1];
  const double resolution = std::atof(argv[2]);
  const bool binary = std::string(argv[3]) == "binary";

  std::vector<Query> queries;
  for (int i = 4; i + 3 < argc; i += 4) {
    queries.push_back({std::atof(argv[i]), std::atof(argv[i + 1]),
                       std::atof(argv[i + 2]), argv[i + 3]});
  }

  std::ifstream file(path, std::ios::binary);
  if (!file) {
    std::cerr << "cannot open " << path << "\n";
    return 2;
  }
  std::stringstream payload;
  payload << file.rdbuf();

  if (payload.str().empty()) {
    std::cerr << "payload is empty; the node published a map with no nodes\n";
    return 1;
  }

  // Exactly what octomap_msgs::binaryMsgToMap and fullMsgToMap do: construct
  // at the message's resolution, then read the headerless node payload.
  octomap::OcTree tree(resolution);
  if (binary) {
    tree.readBinaryData(payload);
  } else {
    tree.readData(payload);
  }

  double min_x, min_y, min_z, max_x, max_y, max_z;
  tree.getMetricMin(min_x, min_y, min_z);
  tree.getMetricMax(max_x, max_y, max_z);

  std::cout << "decoded by the C++ OctoMap library\n"
            << "  nodes:      " << tree.size() << "\n"
            << "  leaves:     " << tree.getNumLeafNodes() << "\n"
            << "  resolution: " << tree.getResolution() << "\n"
            << "  bounds:     [" << min_x << ", " << min_y << ", " << min_z
            << "] .. [" << max_x << ", " << max_y << ", " << max_z << "]\n";

  if (tree.size() == 0) {
    std::cerr << "the payload decoded to an empty tree\n";
    return 1;
  }

  int failures = 0;
  for (const Query& q : queries) {
    const std::string got = stateAt(tree, q.x, q.y, q.z);
    const bool ok = got == q.expected;
    std::cout << "  (" << q.x << ", " << q.y << ", " << q.z << ") -> " << got
              << " (expected " << q.expected << ")" << (ok ? "" : "   <-- FAIL")
              << "\n";
    if (!ok) {
      ++failures;
    }
  }

  if (failures > 0) {
    std::cerr << failures << " of " << queries.size() << " queries disagreed\n";
    return 1;
  }
  return 0;
}
