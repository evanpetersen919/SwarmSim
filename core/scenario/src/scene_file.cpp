#include "swarm/scenario/scene_file.hpp"

#include <cstdint>
#include <fstream>
#include <iterator>
#include <vector>

namespace swarm {

bool read_scene_file(const std::string& path, SceneData& out, std::string& error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    error = "cannot read the scene file " + path;
    return false;
  }
  const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                                        std::istreambuf_iterator<char>());
  SceneData scene;
  const SceneStatus status = parse_scene(bytes.data(), bytes.size(), scene);
  if (status != SceneStatus::kOk) {
    error = "the scene file " + path + " is not valid: " + to_string(status);
    return false;
  }
  std::string reason;
  if (!validate(scene, &reason)) {
    error = "the scene file " + path + " is not valid: " + reason;
    return false;
  }
  out = std::move(scene);
  return true;
}

}  // namespace swarm
