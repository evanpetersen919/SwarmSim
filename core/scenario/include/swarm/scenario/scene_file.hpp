#pragma once

#include <string>

#include "swarm/format/scene.hpp"

namespace swarm {

// Reads a scene file (the format in swarm/format/scene.hpp) into `out`. Returns false and sets
// `error` if the file cannot be read, is not a valid scene file, or holds a scene that fails
// validate().
bool read_scene_file(const std::string& path, SceneData& out, std::string& error);

}  // namespace swarm
