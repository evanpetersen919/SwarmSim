#pragma once

#include <fstream>
#include <iterator>
#include <string>

#include "swarm/experiment/experiment.hpp"

namespace swarm::testing {

// Reads a file under a directory, for load_experiment.
inline swarm::ReadFile read_from(const std::string& directory) {
  return [directory](const std::string& path, std::string& text) {
    std::ifstream in(directory + "/" + path, std::ios::binary);
    if (!in) {
      return false;
    }
    text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
  };
}

}  // namespace swarm::testing
