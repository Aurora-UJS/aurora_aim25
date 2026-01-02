#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <yaml-cpp/yaml.h>

#include "rm_utils/logger/log.hpp"

namespace aurora {
inline YAML::Node load(const std::string &path) {
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile &e) {
    AURORA_ERROR("YAML", "Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException &e) {
    AURORA_ERROR("YAML", "Parser error: {}", e.what());
    exit(1);
  }
}

template <typename T>
inline T read(const YAML::Node &yaml, const std::string &key) {
  if (yaml[key]) return yaml[key].as<T>();
  AURORA_ERROR("YAML", "{} not found!", key);
  exit(1);
}

}  // namespace aurora

#endif  // TOOLS__YAML_HPP