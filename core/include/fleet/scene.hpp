#pragma once
// Reads the parts of a simulator scene JSON the planner needs: the world
// order (world ids are indexes into "worlds") and each world's layout.
#include <string>
#include <vector>

#include "fleet/map.hpp"

namespace fleet {

struct WorldInfo {
    std::string name;
    bs2d::Map map;
};

struct SceneInfo {
    std::vector<WorldInfo> worlds;
};

bool loadSceneInfo(const std::string &path, SceneInfo &out, std::string &err);

} // namespace fleet
