#include "fleet/scene.hpp"

#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace fleet {

bool loadSceneInfo(const std::string &path, SceneInfo &out, std::string &err)
{
    std::ifstream f(path);
    if (!f) { err = "cannot open scene file " + path; return false; }
    nlohmann::json root = nlohmann::json::parse(f, nullptr, false);
    if (root.is_discarded()) { err = "scene file is not valid JSON: " + path; return false; }
    if (!root.contains("worlds") || !root["worlds"].is_array()) {
        err = "scene file needs a \"worlds\" array";
        return false;
    }
    std::filesystem::path base = std::filesystem::path(path).parent_path();
    out.worlds.clear();
    for (const auto &wj : root["worlds"]) {
        if (!wj.contains("map")) { err = "each world needs \"map\""; return false; }
        WorldInfo w;
        std::filesystem::path mp = base / wj["map"].get<std::string>();
        if (!bs2d::loadMapCSV(mp.string(), w.map, err)) return false;
        w.name = wj.value("name", mp.stem().string());
        out.worlds.push_back(std::move(w));
    }
    return true;
}

} // namespace fleet
