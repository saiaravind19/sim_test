#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace bs2d {

// Occupancy grid. Each cell is 1m x 1m. Cell (0,0) is the bottom-left corner
// of the map, at world position (originX, originY).
struct Map {
    int32_t width = 0;   // cells in x
    int32_t height = 0;  // cells in y
    float originX = 0.f;
    float originY = 0.f;
    std::vector<uint8_t> occupied; // 1 = wall, 0 = free; row-major
    // Direction rules from "(a,b,c,d)" cells, row-major like occupied:
    // bit 0 '^' (+y), bit 1 'v' (-y), bit 2 '>' (+x), bit 3 '<' (-x).
    // 0 for cells without arrows.
    std::vector<uint8_t> dirs;

    bool isOccupied(int32_t cx, int32_t cy) const
    {
        if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
            return true; // outside the map counts as wall
        }
        return occupied[cy * width + cx] != 0;
    }
};

// Load a map from CSV: one row per line, values 0 (free) or 1 (wall).
// Rows are listed top to bottom in the file; they are flipped so y grows up.
bool loadMapCSV(const std::string &path, Map &out, std::string &err);

} // namespace bs2d
