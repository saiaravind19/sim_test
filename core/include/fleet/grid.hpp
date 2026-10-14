#pragma once
// Grid helpers for one world: which cells robots may use, shortest-path
// distance fields that follow the layout's arrow rules, and random goals.
#include <cstdint>
#include <random>
#include <vector>

#include "fleet/map.hpp"

namespace fleet {

struct Cell { int32_t x = 0, y = 0; };

class Grid {
public:
    explicit Grid(const bs2d::Map &map);

    int32_t width() const { return map_.width; }
    int32_t height() const { return map_.height; }
    const bs2d::Map &map() const { return map_; }

    // A cell robots may drive on. In a layout with arrows these are the cells
    // that have an arrow; otherwise every free cell.
    bool movable(int32_t x, int32_t y) const
    {
        if (x < 0 || y < 0 || x >= map_.width || y >= map_.height) return false;
        return movable_[y * map_.width + x] != 0;
    }
    int32_t movableCount() const { return (int32_t)movableCells_.size(); }
    const std::vector<Cell> &movableCells() const { return movableCells_; }
    bool hasRules() const { return hasRules_; }

    Cell cellAt(float x, float y) const;
    // Centre of a cell in metres.
    void centre(Cell c, float &x, float &y) const;

    // Steps to reach `goal` from every cell, driving only the way the arrows
    // allow (-1 = cannot reach). Fills `dist` (width * height).
    void distanceTo(Cell goal, std::vector<int32_t> &dist) const;

    // True if a strip `margin` metres to each side of the straight line only
    // crosses movable cells (so a robot of that radius can drive it).
    bool lineClear(float x0, float y0, float x1, float y1, float margin = 0.f) const;

    // A random movable cell.
    Cell randomCell(std::mt19937 &rng) const;

private:
    // Moves allowed out of a cell: bit 0 +y, bit 1 -y, bit 2 +x, bit 3 -x.
    uint8_t moves(int32_t x, int32_t y) const;

    bs2d::Map map_;
    bool hasRules_ = false;
    std::vector<uint8_t> movable_;
    std::vector<Cell> movableCells_;
};

} // namespace fleet
