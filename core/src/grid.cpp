#include "fleet/grid.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

namespace fleet {

Grid::Grid(const bs2d::Map &map) : map_(map)
{
    for (uint8_t d : map_.dirs) {
        if (d) { hasRules_ = true; break; }
    }
    movable_.assign((size_t)map_.width * map_.height, 0);
    for (int32_t y = 0; y < map_.height; y++) {
        for (int32_t x = 0; x < map_.width; x++) {
            size_t i = (size_t)y * map_.width + x;
            bool free = !map_.occupied[i];
            bool ok = free && (!hasRules_ || map_.dirs[i] != 0);
            movable_[i] = ok;
            if (ok) movableCells_.push_back({x, y});
        }
    }
}

Cell Grid::cellAt(float x, float y) const
{
    return { (int32_t)std::floor(x - map_.originX), (int32_t)std::floor(y - map_.originY) };
}

void Grid::centre(Cell c, float &x, float &y) const
{
    x = map_.originX + c.x + 0.5f;
    y = map_.originY + c.y + 0.5f;
}

uint8_t Grid::moves(int32_t x, int32_t y) const
{
    if (!hasRules_) return 15;
    return map_.dirs[(size_t)y * map_.width + x];
}

void Grid::distanceTo(Cell goal, std::vector<int32_t> &dist) const
{
    dist.assign((size_t)map_.width * map_.height, -1);
    if (!movable(goal.x, goal.y)) return;
    // Search backwards from the goal: cell n can be reached from cell p if p
    // allows the move p -> n.
    static const int dx[4] = { 0, 0, 1, -1 };
    static const int dy[4] = { 1, -1, 0, 0 };
    std::deque<Cell> q;
    dist[(size_t)goal.y * map_.width + goal.x] = 0;
    q.push_back(goal);
    while (!q.empty()) {
        Cell c = q.front();
        q.pop_front();
        int32_t d = dist[(size_t)c.y * map_.width + c.x];
        for (int k = 0; k < 4; k++) {
            // Predecessor p sits at c - dir(k) and must move in direction k.
            int32_t px = c.x - dx[k], py = c.y - dy[k];
            if (!movable(px, py)) continue;
            if (!(moves(px, py) & (1 << k))) continue;
            size_t pi = (size_t)py * map_.width + px;
            if (dist[pi] >= 0) continue;
            dist[pi] = d + 1;
            q.push_back({px, py});
        }
    }
}

bool Grid::lineClear(float x0, float y0, float x1, float y1, float margin) const
{
    float len = std::hypot(x1 - x0, y1 - y0);
    float nx = 0.f, ny = 0.f;
    if (len > 1e-6f) { nx = -(y1 - y0) / len * margin; ny = (x1 - x0) / len * margin; }
    int n = std::max(1, (int)std::ceil(len / 0.2f));
    for (int i = 0; i <= n; i++) {
        float t = (float)i / n;
        float px = x0 + (x1 - x0) * t, py = y0 + (y1 - y0) * t;
        for (int k = -1; k <= 1; k++) {
            if (k != 0 && margin <= 0.f) continue;
            Cell c = cellAt(px + nx * k, py + ny * k);
            if (!movable(c.x, c.y)) return false;
        }
    }
    return true;
}

Cell Grid::randomCell(std::mt19937 &rng) const
{
    if (movableCells_.empty()) return {};
    std::uniform_int_distribution<size_t> pick(0, movableCells_.size() - 1);
    return movableCells_[pick(rng)];
}

} // namespace fleet
