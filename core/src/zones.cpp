#include "fleet/zones.hpp"

#include <cstdio>

namespace fleet {

ZoneLayout ZoneLayout::build(const Grid &grid)
{
    ZoneLayout z;
    const int W = grid.width(), H = grid.height();
    std::vector<uint8_t> used((size_t)W * H, 0);
    auto idx = [&](Cell c) { return (size_t)c.y * W + c.x; };

    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            if (!grid.movable(x, y)) continue;
            uint8_t t = grid.zone(x, y);
            if (t == bs2d::ZoneFeeding) z.feeders.push_back({ { x, y }, {}, {} });
            if (t == bs2d::ZoneDumping) z.dumps.push_back({ { x, y }, {} });
        }

    // Lane of each feeder: walk backwards over Q cells that drive into the
    // current cell. Prefer going straight on, so a lane does not bend into a
    // neighbouring lane.
    static const int dx[4] = { 0, 0, 1, -1 };
    static const int dy[4] = { 1, -1, 0, 0 };
    for (Feeder &f : z.feeders) {
        Cell cur = f.cell;
        int lastDir = -1;
        for (;;) {
            int best = -1;
            for (int pass = 0; pass < 2 && best < 0; pass++)
                for (int k = 0; k < 4; k++) {
                    if (pass == 0 && k != lastDir) continue;   // straight first
                    Cell p { cur.x - dx[k], cur.y - dy[k] };
                    if (grid.zone(p.x, p.y) != bs2d::ZoneQueuing) continue;
                    if (!grid.movable(p.x, p.y) || used[idx(p)]) continue;
                    if (!grid.allows(p, cur)) continue;
                    best = k;
                    break;
                }
            if (best < 0) break;
            cur = { cur.x - dx[best], cur.y - dy[best] };
            used[idx(cur)] = 1;
            f.lane.push_back(cur);
            lastDir = best;
        }
        f.open.assign((size_t)W * H, 0);
        f.open[idx(f.cell)] = 1;
        for (Cell c : f.lane) f.open[idx(c)] = 1;
    }
    z.closed.assign((size_t)W * H, 0);
    for (DumpCell &d : z.dumps) {
        d.open.assign((size_t)W * H, 0);
        d.open[idx(d.cell)] = 1;
    }
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (grid.zone(x, y) == bs2d::ZoneQueuing && !used[(size_t)y * W + x])
                z.strayQueue.push_back({ x, y });
    return z;
}

std::string ZoneLayout::summary() const
{
    std::string s = std::to_string(feeders.size()) + " feeding, " +
                    std::to_string(dumps.size()) + " dumping cells";
    size_t q = 0;
    for (const Feeder &f : feeders) q += f.lane.size();
    s += ", " + std::to_string(q) + " queue slots";
    if (!strayQueue.empty())
        s += " (" + std::to_string(strayQueue.size()) + " Q cells lead into no feeding cell)";
    return s;
}

} // namespace fleet
