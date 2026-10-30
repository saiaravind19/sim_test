#pragma once
// Sorting zones read from the layout's zone tags:
//   F  feeding cell   one robot at a time loads a parcel here
//   Q  queuing cell   ordered slots in front of a feeding cell
//   D  dumping cell   the parcel is dropped here (one destination per cell)
// Each feeding cell owns a queue lane: the chain of Q cells that leads into it,
// found by following the arrows backwards from the feeding cell.
#include <string>
#include <vector>

#include "fleet/grid.hpp"

namespace fleet {

struct Feeder {
    Cell cell;
    std::vector<Cell> lane;          // lane[0] is the slot next to the feeding cell
    std::vector<uint8_t> open;       // zone cells a route to this feeder may use
};

struct DumpCell {
    Cell cell;
    std::vector<uint8_t> open;       // zone cells a route to this dump may use
};

struct ZoneLayout {
    std::vector<Feeder> feeders;
    std::vector<DumpCell> dumps;
    std::vector<Cell> strayQueue;    // Q cells that lead into no feeding cell
    std::vector<uint8_t> closed;     // no zone cells open (routes on plain cells)

    // Find feeders, their lanes and dump cells in a grid.
    static ZoneLayout build(const Grid &grid);
    // Usable for the sorting cycle: at least one feeder and one dump.
    bool usable() const { return !feeders.empty() && !dumps.empty(); }
    std::string summary() const;
};

} // namespace fleet
