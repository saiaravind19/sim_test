#include "fleet/map.hpp"

#include <fstream>
#include <vector>

namespace bs2d {

namespace {

// Split one CSV line. Commas inside double quotes do not split fields.
std::vector<std::string> splitCSV(const std::string &line)
{
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false;
    for (char c : line) {
        if (c == '"') {
            quoted = !quoted;
        } else if (c == ',' && !quoted) {
            out.push_back(cur);
            cur.clear();
        } else if (c != '\r') {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    for (auto &f : out) {
        size_t a = f.find_first_not_of(" \t");
        size_t b = f.find_last_not_of(" \t");
        f = a == std::string::npos ? "" : f.substr(a, b - a + 1);
    }
    return out;
}

enum class Cell { Empty, Free, Wall, Invalid };

// Layout rules (see docs/layout_rules.md):
//   "(a,b,c,d)"  drivable cell (arrows are kept as direction rules in Map::dirs)
//   "0"          drivable cell (legacy occupancy format)
//   "1"          wall cell (legacy occupancy format)
//   ""           empty: outside the layout, treated as wall
Cell classify(const std::string &f)
{
    if (f.empty()) return Cell::Empty;
    if (f.front() == '(' && f.back() == ')') return Cell::Free;
    if (f == "0") return Cell::Free;
    if (f == "1") return Cell::Wall;
    return Cell::Invalid;
}

} // namespace

bool loadMapCSV(const std::string &path, Map &out, std::string &err)
{
    std::ifstream f(path);
    if (!f) {
        err = "cannot open " + path;
        return false;
    }

    std::vector<std::vector<std::string>> rows;
    std::string line;
    while (std::getline(f, line)) {
        rows.push_back(splitCSV(line));
    }

    // The grid is the block of columns and rows that holds any drivable or
    // wall cell. Text columns to the right (legends) and blank rows below
    // are not part of the grid.
    int32_t width = 0;
    int32_t height = 0;
    for (size_t r = 0; r < rows.size(); r++) {
        for (size_t c = 0; c < rows[r].size(); c++) {
            Cell k = classify(rows[r][c]);
            if (k == Cell::Invalid) {
                // Text outside the grid is fine (legend); text inside it is not.
                continue;
            }
            if (k != Cell::Empty) {
                if ((int32_t)c + 1 > width) width = (int32_t)c + 1;
                if ((int32_t)r + 1 > height) height = (int32_t)r + 1;
            }
        }
    }
    if (width == 0 || height == 0) {
        err = "no drivable cells found in " + path;
        return false;
    }

    out.width = width;
    out.height = height;
    out.originX = 0.f;
    out.originY = 0.f;
    out.occupied.assign(width * height, 1); // wall unless drivable
    out.dirs.assign(width * height, 0);

    for (int32_t r = 0; r < height; r++) {
        for (int32_t c = 0; c < width; c++) {
            std::string field;
            if (r < (int32_t)rows.size() && c < (int32_t)rows[r].size()) {
                field = rows[r][c];
            }
            Cell k = classify(field);
            if (k == Cell::Invalid) {
                err = "unknown cell '" + field + "' at row " +
                      std::to_string(r + 1) + ", column " + std::to_string(c + 1) +
                      " in " + path;
                return false;
            }
            // File row 0 is the top of the layout; y grows upward.
            bool drivable = k == Cell::Free;
            out.occupied[(height - 1 - r) * width + c] = drivable ? 0 : 1;
            if (drivable) {
                uint8_t d = 0;
                for (char ch : field) {
                    if (ch == '^') d |= 1;
                    if (ch == 'v') d |= 2;
                    if (ch == '>') d |= 4;
                    if (ch == '<') d |= 8;
                }
                out.dirs[(height - 1 - r) * width + c] = d;
            }
        }
    }
    return true;
}

} // namespace bs2d
