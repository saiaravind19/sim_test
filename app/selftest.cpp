// fleet_selftest: runs the planner against a tiny built-in robot model, with no
// simulator, to tell planner problems from simulator-coupling problems.
//   holonomic  robots move exactly as ORCA says (ideal case)
//   diffdrive  robots get the planner's (linear, angular) command and move like
//              the simulator's robots (acceleration and turn-rate limits)
// If the layout has F (feeding) and D (dumping) cells, robots run the
// behaviour-tree sorting cycle instead of random goals (set NO_TASKS=1 to skip).
// Env: TREE=file.xml custom tree, DUMP=1 print every robot's state at each
// report, WALLLOG=1 print where robots touch walls.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "fleet/planner.hpp"
#include "fleet/scene.hpp"
#include "fleet/task_manager.hpp"

using namespace fleet;

int main(int argc, char **argv)
{
    if (argc < 5) {
        fprintf(stderr, "usage: %s scene.json world robots seconds [holonomic|diffdrive] [radius]\n", argv[0]);
        return 1;
    }
    SceneInfo scene;
    std::string err;
    if (!loadSceneInfo(argv[1], scene, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    int world = std::atoi(argv[2]);
    int n = std::atoi(argv[3]);
    double seconds = std::atof(argv[4]);
    bool holo = argc > 5 && !std::strcmp(argv[5], "holonomic");
    Params prm;
    if (argc > 6) prm.radius = (float)std::atof(argv[6]);
    const float simR = 0.3f, amax = 1.f, vmaxSim = 2.f, wmax = 1.f, dt = 0.04f;

    WorldPlanner pl(scene.worlds[world].map, prm, (uint32_t)world);
    const Grid &g = pl.grid();
    std::unique_ptr<TaskManager> tasks;
    ZoneLayout zl = ZoneLayout::build(g);
    if (zl.usable() && !std::getenv("NO_TASKS")) {
        TaskParams tp;
        if (std::getenv("TREE")) tp.treeFile = std::getenv("TREE");
        tasks = std::make_unique<TaskManager>(pl, zl, tp, (uint32_t)world);
        printf("sorting cycle: %s\n", zl.summary().c_str());
        for (size_t i = 0; i < zl.feeders.size(); i++) {
            printf("  feeder %zu at (%d,%d), lane:", i, zl.feeders[i].cell.x, zl.feeders[i].cell.y);
            for (Cell c : zl.feeders[i].lane) printf(" (%d,%d)", c.x, c.y);
            printf("\n");
        }
    }

    // Start robots on distinct movable cells, facing the lane direction.
    std::mt19937 rng(5);
    std::vector<RobotObs> rb;
    std::vector<char> used(g.width() * g.height(), 0);
    while ((int)rb.size() < n) {
        Cell c = g.randomCell(rng);
        if (used[c.y * g.width() + c.x]) continue;
        if (g.zone(c.x, c.y) != bs2d::ZonePlain) continue;   // start outside the zones
        used[c.y * g.width() + c.x] = 1;
        RobotObs o;
        o.id = (uint32_t)rb.size();
        g.centre(c, o.x, o.y);
        uint8_t d = g.map().dirs[c.y * g.width() + c.x];
        o.theta = (d & 1) ? 1.5708f : (d & 2) ? -1.5708f : (d & 4) ? 0.f : (d & 8) ? 3.14159f : 0.f;
        rb.push_back(o);
    }

    std::vector<VelCmd> cmds(n);
    std::vector<double> slowFor(n, 0);
    double t = 0;
    uint64_t overlapSteps = 0, wallSteps = 0;
    double nextPrint = 30;
    for (int step = 0; t < seconds; step++, t += dt) {
        std::vector<VelCmd> out;
        if (pl.update(t, rb, out)) {
            for (const VelCmd &c : out) cmds[c.robot] = c;
            if (tasks) tasks->tick(t);
        }
        for (int i = 0; i < n; i++) {
            RobotObs &o = rb[i];
            if (holo) {
                float vx = 0, vy = 0;
                pl.lastVelocity(i, vx, vy);
                o.x += vx * dt; o.y += vy * dt;
                o.v = std::hypot(vx, vy);
                if (o.v > 0.05f) o.theta = std::atan2(vy, vx);
            } else {
                float want = std::max(-vmaxSim, std::min(vmaxSim, cmds[i].linear));
                float dv = want - o.v;
                float lim = amax * dt;
                o.v += std::max(-lim, std::min(lim, dv));
                float w = std::max(-wmax, std::min(wmax, cmds[i].angular));
                o.theta += w * dt;
                o.x += o.v * std::cos(o.theta) * dt;
                o.y += o.v * std::sin(o.theta) * dt;
            }
            if (std::fabs(o.v) < 0.05f) slowFor[i] += dt; else slowFor[i] = 0;
        }
        // Count contacts the real sim would have halted on.
        for (int i = 0; i < n; i++) {
            Cell c = g.cellAt(rb[i].x, rb[i].y);
            // wall contact: circle of radius simR against any non-free cell
            bool wall = false;
            for (int dy = -1; dy <= 1 && !wall; dy++)
                for (int dx = -1; dx <= 1 && !wall; dx++) {
                    int cx = c.x + dx, cy = c.y + dy;
                    bool occ = cx < 0 || cy < 0 || cx >= g.width() || cy >= g.height() ||
                               g.map().occupied[cy * g.width() + cx];
                    if (!occ) continue;
                    float nx = std::max((float)cx, std::min(rb[i].x, (float)cx + 1));
                    float ny = std::max((float)cy, std::min(rb[i].y, (float)cy + 1));
                    if (std::hypot(rb[i].x - nx, rb[i].y - ny) < simR) wall = true;
                }
            if (wall) wallSteps++;
            if (wall && tasks && std::getenv("WALLLOG")) {
                static std::vector<int> cnt(64 * 64 * 8, 0);
                int key = (c.y * 64 + c.x) * 8 + (int)tasks->state(i);
                if (cnt[key]++ == 0)
                    printf("  wall contact: robot %d %s cell (%d,%d) pos (%.2f,%.2f) th %.2f\n", i,
                           stateName(tasks->state(i)), c.x, c.y, rb[i].x, rb[i].y, rb[i].theta);
            }
            for (int j = i + 1; j < n; j++)
                if (std::hypot(rb[i].x - rb[j].x, rb[i].y - rb[j].y) < 2 * simR) overlapSteps++;
        }
        if (std::getenv("TRACE") && step % 5 == 0 && t > 38.5 && t < 41) {
            float gx = 0, gy = 0; pl.goalOf(0, gx, gy);
            float vx = 0, vy = 0; pl.lastVelocity(0, vx, vy);
            float pvx = 0, pvy = 0; pl.lastPreferred(0, pvx, pvy);
            printf("  t=%5.1f pos=(%.2f,%.2f) goal=(%.1f,%.1f) steps left %d pref=(%.2f,%.2f) orca=(%.2f,%.2f)\n",
                   t, rb[0].x, rb[0].y, gx, gy, pl.stepsLeft(0, rb[0].x, rb[0].y), pvx, pvy, vx, vy);
        }
        if (std::getenv("TRACE") && step == 1000) {
            printf("  cell map (dirs mask ^1 v2 >4 <8 | steps left), y from 4 down to 0:\n");
            for (int y = 4; y >= 0; y--) {
                printf("   y=%d ", y);
                for (int x = 0; x < 4; x++)
                    printf(" [%2d |%3d]", g.map().dirs[y * g.width() + x],
                           pl.stepsLeft(0, x + 0.5f, y + 0.5f));
                printf("\n");
            }
        }
        if (t >= nextPrint) {
            nextPrint += 30;
            int stalled = 0;
            double sp = 0;
            for (int i = 0; i < n; i++) { if (slowFor[i] > 5.0) stalled++; sp += std::fabs(rb[i].v); }
            printf("t=%5.0fs goals=%llu (%.2f/s) mean speed %.2f stalled %d | contact steps: robot %llu wall %llu\n",
                   t, (unsigned long long)pl.stats().goalsReached, pl.stats().goalsReached / t,
                   sp / n, stalled, (unsigned long long)overlapSteps, (unsigned long long)wallSteps);
            if (tasks && std::getenv("DUMP")) {
                for (int i = 0; i < n; i++) {
                    float gx = 0, gy = 0;
                    pl.goalOf(i, gx, gy);
                    printf("    robot %2d %-9s at (%.2f,%.2f) th=%5.2f v=%.2f goal (%.1f,%.1f)%s\n", i,
                           stateName(tasks->state(i)), rb[i].x, rb[i].y, rb[i].theta, rb[i].v, gx, gy,
                           pl.atGoal(i) ? " arrived" : "");
                }
            }
            if (tasks) {
                const TaskStats &ts = tasks->stats();
                printf("         parcels fed %llu dumped %llu | configure %u queuing %u feeding %u approval %u feeding_done %u dumping %u waiting %u\n",
                       (unsigned long long)ts.fed, (unsigned long long)ts.dumped, ts.inState[0],
                       ts.inState[1], ts.inState[2], ts.inState[3], ts.inState[4], ts.inState[5], ts.inState[6]);
            }
        }
    }
    return 0;
}
