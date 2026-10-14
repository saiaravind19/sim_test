#pragma once
// Goal-driven fleet planner for ONE world. It knows nothing about transports:
// feed it robot observations, it returns velocity commands. Each robot gets a
// random goal inside the layout; when the robot reaches it, the next goal is
// picked. Between goals the robot follows the shortest route on the grid
// (respecting arrow rules), and RVO2 (ORCA) bends that motion around other
// robots and walls.
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "fleet/grid.hpp"

namespace RVO { class RVOSimulator; }

namespace fleet {

struct Params {
    float period = 0.1f;          // seconds between control steps (sim time)
    // Robot limits used by the planner (keep at or below the sim's limits).
    float radius = 0.42f;         // planner safety radius (sim robots are 0.3 m)
    float vmax = 1.5f;            // m/s
    float amax = 1.0f;            // m/s^2, used to slow down near the goal
    float omegamax = 1.0f;        // rad/s
    // ORCA settings.
    float neighborDist = 3.0f;
    int maxNeighbors = 12;
    float timeHorizon = 2.5f;
    float timeHorizonObst = 1.5f;
    float goalTol = 0.35f;        // metres: the goal counts as reached
    float minGoalDist = 3.0f;     // new goals are at least this far away
    float turnGain = 2.0f;        // rad/s of turn rate per rad of heading error
    uint32_t seed = 1;
};

struct RobotObs {
    uint32_t id = 0;
    float x = 0, y = 0, theta = 0, v = 0;
    bool stopped = false;
    bool hitRobot = false;
};

struct VelCmd {
    uint32_t robot = 0;
    float linear = 0;   // m/s
    float angular = 0;  // rad/s
};

struct Stats {
    uint64_t goalsReached = 0;
    uint64_t steps = 0;       // control steps run
    uint32_t robots = 0;
    uint32_t stalled = 0;     // robots that have barely moved for 5 s
    uint32_t stopped = 0;     // robots halted by the sim (contact)
    uint32_t hitRobot = 0;    // of those, halted by touching another robot
    double meanSpeed = 0;     // m/s over all robots at the last step
};

class WorldPlanner {
public:
    WorldPlanner(const bs2d::Map &map, const Params &params, uint32_t worldId);
    ~WorldPlanner();

    // Run one control step if `t` (sim seconds) is at least one period after
    // the last one. Returns true and fills `cmds` when it ran.
    bool update(double t, const std::vector<RobotObs> &obs, std::vector<VelCmd> &cmds);

    const Stats &stats() const { return stats_; }
    const Grid &grid() const { return grid_; }

    // Goal of a robot in metres (false if the robot is unknown).
    bool goalOf(uint32_t robot, float &x, float &y) const;

private:
    struct Robot {
        size_t agent = 0;
        Cell goal;
        std::vector<int32_t> dist;   // steps to the goal from every cell
        double slowSince = -1;       // sim time it started moving slowly
        bool known = false;
    };

    bool pickGoal(Robot &r, const RobotObs &o);
    void prefVelocity(const Robot &r, const RobotObs &o, float &px, float &py) const;

    Grid grid_;
    Params p_;
    uint32_t world_;
    std::unique_ptr<RVO::RVOSimulator> sim_;
    std::vector<Robot> robots_;   // indexed by robot id
    std::mt19937 rng_;
    double lastT_ = -1e9;
    Stats stats_;
};

} // namespace fleet
