#pragma once
// Sorting task manager for ONE world, built on BehaviorTree.CPP (v4).
//
// Every robot runs its own tree (default: trees/sorting_cycle.xml):
//
//   Configure  ->  repeat [ Queuing -> Feeding -> DumpApproval -> Dumping ]
//
//   Configure  register the robot with the task manager (once)
//   Queuing    join the shortest queue lane of a feeder, move up slot by slot,
//              until the feeding cell is free and it is this robot's turn
//   Feeding    drive onto the feeding cell, load a parcel (feed time)
//   DumpApproval  wait for approval: the task manager picks the parcel's
//              dumping cell (random, among cells no robot holds), reserves it
//              and gives its index and position (dump_x, dump_y)
//              -> state FeedingDone: the dump position goes to the path
//              planner, which plans the route there
//   Dumping    drive onto the approved dumping cell, drop the parcel (dump time)
//
// The task manager only sets goals on the WorldPlanner (external goal mode);
// the planner's route + ORCA does the driving. A feeding or dumping cell is
// freed once its robot has driven off it.
#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "fleet/planner.hpp"
#include "fleet/zones.hpp"

namespace BT { class BehaviorTreeFactory; class Tree; }

namespace fleet {

struct TaskParams {
    double feedTime = 2.0;   // sim seconds spent loading at a feeding cell
    double dumpTime = 1.0;   // sim seconds spent dropping at a dumping cell
    std::string treeFile;    // BT XML; empty = built-in default tree
    uint32_t seed = 1;
};

// Configure -> Queuing -> Feeding -> Approval -> FeedingDone -> Dumping -> Queuing ...
// Waiting: every queue lane is full. FeedingDone: dump position approved,
// the path planner is given the route to it.
enum class RobotState : uint8_t { Configure, Queuing, Feeding, Approval, FeedingDone, Dumping, Waiting };
constexpr int kRobotStates = 7;
const char *stateName(RobotState s);

struct TaskStats {
    uint64_t fed = 0;        // parcels loaded
    uint64_t approved = 0;   // dump approvals given
    uint64_t dumped = 0;     // parcels dropped
    uint32_t inState[kRobotStates] = {};   // robots per RobotState
};

class TaskManager {
public:
    TaskManager(WorldPlanner &planner, const ZoneLayout &zones, const TaskParams &params,
                uint32_t worldId);
    ~TaskManager();

    // Tick every robot's tree once. Call after planner.update() ran at sim
    // time `t`; the goals set here are used from the next control step.
    void tick(double t);

    const TaskStats &stats() const { return stats_; }
    const ZoneLayout &zones() const { return zones_; }
    RobotState state(uint32_t robot) const;

    // The default tree (also written to trees/sorting_cycle.xml).
    static const char *defaultTreeXML();

    // ---- Used by the tree nodes ----------------------------------------
    double now() const { return now_; }
    void configure(uint32_t robot);
    // Queuing: true once this robot may drive onto `feeder`.
    bool queueStep(uint32_t robot, int &feeder);
    void leaveQueue(uint32_t robot);
    // Feeding: true once the parcel is loaded.
    bool feedStep(uint32_t robot, int feeder, bool start);
    // DumpApproval: true once a dumping cell is approved and reserved for the
    // robot; `dest` = its index, (x, y) = its centre in metres.
    bool approveDump(uint32_t robot, int &dest, float &x, float &y);
    // Dumping: true once the parcel is dropped.
    bool dumpStep(uint32_t robot, int dest, bool start);
    void abortTask(uint32_t robot);

private:
    struct FeederState {
        std::deque<uint32_t> line;   // robots inside the lane, front = next to feed
        uint32_t approaching = 0;    // robots heading to the lane, not in it yet
        int occupant = -1;           // robot granted the feeding cell
    };
    struct RobotTask {
        RobotState state = RobotState::Configure;
        bool configured = false;
        int feeder = -1;             // queue joined / feeder in use
        int slot = -1;               // queue slot its goal points at
        bool inLane = false;         // has entered its feeder's lane
        bool stepOffSet = false;     // waiting: moving off a zone cell
        bool goalSet = false;
        bool dwelling = false;
        double dwellUntil = 0;
        int releaseFeeder = -1;      // free this feeder once the robot is off it
        int releaseDump = -1;        // free this dump once the robot is off it
        std::unique_ptr<BT::Tree> tree;
    };

    RobotTask &task(uint32_t robot);
    bool goTo(uint32_t robot, Cell c, const std::vector<uint8_t> &open);
    bool onCell(uint32_t robot, Cell c) const;
    void releaseCells(uint32_t robot);
    void stepOffZone(uint32_t robot);

    WorldPlanner &pl_;
    ZoneLayout zones_;
    TaskParams p_;
    uint32_t world_;
    std::unique_ptr<BT::BehaviorTreeFactory> factory_;
    std::vector<FeederState> feeders_;
    std::vector<int> dumpOwner_;     // robot that reserved each dump, -1 = free
    std::vector<RobotTask> robots_;
    std::mt19937 rng_;
    double now_ = 0;
    TaskStats stats_;
};

} // namespace fleet
