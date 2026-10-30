#include "fleet/task_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include <behaviortree_cpp/bt_factory.h>

namespace fleet {

namespace {

const char *kDefaultTree = R"(<?xml version="1.0"?>
<!-- Sorting cycle, one tree per robot. robot_id is set on the blackboard. -->
<root BTCPP_format="4" main_tree_to_execute="SortingCycle">
  <BehaviorTree ID="SortingCycle">
    <Sequence>
      <Configure/>
      <KeepRunningUntilFailure>
        <Sequence>
          <Queuing feeder="{feeder}"/>
          <Feeding feeder="{feeder}"/>
          <DumpApproval destination="{destination}" dump_x="{dump_x}" dump_y="{dump_y}"/>
          <Dumping destination="{destination}"/>
        </Sequence>
      </KeepRunningUntilFailure>
    </Sequence>
  </BehaviorTree>
</root>
)";

uint32_t robotOf(const BT::NodeConfig &cfg)
{
    return (uint32_t)cfg.blackboard->get<int>("robot_id");
}

// Configure: register the robot (runs once, at the start of the tree).
class ConfigureNode : public BT::SyncActionNode {
public:
    ConfigureNode(const std::string &name, const BT::NodeConfig &cfg, TaskManager *tm)
        : BT::SyncActionNode(name, cfg), tm_(tm), id_(robotOf(cfg)) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override
    {
        tm_->configure(id_);
        return BT::NodeStatus::SUCCESS;
    }
private:
    TaskManager *tm_;
    uint32_t id_;
};

// Queuing: wait in a feeder's lane until it is this robot's turn.
class QueuingNode : public BT::StatefulActionNode {
public:
    QueuingNode(const std::string &name, const BT::NodeConfig &cfg, TaskManager *tm)
        : BT::StatefulActionNode(name, cfg), tm_(tm), id_(robotOf(cfg)) {}
    static BT::PortsList providedPorts() { return { BT::OutputPort<int>("feeder") }; }
    BT::NodeStatus onStart() override { return onRunning(); }
    BT::NodeStatus onRunning() override
    {
        int f = -1;
        if (!tm_->queueStep(id_, f)) return BT::NodeStatus::RUNNING;
        setOutput("feeder", f);
        return BT::NodeStatus::SUCCESS;
    }
    void onHalted() override { tm_->leaveQueue(id_); }
private:
    TaskManager *tm_;
    uint32_t id_;
};

// Feeding: drive onto the feeding cell and load a parcel.
class FeedingNode : public BT::StatefulActionNode {
public:
    FeedingNode(const std::string &name, const BT::NodeConfig &cfg, TaskManager *tm)
        : BT::StatefulActionNode(name, cfg), tm_(tm), id_(robotOf(cfg)) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("feeder") }; }
    BT::NodeStatus onStart() override { return step(true); }
    BT::NodeStatus onRunning() override { return step(false); }
    void onHalted() override { tm_->abortTask(id_); }
private:
    BT::NodeStatus step(bool start)
    {
        auto f = getInput<int>("feeder");
        if (!f) return BT::NodeStatus::FAILURE;
        return tm_->feedStep(id_, f.value(), start) ? BT::NodeStatus::SUCCESS
                                                    : BT::NodeStatus::RUNNING;
    }
    TaskManager *tm_;
    uint32_t id_;
};

// DumpApproval: ask the task manager for a dumping cell for the loaded parcel.
// RUNNING (robot waits where it is) until one is approved; then outputs the
// cell's index and its position (cell centre, metres). The cell is reserved
// for this robot until it has dropped the parcel and driven off.
class DumpApprovalNode : public BT::StatefulActionNode {
public:
    DumpApprovalNode(const std::string &name, const BT::NodeConfig &cfg, TaskManager *tm)
        : BT::StatefulActionNode(name, cfg), tm_(tm), id_(robotOf(cfg)) {}
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<int>("destination"), BT::OutputPort<double>("dump_x"),
                 BT::OutputPort<double>("dump_y") };
    }
    BT::NodeStatus onStart() override { return onRunning(); }
    BT::NodeStatus onRunning() override
    {
        int dest = -1;
        float x = 0, y = 0;
        if (!tm_->approveDump(id_, dest, x, y)) return BT::NodeStatus::RUNNING;
        setOutput("destination", dest);
        setOutput("dump_x", (double)x);
        setOutput("dump_y", (double)y);
        return BT::NodeStatus::SUCCESS;
    }
    void onHalted() override {}
private:
    TaskManager *tm_;
    uint32_t id_;
};

// Dumping: drive onto the approved dumping cell and drop the parcel.
class DumpingNode : public BT::StatefulActionNode {
public:
    DumpingNode(const std::string &name, const BT::NodeConfig &cfg, TaskManager *tm)
        : BT::StatefulActionNode(name, cfg), tm_(tm), id_(robotOf(cfg)) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("destination") }; }
    BT::NodeStatus onStart() override { return step(true); }
    BT::NodeStatus onRunning() override { return step(false); }
    void onHalted() override { tm_->abortTask(id_); }
private:
    BT::NodeStatus step(bool start)
    {
        auto d = getInput<int>("destination");
        if (!d) return BT::NodeStatus::FAILURE;
        return tm_->dumpStep(id_, d.value(), start) ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::RUNNING;
    }
    TaskManager *tm_;
    uint32_t id_;
};

} // namespace

const char *stateName(RobotState s)
{
    switch (s) {
    case RobotState::Configure: return "configure";
    case RobotState::Queuing: return "queuing";
    case RobotState::Feeding: return "feeding";
    case RobotState::Approval: return "approval";
    case RobotState::FeedingDone: return "feeding_done";
    case RobotState::Dumping: return "dumping";
    case RobotState::Waiting: return "waiting";
    }
    return "?";
}

const char *TaskManager::defaultTreeXML() { return kDefaultTree; }

TaskManager::TaskManager(WorldPlanner &planner, const ZoneLayout &zones,
                         const TaskParams &params, uint32_t worldId)
    : pl_(planner), zones_(zones), p_(params), world_(worldId),
      factory_(new BT::BehaviorTreeFactory()),
      feeders_(zones.feeders.size()), dumpOwner_(zones.dumps.size(), -1),
      rng_(params.seed * 104729u + worldId)
{
    pl_.setExternalGoals(true);
    factory_->registerNodeType<ConfigureNode>("Configure", this);
    factory_->registerNodeType<QueuingNode>("Queuing", this);
    factory_->registerNodeType<FeedingNode>("Feeding", this);
    factory_->registerNodeType<DumpApprovalNode>("DumpApproval", this);
    factory_->registerNodeType<DumpingNode>("Dumping", this);
    if (p_.treeFile.empty())
        factory_->registerBehaviorTreeFromText(kDefaultTree);
    else
        factory_->registerBehaviorTreeFromFile(p_.treeFile);
}

TaskManager::~TaskManager()
{
    // Trees hold nodes that point back at this manager: drop them first.
    for (RobotTask &t : robots_) t.tree.reset();
}

TaskManager::RobotTask &TaskManager::task(uint32_t robot)
{
    if (robot >= robots_.size()) robots_.resize(robot + 1);
    return robots_[robot];
}

RobotState TaskManager::state(uint32_t robot) const
{
    return robot < robots_.size() ? robots_[robot].state : RobotState::Configure;
}

bool TaskManager::goTo(uint32_t robot, Cell c, const std::vector<uint8_t> &open)
{
    return pl_.setGoal(robot, c, &open);
}

bool TaskManager::onCell(uint32_t robot, Cell c) const
{
    float x, y;
    if (!pl_.position(robot, x, y)) return false;
    Cell h = pl_.grid().cellAt(x, y);
    return h.x == c.x && h.y == c.y;
}

void TaskManager::releaseCells(uint32_t robot)
{
    RobotTask &t = task(robot);
    if (t.releaseFeeder >= 0 && !onCell(robot, zones_.feeders[t.releaseFeeder].cell)) {
        if (feeders_[t.releaseFeeder].occupant == (int)robot) feeders_[t.releaseFeeder].occupant = -1;
        t.releaseFeeder = -1;
    }
    if (t.releaseDump >= 0 && !onCell(robot, zones_.dumps[t.releaseDump].cell)) {
        if (dumpOwner_[t.releaseDump] == (int)robot) dumpOwner_[t.releaseDump] = -1;
        t.releaseDump = -1;
    }
}

void TaskManager::configure(uint32_t robot)
{
    RobotTask &t = task(robot);
    t.state = RobotState::Configure;
    t.configured = true;
}

bool TaskManager::queueStep(uint32_t robot, int &feeder)
{
    RobotTask &t = task(robot);
    t.state = RobotState::Queuing;

    if (t.feeder < 0) {
        // Reserve a place at the least loaded feeder with room in its lane;
        // ties go to the nearest one.
        float rx = 0, ry = 0;
        pl_.position(robot, rx, ry);
        int best = -1;
        size_t bestLoad = std::numeric_limits<size_t>::max();
        float bestDist = std::numeric_limits<float>::max();
        for (size_t i = 0; i < feeders_.size(); i++) {
            const FeederState &fs = feeders_[i];
            size_t cap = std::max<size_t>(1, zones_.feeders[i].lane.size());
            size_t queued = fs.line.size() + fs.approaching;
            if (queued >= cap) continue;
            size_t load = queued + (fs.occupant >= 0 ? 1 : 0);
            float fx, fy;
            pl_.grid().centre(zones_.feeders[i].cell, fx, fy);
            float d = std::hypot(fx - rx, fy - ry);
            if (load < bestLoad || (load == bestLoad && d < bestDist)) {
                best = (int)i; bestLoad = load; bestDist = d;
            }
        }
        if (best < 0) {   // every lane is full: wait, but not on a zone cell
            t.state = RobotState::Waiting;
            if (!t.stepOffSet) stepOffZone(robot);
            return false;
        }
        t.stepOffSet = false;
        t.feeder = best;
        t.slot = -1;
        t.inLane = false;
        t.goalSet = false;
        feeders_[best].approaching++;
    }

    FeederState &fs = feeders_[t.feeder];
    const Feeder &fz = zones_.feeders[t.feeder];
    auto grant = [&]() {
        fs.occupant = (int)robot;
        feeder = t.feeder;
        t.slot = -1;
        t.inLane = false;
        t.goalSet = false;
        return true;
    };

    // A feeder without a lane: wait where you are until the cell is free.
    if (fz.lane.empty()) {
        if (fs.occupant >= 0) return false;
        fs.approaching--;
        return grant();
    }

    // Not in the lane yet: drive to its tail. The line order is the order in
    // which robots enter the lane, so nobody has to pass anybody inside it.
    if (!t.inLane) {
        bool entered = false;
        for (Cell c : fz.lane)
            if (onCell(robot, c)) { entered = true; break; }
        if (!entered) {
            if (!t.goalSet) t.goalSet = goTo(robot, fz.lane.back(), fz.open);
            return false;
        }
        fs.approaching--;
        fs.line.push_back(robot);
        t.inLane = true;
        t.slot = -1;
    }

    // The line follows where robots really are in the lane (front = nearest the
    // feeding cell), so the order can never ask a robot to pass another one.
    auto laneIndex = [&](uint32_t r) {
        if (onCell(r, fz.cell)) return -1;
        for (size_t i = 0; i < fz.lane.size(); i++)
            if (onCell(r, fz.lane[i])) return (int)i;
        return (int)fz.lane.size();   // just outside it: at the back
    };
    std::stable_sort(fs.line.begin(), fs.line.end(),
                     [&](uint32_t a, uint32_t b) { return laneIndex(a) < laneIndex(b); });
    size_t k = std::find(fs.line.begin(), fs.line.end(), robot) - fs.line.begin();
    // Front of the line, in the front slot (or already pushed onto the
    // feeding cell), and the feeding cell is free.
    if (k == 0 && fs.occupant < 0 && (onCell(robot, fz.lane[0]) || onCell(robot, fz.cell))) {
        fs.line.pop_front();
        return grant();
    }
    // Otherwise move up to slot k (slot 0 is next to the feeding cell).
    int slot = (int)std::min(k, fz.lane.size() - 1);
    if (slot != t.slot && goTo(robot, fz.lane[slot], fz.open)) t.slot = slot;
    return false;
}

void TaskManager::stepOffZone(uint32_t robot)
{
    RobotTask &t = task(robot);
    float x, y;
    if (!pl_.position(robot, x, y)) return;
    const Grid &g = pl_.grid();
    Cell here = g.cellAt(x, y);
    if (g.zone(here.x, here.y) == bs2d::ZonePlain) { t.stepOffSet = true; return; }
    static const int dx[4] = { 0, 0, 1, -1 };
    static const int dy[4] = { 1, -1, 0, 0 };
    for (int k = 0; k < 4; k++) {
        Cell n { here.x + dx[k], here.y + dy[k] };
        if (g.zone(n.x, n.y) != bs2d::ZonePlain || !g.allows(here, n)) continue;
        if (goTo(robot, n, zones_.closed)) { t.stepOffSet = true; return; }
    }
}

void TaskManager::leaveQueue(uint32_t robot)
{
    RobotTask &t = task(robot);
    if (t.feeder >= 0) {
        FeederState &fs = feeders_[t.feeder];
        if (t.inLane)
            fs.line.erase(std::remove(fs.line.begin(), fs.line.end(), robot), fs.line.end());
        else if (fs.approaching > 0)
            fs.approaching--;
    }
    t.feeder = -1;
    t.slot = -1;
    t.inLane = false;
}

bool TaskManager::feedStep(uint32_t robot, int feeder, bool start)
{
    RobotTask &t = task(robot);
    t.state = RobotState::Feeding;
    if (start) { t.goalSet = false; t.dwelling = false; t.feeder = feeder; }
    if (feeder < 0 || feeder >= (int)zones_.feeders.size()) return false;
    const Feeder &fz = zones_.feeders[feeder];
    if (!t.goalSet) t.goalSet = goTo(robot, fz.cell, fz.open);
    if (!t.goalSet) return false;
    if (!t.dwelling) {
        if (!pl_.atGoal(robot)) return false;
        t.dwelling = true;
        t.dwellUntil = now_ + p_.feedTime;
    }
    if (now_ < t.dwellUntil) return false;
    stats_.fed++;
    t.releaseFeeder = feeder;   // freed once the robot is off the feeding cell
    t.feeder = -1;
    t.goalSet = false;
    t.dwelling = false;
    return true;
}

bool TaskManager::approveDump(uint32_t robot, int &dest, float &x, float &y)
{
    RobotTask &t = task(robot);
    t.state = RobotState::Approval;
    // The parcel's destination: a random dumping cell that no other robot
    // holds. None free -> no approval yet; the robot waits.
    std::vector<int> freeDumps;
    for (size_t i = 0; i < dumpOwner_.size(); i++)
        if (dumpOwner_[i] < 0) freeDumps.push_back((int)i);
    if (freeDumps.empty()) return false;
    std::uniform_int_distribution<size_t> pick(0, freeDumps.size() - 1);
    dest = freeDumps[pick(rng_)];
    dumpOwner_[dest] = (int)robot;
    stats_.approved++;
    pl_.grid().centre(zones_.dumps[dest].cell, x, y);
    // Feeding done: hand the dump position to the path planner now.
    t.state = RobotState::FeedingDone;
    t.goalSet = goTo(robot, zones_.dumps[dest].cell, zones_.dumps[dest].open);
    t.dwelling = false;
    return true;
}

bool TaskManager::dumpStep(uint32_t robot, int dest, bool start)
{
    RobotTask &t = task(robot);
    if (start) t.dwelling = false;
    if (dest < 0 || dest >= (int)zones_.dumps.size()) return false;
    const DumpCell &dz = zones_.dumps[dest];
    // The route was planned at approval; plan again only if that failed.
    if (!t.goalSet) t.goalSet = goTo(robot, dz.cell, dz.open);
    if (!t.goalSet) { t.state = RobotState::FeedingDone; return false; }
    t.state = RobotState::Dumping;
    if (!t.dwelling) {
        if (!pl_.atGoal(robot)) return false;
        t.dwelling = true;
        t.dwellUntil = now_ + p_.dumpTime;
    }
    if (now_ < t.dwellUntil) return false;
    stats_.dumped++;
    t.releaseDump = dest;       // freed once the robot is off the dumping cell
    t.goalSet = false;
    t.dwelling = false;
    return true;
}

void TaskManager::abortTask(uint32_t robot)
{
    RobotTask &t = task(robot);
    for (FeederState &fs : feeders_)
        if (fs.occupant == (int)robot) fs.occupant = -1;
    for (int &o : dumpOwner_)
        if (o == (int)robot) o = -1;
    t.releaseFeeder = t.releaseDump = -1;
    t.goalSet = t.dwelling = false;
}

void TaskManager::tick(double t)
{
    now_ = t;
    TaskStats &s = stats_;
    std::fill(std::begin(s.inState), std::end(s.inState), 0u);
    for (uint32_t id = 0; id < pl_.robotSlots(); id++) {
        if (!pl_.known(id)) continue;
        RobotTask &rt = task(id);
        if (!rt.tree) {
            auto bb = BT::Blackboard::create();
            bb->set("robot_id", (int)id);
            auto names = factory_->registeredBehaviorTrees();
            std::string main = std::find(names.begin(), names.end(), "SortingCycle") != names.end()
                                   ? "SortingCycle" : names.front();
            rt.tree = std::make_unique<BT::Tree>(factory_->createTree(main, bb));
        }
        releaseCells(id);
        rt.tree->tickOnce();
        s.inState[(int)rt.state]++;
    }
}

} // namespace fleet
