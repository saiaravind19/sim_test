// rvo_planner_zmq: talks to the simulator directly over ZeroMQ.
//
//   sim state  (SUB)  /world_<w>/state   -> robot poses
//   sim cmd    (PUB)  /cmd_vel           <- velocity commands
//   sim mode   (REQ)  ControlModeRequest -> robots are put in velocity mode
//
// One planner per world (OpenMP across worlds). Pick worlds with --worlds.
#include <zmq.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "fleet/planner.hpp"
#include "fleet/scene.hpp"
#include "sim2d_msgs.pb.h"

using namespace fleet;
using Clock = std::chrono::steady_clock;

namespace {

struct Options {
    std::string scene;
    std::string host = "localhost";
    int statePort = 5555, cmdPort = 5556, svcPort = 5557;
    std::vector<int> worlds;   // empty = all
    double duration = 0;       // wall seconds, 0 = until Ctrl-C
    double report = 5.0;       // wall seconds between reports
    Params params;
};

bool decodeRobot(pb_istream_t *s, const pb_field_t *, void **arg)
{
    auto *v = (std::vector<sim2d_msgs_RobotState> *)*arg;
    sim2d_msgs_RobotState r = sim2d_msgs_RobotState_init_zero;
    if (!pb_decode(s, sim2d_msgs_RobotState_fields, &r)) return false;
    v->push_back(r);
    return true;
}

struct WorldFeed {
    bool fresh = false;
    double t = 0;
    std::vector<sim2d_msgs_RobotState> robots;
};

void usage(const char *a)
{
    fprintf(stderr,
        "usage: %s scene.json [--host H] [--worlds 0,1] [--period S] [--vmax M]\n"
        "          [--radius R] [--seed N] [--duration S] [--report S]\n"
        "          [--state-port P] [--cmd-port P] [--service-port P]\n", a);
}

bool parse(int argc, char **argv, Options &o)
{
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&](const char *name) -> const char * {
            if (i + 1 >= argc) { fprintf(stderr, "%s needs a value\n", name); return nullptr; }
            return argv[++i];
        };
        const char *v = nullptr;
        if (a == "--host") { if (!(v = next("--host"))) return false; o.host = v; }
        else if (a == "--worlds") {
            if (!(v = next("--worlds"))) return false;
            std::string s = v;
            size_t pos = 0;
            while (pos < s.size()) {
                size_t c = s.find(',', pos);
                o.worlds.push_back(std::atoi(s.substr(pos, c - pos).c_str()));
                if (c == std::string::npos) break;
                pos = c + 1;
            }
        }
        else if (a == "--period") { if (!(v = next("--period"))) return false; o.params.period = (float)std::atof(v); }
        else if (a == "--vmax") { if (!(v = next("--vmax"))) return false; o.params.vmax = (float)std::atof(v); }
        else if (a == "--radius") { if (!(v = next("--radius"))) return false; o.params.radius = (float)std::atof(v); }
        else if (a == "--seed") { if (!(v = next("--seed"))) return false; o.params.seed = (uint32_t)std::atoi(v); }
        else if (a == "--duration") { if (!(v = next("--duration"))) return false; o.duration = std::atof(v); }
        else if (a == "--report") { if (!(v = next("--report"))) return false; o.report = std::atof(v); }
        else if (a == "--state-port") { if (!(v = next("--state-port"))) return false; o.statePort = std::atoi(v); }
        else if (a == "--cmd-port") { if (!(v = next("--cmd-port"))) return false; o.cmdPort = std::atoi(v); }
        else if (a == "--service-port") { if (!(v = next("--service-port"))) return false; o.svcPort = std::atoi(v); }
        else if (!a.empty() && a[0] != '-') o.scene = a;
        else return false;
    }
    return !o.scene.empty();
}

// Ask the sim to put one robot in velocity mode. Returns true on confirmation.
class ModeClient {
public:
    ModeClient(void *ctx, std::string endpoint) : ctx_(ctx), ep_(std::move(endpoint)) { open(); }
    ~ModeClient() { if (sock_) zmq_close(sock_); }

    bool setVelocity(uint32_t world, uint32_t robot)
    {
        sim2d_msgs_ControlModeRequest rq = sim2d_msgs_ControlModeRequest_init_zero;
        rq.world_id = world;
        rq.robot_id = robot;
        rq.mode = sim2d_msgs_ControlMode_CONTROL_MODE_VELOCITY;
        uint8_t buf[128];
        pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
        if (!pb_encode(&os, sim2d_msgs_ControlModeRequest_fields, &rq)) return false;
        zmq_send(sock_, buf, os.bytes_written, 0);
        uint8_t in[256];
        int n = zmq_recv(sock_, in, sizeof(in), 0);
        if (n < 0) { open(); return false; }   // timed out: a REQ socket must be reopened
        sim2d_msgs_ControlModeResponse rs = sim2d_msgs_ControlModeResponse_init_zero;
        pb_istream_t is = pb_istream_from_buffer(in, (size_t)n);
        if (!pb_decode(&is, sim2d_msgs_ControlModeResponse_fields, &rs)) return false;
        return rs.success && rs.mode == sim2d_msgs_ControlMode_CONTROL_MODE_VELOCITY;
    }

private:
    void open()
    {
        if (sock_) zmq_close(sock_);
        sock_ = zmq_socket(ctx_, ZMQ_REQ);
        int ms = 1000, linger = 0;
        zmq_setsockopt(sock_, ZMQ_RCVTIMEO, &ms, sizeof(ms));
        zmq_setsockopt(sock_, ZMQ_SNDTIMEO, &ms, sizeof(ms));
        zmq_setsockopt(sock_, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_connect(sock_, ep_.c_str());
    }
    void *ctx_;
    std::string ep_;
    void *sock_ = nullptr;
};

} // namespace

int main(int argc, char **argv)
{
    Options opt;
    if (!parse(argc, argv, opt)) { usage(argv[0]); return 1; }

    SceneInfo scene;
    std::string err;
    if (!loadSceneInfo(opt.scene, scene, err)) {
        fprintf(stderr, "scene error: %s\n", err.c_str());
        return 1;
    }
    if (opt.worlds.empty())
        for (size_t i = 0; i < scene.worlds.size(); i++) opt.worlds.push_back((int)i);
    for (int w : opt.worlds) {
        if (w < 0 || w >= (int)scene.worlds.size()) {
            fprintf(stderr, "world %d is not in the scene (%zu worlds)\n", w, scene.worlds.size());
            return 1;
        }
    }

    std::map<int, std::unique_ptr<WorldPlanner>> planners;
    for (int w : opt.worlds) {
        planners[w] = std::make_unique<WorldPlanner>(scene.worlds[w].map, opt.params, (uint32_t)w);
        printf("world %d (%s): %d movable cells, %s\n", w, scene.worlds[w].name.c_str(),
               planners[w]->grid().movableCount(),
               planners[w]->grid().hasRules() ? "arrow rules on" : "no arrow rules");
    }

    void *ctx = zmq_ctx_new();
    void *sub = zmq_socket(ctx, ZMQ_SUB);
    void *pub = zmq_socket(ctx, ZMQ_PUB);
    zmq_connect(sub, ("tcp://" + opt.host + ":" + std::to_string(opt.statePort)).c_str());
    zmq_connect(pub, ("tcp://" + opt.host + ":" + std::to_string(opt.cmdPort)).c_str());
    for (int w : opt.worlds) {
        std::string t = "/world_" + std::to_string(w) + "/state";
        zmq_setsockopt(sub, ZMQ_SUBSCRIBE, t.data(), t.size());
    }
    ModeClient modes(ctx, "tcp://" + opt.host + ":" + std::to_string(opt.svcPort));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let the sockets connect

    std::map<int, WorldFeed> feeds;
    std::map<int, std::set<uint32_t>> inVelocity;   // robots confirmed in velocity mode
    auto start = Clock::now();
    auto lastReport = start;
    uint64_t lastGoals = 0;
    double firstSimT = -1, lastSimT = 0;

    printf("planning %zu world(s); waiting for the simulator...\n", opt.worlds.size());
    for (;;) {
        // Drain every queued state message; keep the newest per world.
        zmq_pollitem_t pi { sub, 0, ZMQ_POLLIN, 0 };
        zmq_poll(&pi, 1, 20);
        for (;;) {
            char topic[64];
            int tn = zmq_recv(sub, topic, sizeof(topic) - 1, ZMQ_DONTWAIT);
            if (tn < 0) break;
            topic[std::min(tn, 63)] = 0;
            static std::vector<uint8_t> buf(1 << 20);
            int n = zmq_recv(sub, buf.data(), buf.size(), 0);
            if (n < 0 || n > (int)buf.size()) continue;
            int w = -1;
            if (std::sscanf(topic, "/world_%d/state", &w) != 1) continue;
            WorldFeed &f = feeds[w];
            f.robots.clear();
            sim2d_msgs_WorldState ws = sim2d_msgs_WorldState_init_zero;
            ws.robots.funcs.decode = decodeRobot;
            ws.robots.arg = &f.robots;
            pb_istream_t is = pb_istream_from_buffer(buf.data(), (size_t)n);
            if (!pb_decode(&is, sim2d_msgs_WorldState_fields, &ws)) continue;
            f.t = ws.header.stamp_ms / 1000.0;
            f.fresh = true;
        }

        // Robots must be in velocity mode before the sim obeys /cmd_vel.
        for (auto &kv : feeds) {
            if (!kv.second.fresh) continue;
            for (const auto &r : kv.second.robots) {
                if (r.control_mode == sim2d_msgs_ControlMode_CONTROL_MODE_VELOCITY) continue;
                if (modes.setVelocity((uint32_t)kv.first, r.robot_id)) inVelocity[kv.first].insert(r.robot_id);
            }
        }

        // One control step per world, in parallel.
        std::vector<int> ready;
        for (int w : opt.worlds)
            if (feeds.count(w) && feeds[w].fresh) ready.push_back(w);
        std::vector<std::vector<VelCmd>> out(ready.size());
        std::vector<char> ran(ready.size(), 0);
#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < (int)ready.size(); i++) {
            WorldFeed &f = feeds[ready[i]];
            std::vector<RobotObs> obs;
            obs.reserve(f.robots.size());
            for (const auto &r : f.robots) {
                RobotObs o;
                o.id = r.robot_id;
                o.x = r.pose.x; o.y = r.pose.y; o.theta = r.pose.theta; o.v = r.v;
                o.stopped = r.stopped; o.hitRobot = r.hit_robot;
                obs.push_back(o);
            }
            ran[i] = planners[ready[i]]->update(f.t, obs, out[i]);
        }
        for (size_t i = 0; i < ready.size(); i++) {
            WorldFeed &f = feeds[ready[i]];
            f.fresh = false;
            if (firstSimT < 0) firstSimT = f.t;
            lastSimT = std::max(lastSimT, f.t);
            if (!ran[i]) continue;
            for (const VelCmd &c : out[i]) {
                sim2d_msgs_VelocityCommand vc = sim2d_msgs_VelocityCommand_init_zero;
                vc.world_id = (uint32_t)ready[i];
                vc.robot_id = c.robot;
                vc.linear = c.linear;
                vc.angular = c.angular;
                uint8_t buf[96];
                pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
                if (!pb_encode(&os, sim2d_msgs_VelocityCommand_fields, &vc)) continue;
                zmq_send(pub, "/cmd_vel", 8, ZMQ_SNDMORE);
                zmq_send(pub, buf, os.bytes_written, 0);
            }
        }

        auto now = Clock::now();
        double wall = std::chrono::duration<double>(now - start).count();
        double since = std::chrono::duration<double>(now - lastReport).count();
        if (since >= opt.report) {
            lastReport = now;
            uint64_t goals = 0;
            uint32_t robots = 0, stalled = 0, stopped = 0, hit = 0;
            double speed = 0;
            for (int w : opt.worlds) {
                const Stats &s = planners[w]->stats();
                goals += s.goalsReached;
                robots += s.robots; stalled += s.stalled; stopped += s.stopped; hit += s.hitRobot;
                speed += s.meanSpeed * s.robots;
            }
            double simSpan = lastSimT - std::max(0.0, firstSimT);
            printf("[%5.1fs wall | sim %6.1fs] robots %u | goals %llu (+%.1f/s) | mean speed %.2f m/s | stalled %u | halted by sim %u (robot contact %u)\n",
                   wall, lastSimT, robots, (unsigned long long)goals,
                   (goals - lastGoals) / since, robots ? speed / robots : 0.0, stalled, stopped, hit);
            (void)simSpan;
            if (std::getenv("FLEET_DEBUG")) {
                for (auto &kv : feeds)
                    for (const auto &r : kv.second.robots) {
                        if (kv.second.robots.size() <= 3) {
                            float gx = 0, gy = 0;
                            planners[kv.first]->goalOf(r.robot_id, gx, gy);
                            printf("  robot %u at (%.2f,%.2f) th=%.2f v=%.2f goal=(%.1f,%.1f) steps left %d\n",
                                   r.robot_id, r.pose.x, r.pose.y, r.pose.theta, r.v, gx, gy,
                                   planners[kv.first]->stepsLeft(r.robot_id, r.pose.x, r.pose.y));
                        }
                        if (r.stopped) {
                            float gx = 0, gy = 0;
                            planners[kv.first]->goalOf(r.robot_id, gx, gy);
                            printf("  halted robot %u at (%.2f,%.2f) th=%.2f v=%.2f hit_robot=%d goal=(%.1f,%.1f)\n",
                                   r.robot_id, r.pose.x, r.pose.y, r.pose.theta, r.v, r.hit_robot, gx, gy);
                        }
                    }
            }
            lastGoals = goals;
            fflush(stdout);
        }
        if (opt.duration > 0 && wall >= opt.duration) break;
    }

    uint64_t goals = 0;
    for (int w : opt.worlds) goals += planners[w]->stats().goalsReached;
    printf("done: %llu goals reached in %.1f s of simulated time\n",
           (unsigned long long)goals, lastSimT - std::max(0.0, firstSimT));
    zmq_close(sub);
    zmq_close(pub);
    zmq_ctx_term(ctx);
    return 0;
}
