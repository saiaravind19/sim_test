#include "fleet/planner.hpp"

#include <algorithm>
#include <cmath>

#include "RVO.h"

namespace fleet {

namespace {
float wrapPi(float a)
{
    while (a > (float)M_PI) a -= 2.f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.f * (float)M_PI;
    return a;
}
} // namespace

WorldPlanner::WorldPlanner(const bs2d::Map &map, const Params &params, uint32_t worldId)
    : grid_(map), p_(params), world_(worldId),
      sim_(new RVO::RVOSimulator()), rng_(params.seed * 7919u + worldId)
{
    sim_->setTimeStep(p_.period);
    sim_->setAgentDefaults(p_.neighborDist, (size_t)p_.maxNeighbors, p_.timeHorizon,
                           p_.timeHorizonObst, p_.radius, p_.vmax);

    // Every wall cell that touches a free cell becomes a square obstacle.
    const bs2d::Map &m = grid_.map();
    for (int32_t y = 0; y < m.height; y++) {
        for (int32_t x = 0; x < m.width; x++) {
            if (!m.occupied[(size_t)y * m.width + x]) continue;
            bool touches = false;
            for (int dy = -1; dy <= 1 && !touches; dy++)
                for (int dx = -1; dx <= 1 && !touches; dx++) {
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= m.width || ny >= m.height) continue;
                    if (!m.occupied[(size_t)ny * m.width + nx]) touches = true;
                }
            if (!touches) continue;
            float x0 = m.originX + x, y0 = m.originY + y;
            sim_->addObstacle({ RVO::Vector2(x0, y0), RVO::Vector2(x0 + 1, y0),
                                RVO::Vector2(x0 + 1, y0 + 1), RVO::Vector2(x0, y0 + 1) });
        }
    }
    // The sim treats everything outside the grid as wall, so box the grid in.
    {
        float x0 = m.originX, y0 = m.originY, x1 = x0 + m.width, y1 = y0 + m.height;
        auto box = [&](float ax, float ay, float bx, float by) {
            sim_->addObstacle({ RVO::Vector2(ax, ay), RVO::Vector2(bx, ay),
                                RVO::Vector2(bx, by), RVO::Vector2(ax, by) });
        };
        box(x0 - 2.f, y0 - 2.f, x1 + 2.f, y0);   // below
        box(x0 - 2.f, y1, x1 + 2.f, y1 + 2.f);   // above
        box(x0 - 2.f, y0, x0, y1);               // left
        box(x1, y0, x1 + 2.f, y1);               // right
    }
    sim_->processObstacles();
}

WorldPlanner::~WorldPlanner() = default;

bool WorldPlanner::goalOf(uint32_t robot, float &x, float &y) const
{
    if (robot >= robots_.size() || !robots_[robot].known) return false;
    grid_.centre(robots_[robot].goal, x, y);
    return true;
}

bool WorldPlanner::lastVelocity(uint32_t robot, float &vx, float &vy) const
{
    if (robot >= robots_.size() || !robots_[robot].known) return false;
    vx = robots_[robot].orcaVx;
    vy = robots_[robot].orcaVy;
    return true;
}

bool WorldPlanner::lastPreferred(uint32_t robot, float &vx, float &vy) const
{
    if (robot >= robots_.size() || !robots_[robot].known) return false;
    vx = robots_[robot].prefVx;
    vy = robots_[robot].prefVy;
    return true;
}

int WorldPlanner::stepsLeft(uint32_t robot, float x, float y) const
{
    if (robot >= robots_.size() || !robots_[robot].known || robots_[robot].dist.empty()) return -1;
    Cell c = grid_.cellAt(x, y);
    if (c.x < 0 || c.y < 0 || c.x >= grid_.width() || c.y >= grid_.height()) return -1;
    return robots_[robot].dist[(size_t)c.y * grid_.width() + c.x];
}

bool WorldPlanner::planPath(Robot &r, Cell from) const
{
    r.path.clear();
    r.pathIdx = 0;
    const int W = grid_.width();
    if (!grid_.movable(from.x, from.y) || r.dist[(size_t)from.y * W + from.x] < 0) return false;
    static const int dx[4] = { 0, 0, 1, -1 };
    static const int dy[4] = { 1, -1, 0, 0 };
    Cell c = from;
    r.path.push_back(c);
    while (r.dist[(size_t)c.y * W + c.x] > 0) {
        int32_t d = r.dist[(size_t)c.y * W + c.x];
        bool found = false;
        for (int k = 0; k < 4 && !found; k++) {
            int nx = c.x + dx[k], ny = c.y + dy[k];
            if (!grid_.movable(nx, ny)) continue;
            if (r.dist[(size_t)ny * W + nx] == d - 1) { c = { nx, ny }; found = true; }
        }
        if (!found) return false;
        r.path.push_back(c);
    }
    return true;
}

bool WorldPlanner::pickGoal(Robot &r, const RobotObs &o)
{
    Cell here = grid_.cellAt(o.x, o.y);
    for (int attempt = 0; attempt < 40; attempt++) {
        Cell g = grid_.randomCell(rng_);
        float gx, gy;
        grid_.centre(g, gx, gy);
        float far = std::hypot(gx - o.x, gy - o.y);
        // Relax the distance rule as attempts run out (tiny layouts).
        if (far < p_.minGoalDist * (1.f - attempt / 40.f)) continue;
        if (g.x == here.x && g.y == here.y) continue;
        grid_.distanceTo(g, r.dist);
        if (grid_.movable(here.x, here.y) &&
            r.dist[(size_t)here.y * grid_.width() + here.x] < 0) {
            continue;  // arrows do not let this robot get there
        }
        r.goal = g;
        planPath(r, here);
        return true;
    }
    return false;
}

void WorldPlanner::prefVelocity(Robot &r, const RobotObs &o, float &px, float &py) const
{
    float gx, gy;
    grid_.centre(r.goal, gx, gy);
    float tx = gx, ty = gy;

    Cell cur = grid_.cellAt(o.x, o.y);
    const int W = grid_.width();

    // Where is the robot on its stored route?
    bool onPath = false;
    if (!r.path.empty()) {
        size_t last = std::min(r.pathIdx + 4, r.path.size() - 1);
        for (size_t i = r.pathIdx; i <= last && !onPath; i++)
            if (r.path[i].x == cur.x && r.path[i].y == cur.y) { r.pathIdx = i; onPath = true; }
    }
    if (!onPath && grid_.movable(cur.x, cur.y) && !r.dist.empty() &&
        r.dist[(size_t)cur.y * W + cur.x] >= 0) {
        onPath = planPath(r, cur);   // knocked off the route: plan again from here
    }

    if (onPath && r.pathIdx + 1 < r.path.size()) {
        // Route cells ahead of the robot (current cell first).
        Cell route[5];
        int nRoute = 0;
        for (size_t i = r.pathIdx; i < r.path.size() && nRoute < 5; i++) route[nRoute++] = r.path[i];
        // Aim at the furthest of them reachable in a straight line that visits
        // route cells in order, never skipping one: cutting a corner through
        // another lane would break the arrow rules and lengthen the trip.
        auto routeIndex = [&](float px, float py, int upTo) {
            Cell q = grid_.cellAt(px, py);
            for (int i = 0; i <= upTo; i++)
                if (route[i].x == q.x && route[i].y == q.y) return i;
            return -1;
        };
        float bestX = 0, bestY = 0;
        bool have = false;
        for (int k = 1; k < nRoute; k++) {
            float cx, cy;
            grid_.centre(route[k], cx, cy);
            float len = std::hypot(cx - o.x, cy - o.y);
            int n = std::max(1, (int)std::ceil(len / 0.1f));
            bool ok = true;
            int idx = 0;
            for (int i = 0; i <= n && ok; i++) {
                float t = (float)i / n;
                int ri = routeIndex(o.x + (cx - o.x) * t, o.y + (cy - o.y) * t, k);
                if (ri < 0 || ri < idx || ri > idx + 1) ok = false; else idx = ri;
            }
            if (!ok) break;
            bestX = cx; bestY = cy; have = true;
        }
        if (!have && nRoute > 1) grid_.centre(route[1], bestX, bestY), have = true;
        if (have) { tx = bestX; ty = bestY; }
    }

    float ex = tx - o.x, ey = ty - o.y;
    float dist = std::hypot(ex, ey);
    float gd = std::hypot(gx - o.x, gy - o.y);
    // Slow down smoothly for the last stretch.
    float speed = std::min(p_.vmax, std::max(0.25f, 1.2f * std::sqrt(2.f * p_.amax * gd)));
    if (dist < 1e-4f) { px = py = 0; return; }
    px = ex / dist * speed;
    py = ey / dist * speed;
}

bool WorldPlanner::update(double t, const std::vector<RobotObs> &obs, std::vector<VelCmd> &cmds)
{
    if (t < lastT_ + p_.period - 1e-9) return false;
    lastT_ = t;
    cmds.clear();

    // Register robots we have not seen before (ids are dense: 0, 1, 2, ...).
    for (const RobotObs &o : obs) {
        if (o.id >= robots_.size()) robots_.resize(o.id + 1);
        Robot &r = robots_[o.id];
        if (!r.known) {
            r.known = true;
            r.agent = sim_->addAgent(RVO::Vector2(o.x, o.y));
            if (!pickGoal(r, o)) r.goal = grid_.cellAt(o.x, o.y);
        }
    }

    // Goals reached -> next goal; then push state and preferred velocity in.
    uint32_t stalled = 0, stoppedN = 0, hitN = 0;
    double speedSum = 0;
    for (const RobotObs &o : obs) {
        Robot &r = robots_[o.id];
        float gx, gy;
        grid_.centre(r.goal, gx, gy);
        if (std::hypot(gx - o.x, gy - o.y) < p_.goalTol) {
            stats_.goalsReached++;
            if (!pickGoal(r, o)) r.goal = grid_.cellAt(o.x, o.y);
        }
        // ORCA agent = a point `offset` ahead of the centre (differential drive).
        float w = r.havePrev ? wrapPi(o.theta - r.prevTheta) / p_.period : 0.f;
        r.prevTheta = o.theta;
        r.havePrev = true;
        float ct = std::cos(o.theta), st = std::sin(o.theta);
        sim_->setAgentPosition(r.agent, RVO::Vector2(o.x + p_.offset * ct, o.y + p_.offset * st));
        sim_->setAgentVelocity(r.agent, RVO::Vector2(o.v * ct - p_.offset * w * st,
                                                     o.v * st + p_.offset * w * ct));
        float px, py;
        prefVelocity(r, o, px, py);
        sim_->setAgentPrefVelocity(r.agent, RVO::Vector2(px, py));
        r.prefVx = px;
        r.prefVy = py;

        if (std::fabs(o.v) < 0.05f) {
            if (r.slowSince < 0) r.slowSince = t;
            if (t - r.slowSince > 5.0) stalled++;
        } else {
            r.slowSince = -1;
        }
        if (o.stopped) stoppedN++;
        if (o.hitRobot) hitN++;
        speedSum += std::fabs(o.v);
    }

    sim_->doStep();

    for (const RobotObs &o : obs) {
        Robot &r = robots_[o.id];
        RVO::Vector2 v = sim_->getAgentVelocity(r.agent);
        r.orcaVx = v.x();
        r.orcaVy = v.y();
        // Map the point's velocity back to the robot's (speed, turn rate).
        float ct = std::cos(o.theta), st = std::sin(o.theta);
        VelCmd c;
        c.robot = o.id;
        c.linear = std::max(0.f, std::min(p_.vmax, v.x() * ct + v.y() * st));
        c.angular = std::max(-p_.omegamax,
                             std::min(p_.omegamax, (-v.x() * st + v.y() * ct) / p_.offset));
        cmds.push_back(c);
    }

    stats_.steps++;
    stats_.robots = (uint32_t)obs.size();
    stats_.stalled = stalled;
    stats_.stopped = stoppedN;
    stats_.hitRobot = hitN;
    stats_.meanSpeed = obs.empty() ? 0.0 : speedSum / obs.size();
    return true;
}

} // namespace fleet
