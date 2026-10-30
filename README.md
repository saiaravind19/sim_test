# sim_rvo_fleet

A goal-driven fleet planner for the `parallelverse` multi-world simulator
([multi_world_sim](https://github.com/saiaravind19/multi_world_sim)).

- Each world gets random goals inside its layout, generated while it runs.
- Every robot is driven in **velocity mode**. The planner puts robots in that
  mode itself and checks the sim's confirmation.
- A robot follows the shortest route to its goal on the grid (it respects the
  layout's arrow rules). [RVO2](https://github.com/snape/RVO2) (ORCA) bends
  that motion around other robots and walls.
- When a robot reaches its goal (within 0.35 m) it gets the next random goal.
- RVO2 returns a velocity in any direction, but the robots are differential
  drive. The planner turns it into a (forward speed, turn rate) command: it
  slows down while the heading is off and turns in place for a large error.
- One planner process can serve one world or all of them (OpenMP across
  worlds). Run several processes with `--worlds` to split a big scene.

- If a world's layout has zone cells (`F` feeding, `Q` queuing, `D` dumping),
  robots run a **sorting cycle** from a behaviour tree
  ([BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) v4)
  instead of random goals. See [Sorting cycle](#sorting-cycle).

It is a client of the simulator: it never links Madrona. The same planner core
(`core/`) has no transport, so other front ends (for example ROS 2) can be
added next to `app/zmq_planner.cpp`.

## Setup

Ubuntu / Debian:

```bash
sudo apt install git cmake g++ libzmq3-dev nlohmann-json3-dev
```

macOS: `brew install cmake zeromq nlohmann-json libomp`, then configure with
`cmake -S . -B build -DCMAKE_PREFIX_PATH=$(brew --prefix)` (libomp is optional:
without it the planner uses one thread).

RVO2 and BehaviorTree.CPP are downloaded by CMake the first time (needs
internet access to GitHub). BehaviorTree.CPP is built as a static library
with only its core (no Groot2 publisher, no SQLite logger).

```bash
git clone https://github.com/saiaravind19/sim_test.git
cd sim_test
cmake -S . -B build
cmake --build build -j
```

## Run

Start the simulator, then the planner with the **same scene file**. The planner
reads the scene for the layouts and the world order (world ids are the order of
`worlds` in the scene).

```bash
# terminal 1 (from the multi_world_sim repo)
cd build && ./parallelverse ../scenes/sorting_map_scene.json --robots 40

# terminal 2 (from this repo)
./build/rvo_planner_zmq /path/to/multi_world_sim/scenes/sorting_map_scene.json
```

Options: `--host`, `--worlds 0,1`, `--period 0.1` (seconds between control
steps), `--vmax 1.5`, `--radius 0.42` (planner safety radius; the sim robots are
0.3 m), `--seed`, `--duration S`, `--report S`, `--state-port`, `--cmd-port`,
`--service-port`. Set `FLEET_DEBUG=1` to print robots the sim halted.

The planner prints a line every few seconds: robots, goals reached, mean speed,
robots that barely moved for 5 s ("stalled"), and robots the sim halted on
contact.

Robots that appear while it runs (added in the simulator) are picked up
automatically.

Sorting-cycle options: `--tree file.xml` (your own tree; default is
`trees/sorting_cycle.xml`, built in), `--feed-time S` (default 2),
`--dump-time S` (default 1), `--random-goals` (ignore the zones).

## Sorting cycle

Tag cells in the layout CSV with a zone letter in front of the arrow tuple
(see `docs/layout_rules.md` in the simulator repo):

| Tag | Zone | Meaning |
| --- | --- | --- |
| `F(...)` | feeding | a robot loads a parcel here, one at a time |
| `Q(...)` | queuing | queue slots; a feeder's lane is the chain of `Q` cells whose arrows lead into it |
| `D(...)` | dumping | the parcel is dropped here; one destination per cell |
| `T(...)` or no tag | traversable | ordinary drivable cell |

A bare letter (`"D"`) is a cell without arrows, which may be left any way.
A world runs the sorting cycle when it has at least one `F` and one `D` cell.

Every robot runs its own tree (`trees/sorting_cycle.xml`):

```
Sequence
├── Configure                      register the robot (once)
└── KeepRunningUntilFailure
    └── Sequence
        ├── Queuing       feeder → {feeder}
        ├── Feeding       {feeder}
        ├── DumpApproval  → {destination}, {dump_x}, {dump_y}
        └── Dumping       {destination}
```

- **Configure**: registers the robot with the task manager.
- **Queuing**: reserves a place at the least loaded feeder (ties: nearest),
  drives to the tail of its lane, then moves up slot by slot. The line order
  is where robots are in the lane, so no robot has to pass another. When it
  is in the front slot and the feeding cell is free, it may feed.
- **Feeding**: drives onto the feeding cell and loads (`--feed-time`).
- **DumpApproval**: the robot waits on the feeding cell for approval. The
  task manager picks the parcel's dumping cell (random, among cells no other
  robot holds), reserves it, and outputs its index and position. The robot
  is then *feeding done*: the dump position goes to the path planner.
- **Dumping**: drives onto the approved cell and drops (`--dump-time`), then
  the cycle starts again at Queuing.

A feeding or dumping cell is freed once its robot has driven off it. Routes
only cross zone cells on the way into the robot's own lane, feeder or dump.
A robot waiting for its next step stands still (others steer around it). If
every lane is full, the robot steps off any zone cell and waits.

The report adds a line per interval: parcels fed and dumped, and how many
robots are in each state (configure, queuing, feeding, approval,
feeding_done, dumping, waiting).

The task manager (`core/include/fleet/task_manager.hpp`) only sets goals; the
planner's route + ORCA still does the driving. Nodes are registered as
`Configure`, `Queuing`, `Feeding`, `DumpApproval`, `Dumping`, so a custom tree
can rearrange or extend them.

## Layout

```
core/    planner, grid, scene and zone code (no transport); task manager
trees/   behaviour tree XML (sorting cycle)
app/     rvo_planner_zmq: the ZeroMQ front end
proto/   message definitions shared with the simulator
gen/     generated nanopb code (copy from the simulator repo)
```

If the simulator's `proto/sim2d_msgs.proto` changes, copy it and the generated
`gen/` files here.

## Known limits

- ORCA does local avoidance only. In one-way lanes of one cell, robots can
  still block each other (the "stalled" count). This is intended to be
  measured, not hidden.
- Tuned and tested against the sorting-map layout only so far.
- ROS 2 is not included yet.
- Sorting cycle: tight 90-degree turns into and out of one-cell pockets
  (a dumping cell between walls, a feeding cell exiting into an aisle) are
  where robots most often touch walls; layouts with room to turn work best.
  Robots wait in place for a free lane or a dump approval, which can block a
  1-wide aisle.
