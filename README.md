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

RVO2 is downloaded by CMake the first time (needs internet access to GitHub).

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

## Layout

```
core/    planner, grid and scene code (no transport)
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
