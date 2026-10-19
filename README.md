# Conflict-Based Search for Multi-Agent Path Finding

A C++14 implementation of Conflict-Based Search (CBS) for coordinating multiple agents on a two-dimensional grid. Space-time A* plans individual routes; a constraint tree resolves collisions while minimizing the sum of arrival times.

The project demonstrates heuristic search, time-dependent constraints, collision detection, explicit resource limits, and regression testing against an independent joint-state Dijkstra solver.

## Example

The demo coordinates two agents on a 4 × 4 grid with blocked corners:

```text
Agent 1
(1, 0), t: 0
(2, 0), t: 1
(2, 1), t: 2
(2, 2), t: 3
(2, 3), t: 4
Agent 2
(0, 1), t: 0
(1, 1), t: 1
(1, 2), t: 2
(1, 2), t: 3
(2, 2), t: 4
(3, 2), t: 5
Sum of costs: 9
```

Equally optimal routes may differ. Waiting is a legal action and costs one time step.

## Build and run

Requirements: CMake 3.16 or newer, a C++14 compiler, and Git to restore the pinned GoogleTest dependency. GoogleTest is included as a submodule; no system-wide installation is needed.

```sh
git clone --recurse-submodules https://github.com/MateuszAmbrozy/CBS.git
cd CBS
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 2
ctest --test-dir build -C Release --output-on-failure
```

For an existing checkout, run `git submodule update --init --recursive` before configuring. CMake can also use an installed GoogleTest package when the submodule sources are absent. Configuration does not download dependencies.

Run on Linux or macOS:

```sh
./build/cbs_demo
```

Run with a Windows multi-configuration generator:

```powershell
.\build\Release\cbs_demo.exe
```

Build only the demo without GoogleTest:

```sh
cmake -S . -B build-demo -DBUILD_TESTING=OFF
cmake --build build-demo --config Release
```

## How it works

- The rectangular grid is indexed as `map.cells[x][y]`. Obstacles are static.
- At each integer time step an agent can move to an adjacent horizontal/vertical cell or wait.
- Vertex collisions and simultaneous swaps across an edge are forbidden.
- Agents remain at their goals after their final arrival; they may visit a goal and leave before committing to it.
- Constraints use unique `Agent::id` values, which need not be consecutive. Paths follow the order of `map.agents`.
- Time starts at zero. Edge constraints specify a source, a destination, and the **arrival** time.
- Cost is `sum(path.size() - 1)`: the sum of final arrival times, including waits before arrival.

CBS explores nodes in cost order. Its two branches exclude one conflicting agent's vertex occupancy or directed transition. Each child replans only the affected agent. A* searches `(x, y, time)` states with a Manhattan heuristic and owned predecessor records. Once the last finite constraint expires, a simple static route needs fewer than the number of grid cells; this gives the low-level search a finite horizon.

Each constraint-tree node is prioritized by its sum of arrival times. The test suite compares solution costs with an independent optimal solver on small grids.

## Library usage

```cpp
#include "HighLevel.h"

Map map;
map.cells = {{Cell(Position(0, 0))}, {Cell(Position(1, 0))}};
map.agents.emplace_back(42, Position(0, 0), Position(1, 0));

HighLevel solver(map);
auto paths = solver.solve();
if (solver.status() == SearchStatus::Success) {
    // paths[0] contains (0,0) at t=0 and (1,0) at t=1.
}
```

Link your target with `cbs`. `LowLevel::aStarSearch(map, agentID, constraints)` also exposes independent single-agent planning. Constraints can forbid a vertex (`Constraint(id, position, time)`) or an edge (`Constraint::edge(id, from, to, arrivalTime)`), including waiting edges.

`HighLevel::solve()` and the low-level planner expose these statuses:

| Status | Meaning |
| --- | --- |
| `NotStarted` | No completed search yet |
| `Success` | Complete solution found; a zero-agent problem has an empty successful solution |
| `NoSolution` | Infeasibility established by the search or an unavoidable endpoint collision |
| `ResourceLimit` | The configured search budget was reached; feasibility is unresolved |

Always inspect the status rather than inferring it from an empty result. Invalid grids, duplicate IDs, invalid endpoints, and malformed constraints throw `std::invalid_argument`.

`SearchLimits` defaults to 10,000 expanded high-level nodes and 1,000,000 stored states per low-level search. Pass different limits as the third `HighLevel` constructor argument or to `LowLevel`. Limits are work bounds, not wall-clock deadlines. General infeasible CBS instances can keep generating constraints, so reaching a bound is explicitly reported instead of being called “no solution.”

## Tests and memory checks

The 19 GoogleTest tests cover waiting, directed edge constraints, future goal restrictions, revisits, failed branches, input validation, resource statuses, multiple solver instances, and collision-free coordination of three agents. The solution validator checks endpoints, obstacles, time indices, legal steps, supplied constraints, vertex conflicts, and edge swaps after padding completed paths at their goals.

An independent joint-state Dijkstra solver checks optimal costs for all 144 distinct-start/distinct-goal two-agent scenarios on a 2 × 2 grid, plus more than 900 feasible scenarios on 3 × 2 grids with and without obstacles.

Run AddressSanitizer, LeakSanitizer, and UndefinedBehaviorSanitizer with GCC or Clang on a Unix platform:

```sh
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug -DCBS_ENABLE_SANITIZERS=ON
cmake --build build-sanitize --parallel 2
ctest --test-dir build-sanitize --output-on-failure
```

[GitHub Actions](.github/workflows/ci.yml) runs the test suite on Linux, Windows, and macOS, with an additional Linux sanitizer job.

## Project layout

| Path | Responsibility |
| --- | --- |
| `lib/LowLevel.*` | Space-time A*, map validation, and single-agent feasibility |
| `lib/HighLevel.*` | CBS branching, affected-agent replanning, and search statuses |
| `lib/ConstraintTree.*` | Path costs and stateless conflict detection |
| `lib/util.h` | Grid, agent, constraint, conflict, and limit types |
| `main.cpp` | Self-contained example scenario |
| `gtest/tests.cpp` | Regression tests, solution validator, and independent optimality oracle |

## Scope

Scenarios are configured in C++. The solver supports rectangular grids, static obstacles, and unit-cost moves. CBS can require substantial search as congestion and agent count grow; configurable search limits bound the work. Performance on large maps has not been benchmarked.

## License

This repository does not currently specify a project license. GoogleTest is distributed under its [BSD 3-Clause license](googletest/LICENSE).
