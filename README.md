# cpp-planning

Motion planning algorithms implemented from scratch in modern C++20, built toward a
custom global planner for the ROS 2 Nav2 stack.

This is a learning-in-public repository. Each algorithm is written from the primary
source rather than ported from an existing library, and each comes with tests. The
library itself is ROS-free and testable on its own. ROS 2 integration will be a thin
adapter on top of it in the future.

## Demo

Breadth-first search around an obstacle (`S` start, `G` goal, `*` path, `#` wall):

```
S*********
..####...*
..#...#..*
..#...#..*
..#####..*
.........G
```

Every planner on the same map, where `~` is mud costing 5× open ground. BFS
minimises steps and goes straight through the mud; the cost-aware planners detour
around it:

```
............
.~~~~~~~~~~.
S~~~~~~~~~~G      start (0,2) → goal (11,2)
.~~~~~~~~~~.
............
```

```
planner                  cells      cost  expansions   inserts  removals      peak
--------------------------------------------------------------------------------
BFS                         12       510          50        53         0         7
A*                          16       150          18        30         0        14
Weighted A*                 16       150          25        49         0        26
Dijkstra (4-conn, heap)     16       150          49        57         0        15
Dijkstra (8-conn, heap)     14       138          46        83        12        33
Dijkstra (4-conn, set)      16       150          48        57         0        15
Dijkstra (8-conn, set)      14       138          46        83        25        18
```

Costs are integers: 10 per orthogonal step and 14 per diagonal, times the terrain
cost of the cell entered. `removals` counts stale entries discarded by the heap
frontier, or decrease-key erases for the ordered-set frontier.

## Build

Requires CMake 3.16+ and a C++20 compiler. Tested with Apple clang 17 and GCC 15.
The first configure downloads GoogleTest.

```bash
git clone https://github.com/mjones57/cpp-planning.git
cd cpp-planning
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
(cd build && ctest)
./build/planner_compare
```

With AddressSanitizer and UndefinedBehaviorSanitizer:

```bash
cmake -S . -B build-asan -DENABLE_SANITIZERS=ON
cmake --build build-asan
(cd build-asan && ctest)
```

Add `-DBUILD_TESTING_PLANNING=OFF` to skip the tests and the GoogleTest download.

### Demos

| Executable | Shows |
|---|---|
| `bfs_demo` | BFS on a small map, plus an unreachable goal |
| `planner_compare` | every planner on the mud map above |
| `frontier_compare` | Dijkstra's two frontier implementations on terrain with varying cost |
| `node_pool_demo` | building a 21-node chain in `NodePool` and walking the leaf back to the root |

## Using the library

```cpp
#include "planning/grid_map.hpp"
#include "planning/planner_factory.hpp"

const planning::GridMap map = planning::make_grid({
    "..........",
    "..####....",
    "..#..#....",
    ".........."});

auto planner = planning::make_planner("astar");   // std::unique_ptr<planning::Planner>
planner->configure(map);                           // non-owning: map must outlive the planner
const auto path = planner->createPlan({0, 0}, {9, 3});
// planner->last_stats().expansions == 13
```

## Implemented

| Planner | Moves | Result | Notes |
|---|---|---|---|
| Breadth-first search | 4-connected | fewest steps; ignores terrain cost | |
| Dijkstra | 4- or 8-connected | cheapest path | heap frontier with lazy deletion, or `std::set` frontier with decrease-key |
| A\* | 4-connected | cheapest path | Manhattan distance × cheapest terrain cost (admissible) |
| Weighted A\* | 4-connected | within a factor *w* of the cheapest | same heuristic, inflated by *w* |

Building blocks for continuous-space planning (in progress, toward BIT\*):

- **`Planner`**: an abstract interface (`configure`, `createPlan`, `name`), with a
  `make_planner` factory that builds any planner from a string id.
- **`NodePool<State>`**: tree storage for sampling-based planners: add, rewire, prune,
  and walk a node back to the root.
- **`Point2`**: a continuous state, with conversions to and from grid cells.
- **`CollisionChecker`**: an interface for point and segment queries, with a grid
  implementation that samples segments every quarter cell.

## Testing

30 GoogleTest tests, run under both normal and sanitizer builds. Several different
paths often share the optimal cost, so the tests don't pin exact paths. They check structural
invariants instead (valid moves, no blocked cells, correct endpoints) and cross-check
planners against each other: A\* must match Dijkstra's cost and expand no more cells,
and the heuristic is checked for admissibility against Dijkstra's true cost-to-go at
every free cell. Key tests were checked by deliberately reintroducing the bug they
target and confirming they fail.

## Layout

```
include/planning/   public headers
src/                implementations
apps/               demo executables
tests/              GoogleTest suites
```

Algorithms live in the `planning` library; `apps/` only wires them up, so everything
is testable without a `main`.

## Design notes

**Flat grid storage.** The occupancy grid is one `std::vector` in row-major order, not
a vector of vectors. That's one allocation instead of one per row, contiguous memory,
and a single integer that identifies a cell everywhere, so per-cell search state is a
plain vector rather than a hash map.

**Integer edge costs.** Diagonal steps cost 14 against 10 for orthogonal ones, rather
than √2 against 1. Comparisons stay exact and tie-breaking stays deterministic. Making
costs depend on the move direction is also what lets a cell's cost improve after it's
first discovered, which is the case lazy deletion and decrease-key exist for;
`frontier_compare` measures the two.

**A ROS-free interface shaped like Nav2's.** `Planner` mirrors the shape of
`nav2_core::GlobalPlanner`: configure once, then plan many times. Options such as the
A\* weight are set through constructors rather than default arguments on the virtual
`createPlan`, because a default argument is chosen from the static type while the body
is chosen dynamically (C++ Core Guidelines C.140).

**Indices, not pointers, in the node pool.** Parent pointers into a growing
`std::vector` dangle the moment it reallocates. In testing, a walk through dangling
pointers silently skipped 15 of 21 nodes and still ended at the root. Indices survive
reallocation. Pruning marks a node dead rather than erasing it, because erasing shifts
every later index.

**One corner-cutting rule.** A diagonal move must not squeeze between two walls that
meet at a corner. 8-connected search and segment collision checks both call the same
function for this, so planners compared on the same map follow the same rule.

## Roadmap

- [ ] BIT\* (Batch Informed Trees) in continuous space, built on `NodePool<Point2>`
      and `CollisionChecker`
- [ ] A `nav2_core::GlobalPlanner` plugin around it, benchmarked against Nav2's Smac
      Planner (Hybrid-A\*, State Lattice) and NavFn on identical costmaps
- [ ] Continuous integration on clang and GCC, including the sanitizer build

## References

- S. M. LaValle, *Planning Algorithms*, Cambridge University Press, 2006, Ch. 2
- P. E. Hart, N. J. Nilsson, B. Raphael, "A Formal Basis for the Heuristic
  Determination of Minimum Cost Paths," *IEEE Trans. Systems Science and
  Cybernetics*, 1968
- I. Pohl, "Heuristic Search Viewed as Path Finding in a Graph," *Artificial
  Intelligence*, 1970
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
- [Nav2: Writing a New Planner Plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html)

## License

MIT. See [LICENSE](LICENSE).
