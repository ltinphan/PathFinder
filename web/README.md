# PathFinder · Vietnamese teaching edition

A buildless browser adaptation of Kevin Wang's Java/Swing [PathFinder](https://github.com/kevinwang1975/PathFinder), for the undergraduate path-planning course at [tinrobotics Teaching](https://tinrobotics.com/teaching/lap-lo-trinh-di-chuyen/).

Live simulator: <https://tinrobotics.com/teaching/lap-lo-trinh-di-chuyen/mo-phong-thuat-toan/>

## Run locally

From this repository: `python3 -m http.server 8768 --directory web`, then open <http://localhost:8768/>. ES modules require HTTP rather than opening `index.html` as a local file. No package installation, backend, telemetry, or external runtime library is needed.

Verify all five planners: `node --test web/tests/*.test.mjs` (Node 18+).

## What carries over

- Side-by-side A\* and Dijkstra on the same editable grid.
- Priority by accumulated cost plus a heuristic; Dijkstra uses a zero heuristic.
- Original edge costs: 10 orthogonal, 14 diagonal.
- Walls, start/goal selection, random maps, and search animation.

The Java source and its build remain unchanged. This is a new JavaScript implementation of the same algorithms and teaching concept, not a Java runtime embedded in the browser.

## Changes made for teaching

Added by Luong Tin Phan, November 2026, based on upstream commit `d067f683a942d3978975a24ac9e4b5fa2c418d49`:

- Vietnamese responsive UI, keyboard editing, run/pause/step, and comparable expansion counts.
- Four-neighbor Manhattan heuristic and eight-neighbor octile heuristic consistent with the 10/14 costs. The original Java evaluator uses Manhattan even with diagonal edges, which can overestimate. This edition uses an admissible, consistent heuristic by default and explains optimality conditions.
- A diagonal is blocked when **either** adjacent side cell is a wall. The original Java `XMatrix.isCrossCutBlocked` blocks it only when **both** side cells are walls. The web version intentionally prevents slipping past wall corners.
- Deterministic insertion-order tie breaking, lazy removal of stale priority-queue entries, and independently tested path costs.
- Explicit model limits: a grid point is not an Ackermann vehicle, and animation speed is not a runtime benchmark.

`search.mjs` contains the grid engine; `sampling.mjs` adds RRT, RRT\* and PRM; `app.mjs` handles canvas rendering and controls. `index.html` and `style.css` are standalone. The website carries a pinned copy of these six runtime/license files, with the matching fork commit recorded in its adjacent `SOURCE.md`. Tests and documentation stay in this repository. There is no practice ZIP/download button.

## RRT, RRT\* and PRM extension

These three algorithms were added by Luong Tin Phan; they are **not present in the upstream Java repository**. Switch modes in the UI, or use the direct link <https://tinrobotics.com/teaching/lap-lo-trinh-di-chuyen/mo-phong-thuat-toan/?mode=sampling>.

- RRT: nearest node, bounded straight-line extension, whole-segment collision checking, and connection to the goal within one extension length. Continues to the trial budget rather than stopping at the first path.
- RRT\*: same proposed samples and accepted node geometry as RRT, with least-cost parent choice and rewiring. All descendant costs are updated after a rewire; the best goal connection is recomputed. The neighbor radius is min(2η, 2√A √(ln(n+1)/(n+1))), where η is extension length and A is map area.
- PRM: free-space samples, collision-checked connections within a configurable radius, and a Dijkstra query after roadmap construction. S and G are included in the graph from the beginning. This teaching UI rebuilds the roadmap after endpoint/map changes; a production multi-query PRM would retain a valid roadmap.

All samples are continuous points. A seed, trial budget, RRT extension length and PRM connection radius are exposed. The deterministic generator consumes three values per trial. RRT/RRT\* use 10% goal bias; PRM uses uniform samples, so the same seed reproduces each algorithm without claiming identical proposal distributions. A trial counts even if its sample or extension is rejected.

Lengths in this mode are Euclidean cell widths, **not** the grid mode's 10/14 cost. Finite-budget failure is `unresolved`, never proof of infeasibility. Finite runs do not certify global optimality for RRT\* or PRM. All connections are checked analytically against the closed obstacle rectangles; touching a blocked corner is rejected.

Tests independently verify segment/rectangle intersections, tree acyclicity and descendant costs after rewiring, monotone RRT\* best costs, deterministic replay, legal returned paths, and PRM query costs against Bellman-Ford on its roadmap.

References: [LaValle, Chapter 5](https://lavalle.pl/planning/ch5.pdf); [Karaman and Frazzoli, 2011](https://arxiv.org/abs/1105.1186).

## Attribution and license

Original PathFinder author: **Kevin Wang** (`kevinwang1975`). Upstream repository: <https://github.com/kevinwang1975/PathFinder>. Original project: Apache License 2.0.

Browser adaptation and explanatory material: **Luong Tin Phan**, Apache License 2.0. The full license is provided in `../LICENSE` and `LICENSE.txt`. The original Java files and attribution notices are retained. The new browser files identify the adaptation and behavioral changes above.
