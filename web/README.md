# PathFinder · Vietnamese teaching edition

A buildless browser adaptation of Kevin Wang's Java/Swing [PathFinder](https://github.com/kevinwang1975/PathFinder), for the undergraduate path-planning course at [tinrobotics Teaching](https://tinrobotics.com/teaching/lap-lo-trinh-di-chuyen/).

Live simulator: <https://tinrobotics.com/teaching/lap-lo-trinh-di-chuyen/mo-phong-thuat-toan/>

## Run locally

From this repository: `python3 -m http.server 8768 --directory web`, then open <http://localhost:8768/>. ES modules require HTTP rather than opening `index.html` as a local file. No package installation, backend, telemetry, or external runtime library is needed.

Verify the algorithm: `node --test web/tests/search.test.mjs` (Node 18+).

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

`search.mjs` contains the incremental engine; `app.mjs` handles canvas rendering and controls. `index.html` and `style.css` are standalone. The website carries a pinned copy of these five runtime/license files, with the matching fork commit recorded in its adjacent `SOURCE.md`. Tests and documentation stay in this repository. There is no practice ZIP/download button.

## Attribution and license

Original PathFinder author: **Kevin Wang** (`kevinwang1975`). Upstream repository: <https://github.com/kevinwang1975/PathFinder>. Original project: Apache License 2.0.

Browser adaptation and explanatory material: **Luong Tin Phan**, Apache License 2.0. The full license is provided in `../LICENSE` and `LICENSE.txt`. The original Java files and attribution notices are retained. The new browser files identify the adaptation and behavioral changes above.
