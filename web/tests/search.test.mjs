// SPDX-License-Identifier: Apache-2.0. Browser-port verification, added by Luong Tin Phan.
import test from "node:test";
import assert from "node:assert/strict";
import { Search, preset, neighbors, heuristic } from "../search.mjs";

// Independent reference: form the whole adjacency matrix geometrically, then
// relax every edge until no distance changes (Bellman-Ford, no production queue).
function reference(map) {
  const n = map.rows * map.cols,
    edges = [],
    dist = Array(n).fill(Infinity);
  for (let a = 0; a < n; a++)
    for (let b = 0; b < n; b++) {
      if (a === b || map.walls[a] || map.walls[b]) continue;
      const ra = Math.floor(a / map.cols),
        rb = Math.floor(b / map.cols),
        ca = a % map.cols,
        cb = b % map.cols;
      const dr = Math.abs(ra - rb),
        dc = Math.abs(ca - cb);
      if (dr > 1 || dc > 1 || (!map.diagonal && dr + dc !== 1)) continue;
      if (dr && dc && (map.walls[ra * map.cols + cb] || map.walls[rb * map.cols + ca])) continue;
      edges.push([a, b, dr + dc === 2 ? 14 : 10]);
    }
  dist[map.start] = 0;
  for (let i = 0; i < n - 1; i++) {
    let changed = false;
    for (const [a, b, w] of edges)
      if (dist[b] > dist[a] + w) {
        dist[b] = dist[a] + w;
        changed = true;
      }
    if (!changed) break;
  }
  return { cost: dist[map.goal], edges };
}
function check(map) {
  const expected = reference(map);
  for (const name of ["astar", "dijkstra"]) {
    const s = new Search(map, name).finish();
    assert.equal(s.g[map.goal], expected.cost, `${name} must agree with independent solver`);
    assert.equal(s.status, expected.cost === Infinity ? "no-path" : "found");
    assert.ok(s.expanded <= map.rows * map.cols);
    if (s.status === "found") {
      assert.equal(s.path[0], map.start);
      assert.equal(s.path.at(-1), map.goal);
      let sum = 0;
      for (let i = 1; i < s.path.length; i++) {
        const edge = expected.edges.find(([a, b]) => a === s.path[i - 1] && b === s.path[i]);
        assert.ok(edge, "Path may not cross a wall or a blocked corner");
        sum += edge[2];
      }
      assert.equal(sum, expected.cost);
    } else assert.deepEqual(s.path, []);
  }
}
test("all 128 wall patterns on a 3×3 grid, with four and eight neighbors", () => {
  for (const diagonal of [false, true])
    for (let bits = 0; bits < 128; bits++) {
      const walls = new Uint8Array(9);
      for (let i = 1; i < 8; i++) walls[i] = (bits >> (i - 1)) & 1;
      check({ rows: 3, cols: 3, walls, start: 0, goal: 8, diagonal });
    }
});
test("seeded larger grids and every teaching scenario match reference costs", () => {
  for (const diagonal of [false, true]) {
    for (let seed = 1; seed <= 16; seed++) check({ ...preset("random", 7, 9, seed), diagonal });
    for (const scenario of ["empty", "parking", "narrow", "blocked"]) check({ ...preset(scenario, 7, 9), diagonal });
  }
});
test("octile is consistent on every legal edge in eight-neighbor maps", () => {
  const map = preset("random", 7, 9, 12);
  assert.equal(heuristic(0, 10, 9, true), 14);
  for (let id = 0; id < map.walls.length; id++)
    for (const [to, cost] of neighbors(id, map)) {
      assert.ok(heuristic(id, map.goal, map.cols, true) <= cost + heuristic(to, map.goal, map.cols, true));
    }
});
test("one-cell map, identical endpoints, and impossible corner", () => {
  check({ rows: 1, cols: 1, walls: [0], start: 0, goal: 0, diagonal: true });
  check({ rows: 2, cols: 2, walls: [0, 1, 1, 0], start: 0, goal: 3, diagonal: true });
  check({ rows: 1, cols: 5, walls: [0, 0, 1, 0, 0], start: 0, goal: 4, diagonal: false });
});
test("step/finish agree, completed search is stable, maps are isolated", () => {
  const map = preset("parking");
  const stepped = new Search(map),
    direct = new Search(map).finish();
  map.walls.fill(1);
  while (!stepped.done) stepped.step();
  assert.deepEqual(stepped.path, direct.path);
  assert.equal(stepped.expanded, direct.expanded);
  assert.equal(stepped.step(), false);
  assert.equal(new Search(preset()).expanded, 0);
});
