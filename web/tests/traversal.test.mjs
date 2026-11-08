// SPDX-License-Identifier: Apache-2.0. BFS/DFS teaching extension by Luong Tin Phan.
import test from "node:test";
import assert from "node:assert/strict";
import { Search, preset } from "../search.mjs";

// Independent adjacency construction and all-pairs unit-edge distances.
// This does not use the production neighbors() or either frontier policy.
function reference(map) {
  const n = map.rows * map.cols;
  const dist = Array.from({ length: n }, (_, a) => Array.from({ length: n }, (_, b) => (a === b ? 0 : Infinity)));
  const adjacent = (a, b) => {
    if (map.walls[a] || map.walls[b]) return false;
    const ar = Math.floor(a / map.cols),
      ac = a % map.cols,
      br = Math.floor(b / map.cols),
      bc = b % map.cols;
    const dr = Math.abs(ar - br),
      dc = Math.abs(ac - bc);
    if (dr + dc === 1) return true;
    return map.diagonal && dr === 1 && dc === 1 && !map.walls[ar * map.cols + bc] && !map.walls[br * map.cols + ac];
  };
  for (let a = 0; a < n; a++) for (let b = 0; b < n; b++) if (adjacent(a, b)) dist[a][b] = 1;
  for (let k = 0; k < n; k++) for (let a = 0; a < n; a++) for (let b = 0; b < n; b++) dist[a][b] = Math.min(dist[a][b], dist[a][k] + dist[k][b]);
  return { edges: dist[map.start][map.goal], adjacent };
}
function check(map) {
  const expected = reference(map);
  for (const algorithm of ["bfs", "dfs"]) {
    const search = new Search(map, algorithm),
      pops = [];
    while (!search.done) {
      search.step();
      if (search.current >= 0) pops.push(search.current);
    }
    assert.equal(search.status, Number.isFinite(expected.edges) ? "found" : "no-path");
    assert.equal(new Set(pops).size, pops.length, "never expand a node twice");
    assert.ok(search.expanded <= map.walls.length);
    if (search.status === "found") {
      assert.equal(search.path[0], map.start);
      assert.equal(search.path.at(-1), map.goal);
      assert.equal(new Set(search.path).size, search.path.length, "parent chain is acyclic");
      let cost = 0;
      for (let i = 1; i < search.path.length; i++) {
        const a = search.path[i - 1],
          b = search.path[i];
        assert.ok(expected.adjacent(a, b), "path never crosses a wall or a blocked corner");
        cost += Math.abs((a % map.cols) - (b % map.cols)) + Math.abs(Math.floor(a / map.cols) - Math.floor(b / map.cols)) === 2 ? 14 : 10;
      }
      assert.equal(search.g[map.goal], cost);
      assert.equal(search.depth[map.goal], search.path.length - 1);
      if (algorithm === "bfs") assert.equal(search.path.length - 1, expected.edges);
      else assert.ok(search.path.length - 1 >= expected.edges);
    } else assert.deepEqual(search.path, []);
    const snapshot = JSON.stringify(search);
    assert.equal(search.step(), false);
    search.finish();
    assert.equal(JSON.stringify(search), snapshot, "finished searches remain unchanged");
  }
}
test("BFS minimizes edges; DFS returns a legal path for every 3×3 obstacle pattern", () => {
  for (const diagonal of [false, true])
    for (let bits = 0; bits < 128; bits++) {
      const walls = new Uint8Array(9);
      for (let i = 1; i < 8; i++) walls[i] = (bits >> (i - 1)) & 1;
      check({ rows: 3, cols: 3, walls, start: 0, goal: 8, diagonal });
    }
});
test("FIFO and LIFO use the same neighbor insertion order and mark on push", () => {
  const map = { rows: 3, cols: 3, walls: new Uint8Array(9), start: 4, goal: 0, diagonal: false };
  const bfs = new Search(map, "bfs"),
    dfs = new Search(map, "dfs");
  bfs.step();
  dfs.step();
  assert.deepEqual(bfs.frontier(), [1, 5, 7, 3]); // north, east, south, west
  assert.deepEqual(dfs.frontier(), [3, 7, 5, 1]); // top first
  assert.deepEqual([...bfs.parent], [...dfs.parent]);
  bfs.step();
  dfs.step();
  assert.equal(bfs.current, 1);
  assert.equal(dfs.current, 3);
  assert.equal(dfs.parent[1], 4, "a discovered node keeps its first parent");
  dfs.step();
  assert.equal(dfs.current, 6, "last newly added node is taken first");
  assert.deepEqual(
    [...dfs.open].sort((a, b) => a - b),
    dfs.frontier(99).sort((a, b) => a - b)
  );
});
test("same endpoints, disconnected goals and seeded four/eight-direction maps", () => {
  check({ rows: 1, cols: 1, walls: [0], start: 0, goal: 0, diagonal: false });
  check({ rows: 1, cols: 5, walls: [0, 0, 1, 0, 0], start: 0, goal: 4, diagonal: false });
  for (const diagonal of [false, true]) {
    for (const scenario of ["empty", "parking", "narrow", "blocked"]) check({ ...preset(scenario, 7, 9), diagonal });
    for (let seed = 1; seed <= 12; seed++) check({ ...preset("random", 7, 9, seed), diagonal });
  }
});
test("DFS can be longer than BFS; neither traversal uses weighted priority", () => {
  const map = { ...preset("empty", 5, 7), diagonal: false };
  const bfs = new Search(map, "bfs").finish(),
    dfs = new Search(map, "dfs").finish();
  assert.ok(dfs.path.length > bfs.path.length);
  assert.equal(bfs.path.length - 1, 2);
  assert.equal(bfs.h(map.start), 0);
  assert.equal(dfs.h(map.start), 0);
  const weighted = preset("random", 9, 11, 28);
  const fewestEdges = new Search(weighted, "bfs").finish();
  const leastCost = new Search(weighted, "dijkstra").finish();
  assert.equal(fewestEdges.path.length - 1, 9);
  assert.equal(fewestEdges.g[weighted.goal], 102);
  assert.equal(leastCost.g[weighted.goal], 94, "BFS is not a weighted shortest-path algorithm");
});
