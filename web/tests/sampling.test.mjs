// SPDX-License-Identifier: Apache-2.0. Verification of the new sampling-based extension.
import test from "node:test";
import assert from "node:assert/strict";
import { SamplingPlanner, segmentFree, distance } from "../sampling.mjs";
import { preset } from "../search.mjs";

// Independent collision oracle: intersect a path segment with each rectangle's
// four boundary segments, plus endpoint containment (not the production slab test).
function orient(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
function intersects(a, b, c, d) {
  const eps = 1e-8;
  if (
    Math.max(a.x, b.x) < Math.min(c.x, d.x) - eps ||
    Math.max(c.x, d.x) < Math.min(a.x, b.x) - eps ||
    Math.max(a.y, b.y) < Math.min(c.y, d.y) - eps ||
    Math.max(c.y, d.y) < Math.min(a.y, b.y) - eps
  )
    return false;
  return orient(a, b, c) * orient(a, b, d) <= eps && orient(c, d, a) * orient(c, d, b) <= eps;
}
function oracleFree(a, b, map) {
  for (let id = 0; id < map.walls.length; id++)
    if (map.walls[id]) {
      const x = id % map.cols,
        y = Math.floor(id / map.cols);
      if ([a, b].some((p) => p.x >= x && p.x <= x + 1 && p.y >= y && p.y <= y + 1)) return false;
      const corners = [
        { x, y },
        { x: x + 1, y },
        { x: x + 1, y: y + 1 },
        { x, y: y + 1 },
      ];
      if (corners.some((p, i) => intersects(a, b, p, corners[(i + 1) % 4]))) return false;
    }
  return true;
}
function verify(p) {
  const pairs = p.algorithm === "prm" ? p.edges : p.nodes.flatMap((n, i) => (n.parent < 0 ? [] : [[i, n.parent]]));
  for (const [a, b] of pairs) assert.ok(oracleFree(p.nodes[a], p.nodes[b], p.map), "Every edge is collision-free, including corners");
  if (p.algorithm !== "prm")
    p.nodes.forEach((n, i) => {
      if (i === 0) return;
      assert.ok(Math.abs(n.cost - p.nodes[n.parent].cost - distance(n, p.nodes[n.parent])) < 1e-7, "Rewiring updates descendant costs");
      const seen = new Set();
      let j = i;
      while (j !== -1) {
        assert.ok(!seen.has(j), "Parent chain has no cycles");
        seen.add(j);
        j = p.nodes[j].parent;
      }
    });
  if (p.path.length) {
    assert.deepEqual(p.path[0], p.start);
    assert.deepEqual(p.path.at(-1), p.goal);
    let length = 0;
    for (let i = 1; i < p.path.length; i++) {
      assert.ok(oracleFree(p.path[i - 1], p.path[i], p.map));
      length += distance(p.path[i - 1], p.path[i]);
    }
    assert.ok(Math.abs(length - p.bestCost) < 1e-7);
    assert.ok(p.bestCost + 1e-8 >= distance(p.start, p.goal));
  }
}
test("segment checks catch wall crossings, tangencies, and blocked corners", () => {
  const map = { rows: 3, cols: 3, walls: [0, 0, 0, 0, 1, 0, 0, 0, 0] };
  assert.equal(segmentFree({ x: 0.5, y: 1.5 }, { x: 2.5, y: 1.5 }, map), false);
  assert.equal(segmentFree({ x: 0.5, y: 0.5 }, { x: 2.5, y: 2.5 }, map), false);
  assert.equal(segmentFree({ x: 0.5, y: 1 }, { x: 2.5, y: 1 }, map), false);
  assert.equal(segmentFree({ x: 0.5, y: 0.5 }, { x: 2.5, y: 0.5 }, map), true);
  assert.equal(segmentFree({ x: -0.1, y: 0.5 }, { x: 0.5, y: 0.5 }, map), false);
});
test("default classroom example finds legal paths for all three planners", () => {
  for (const algorithm of ["rrt", "rrtstar", "prm"]) {
    const p = new SamplingPlanner(preset("parking"), algorithm).finish();
    assert.equal(p.status, "found");
    assert.equal(p.iterations, 800);
    verify(p);
    assert.equal(p.step(), false);
  }
});
test("seed repeats exactly; step and finish agree without mutating the source map", () => {
  for (const algorithm of ["rrt", "rrtstar", "prm"]) {
    const map = preset("narrow"),
      a = new SamplingPlanner(map, algorithm, { budget: 200, seed: 7 }),
      b = new SamplingPlanner(map, algorithm, { budget: 200, seed: 7 }).finish();
    map.walls.fill(1);
    while (!a.done) a.step();
    assert.deepEqual(a.nodes, b.nodes);
    assert.deepEqual(a.path, b.path);
    assert.equal(a.bestCost, b.bestCost);
    verify(a);
  }
});
test("RRT* best path never gets longer as rewiring continues", () => {
  const p = new SamplingPlanner(preset("empty"), "rrtstar", { budget: 800 });
  let previous = Infinity;
  while (!p.done) {
    p.step();
    assert.ok(p.bestCost <= previous + 1e-8);
    previous = p.bestCost;
  }
  assert.ok(p.rewires > 0);
  verify(p);
  assert.ok(p.bestCost <= new SamplingPlanner(preset("empty"), "rrt", { budget: 800 }).finish().bestCost);
});
test("PRM query agrees with independent Bellman-Ford distances on its roadmap", () => {
  const p = new SamplingPlanner(preset("parking"), "prm", { budget: 250 }).finish();
  const costs = Array(p.nodes.length).fill(Infinity);
  costs[0] = 0;
  for (let i = 0; i < p.nodes.length - 1; i++) {
    let changed = false;
    for (const [a, b] of p.edges) {
      const w = distance(p.nodes[a], p.nodes[b]);
      if (costs[b] > costs[a] + w) {
        costs[b] = costs[a] + w;
        changed = true;
      }
      if (costs[a] > costs[b] + w) {
        costs[a] = costs[b] + w;
        changed = true;
      }
    }
    if (!changed) break;
  }
  assert.equal(p.bestCost, costs[1]);
  verify(p);
});
test("budget exhaustion never claims proof of infeasibility", () => {
  for (const algorithm of ["rrt", "rrtstar", "prm"]) {
    const p = new SamplingPlanner(preset("blocked"), algorithm, { budget: 120 }).finish();
    assert.equal(p.status, "unresolved");
    assert.deepEqual(p.path, []);
    assert.equal(p.bestCost, Infinity);
    verify(p);
  }
});
test("random maps retain legal edges under different seeds and connection sizes", () => {
  for (const algorithm of ["rrt", "rrtstar", "prm"])
    for (const seed of [3, 19, 87]) {
      verify(new SamplingPlanner(preset("random", 9, 13, seed), algorithm, { seed, budget: 160, step: 3, radius: 8 }).finish());
    }
});
