/* SPDX-License-Identifier: Apache-2.0
 * Sampling-based extension by Luong Tin Phan. RRT, RRT-star and PRM are additions
 * to the browser edition, not algorithms from upstream PathFinder. */
const EPS = 1e-9;
export const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
export const center = (id, cols) => ({ x: (id % cols) + 0.5, y: Math.floor(id / cols) + 0.5 });

// Segment versus closed axis-aligned obstacle cells (slab intersection).
// Checking the entire segment prevents two free endpoints from hiding a collision.
export function segmentFree(a, b, map) {
  for (const p of [a, b]) if (p.x < 0 || p.y < 0 || p.x > map.cols || p.y > map.rows) return false;
  const x0 = Math.max(0, Math.floor(Math.min(a.x, b.x) - EPS));
  const x1 = Math.min(map.cols - 1, Math.floor(Math.max(a.x, b.x) + EPS));
  const y0 = Math.max(0, Math.floor(Math.min(a.y, b.y) - EPS));
  const y1 = Math.min(map.rows - 1, Math.floor(Math.max(a.y, b.y) + EPS));
  for (let r = y0; r <= y1; r++)
    for (let c = x0; c <= x1; c++) {
      if (!map.walls[r * map.cols + c]) continue;
      let lo = 0,
        hi = 1,
        hit = true;
      for (const [p, d, min, max] of [
        [a.x, b.x - a.x, c - EPS, c + 1 + EPS],
        [a.y, b.y - a.y, r - EPS, r + 1 + EPS],
      ]) {
        if (Math.abs(d) < EPS) {
          if (p < min || p > max) {
            hit = false;
            break;
          }
        } else {
          let t0 = (min - p) / d,
            t1 = (max - p) / d;
          if (t0 > t1) [t0, t1] = [t1, t0];
          lo = Math.max(lo, t0);
          hi = Math.min(hi, t1);
          if (lo > hi) {
            hit = false;
            break;
          }
        }
      }
      if (hit) return false;
    }
  return true;
}

export class SamplingPlanner {
  constructor(map, algorithm = "rrt", { seed = 42, budget = 800, step = 2, radius = 5 } = {}) {
    if (!["rrt", "rrtstar", "prm"].includes(algorithm)) throw new Error("Unknown planner");
    if (
      !Number.isInteger(seed) ||
      seed < 0 ||
      seed > 4294967295 ||
      !Number.isInteger(budget) ||
      budget < 1 ||
      budget > 5000 ||
      !(step > 0) ||
      !(radius > 0)
    )
      throw new Error("Invalid sampling parameters");
    this.map = { ...map, walls: Uint8Array.from(map.walls) };
    this.start = center(map.start, map.cols);
    this.goal = center(map.goal, map.cols);
    if (!segmentFree(this.start, this.start, this.map) || !segmentFree(this.goal, this.goal, this.map)) throw new Error("Blocked endpoint");
    this.algorithm = algorithm;
    this.seed = seed;
    this.rng = seed >>> 0;
    this.budget = budget;
    this.stepLength = step;
    this.radius = radius;
    this.iterations = 0;
    this.rewires = 0;
    this.lastRewired = [];
    this.lastSample = null;
    this.nodes = [{ ...this.start, parent: -1, cost: 0 }];
    this.edges = [];
    this.adjacency = [[]];
    this.path = [];
    this.bestCost = Infinity;
    this.status = "ready";
    this.message = "Chưa lấy mẫu.";
    if (algorithm === "prm") {
      this.nodes.push({ ...this.goal, parent: -1, cost: Infinity });
      this.adjacency.push([]);
      if (distance(this.start, this.goal) <= radius && segmentFree(this.start, this.goal, this.map)) this.connect(0, 1);
    } else {
      this.updatePath();
      // A nearby goal may already have a valid direct connection from S.
      if (this.algorithm === "rrt" && this.path.length) {
        this.status = "found";
        this.message = "Đã nối trực tiếp S với G; cây RRT đã dừng, không cần lấy mẫu.";
      }
    }
  }
  random() {
    this.rng = (Math.imul(1664525, this.rng) + 1013904223) >>> 0;
    return this.rng / 4294967296;
  }
  get done() {
    // RRT solves a feasibility query; RRT* keeps optimizing until its budget.
    return (this.algorithm === "rrt" && this.path.length > 0) || this.iterations >= this.budget;
  }
  connect(a, b) {
    const cost = distance(this.nodes[a], this.nodes[b]);
    this.edges.push([a, b]);
    this.adjacency[a].push([b, cost]);
    this.adjacency[b].push([a, cost]);
  }
  step() {
    if (this.done) return false;
    this.iterations++;
    this.status = "searching";
    this.lastRewired = [];
    // Consume the same three random values per trial, regardless of acceptance.
    const x = this.random() * this.map.cols,
      y = this.random() * this.map.rows,
      bias = this.random();
    const sample = this.algorithm !== "prm" && bias < 0.1 ? { ...this.goal } : { x, y };
    this.lastSample = sample;
    if (this.algorithm === "prm") this.growRoadmap(sample);
    else this.growTree(sample);
    if (this.algorithm !== "prm") this.updatePath();
    if (this.done) {
      if (this.algorithm === "prm") this.queryRoadmap();
      this.status = this.path.length ? "found" : "unresolved";
      if (this.algorithm === "rrt" && this.path.length) this.message = "Đã tìm được đường đầu tiên; cây RRT đã dừng.";
    }
    return true;
  }
  growRoadmap(sample) {
    if (!segmentFree(sample, sample, this.map)) {
      this.message = "Mẫu nằm trong vật cản, không thêm nút.";
      return;
    }
    if (this.nodes.some((p) => distance(p, sample) < EPS)) {
      this.message = "Mẫu trùng nút đã có.";
      return;
    }
    const id = this.nodes.length;
    this.nodes.push({ ...sample, parent: -1, cost: Infinity });
    this.adjacency.push([]);
    let links = 0;
    for (let j = 0; j < id; j++)
      if (distance(this.nodes[j], sample) <= this.radius && segmentFree(this.nodes[j], sample, this.map)) {
        this.connect(id, j);
        links++;
      }
    this.message = `Thêm một nút và ${links} cạnh hợp lệ. Truy vấn đường sau khi hết ngân sách mẫu.`;
  }
  growTree(sample) {
    let nearest = 0,
      closest = Infinity;
    this.nodes.forEach((p, i) => {
      const d = distance(p, sample);
      if (d < closest) {
        nearest = i;
        closest = d;
      }
    });
    if (closest < EPS) {
      this.message = "Mẫu trùng nút đã có.";
      return;
    }
    const from = this.nodes[nearest],
      ratio = Math.min(1, this.stepLength / closest);
    const added = {
      x: from.x + ratio * (sample.x - from.x),
      y: from.y + ratio * (sample.y - from.y),
      parent: nearest,
      cost: from.cost + Math.min(closest, this.stepLength),
    };
    if (!segmentFree(from, added, this.map)) {
      this.message = "Đoạn mở rộng chạm vật cản, không thêm nút.";
      return;
    }
    const id = this.nodes.length,
      near = [];
    if (this.algorithm === "rrtstar") {
      const radius = Math.min(this.stepLength * 2, 2 * Math.sqrt(this.map.rows * this.map.cols) * Math.sqrt(Math.log(id + 1) / (id + 1)));
      this.nodes.forEach((p, i) => {
        if (distance(p, added) <= radius) near.push(i);
      });
      for (const i of near) {
        const cost = this.nodes[i].cost + distance(this.nodes[i], added);
        if (cost + EPS < added.cost && segmentFree(this.nodes[i], added, this.map)) {
          added.parent = i;
          added.cost = cost;
        }
      }
    }
    this.nodes.push(added);
    for (const i of near) {
      if (i === 0 || i === added.parent) continue;
      const cost = added.cost + distance(added, this.nodes[i]);
      if (cost + EPS < this.nodes[i].cost && segmentFree(added, this.nodes[i], this.map)) {
        const delta = cost - this.nodes[i].cost;
        this.nodes[i].parent = id;
        this.nodes[i].cost = cost;
        // Descendants inherit the cost change; geometry does not change.
        const stack = [i];
        while (stack.length) {
          const parent = stack.pop();
          this.nodes.forEach((node, j) => {
            if (node.parent === parent) {
              node.cost += delta;
              stack.push(j);
            }
          });
        }
        this.rewires++;
        this.lastRewired.push(i);
      }
    }
    this.message =
      this.algorithm === "rrtstar"
        ? `Thêm nút; nối lại ${this.lastRewired.length} nhánh trong bước này.`
        : "Nối nút mới vào nút gần nhất bằng một đoạn không va chạm.";
  }
  updatePath() {
    let best = -1,
      cost = Infinity;
    this.nodes.forEach((p, i) => {
      const remaining = distance(p, this.goal),
        candidate = p.cost + remaining;
      if (remaining <= this.stepLength && candidate < cost && segmentFree(p, this.goal, this.map)) {
        best = i;
        cost = candidate;
      }
    });
    this.path = [];
    this.bestCost = cost;
    if (best < 0) return;
    for (let i = best; i !== -1; i = this.nodes[i].parent) this.path.push({ x: this.nodes[i].x, y: this.nodes[i].y });
    this.path.reverse();
    if (distance(this.path.at(-1), this.goal) > EPS) this.path.push({ ...this.goal });
  }
  queryRoadmap() {
    const costs = Array(this.nodes.length).fill(Infinity),
      parent = Array(this.nodes.length).fill(-1),
      visited = new Set();
    costs[0] = 0;
    while (true) {
      let id = -1;
      for (let i = 0; i < costs.length; i++) if (!visited.has(i) && Number.isFinite(costs[i]) && (id < 0 || costs[i] < costs[id])) id = i;
      if (id < 0 || id === 1) break;
      visited.add(id);
      for (const [to, weight] of this.adjacency[id])
        if (costs[id] + weight < costs[to]) {
          costs[to] = costs[id] + weight;
          parent[to] = id;
        }
    }
    this.bestCost = costs[1];
    this.path = [];
    if (!Number.isFinite(costs[1])) return;
    for (let i = 1; i !== -1; i = parent[i]) this.path.push({ x: this.nodes[i].x, y: this.nodes[i].y });
    this.path.reverse();
  }
  finish() {
    while (!this.done) this.step();
    return this;
  }
}
