/* SPDX-License-Identifier: Apache-2.0
 * Browser adaptation of Kevin Wang's PathFinder (AStarPathAlgorithm and cost evaluators).
 * Added by Luong Tin Phan: incremental steps, deterministic queue, admissible octile
 * heuristic, and stricter diagonal collision checks. Original Java files are unchanged.
 */
class MinHeap {
  items = [];
  before(a, b) {
    return a.f < b.f || (a.f === b.f && a.order < b.order);
  }
  push(item) {
    const a = this.items;
    a.push(item);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.before(a[i], a[p])) break;
      [a[i], a[p]] = [a[p], a[i]];
      i = p;
    }
  }
  pop() {
    const a = this.items,
      top = a[0],
      last = a.pop();
    if (a.length) {
      a[0] = last;
      let i = 0;
      while (true) {
        const l = i * 2 + 1,
          r = l + 1;
        let b = i;
        if (l < a.length && this.before(a[l], a[b])) b = l;
        if (r < a.length && this.before(a[r], a[b])) b = r;
        if (b === i) break;
        [a[i], a[b]] = [a[b], a[i]];
        i = b;
      }
    }
    return top;
  }
}

export function heuristic(a, b, cols, diagonal) {
  const dx = Math.abs((a % cols) - (b % cols));
  const dy = Math.abs(Math.floor(a / cols) - Math.floor(b / cols));
  return diagonal ? 14 * Math.min(dx, dy) + 10 * Math.abs(dx - dy) : 10 * (dx + dy);
}

export function neighbors(id, map) {
  const { rows, cols, walls, diagonal } = map;
  const r = Math.floor(id / cols),
    c = id % cols,
    result = [];
  const moves = [
    [-1, 0],
    [0, 1],
    [1, 0],
    [0, -1],
  ];
  if (diagonal) moves.push([-1, 1], [1, 1], [1, -1], [-1, -1]);
  for (const [dr, dc] of moves) {
    const y = r + dr,
      x = c + dc;
    if (y < 0 || y >= rows || x < 0 || x >= cols) continue;
    const to = y * cols + x;
    if (walls[to]) continue;
    // Both side cells must be free: a point cannot slip diagonally past a wall corner.
    if (dr && dc && (walls[r * cols + x] || walls[y * cols + c])) continue;
    result.push([to, dr && dc ? 14 : 10]);
  }
  return result;
}

export class Search {
  constructor(map, algorithm = "astar") {
    if (!["astar", "dijkstra"].includes(algorithm)) throw new Error("Unknown algorithm");
    const n = map.rows * map.cols;
    if (
      !Number.isInteger(n) ||
      n < 1 ||
      map.walls.length !== n ||
      !Number.isInteger(map.start) ||
      !Number.isInteger(map.goal) ||
      map.start < 0 ||
      map.goal < 0 ||
      map.start >= n ||
      map.goal >= n ||
      map.walls[map.start] ||
      map.walls[map.goal]
    )
      throw new Error("Invalid map");
    this.map = { ...map, walls: Uint8Array.from(map.walls) };
    this.algorithm = algorithm;
    this.g = new Float64Array(n).fill(Infinity);
    this.parent = new Int32Array(n).fill(-1);
    this.closed = new Uint8Array(n);
    this.open = new Set([map.start]);
    this.queue = new MinHeap();
    this.order = 0;
    this.expanded = 0;
    this.current = -1;
    this.path = [];
    this.status = "ready";
    this.g[map.start] = 0;
    this.enqueue(map.start);
  }
  h(id) {
    return this.algorithm === "dijkstra" ? 0 : heuristic(id, this.map.goal, this.map.cols, this.map.diagonal);
  }
  enqueue(id) {
    this.queue.push({ id, g: this.g[id], f: this.g[id] + this.h(id), order: this.order++ });
  }
  get done() {
    return this.status === "found" || this.status === "no-path";
  }
  step() {
    if (this.done) return false;
    let entry;
    while (this.queue.items.length) {
      const candidate = this.queue.pop();
      if (!this.closed[candidate.id] && candidate.g === this.g[candidate.id]) {
        entry = candidate;
        break;
      }
    }
    if (!entry) {
      this.status = "no-path";
      this.current = -1;
      return false;
    }
    const id = entry.id;
    this.status = "searching";
    this.current = id;
    this.open.delete(id);
    this.closed[id] = 1;
    this.expanded++;
    if (id === this.map.goal) {
      this.status = "found";
      for (let p = id; p !== -1; p = this.parent[p]) this.path.push(p);
      this.path.reverse();
      return true;
    }
    for (const [to, weight] of neighbors(id, this.map)) {
      if (this.closed[to]) continue;
      const cost = this.g[id] + weight;
      if (cost < this.g[to]) {
        this.g[to] = cost;
        this.parent[to] = id;
        this.open.add(to);
        this.enqueue(to);
      }
    }
    if (!this.open.size) {
      this.status = "no-path";
      this.current = -1;
    }
    return true;
  }
  finish() {
    while (!this.done) this.step();
    return this;
  }
  frontier(limit = 5) {
    return [...this.open].sort((a, b) => this.g[a] + this.h(a) - this.g[b] - this.h(b) || a - b).slice(0, limit);
  }
}

export function preset(name = "parking", rows = 21, cols = 33, seed = 42) {
  const walls = new Uint8Array(rows * cols);
  const start = Math.floor(rows / 2) * cols + 2;
  const goal = Math.floor(rows / 2) * cols + cols - 3;
  const wall = (r, c) => {
    if (r >= 0 && r < rows && c >= 0 && c < cols) walls[r * cols + c] = 1;
  };
  if (name === "parking") {
    for (let c = 6; c < cols - 4; c += 6) {
      for (let r = 3; r < rows - 3; r++) {
        if ((c % 12 === 6 && r < rows - 6) || (c % 12 === 0 && r > 5)) wall(r, c);
      }
    }
  } else if (name === "narrow") {
    for (let r = 0; r < rows; r++) if (r !== Math.floor(rows / 2) - 3) wall(r, Math.floor(cols / 2));
  } else if (name === "blocked") {
    for (let r = 0; r < rows; r++) wall(r, Math.floor(cols / 2));
  } else if (name === "random") {
    let x = seed >>> 0;
    for (let i = 0; i < walls.length; i++) {
      x = (Math.imul(1664525, x) + 1013904223) >>> 0;
      walls[i] = x / 4294967296 < 0.25 ? 1 : 0;
    }
  }
  walls[start] = 0;
  walls[goal] = 0;
  return { rows, cols, walls, start, goal, diagonal: true };
}
