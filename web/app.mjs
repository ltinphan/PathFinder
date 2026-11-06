/* SPDX-License-Identifier: Apache-2.0
 * New browser UI by Luong Tin Phan, adapted from Kevin Wang's PathFinder demo.
 * Java source is unchanged; see README.md for mathematical and collision-rule changes.
 */
import { Search, preset } from "./search.mjs";
import { SamplingPlanner } from "./sampling.mjs";
const $ = (id) => document.getElementById(id);
const gridNames = ["astar", "dijkstra"],
  sampleNames = ["rrt", "rrtstar", "prm"];
let names = gridNames,
  samplingMode = false;
const colors = { wall: "#344054", closed: "#c8e4f3", open: "#ffe5a3", current: "#ffb26b", path: "#166fe5" };
let map = preset(),
  searches,
  running = false,
  frame = 0,
  last = 0,
  credit = 0;
let tool = "wall",
  cursor = map.start,
  dragging = false,
  previous = null,
  randomSeed = 42;
const point = (id) => `(${(id % map.cols) + 1}, ${Math.floor(id / map.cols) + 1})`;
const value = (n) => (Number.isFinite(n) ? String(n) : "∞");
function say(text) {
  $("status").textContent = text;
}
function stop() {
  running = false;
  cancelAnimationFrame(frame);
  frame = 0;
  $("run").textContent = "▶ Chạy so sánh";
  $("run").setAttribute("aria-pressed", "false");
}
function reset(message = "Sẵn sàng. Các thuật toán dùng cùng một bản đồ.") {
  stop();
  const rawSeed = Number($("sample-seed").value);
  const seed = Math.max(0, Math.min(4294967295, Number.isFinite(rawSeed) ? Math.floor(rawSeed) : 42));
  $("sample-seed").value = seed;
  const options = { seed, budget: Number($("sample-budget").value), step: Number($("sample-step").value), radius: Number($("sample-radius").value) };
  searches = names.map((name) => (samplingMode ? new SamplingPlanner(map, name, options) : new Search(map, name)));
  $("result").textContent = samplingMode
    ? "Hãy quan sát cây RRT, những lần nối lại của RRT* và mạng PRM. PRM sẽ truy vấn đường khi hết ngân sách mẫu."
    : "Thử đoán trước: A* có cần xét hết vùng quanh điểm đầu không?";
  say(message);
  render();
}
function resize(canvas) {
  const w = canvas.getBoundingClientRect().width;
  const scale = Math.min(window.devicePixelRatio || 1, 3);
  const width = Math.round(w * scale),
    height = Math.round(((w * map.rows) / map.cols) * scale);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}
function draw(canvas, search) {
  resize(canvas);
  const ctx = canvas.getContext("2d"),
    w = canvas.width / map.cols,
    h = canvas.height / map.rows;
  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  for (let i = 0; i < map.walls.length; i++) {
    const fill = map.walls[i]
      ? colors.wall
      : i === search.current
        ? colors.current
        : search.closed[i]
          ? colors.closed
          : search.open.has(i)
            ? colors.open
            : null;
    if (fill) {
      ctx.fillStyle = fill;
      ctx.fillRect((i % map.cols) * w, Math.floor(i / map.cols) * h, w, h);
    }
  }
  ctx.strokeStyle = "#dfe5ec";
  ctx.lineWidth = Math.max(1, canvas.width / 1200);
  ctx.beginPath();
  for (let c = 0; c <= map.cols; c++) {
    ctx.moveTo(c * w, 0);
    ctx.lineTo(c * w, canvas.height);
  }
  for (let r = 0; r <= map.rows; r++) {
    ctx.moveTo(0, r * h);
    ctx.lineTo(canvas.width, r * h);
  }
  ctx.stroke();
  if (search.path.length) {
    ctx.strokeStyle = colors.path;
    ctx.lineWidth = w * 0.24;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    search.path.forEach((id, i) => {
      const x = ((id % map.cols) + 0.5) * w,
        y = (Math.floor(id / map.cols) + 0.5) * h;
      if (i) ctx.lineTo(x, y);
      else ctx.moveTo(x, y);
    });
    ctx.stroke();
  }
  for (const [id, label, color] of [
    [map.start, "S", "#087443"],
    [map.goal, "G", "#b4235c"],
  ]) {
    const x = ((id % map.cols) + 0.5) * w,
      y = (Math.floor(id / map.cols) + 0.5) * h;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(x, y, w * 0.43, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillStyle = "#fff";
    ctx.font = `bold ${w * 0.6}px system-ui`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, x, y + h * 0.02);
  }
  if (document.activeElement === canvas) {
    ctx.strokeStyle = "#111827";
    ctx.lineWidth = Math.max(2, w * 0.1);
    ctx.strokeRect((cursor % map.cols) * w + 2, Math.floor(cursor / map.cols) * h + 2, w - 4, h - 4);
  }
}
function render() {
  if (samplingMode) {
    renderSampling();
    return;
  }
  searches.forEach((s, k) => {
    const name = names[k];
    draw($(name), s);
    const metrics = $(`${name}-metrics`);
    const values = {
      expanded: s.expanded,
      open: s.open.size,
      cost: s.status === "found" ? s.g[map.goal] : s.status === "no-path" ? "Không có" : "—",
      edges: s.status === "found" ? s.path.length - 1 : "—",
    };
    for (const [key, v] of Object.entries(values)) metrics.querySelector(`[data-metric="${key}"]`).textContent = v;
    $(`${name}-current`).textContent =
      s.current < 0
        ? s.done
          ? "Hết ô có thể đi tới; không tìm được đường."
          : "Chưa lấy ô nào ra khỏi hàng đợi."
        : `Vừa chọn ${point(s.current)} · g = ${s.g[s.current]} · h = ${s.h(s.current)} · f = ${s.g[s.current] + s.h(s.current)}${
            s.status === "found" ? " · Đã đến đích." : ""
          }`;
    const list = $(`${name}-queue`);
    list.replaceChildren();
    for (const id of s.frontier()) {
      const li = document.createElement("li");
      li.textContent = `${point(id)}: g = ${s.g[id]}, h = ${s.h(id)}, f = ${s.g[id] + s.h(id)}`;
      list.append(li);
    }
    if (!s.open.size) {
      const li = document.createElement("li");
      li.textContent = "Hàng đợi trống.";
      list.append(li);
    }
  });
  $("inspector").textContent = `Ô ${point(cursor)}${
    map.walls[cursor]
      ? " là vật cản."
      : ": " +
        searches
          .map((s, k) => `${k ? "Dijkstra" : "A*"}: g = ${value(s.g[cursor])}, h = ${s.h(cursor)}, f = ${value(s.g[cursor] + s.h(cursor))}`)
          .join(" · ")
  } Tọa độ (cột, hàng) bắt đầu từ 1 ở góc trên trái.`;
  if (searches.every((s) => s.done)) {
    stop();
    const [a, d] = searches;
    if (a.status === "found" && d.status === "found") {
      $("result").textContent = `Cả hai tìm được đường có chi phí ${a.g[map.goal]}. A* xét ${a.expanded} ô; Dijkstra xét ${d.expanded} ô. ${
        a.expanded < d.expanded
          ? "Ở bản đồ này, heuristic giúp A* xét ít ô hơn."
          : "Ở bản đồ này, A* không xét ít ô hơn; lợi ích của heuristic còn phụ thuộc bản đồ."
      } Hai đường không nhất thiết trùng nhau.`;
      say("Đã hoàn tất phép so sánh.");
    } else {
      $("result").textContent =
        `Không có đường từ S đến G. A* đã xét ${a.expanded} ô, Dijkstra đã xét ${d.expanded} ô; mỗi thuật toán dừng khi không còn ô chờ xử lý trong vùng đi tới được.`;
      say("Đã hoàn tất: không có đường.");
    }
  }
  $("step").disabled = searches.every((s) => s.done);
  $("finish").disabled = searches.every((s) => s.done);
}
function tick(now) {
  if (!running) return;
  credit += (Math.min(now - last, 250) * Number($("speed").value)) / 1000;
  last = now;
  while (credit >= 1 && searches.some((s) => !s.done)) {
    searches.forEach((s) => s.step());
    credit--;
  }
  render();
  if (running) frame = requestAnimationFrame(tick);
}
$("run").addEventListener("click", () => {
  if (running) {
    stop();
    say("Đang tạm dừng. Có thể tiến từng bước hoặc tiếp tục chạy.");
    return;
  }
  if (searches.every((s) => s.done)) reset();
  running = true;
  last = performance.now();
  credit = 0;
  $("run").textContent = "Ⅱ Tạm dừng";
  $("run").setAttribute("aria-pressed", "true");
  say(
    samplingMode
      ? "Đang lấy mẫu. RRT* tiếp tục cải thiện đường; PRM truy vấn sau khi dựng mạng."
      : "Đang chạy cùng tốc độ lấy ô cho cả hai thuật toán."
  );
  frame = requestAnimationFrame(tick);
});
$("step").addEventListener("click", () => {
  stop();
  searches.forEach((s) => s.step());
  say(samplingMode ? "Mỗi thuật toán vừa thử thêm một mẫu." : "Mỗi thuật toán còn chạy vừa xử lý thêm một ô.");
  render();
});
$("finish").addEventListener("click", () => {
  stop();
  searches.forEach((s) => s.finish());
  render();
});
$("reset").addEventListener("click", () => reset());
$("speed").addEventListener("input", () => {
  $("speed-value").textContent = $("speed").value;
});
function newMap() {
  const diagonal = $("movement").value === "8";
  map = preset($("scenario").value, 21, 33, randomSeed++);
  map.diagonal = diagonal;
  cursor = map.start;
  reset("Đã tạo bản đồ. Các thuật toán sẽ dùng cùng bản đồ này.");
}
$("scenario").addEventListener("change", newMap);
$("new-map").addEventListener("click", newMap);
$("movement").addEventListener("change", () => {
  map.diagonal = $("movement").value === "8";
  reset("Đã đổi cách di chuyển và tính lại heuristic.");
});
$("clear-walls").addEventListener("click", () => {
  map.walls.fill(0);
  reset("Đã xóa hết vật cản; giữ nguyên S và G.");
});
document.querySelectorAll("[data-tool]").forEach((button) =>
  button.addEventListener("click", () => {
    tool = button.dataset.tool;
    document.querySelectorAll("[data-tool]").forEach((b) => b.setAttribute("aria-pressed", String(b === button)));
  })
);
function paint(id) {
  cursor = id;
  if (tool === "start" || tool === "goal") {
    if (id === map[tool === "start" ? "goal" : "start"]) {
      say("Hãy chọn hai ô khác nhau cho điểm đầu và điểm đích.");
      return;
    }
    map[tool] = id;
    map.walls[id] = 0;
  } else {
    if (id === map.start || id === map.goal) return;
    map.walls[id] = tool === "wall" ? 1 : 0;
  }
  reset(`Đã sửa ô ${point(id)}. Kết quả cũ đã được xóa để chạy lại trên bản đồ mới.`);
}
function cellAt(event, canvas) {
  const b = canvas.getBoundingClientRect();
  return (
    Math.max(0, Math.min(map.rows - 1, Math.floor(((event.clientY - b.top) / b.height) * map.rows))) * map.cols +
    Math.max(0, Math.min(map.cols - 1, Math.floor(((event.clientX - b.left) / b.width) * map.cols)))
  );
}
function stroke(from, to) {
  let x = from % map.cols,
    y = Math.floor(from / map.cols);
  const x2 = to % map.cols,
    y2 = Math.floor(to / map.cols),
    dx = Math.abs(x2 - x),
    dy = -Math.abs(y2 - y),
    sx = x < x2 ? 1 : -1,
    sy = y < y2 ? 1 : -1;
  let error = dx + dy;
  while (true) {
    paint(y * map.cols + x);
    if (x === x2 && y === y2) break;
    const e2 = 2 * error;
    if (e2 >= dy) {
      error += dy;
      x += sx;
    }
    if (e2 <= dx) {
      error += dx;
      y += sy;
    }
  }
}
[...gridNames, ...sampleNames].forEach((name) => {
  const canvas = $(name);
  canvas.addEventListener("pointerdown", (e) => {
    if (e.button !== 0) return;
    e.preventDefault();
    canvas.focus();
    dragging = true;
    canvas.setPointerCapture(e.pointerId);
    previous = cellAt(e, canvas);
    paint(previous);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    const id = cellAt(e, canvas);
    if (id !== previous) {
      if (tool === "wall" || tool === "erase") stroke(previous, id);
      else paint(id);
      previous = id;
    }
  });
  const release = () => {
    dragging = false;
    previous = null;
  };
  canvas.addEventListener("pointerup", release);
  canvas.addEventListener("pointercancel", release);
  canvas.addEventListener("lostpointercapture", release);
  canvas.addEventListener("focus", render);
  canvas.addEventListener("blur", render);
  canvas.addEventListener("keydown", (e) => {
    const r = Math.floor(cursor / map.cols),
      c = cursor % map.cols;
    const moves = { ArrowUp: [-1, 0], ArrowDown: [1, 0], ArrowLeft: [0, -1], ArrowRight: [0, 1] };
    if (moves[e.key]) {
      e.preventDefault();
      const [dr, dc] = moves[e.key];
      cursor = Math.max(0, Math.min(map.rows - 1, r + dr)) * map.cols + Math.max(0, Math.min(map.cols - 1, c + dc));
      say(`Đang chọn ô ${point(cursor)}. Nhấn Enter để dùng công cụ hiện tại.`);
      render();
    } else if (e.key === "Enter" || e.key === " ") {
      e.preventDefault();
      paint(cursor);
    }
  });
});
function drawSampling(canvas, planner) {
  // Reuse the obstacle/grid/endpoint layer, then overlay continuous geometry.
  draw(canvas, { current: -1, closed: [], open: new Set(), path: [] });
  const ctx = canvas.getContext("2d"),
    w = canvas.width / map.cols,
    h = canvas.height / map.rows;
  const line = (a, b) => {
    ctx.moveTo(a.x * w, a.y * h);
    ctx.lineTo(b.x * w, b.y * h);
  };
  ctx.strokeStyle = planner.algorithm === "prm" ? "rgba(53,112,127,.18)" : "rgba(35,128,116,.55)";
  ctx.lineWidth = Math.max(0.65, w * 0.045);
  ctx.beginPath();
  if (planner.algorithm === "prm") for (const [a, b] of planner.edges) line(planner.nodes[a], planner.nodes[b]);
  else
    planner.nodes.forEach((p) => {
      if (p.parent >= 0) line(p, planner.nodes[p.parent]);
    });
  ctx.stroke();
  ctx.fillStyle = "#237b70";
  for (const p of planner.nodes) {
    ctx.beginPath();
    ctx.arc(p.x * w, p.y * h, Math.max(0.8, w * 0.065), 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.strokeStyle = "#9c36b5";
  ctx.lineWidth = Math.max(2, w * 0.13);
  ctx.beginPath();
  for (const id of planner.lastRewired) line(planner.nodes[id], planner.nodes[planner.nodes[id].parent]);
  ctx.stroke();
  if (planner.lastSample) {
    ctx.strokeStyle = "#d45a12";
    ctx.lineWidth = Math.max(1.5, w * 0.08);
    ctx.beginPath();
    ctx.arc(planner.lastSample.x * w, planner.lastSample.y * h, w * 0.3, 0, Math.PI * 2);
    ctx.stroke();
  }
  if (planner.path.length) {
    ctx.strokeStyle = colors.path;
    ctx.lineWidth = Math.max(2.5, w * 0.22);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    planner.path.forEach((p, i) => {
      if (i) ctx.lineTo(p.x * w, p.y * h);
      else ctx.moveTo(p.x * w, p.y * h);
    });
    ctx.stroke();
  }
  // Endpoint labels stay visible over the roadmap and path.
  for (const [id, label, color] of [
    [map.start, "S", "#087443"],
    [map.goal, "G", "#b4235c"],
  ]) {
    const x = ((id % map.cols) + 0.5) * w,
      y = (Math.floor(id / map.cols) + 0.5) * h;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(x, y, w * 0.43, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#fff";
    ctx.font = `bold ${w * 0.6}px system-ui`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, x, y + h * 0.02);
  }
  if (document.activeElement === canvas) {
    ctx.strokeStyle = "#111827";
    ctx.lineWidth = Math.max(2, w * 0.1);
    ctx.strokeRect((cursor % map.cols) * w + 2, Math.floor(cursor / map.cols) * h + 2, w - 4, h - 4);
  }
}
function renderSampling() {
  searches.forEach((s, k) => {
    const name = names[k];
    drawSampling($(name), s);
    const metrics = $(`${name}-metrics`),
      values = {
        attempts: s.iterations,
        nodes: s.nodes.length,
        length: Number.isFinite(s.bestCost) ? s.bestCost.toFixed(2) : s.done ? "Chưa có" : "—",
        extra: name === "rrtstar" ? s.rewires : name === "prm" ? s.edges.length : s.nodes.length - 1,
      };
    for (const [key, v] of Object.entries(values)) metrics.querySelector(`[data-metric="${key}"]`).textContent = v;
    const message =
      name === "prm" && s.done
        ? s.path.length
          ? "Đã truy vấn bằng Dijkstra và tìm được đường trên mạng."
          : "Đã truy vấn; S và G chưa được nối trong mạng này."
        : s.message;
    $(`${name}-current`).textContent = `${s.iterations}/${s.budget} lần thử · ${message}${
      s.done && !s.path.length ? " Hết ngân sách, chưa tìm được đường." : ""
    }`;
  });
  const done = searches.every((s) => s.done);
  $("step").disabled = done;
  $("finish").disabled = done;
  if (done) {
    stop();
    const labels = ["RRT", "RRT*", "PRM"];
    $("result").textContent =
      searches.map((s, k) => `${labels[k]}: ${s.path.length ? s.bestCost.toFixed(2) + " ô" : "chưa tìm được đường"}`).join(" · ") +
      ". Cùng ngân sách hữu hạn; chưa tìm được đường không chứng minh rằng không có đường.";
    say("Đã chạy hết ngân sách. Có thể tăng số lần thử hoặc đổi seed rồi chạy lại.");
  } else {
    const found = searches.slice(0, 2).filter((s) => s.path.length).length;
    $("result").textContent = searches.every((s) => s.iterations === 0)
      ? "Nhấn Tiến một bước để quan sát từng mẫu, hoặc Chạy so sánh để xem cây và mạng phát triển."
      : found
        ? `Đã có đường ở ${found} cây. RRT* vẫn tiếp tục thử nối lại để giảm độ dài; PRM đang dựng mạng và sẽ truy vấn ở cuối.`
        : "Đang xây cây và mạng. Mẫu hoặc đoạn nối không hợp lệ bị loại nhưng vẫn tính là một lần thử.";
  }
}
function chooseMode(sample) {
  samplingMode = sample;
  names = sample ? sampleNames : gridNames;
  document.querySelectorAll(".grid-only").forEach((e) => {
    e.hidden = sample;
  });
  for (const id of ["sampling-settings", "sampling-boards", "sampling-legend", "sampling-reading"]) $(id).hidden = !sample;
  $("mode-grid").setAttribute("aria-pressed", String(!sample));
  $("mode-sampling").setAttribute("aria-pressed", String(sample));
  $("speed-unit").textContent = sample ? "lần thử/s" : "bước/s";
  $("reading-link").setAttribute("href", sample ? "#sampling-reading" : "#reading");
  const url = new URL(window.location.href);
  if (sample) url.searchParams.set("mode", "sampling");
  else url.searchParams.delete("mode");
  history.replaceState(null, "", url);
  reset(sample ? "Chế độ lấy mẫu: RRT, RRT* và PRM cùng dùng bản đồ hiện tại." : "Chế độ lưới: A* và Dijkstra cùng dùng bản đồ hiện tại.");
}
$("mode-grid").addEventListener("click", () => chooseMode(false));
$("mode-sampling").addEventListener("click", () => chooseMode(true));
for (const id of ["sample-seed", "sample-budget", "sample-step", "sample-radius"])
  $(id).addEventListener("change", () => reset("Đã đổi tham số lấy mẫu. Bản đồ được giữ nguyên."));
$("sample-seed").addEventListener("input", () => {
  if ($("sample-seed").value !== "" && $("sample-seed").validity.valid) reset("Đã đổi seed và bắt đầu lại phép so sánh.");
});
new ResizeObserver(() => {
  if (searches) render();
}).observe($("simulation"));
document.addEventListener("visibilitychange", () => {
  if (document.hidden && running) {
    stop();
    say("Đã tạm dừng khi chuyển sang trang khác.");
  }
});
if (new URLSearchParams(window.location.search).get("mode") === "sampling") chooseMode(true);
else reset();
