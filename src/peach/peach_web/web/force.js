"use strict";

// Five-point force read-only page. Cad coordinates are nominal; CoP is a
// five-point weighted centroid, not a metrology-grade pressure centre.
(function () {
  // Angle clockwise from +Y_tcp; nominal radius 16.35 mm.
  const PADS = [
    {ch: 5, x: 0.00, y: 16.45, ang: 0.0, pos: "上", lp: "above"},
    {ch: 1, x: 15.75, y: 4.30, ang: 74.7, pos: "右上", lp: "right"},
    {ch: 2, x: 9.45, y: -13.30, ang: 144.6, pos: "右下", lp: "below"},
    {ch: 3, x: -9.65, y: -13.15, ang: 216.3, pos: "左下", lp: "below"},
    {ch: 7, x: -15.70, y: 4.55, ang: 286.2, pos: "左上", lp: "left"},
  ];
  const NAMES = {0: "OK", 1: "饱和", 2: "开路", 3: "无效"};
  const {$, safe} = Peach.ui;
  let fov = "full";
  let latest = null;
  let forceAvailable = false;

  function setStatus(kind, text, hint) {
    const card = $("statusCard");
    card.classList.remove("ok", "warn", "bad");
    if (kind) card.classList.add(kind);
    $("status").textContent = text;
    $("hint").textContent = hint;
  }

  function valuesById(data) {
    const out = {};
    if (!data || !data.available) return out;
    const ids = Array.isArray(data.channel_ids) ? data.channel_ids : [];
    ids.forEach((ch, index) => {
      out[Number(ch)] = {
        force: Number(data.force_n?.[index] ?? 0),
        travel: Number(data.travel_mm?.[index] ?? 0),
        ad: data.ad?.[index] ?? null,
        status: Number(data.status?.[index] ?? 0),
      };
    });
    return out;
  }

  function buildPads() {
    $("padLayer").innerHTML = PADS.map((pad) =>
      `<div class="pt" data-ch="${pad.ch}" title="CH${pad.ch} · ${pad.pos} · ${pad.x.toFixed(2)}, ${pad.y.toFixed(2)} mm">`
      + `<div class="btn">${pad.ch}</div>`
      + `<div class="lbl ${pad.lp}"><b>CH${pad.ch} · ${pad.pos}</b>`
      + `<small>${pad.x.toFixed(2)}, ${pad.y.toFixed(2)} mm</small>`
      + `<small class="v">— N</small></div></div>`).join("");
  }

  function applyGeometry() {
    const full = fov === "full";
    const mm = full ? 220 : 72;
    const rp = 16.35 / mm * 100;
    const face = $("face");
    face.classList.toggle("full", full);
    face.classList.toggle("disc", !full);
    $("faceimg").src = full ? "/force_face_full.png" : "/force_face_fruit.png";
    $("plate").style.width = $("plate").style.height = `${(55 / mm * 100).toFixed(2)}%`;
    $("ring").style.width = $("ring").style.height = `${(2 * 16.35 / mm * 100).toFixed(2)}%`;
    for (const pad of PADS) {
      const node = document.querySelector(`.pt[data-ch="${pad.ch}"]`);
      if (!node) continue;
      const rad = pad.ang * Math.PI / 180;
      node.style.left = `${(50 + rp * Math.sin(rad)).toFixed(2)}%`;
      node.style.top = `${(50 - rp * Math.cos(rad)).toFixed(2)}%`;
    }
    $("fovtext").textContent = full
      ? "整机 220 mm 视场 · 完整 CAD 工具视图"
      : "盘面 72 mm 视场 · 盖板 Ø55 · 五点 R 16.35 mm";
    $("viewcap").textContent = full
      ? "CAD 工具视图 · 整机 220 mm（悬停点位看坐标；切盘面看放大标注）· 右 = +X_tcp · +Z 出屏"
      : "CAD 工具视图 · 盘面 72 mm · 右 = +X_tcp · +Z 出屏";
    document.querySelectorAll("[data-fov]").forEach((button) => {
      const active = button.dataset.fov === fov;
      button.classList.toggle("on", active);
      button.setAttribute("aria-pressed", String(active));
    });
  }

  function placeCop(data) {
    const mark = $("copmark");
    if (!data || !data.available) { mark.hidden = true; return; }
    const values = valuesById(data);
    let total = 0;
    let cx = 0;
    let cy = 0;
    PADS.forEach((pad) => {
      const value = values[pad.ch];
      if (!value || (value.status !== 0 && value.status !== 1)) return;
      total += value.force;
      cx += value.force * pad.x;
      cy += value.force * pad.y;
    });
    if (total <= 0.05) { mark.hidden = true; return; }
    const mm = fov === "full" ? 220 : 72;
    const x = 50 + (cx / total) / mm * 100;
    const y = 50 - (cy / total) / mm * 100;
    mark.style.left = `${Math.max(1, Math.min(99, x)).toFixed(2)}%`;
    mark.style.top = `${Math.max(1, Math.min(99, y)).toFixed(2)}%`;
    mark.hidden = false;
    mark.title = `CoP (${(cx / total).toFixed(1)}, ${(cy / total).toFixed(1)}) mm`;
  }

  function renderChannels(data) {
    const values = valuesById(data);
    $("channels").innerHTML = PADS.map((pad) => {
      const value = values[pad.ch];
      return `<div class="channel"><span>CH${pad.ch} ${pad.pos}</span>`
        + `<em>${pad.x.toFixed(2)}, ${pad.y.toFixed(2)} mm</em>`
        + `<b>${value ? `${value.force.toFixed(2)} N` : "—"}</b></div>`;
    }).join("");
    document.querySelectorAll(".pt").forEach((node) => {
      const value = values[Number(node.dataset.ch)];
      const label = node.querySelector(".lbl .v");
      if (label) label.textContent = value ? `${value.force.toFixed(2)} N` : "— N";
      node.title = `CH${node.dataset.ch}${value ? ` · ${value.force.toFixed(2)} N` : " · 无数据"}`;
    });
  }

  function renderTable(data) {
    const values = valuesById(data);
    $("rows").innerHTML = PADS.map((pad) => {
      const value = values[pad.ch];
      return `<tr><td>CH${pad.ch} ${pad.pos}</td>`
        + `<td>${pad.x.toFixed(2)} / ${pad.y.toFixed(2)}</td>`
        + `<td>${value ? value.force.toFixed(3) : "—"}</td>`
        + `<td>${value ? value.travel.toFixed(2) : "—"}</td>`
        + `<td>${value ? safe(value.ad) : "—"}</td>`
        + `<td>${value ? safe(NAMES[value.status] || value.status) : "—"}</td></tr>`;
    }).join("");
    $("frameHint").textContent = data && data.available
      ? `frame_id=${data.frame_id} · seq=${data.sequence} · age=${Number(data.age_s).toFixed(2)} s`
      : "无实时帧";
  }

  function render(data) {
    latest = data;
    if (!data || !data.available) {
      forceAvailable = false;
      const reason = data?.error === "no_frame"
        ? "force_enabled=false 或未发布 /force/sample" : (data?.error || "无实时帧");
      const stale = data?.error === "stale";
      setStatus(stale ? "warn" : "bad", stale ? "数据陈旧" : "力源未启用",
        stale ? `力帧已 ${Number(data.age_s).toFixed(1)} s 未更新；等待新帧。`
          : `${reason}；CAD 点位/坐标系仍显示。`);
      $("total").textContent = "—";
      $("copmag").textContent = "—";
      $("tableHint").textContent = "无实时帧";
      placeCop(null);
      renderChannels(null);
      renderTable(null);
      return;
    }
    forceAvailable = true;
    setStatus("ok", "在线", "只读监控；力值不参与抓取许可。");
    const values = valuesById(data);
    let total = 0;
    let cx = 0;
    let cy = 0;
    PADS.forEach((pad) => {
      const value = values[pad.ch];
      if (!value || (value.status !== 0 && value.status !== 1)) return;
      total += value.force;
      cx += value.force * pad.x;
      cy += value.force * pad.y;
    });
    const sum = (data.force_n || []).reduce((acc, value) => acc + Number(value || 0), 0);
    $("total").textContent = `${sum.toFixed(2)} N`;
    $("copmag").textContent = total > 0.05
      ? `${Math.hypot(cx / total, cy / total).toFixed(1)} mm` : "—";
    $("tableHint").textContent = "frame tcp";
    placeCop(data);
    renderChannels(data);
    renderTable(data);
  }

  function describeConnection(status) {
    if (status.state === "offline") {
      return {label: "连接中断", detail: status.error || "保留最后一次读数"};
    }
    if (status.state === "stale") {
      return {label: "数据陈旧", detail: `${status.ageS?.toFixed?.(1) ?? "—"}s 未更新`};
    }
    if (status.state === "connecting") return {label: "正在连接", detail: "等待 /api/force"};
    return forceAvailable
      ? {label: "已连接", detail: "实时力帧"}
      : {label: "服务在线", detail: latest?.error === "stale" ? "力数据陈旧" : "力源未启用"};
  }

  function clearReadings() {
    forceAvailable = false;
    latest = null;
    $("total").textContent = "—";
    $("copmag").textContent = "—";
    $("tableHint").textContent = "无实时帧";
    placeCop(null);
    renderChannels(null);
    renderTable(null);
  }

  const feed = PeachApi.createFeed({
    name: "force", path: "/api/force", intervalMs: 250, timeoutMs: 6000,
    staleAfterMs: 1500, onData: render,
    onStatus: (status) => {
      if (!forceAvailable || !["stale", "offline"].includes(status.state)) return;
      clearReadings();
      setStatus(status.state === "stale" ? "warn" : "bad", status.label,
        "力投影未更新；旧读数与 CoP 已清除，等待新帧。");
    },
    onError: (error) => {
      setStatus("bad", "连接失败", `${error.message}；CAD 点位仍显示。`);
      clearReadings();
      $("frameHint").textContent = "力源请求失败";
    },
  });

  PeachShell.mount({
    active: "force",
    title: "五点力",
    eyebrow: "PEACH / FIVE-POINT FORCE",
    feed,
    describe: describeConnection,
    footer: ["只读实时页；历史力帧与接触段在 observability 会话 bag。", "力值不参与抓取许可。"],
  });

  buildPads();
  document.querySelectorAll("[data-fov]").forEach((button) => {
    button.addEventListener("click", () => {
      fov = button.dataset.fov;
      applyGeometry();
      placeCop(latest);
    });
  });
  $("faceimg")?.addEventListener("error", () => {
    setStatus("warn", "CAD 图缺失", "素材未安装；点位与读数仍可用。");
  });
  applyGeometry();
  renderChannels(null);
  renderTable(null);
  feed.start();
})();
