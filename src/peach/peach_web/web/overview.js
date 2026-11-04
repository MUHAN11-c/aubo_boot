"use strict";

// System overview: one state feed + one force feed. All cards are read-only;
// no operation is reachable from this page.
(function () {
  const {$, safe} = Peach.ui;
  const BATCH = ["等待就绪", "WAIT_LOCK", "RUNNING", "等待安全暂停点", "PAUSED", "MAINTENANCE",
    "已结算", "需要恢复", "已中断"];
  let stateFeed = null;
  let forceFeed = null;
  let lastImageKey = "";

  function text(id, value) {
    const node = $(id);
    if (node) node.textContent = value == null || value === "" ? "—" : String(value);
  }

  function short(value, n = 34) {
    const textValue = String(value || "");
    return textValue.length > n ? `${textValue.slice(0, n - 1)}…` : textValue;
  }

  function setState(id, value, tone = "") {
    const node = $(id);
    if (!node) return;
    node.textContent = value || "—";
    node.classList.remove("state-ok", "state-warn", "state-bad", "state-idle");
    if (tone) node.classList.add(`state-${tone}`);
  }

  function completionText(row, task) {
    if (row && row.completion_level != null) {
      const level = Number(row.completion_level);
      const names = ["未到达预抓取", "预抓取到位", "预抓取已核验", "套入完成",
        "刀指令已受理", "切断已确认", "撤退已确认", "采摘已确认"];
      return `L${level}（${names[level] || "未知级别"}）`;
    }
    if (task.progress != null) return `${(Number(task.progress) * 100).toFixed(0)}%`;
    return "—";
  }

  function renderImage(image) {
    const img = $("preview");
    const empty = $("previewEmpty");
    const note = $("imageNote");
    const badge = $("imageAge");
    const key = `${image?.stamp_ns || ""}:${image?.available}`;
    if (image?.stamp_ns && (image.available || image.error === "stale")) {
      if (key !== lastImageKey) {
        lastImageKey = key;
        img.src = `/api/images/raw.jpg?ts=${encodeURIComponent(image.stamp_ns)}`;
      }
      img.hidden = false;
      empty.hidden = true;
      badge.textContent = Number.isFinite(Number(image.age_s))
        ? `${Number(image.age_s).toFixed(1)} s` : "—";
      badge.classList.remove("state-warn");
      if (image.error === "stale") badge.classList.add("state-warn");
      note.textContent = `${image.frame_id || "camera"} · ${image.width || "?"}×${image.height || "?"}`
        + (image.available ? " · 实时" : " · 陈旧帧");
    } else {
      img.hidden = true;
      img.removeAttribute("src");
      lastImageKey = "";
      empty.hidden = false;
      empty.textContent = image?.error === "no_frame" || !image
        ? "等待相机帧（camera_enabled）" : `相机不可用：${image.error || "未知原因"}`;
      badge.textContent = "无帧";
      badge.classList.add("state-warn");
      note.textContent = "等待相机；检查 camera_enabled 与相机网络。";
    }
  }

  function updateState(snapshot) {
    const facts = snapshot.startup?.facts || {};
    const task = snapshot.task_executor?.state || {};
    const targets = snapshot.perception?.targets || {};
    const harvest = snapshot.perception?.harvest || {};
    const recon = snapshot.reconstruction?.status || {};
    const diag = snapshot.reconstruction?.diagnostics || {};
    const arm = snapshot.manipulation?.status || {};
    const check = snapshot.selfcheck?.report || {};
    const nodes = snapshot.diagnostics?.nodes || {};
    const record = snapshot.record?.info || {};
    const live = snapshot.ledger?.live || {};
    const rows = live.rows || [];
    const lastRow = rows[rows.length - 1] || null;
    const batch = BATCH[task.batch_state] || `state ${task.batch_state ?? "—"}`;

    setState("mode", facts.hardware_mode === "real" ? "真机" : facts.hardware_mode === "mock" ? "mock" : "—",
      facts.hardware_mode === "real" ? "warn" : "");
    text("runId", short(task.run_id || "—", 30));
    text("phase", task.message || "—");
    setState("recovery", task.recovery_required ? "需人工恢复" : "无",
      task.recovery_required ? "bad" : "ok");

    text("batchBadge", batch);
    text("target", task.target_id || lastRow?.target_id || "—");
    text("batchPhase", task.target_phase ?? "—");
    text("completion", completionText(lastRow, task));
    text("action", task.action_active ? "执行中" : "空闲");
    text("batchNote", task.message || "等待调度状态");

    const targetCount = (targets.observations || []).length;
    setState("sceneState", targetCount ? "有目标" : "无目标", targetCount ? "ok" : "warn");
    text("sceneMain", `${targetCount} 个目标 · ${targets.target_set_locked ? "已锁" : "采集中"}`);
    const timing = harvest.timing || {};
    text("sceneSub", timing.total_ms
      ? `${(1000 / timing.total_ms).toFixed(1)} FPS · ${Number(timing.total_ms).toFixed(0)} ms`
      : (harvest.scene_epoch ? `epoch ${harvest.scene_epoch}` : "等待帧"));

    setState("reconState", recon.state || "等待", recon.state === "READY" ? "ok" : "");
    text("reconMain", diag.target_id ? `target ${diag.target_id}` : "无目标");
    text("reconSub", diag.captured_views != null
      ? `${diag.captured_views} views · ${diag.state || ""}` : (recon.state || "—"));

    setState("supState", batch, task.recovery_required ? "bad" : task.batch_state === 6 ? "ok" : "");
    text("supMain", task.message || "—");
    text("supSub", `target=${task.target_id || "—"} · action=${task.action_active ? "active" : "idle"}`);

    setState("armState", arm.contact_recovery_required ? "需恢复" : (task.action_active ? "执行中" : "待命"),
      arm.contact_recovery_required ? "bad" : "");
    text("armMain", arm.message || "—");
    text("armSub", `exec=${arm.execution_enabled ? "on" : "off"} · grasp=${arm.grasp_enabled ? "on" : "off"} · tool=${arm.tool_enabled ? "on" : "off"}`);

    setState("obsState", record.enabled ? "录制中" : "未录制", record.enabled ? "ok" : "warn");
    text("obsSub", record.session ? short(String(record.session).split("/").pop(), 26) : "无会话");

    setState("checkState", check.passed ? "通过" : (check.trigger ? "有警告" : "等待"),
      check.passed ? "ok" : "warn");
    text("checkSub", check.summary || "等待首次自检");

    const levels = Object.values(nodes).map((entry) => Number(entry.level || 0));
    const valid = levels.length;
    const worst = valid ? Math.max(...levels) : 0;
    const abnormal = levels.filter((level) => level > 0).length;
    setState("diagState", valid ? (worst ? `异常 ${abnormal}` : "正常") : "无数据",
      valid ? (worst > 1 ? "bad" : worst ? "warn" : "ok") : "");
    text("diagSub", `${valid} 个节点 · ${check.status || "自检等待"}`);

    $("ledger").innerHTML = rows.length
      ? rows.slice(-6).map((row) => `<div class="ledger-row"><b>${safe(row.target_id || "—")}</b>`
        + `<span>${safe(row.outcome_name || row.outcome || "—")}</span>`
        + `<span>L${safe(row.completion_level ?? "—")}</span>`
        + `<span>${safe(short(row.reason || "", 70))}</span></div>`).join("")
      : '<p class="empty">等待当前批次入账</p>';

    renderImage(snapshot.images?.raw);
    document.body.classList.toggle("data-stale", stateFeed?.isFresh() !== true);
  }

  function updateForce(force) {
    if (!force || !force.available) {
      const stale = force?.error === "stale";
      setState("forceState", stale ? "数据陈旧" : "未启用", "warn");
      text("forceSub", stale ? `力帧已 ${Number(force.age_s).toFixed(1)} s 未更新`
        : force?.error === "no_frame"
        ? "force_enabled=false 或未发布 /force/sample" : (force?.error || "无实时帧"));
      return;
    }
    const total = (force.force_n || []).reduce((sum, value) => sum + Number(value || 0), 0);
    setState("forceState", "在线", "ok");
    text("forceSub", `${total.toFixed(2)} N · seq ${force.sequence} · age ${Number(force.age_s).toFixed(2)} s`);
  }

  function updateAlert(status) {
    const alert = $("overview-alert");
    if (!alert) return;
    const offline = status.state === "offline" || status.state === "stale";
    alert.hidden = !offline;
    alert.textContent = status.state === "offline"
      ? `状态连接中断：${status.error || "保留最后一次快照"}。页面数据可能陈旧，不能代替实时状态。`
      : status.state === "stale"
        ? `状态快照已 ${status.ageS?.toFixed?.(1) ?? "—"}s 未更新，页面保留最后画面。`
        : "";
    document.body.classList.toggle("data-stale", status.state !== "online");
  }

  stateFeed = PeachApi.createFeed({
    name: "overview-state", path: "/api/state", intervalMs: 1000, timeoutMs: 6000,
    staleAfterMs: 5000, onData: updateState, onStatus: updateAlert,
  });
  forceFeed = PeachApi.createFeed({
    name: "overview-force", path: "/api/force", intervalMs: 750, timeoutMs: 6000,
    staleAfterMs: 1500, onData: updateForce,
    onStatus: (status) => {
      if (status.state === "stale") {
        setState("forceState", "数据陈旧", "warn");
        text("forceSub", "力投影未更新；等待新帧");
      } else if (status.state === "offline") {
        setState("forceState", "不可用", "bad");
        text("forceSub", "力源请求失败");
      }
    },
    onError: () => { setState("forceState", "不可用", "bad"); text("forceSub", "力源请求失败"); },
  });

  PeachShell.mount({
    active: "overview",
    title: "系统总览",
    eyebrow: "PEACH / OPERATIONS",
    feed: stateFeed,
    footer: ["Peach 系统总览 · 8090 · 只读监控与显式调试；不替代硬件急停。", "先核对现场与使能。"],
  });

  $("preview")?.addEventListener("error", () => {
    const img = $("preview");
    if (!img) return;
    img.hidden = true;
    const empty = $("previewEmpty");
    if (empty) { empty.hidden = false; empty.textContent = "相机预览不可用"; }
    text("imageNote", "相机预览请求失败；不改变状态新鲜度。");
  });

  stateFeed.start();
  forceFeed.start();
})();
