"use strict";

// Process monitor entry point. Networking lives in PeachApi feeds; snapshot
// rendering stays in monitor-renderers.js; camera/3D stays in visualization.js.
// This file only composes them, gates the three operation buttons, and keeps
// the page honest about stale data.
(function () {
  const {$, setText, numeric} = Peach.ui;
  const debugApi = PeachDebug;
  const api = PeachApi.api;
  const pendingDebug = new Set();
  const OP_NOTICE_DEFAULT = "操作结果将在此显示，详细响应见单步调试。";
  let visualization = null;
  let trajectoryPayload = null;
  let stateFeed = null;
  let trajectoryFeed = null;

  function stateIsFresh() {
    return stateFeed?.isFresh() === true;
  }

  function labTrajectory() {
    const feedStatus = trajectoryFeed?.status() || {};
    const age = trajectoryFeed?.lastSuccessAt
      ? (Date.now() - trajectoryFeed.lastSuccessAt) / 1000 : Infinity;
    const fresh = trajectoryFeed?.isFresh() === true;
    const frames = Object.fromEntries(Object.entries(trajectoryPayload?.tool_frames || {}).map(([name, tf]) => {
      const frameAge = numeric(tf.age_s) ? Number(tf.age_s) + age : null;
      const valid = fresh && tf.available === true && (frameAge === null || frameAge <= 1);
      return [name, {
        ...tf, age_s: frameAge, available: valid,
        error: valid ? "" : tf.error || (fresh ? "stale" : "trajectory_unavailable"),
      }];
    }));
    return {
      ...(trajectoryPayload || {}),
      feed_state: feedStatus.state || "connecting",
      tool_frames: frames,
    };
  }

  function labState() {
    return {
      ...(monitorState || {}),
      ui: {
        fresh: stateIsFresh(),
        pending: Array.from(pendingDebug),
      },
    };
  }

  function updateLab() {
    visualization?.update(labState(), labTrajectory());
  }

  function setNotice(message, tone = "") {
    const notice = $("debug-notice");
    if (!notice) return;
    notice.textContent = message;
    notice.className = `debug-notice ${tone}`;
  }

  function updateConnectionView(status = stateFeed?.status()) {
    const snapshot = status || {state: "connecting", label: "正在连接", ageS: null};
    const fresh = snapshot.state === "online";
    document.body.classList.toggle("data-stale", !fresh);
    const alert = $("feed-alert");
    if (alert) {
      alert.hidden = fresh;
      if (snapshot.state === "connecting") {
        alert.textContent = "等待状态连接，操作将在数据就绪后开放。";
      } else if (snapshot.state === "offline") {
        const age = snapshot.ageS === null ? "从未收到" : `${snapshot.ageS.toFixed(0)}s 前`;
        alert.textContent = `当前显示最后一次快照（${age}）。${snapshot.error || "状态更新失败"}；普通操作已暂停，可尝试取消在途动作。`;
      } else if (snapshot.state === "stale") {
        alert.textContent = `当前快照已 ${snapshot.ageS === null ? "—" : snapshot.ageS.toFixed(1)}s 未更新。普通操作已暂停，可尝试取消在途动作。`;
      }
    }
    updateDebugControls();
  }

  function actionPayload(el) {
    const action = el.dataset.debug;
    if (action === "control_service") {
      const command = el.dataset.payload ? JSON.parse(el.dataset.payload).command : "CANCEL_NOW";
      const seqUnchecked = command === "PAUSE" || command === "CANCEL_NOW";
      return {command, expected_state_seq: seqUnchecked ? 0 : Number(monitorState?.task_executor?.state?.state_seq) || 0,
        reason: "web 过程页"};
    }
    if (el.dataset.payload) {
      try { return JSON.parse(el.dataset.payload); } catch (_) { return {}; }
    }
    return {};
  }

  function updateDebugControls() {
    const enabled = monitorState?.debug?.enabled === true;
    document.querySelectorAll("[data-debug]").forEach((el) => {
      const action = el.dataset.debug;
      const payload = actionPayload(el);
      const key = debugApi.requestKey(action, payload);
      const busy = pendingDebug.has(key);
      const cancel = debugApi.isCancelRequest(action, payload);
      const block = debugApi.blockReason(action, payload, monitorState);
      if (!el.dataset.label) el.dataset.label = el.textContent;
      el.disabled = !enabled || (!stateIsFresh() && !cancel) || busy || Boolean(block);
      el.setAttribute("aria-busy", String(busy));
      el.textContent = busy ? `${el.dataset.label} · 等待中` : el.dataset.label;
      el.title = busy ? "请求仍在等待结果，取消入口保持独立。"
        : !enabled ? "后端调试入口尚未就绪或已关闭。"
          : !stateIsFresh() && !cancel ? "状态快照已失联，请等待恢复。"
            : block;
    });
  }

  async function postOperation(action, payload, options = {}) {
    const key = debugApi.requestKey(action, payload || {});
    if (pendingDebug.has(key)) {
      setNotice("前一次请求仍在等待结果，请勿重复提交。", "warn");
      return null;
    }
    const cancel = debugApi.isCancelRequest(action, payload || {});
    if (options.confirmText) {
      const confirmed = await PeachShell.confirm({
        title: options.confirmTitle || "确认操作",
        message: options.confirmText,
        confirmText: options.confirmLabel || "确认",
        danger: cancel,
      });
      if (!confirmed) return null;
    }
    if (monitorState?.debug?.enabled !== true || (!stateIsFresh() && !cancel)) {
      updateConnectionView();
      setNotice("请求未发送：调试入口关闭或状态快照已陈旧，请等状态恢复后再操作。", "warn");
      return null;
    }
    const block = debugApi.blockReason(action, payload || {}, monitorState);
    if (block) {
      updateDebugControls();
      setNotice(`请求未发送：${block}。`, "warn");
      return null;
    }
    pendingDebug.add(key);
    updateLab();
    updateDebugControls();
    setNotice("请求已发送，等待后端结果；取消入口仍可使用。");
    let outcome;
    try {
      const {response, body} = await api.debug(action, payload || {});
      if (!body || typeof body !== "object" || Array.isArray(body)) {
        throw new PeachApi.ApiError("后端结果格式无效", {kind: "parse", status: response.status});
      }
      const classification = debugApi.classify(response, body);
      const message = debugApi.messageFor(classification, body);
      setNotice(`${classification.label} · ${message}`, classification.tone);
      outcome = {
        ok: response.ok && body.result?.success === true && !classification.running
          && !classification.unknown && !classification.rejected,
        status: response.status, body, classification, message,
      };
      if (classification.tone === "err") {
        PeachShell.notify(`${classification.label} · ${message}`, {tone: "err", timeout: 8000});
      }
    } catch (error) {
      const message = debugApi.messageFor(
        {unknown: true, label: "结果未知"}, null, error.message || "网络请求失败");
      setNotice(`结果未知 · ${message}`, "warn");
      PeachShell.notify(`结果未知 · ${message}`, {tone: "warn", timeout: 9000});
      outcome = {ok: false, status: null, body: null, message, unknown: true};
    } finally {
      pendingDebug.delete(key);
      updateDebugControls();
      updateLab();
    }
    return outcome;
  }

  function bindOperations() {
    document.querySelectorAll("[data-debug]").forEach((el) => {
      el.addEventListener("click", () => {
        try {
          const action = el.dataset.debug;
          const payload = actionPayload(el);
          postOperation(action, payload, {
            confirmTitle: el.dataset.confirmTitle,
            confirmText: el.dataset.confirm === "1"
              ? el.dataset.confirmText || `确认发送 ${action}？` : "",
            confirmLabel: el.dataset.confirmLabel || "确认",
          });
        } catch (error) {
          setNotice(`请求未发送：${error.message}`, "err");
        }
      });
    });
    setNotice(OP_NOTICE_DEFAULT);
    updateDebugControls();
  }

  function bindLogFilters() {
    const refresh = () => renderLogs((monitorState && monitorState.logs) || []);
    $("log-level")?.addEventListener("change", refresh);
    $("log-node")?.addEventListener("input", refresh);
    $("log-keyword")?.addEventListener("input", refresh);
    $("log-pause")?.addEventListener("click", (event) => {
      logsPaused = !logsPaused;
      event.currentTarget.textContent = logsPaused ? "恢复日志" : "暂停日志";
      if (!logsPaused) refresh();
    });
  }

  function handleState(state) {
    if (!state || typeof state !== "object" || Array.isArray(state)) return;
    monitorState = state;
    renderMonitorSnapshot(state);
    updateLab();
    updateDebugControls();
  }

  function handleTrajectory(payload) {
    trajectoryPayload = payload;
    renderTrajHud(payload);
    renderTcpMini(payload);
    updateLab();
  }

  function handleTrajectoryError(error) {
    updateLab();
    setText("traj-status", `轨迹更新不可用 · ${error.message || "请求失败"} · 保留上次画面`);
  }

  function updateHeader() {
    PeachShell.syncHeaderHeight();
    if (trajectoryPayload) renderTcpMini(trajectoryPayload);
  }

  stateFeed = PeachApi.createFeed({
    name: "state", path: "/api/state", intervalMs: 1000, timeoutMs: 6000,
    staleAfterMs: 5000, onData: handleState, onStatus: updateConnectionView,
    onError: (error) => { if (!monitorState) setNotice(`状态连接失败：${error.message}`, "warn"); },
  });
  trajectoryFeed = PeachApi.createFeed({
    name: "trajectory", path: "/api/trajectory", intervalMs: 400, timeoutMs: 6000,
    staleAfterMs: 5000, onData: handleTrajectory, onError: handleTrajectoryError,
  });

  PeachShell.mount({
    active: "monitor",
    title: "采摘过程监控",
    eyebrow: "PEACH / HARVEST OPERATIONS",
    feed: stateFeed,
    footer: ["过程记录 · 监控 · 轨迹 · 单步调试 · 127.0.0.1:8090", "上电与安全恢复由现场完成"],
  });

  if (window.PeachVisualization && $("monitor-visualization-host")) {
    visualization = window.PeachVisualization.create($("monitor-visualization-host"));
  }

  bindOperations();
  bindLogFilters();
  updateConnectionView(stateFeed.status());
  stateFeed.start();
  trajectoryFeed.start();
  updateLab();

  document.addEventListener("visibilitychange", () => {
    visualization?.setActive(!document.hidden);
  });
  window.addEventListener("resize", updateHeader);
  window.addEventListener("pageshow", (event) => {
    if (!event.persisted) return;
    visualization?.setActive(true);
    updateConnectionView(stateFeed.status());
    stateFeed.start();
    trajectoryFeed.start();
  });
  window.addEventListener("pagehide", (event) => {
    visualization?.setActive(false);
    if (!event.persisted) {
      visualization?.dispose();
      PeachApi.abortAll();
    }
  });
})();
