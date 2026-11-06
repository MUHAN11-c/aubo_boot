"use strict";

// Shared ROSbridge / explicit HTTP access contract for every page:
//  - GET only for periodic snapshots; a feed never overlaps its own requests.
//  - POST is never retried automatically; timeout/network loss stays "unknown".
//  - All waiters receive both the Response and parsed JSON so the caller can
//    classify 202 / 423 / service-level failures instead of collapsing them.
(function (root) {
  class ApiError extends Error {
    constructor(message, options = {}) {
      super(message);
      this.name = "ApiError";
      this.kind = options.kind || "http";
      this.status = options.status ?? null;
      this.body = options.body ?? null;
      this.url = options.url || "";
    }
  }

  const liveControllers = new Set();

  async function requestJson(path, options = {}) {
    const {
      method = "GET",
      body = null,
      timeoutMs = 6000,
      signal = null,
      headers = null,
      throwOnHttpError = true,
    } = options;
    if (!Number.isFinite(timeoutMs) || timeoutMs <= 0 || timeoutMs > 2147483647) {
      throw new TypeError("timeoutMs 必须是有效的正数毫秒时限");
    }
    const deadline = performance.now() + timeoutMs;
    const controller = new AbortController();
    liveControllers.add(controller);
    let timedOut = false;
    let externallyAborted = false;
    const onAbort = () => { externallyAborted = true; controller.abort(); };
    if (signal) {
      if (signal.aborted) onAbort();
      else signal.addEventListener("abort", onAbort, {once: true});
    }
    const timer = setTimeout(() => { timedOut = true; controller.abort(); }, timeoutMs);
    function checkCompletion(body = null, status = null) {
      const partial = Array.isArray(body?.results);
      // Browser long tasks can delay the timer past response completion.
      if (timedOut || performance.now() >= deadline) {
        const message = partial ? "等待响应超时；已保留逐项结果，请重新读回参数"
          : method === "GET" ? "等待响应超时" : "等待响应超时，命令结果未知；请查询状态";
        throw new ApiError(message, {kind: "timeout", url: path, body, status});
      }
      if (controller.signal.aborted) {
        throw new ApiError(partial ? "请求已取消；已保留逐项结果" : "请求已取消", {kind: "abort", url: path, body, status});
      }
    }
    let response;
    let text;
    try {
      checkCompletion();
      if (new URLSearchParams(root.location?.search || "").get("transport") !== "http") {
        if (!root.PeachRosbridge && /^\/api\/(state|force|trajectory|params|debug)(?:[/?]|$)/.test(path)) {
          throw new ApiError("ROSbridge 资源未加载", {kind: "network", url: path});
        }
        const bridged = await root.PeachRosbridge?.request(path, {
          method, body, timeoutMs, deadline, signal: controller.signal,
        });
        if (bridged) {
          checkCompletion(bridged.body, bridged.response.status);
          if (!bridged.response.ok && throwOnHttpError) {
            throw new ApiError(bridged.body?.message || `ROS ${bridged.response.status}`, {
              kind: "http", status: bridged.response.status, body: bridged.body, url: path,
            });
          }
          return bridged;
        }
      }
      checkCompletion();
      const init = {method, cache: "no-store", signal: controller.signal};
      if (headers) init.headers = headers;
      if (body !== null && body !== undefined) init.body = body;
      response = await fetch(path, init);
      text = await response.text();
    } catch (error) {
      if (error instanceof ApiError) throw error;
      checkCompletion();
      if (externallyAborted || error?.name === "AbortError") {
        throw new ApiError("请求已取消", {kind: "abort", url: path});
      }
      throw new ApiError(error?.message || "网络请求失败", {kind: error?.kind || "network", url: path});
    } finally {
      clearTimeout(timer);
      liveControllers.delete(controller);
      if (signal) signal.removeEventListener("abort", onAbort);
    }
    let data = null;
    if (text) {
      try {
        data = JSON.parse(text);
      } catch (_) {
        checkCompletion(null, response.status);
        if (response.ok) {
          throw new ApiError("响应不是合法 JSON", {
            kind: "parse", status: response.status, url: path,
          });
        }
      }
    }
    // Keep confirmed per-item results even when the completed body arrives late.
    checkCompletion(data, response.status);
    if (!response.ok && throwOnHttpError) {
      const message = (data && (data.message || data.error))
        || `HTTP ${response.status}`;
      throw new ApiError(message, {
        kind: "http", status: response.status, body: data, url: path,
      });
    }
    return {response, body: data};
  }

  function abortAll() {
    for (const controller of Array.from(liveControllers)) controller.abort();
  }

  function query(params) {
    const search = new URLSearchParams();
    Object.entries(params || {}).forEach(([key, value]) => {
      if (value !== undefined && value !== null && value !== "") search.set(key, String(value));
    });
    const text = search.toString();
    return text ? `?${text}` : "";
  }

  const api = {
    requestJson,
    abortAll,
    state: (options) => requestJson(`/api/state?t=${Date.now()}`, options),
    trajectory: (options) => requestJson(`/api/trajectory?t=${Date.now()}`, options),
    force: (options) => requestJson("/api/force", options),
    params: (options) => requestJson("/api/params", options),
    setParams: (changes, options) => requestJson("/api/params", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({changes}),
      timeoutMs: 60000,
      throwOnHttpError: false,
      ...options,
    }),
    debug: (action, payload, options) => requestJson(
      `/api/debug/${encodeURIComponent(action)}`, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(payload || {}),
        timeoutMs: 195000,
        throwOnHttpError: false,
        ...options,
      }),
    analysisSuccess: (options) => requestJson("/api/analysis/success", options),
    analysisRecords: (options) => requestJson("/api/analysis/records", options),
    analysisReport: (path, options) => requestJson(
      `/api/analysis${query({path})}`, options),
    image: (stream) => `/api/images/${stream === "raw" ? "raw" : "perception"}.jpg`,
  };

  function createFeed(options = {}) {
    const {
      name = "state",
      path,
      intervalMs = 1000,
      timeoutMs = 6000,
      staleAfterMs = 5000,
      immediate = true,
      onData = null,
      onStatus = null,
      onError = null,
    } = options;
    if (!path) throw new Error("createFeed requires path");
    const listeners = new Set();
    let started = false;
    let timer = 0;
    let statusTimer = 0;
    let inFlight = null;
    let generation = 0;
    let lastSuccessAt = 0;
    let lastError = null;
    let visibleHandler = null;
    let pageHideHandler = null;
    let pageShowHandler = null;

    function ageMs() {
      return lastSuccessAt ? Math.max(0, Date.now() - lastSuccessAt) : null;
    }

    function status() {
      const age = ageMs();
      let state = "connecting";
      if (lastError) state = "offline";
      else if (lastSuccessAt && age > staleAfterMs) state = "stale";
      else if (lastSuccessAt) state = "online";
      const label = {
        connecting: "正在连接",
        online: "已连接",
        stale: "数据陈旧",
        offline: "连接中断",
      }[state];
      return {
        name, state, label, lastSuccessAt,
        ageS: age === null ? null : age / 1000,
        staleAfterS: staleAfterMs / 1000,
        error: lastError ? (lastError.message || String(lastError)) : "",
        kind: lastError?.kind || "",
      };
    }

    function emitStatus() {
      const snapshot = status();
      listeners.forEach((listener) => {
        try { listener(snapshot); } catch (error) { console.error(error); }
      });
      if (typeof onStatus === "function") onStatus(snapshot);
    }

    function schedule(delay = intervalMs) {
      if (!started) return;
      clearTimeout(timer);
      timer = setTimeout(loop, Math.max(50, delay));
    }

    async function fetchOnce() {
      if (inFlight) return inFlight;
      if (root.document?.hidden) {
        emitStatus();
        return null;
      }
      const requestGeneration = generation;
      inFlight = (async () => {
        try {
          const {response, body} = await requestJson(path, {timeoutMs});
          if (!started || requestGeneration !== generation) return null;
          // A ROS cache read is still the same frame, not a new success time.
          lastSuccessAt = Number.isFinite(response.receivedAt)
            ? response.receivedAt : Date.now();
          lastError = null;
          if (typeof onData === "function") onData(body, status());
          return body;
        } catch (error) {
          if (!started || requestGeneration !== generation || error?.kind === "abort") return null;
          lastError = error;
          if (typeof onError === "function") onError(error, status());
          return null;
        } finally {
          emitStatus();
        }
      })();
      try {
        return await inFlight;
      } finally {
        inFlight = null;
      }
    }

    async function loop() {
      if (!started || root.document?.hidden) return;
      await fetchOnce();
      schedule();
    }

    function onVisibility() {
      if (!started) return;
      if (root.document?.hidden) {
        clearTimeout(timer);
        timer = 0;
        emitStatus();
      } else {
        loop();
      }
    }

    function onPageHide(event) {
      if (!event?.persisted) stop({abort: true});
    }

    function onPageShow(event) {
      if (event?.persisted && !started) start();
    }

    function start() {
      if (started) return;
      started = true;
      generation += 1;
      // Keep stale status current even while the next frame request is pending.
      statusTimer = setInterval(emitStatus, Math.max(50, Math.min(250, staleAfterMs / 4)));
      visibleHandler = onVisibility;
      pageHideHandler = onPageHide;
      pageShowHandler = onPageShow;
      root.document?.addEventListener("visibilitychange", visibleHandler);
      root.addEventListener("pagehide", pageHideHandler);
      root.addEventListener("pageshow", pageShowHandler);
      if (!root.document?.hidden) {
        if (immediate) loop();
        else schedule();
      }
      emitStatus();
    }

    function stop({abort = false} = {}) {
      if (!started) return;
      started = false;
      generation += 1;
      clearTimeout(timer);
      clearInterval(statusTimer);
      timer = statusTimer = 0;
      if (visibleHandler) root.document?.removeEventListener("visibilitychange", visibleHandler);
      if (pageHideHandler) root.removeEventListener("pagehide", pageHideHandler);
      if (pageShowHandler) root.removeEventListener("pageshow", pageShowHandler);
      visibleHandler = pageHideHandler = pageShowHandler = null;
      if (abort) lastError = new ApiError("页面已关闭", {kind: "abort"});
      emitStatus();
    }

    async function refresh() {
      clearTimeout(timer);
      const result = await fetchOnce();
      if (started) schedule();
      return result;
    }

    return {
      name,
      start,
      stop,
      refresh,
      status,
      isFresh: () => {
        const snapshot = status();
        return snapshot.state === "online";
      },
      isStale: () => status().state !== "online",
      subscribe(listener) {
        listeners.add(listener);
        listener(status());
        return () => listeners.delete(listener);
      },
      get lastSuccessAt() { return lastSuccessAt; },
    };
  }

  root.PeachApi = Object.freeze({ApiError, requestJson, abortAll, api, createFeed});
})(typeof window !== "undefined" ? window : globalThis);
