"use strict";

// Browser waiting limits do not cancel a ROS goal and must never trigger POST retries.
const stateFeed = {inFlight: false, lastSuccessAt: 0, error: ""};
const pendingDebug = new Set();
const fetchControllers = new Set();
let trajectoryInFlight = false;
let trajectoryPayload = null;

async function requestJson(url, options = {}, timeoutMs = 6000) {
  const controller = new AbortController();
  fetchControllers.add(controller);
  let timedOut = false;
  const timer = setTimeout(() => { timedOut = true; controller.abort(); }, timeoutMs);
  try {
    const response = await fetch(url, {...options, signal: controller.signal});
    const body = await response.json();
    return {response, body};
  } catch (error) {
    if (timedOut) throw new Error("等待响应超时");
    throw error;
  } finally {
    clearTimeout(timer);
    fetchControllers.delete(controller);
  }
}

function classifyDebugResponse(response, body) {
  const running = response.status === 202;
  const unknown = response.status >= 500 && response.status !== 503;
  const failed = body.result?.success === false || body.result?.accepted === false;
  const rejected = !response.ok || body.accepted !== true;
  const tone = running || unknown ? "warn" : failed || rejected ? "err" :
    body.result?.success === true ? "ok" : "info";
  const label = running ? "仍在执行 · 可取消" : unknown ? "结果未知" :
    failed ? "操作失败" : rejected ? "请求被拒" : tone === "ok" ? "操作成功" : "已受理 · 查看结果";
  return {running, unknown, failed, rejected, tone, label};
}
