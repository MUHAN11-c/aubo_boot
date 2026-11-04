"use strict";

// Pure debug-request policy shared by the monitor operation bar and the
// step-debug console. UI feedback mirrors the backend, but never replaces it:
// debug.enabled / debug.motion_enabled / ExecutionAuthority stay authoritative.
(function (root) {
  const MOTION_ACTIONS = new Set([
    "survey_action", "photo_pose_service", "arm_service", "run_harvest_action",
  ]);

  function isCancelRequest(action, payload = {}) {
    if (action === "cancel" || action === "skill_cancel_service") return true;
    return action === "control_service"
      && String(payload?.command || "").toUpperCase() === "CANCEL_NOW";
  }

  function isMotionRequest(action, payload = {}) {
    if (MOTION_ACTIONS.has(action)) return true;
    if (action === "execute_action") {
      return String(payload?.mode || "PREVIEW").toUpperCase() !== "PREVIEW";
    }
    return action === "control_service"
      && ["RESUME", "EXIT_MAINTENANCE"].includes(String(payload?.command || "").toUpperCase());
  }

  function requestKey(action, payload = {}) {
    if (action === "cancel") return `cancel:${payload?.target || ""}`;
    if (action === "control_service"
      && String(payload?.command || "").toUpperCase() === "CANCEL_NOW") {
      return "control_service:CANCEL_NOW";
    }
    return action;
  }

  function blockReason(action, payload = {}, state = null) {
    if (isMotionRequest(action, payload) && state?.debug?.motion_enabled !== true) {
      return "后端动臂入口未放行";
    }
    if (action === "photo_pose_service") {
      const arm = state?.manipulation?.status || {};
      if (arm.contact_recovery_required || state?.task_executor?.state?.recovery_required) {
        return "待人工恢复确认；不会自动确认恢复";
      }
      if (arm.running === true) return "臂周期正在运行，请等待结束或取消执行";
      if (arm.execution_enabled === true && arm.execution_armed !== true) {
        return "实际移动前须通过 /peach_arm/set_execution_armed 完成本次手动授权";
      }
    }
    return "";
  }

  function classify(response, body) {
    const status = response?.status ?? null;
    const result = body?.result || {};
    const running = status === 202;
    const unknown = status === null || (status >= 500 && status !== 503);
    const failed = result.success === false || result.accepted === false;
    const rejected = status === null || !response.ok || body?.accepted !== true;
    const tone = running || unknown ? "warn" : failed || rejected ? "err"
      : result.success === true ? "ok" : "info";
    const label = running ? "仍在执行 · 可取消"
      : unknown ? "结果未知"
        : failed ? "操作失败"
          : rejected ? "请求被拒"
            : tone === "ok" ? "操作成功" : "已受理 · 查看结果";
    return {running, unknown, failed, rejected, tone, label, status};
  }

  function messageFor(classification, body, networkError = "") {
    if (networkError) {
      return `${networkError}。ROS 操作结果未知，请核对状态或使用取消入口，勿直接重复发送。`;
    }
    const detail = body?.message || body?.result?.message || "";
    if (classification.unknown) {
      return `${detail || "后端未返回操作终态"}。请核对状态或使用取消入口，勿直接重复发送。`;
    }
    if (classification.running) {
      return detail || "后端仍在执行；可稍后查询状态或使用取消入口。";
    }
    if (detail) return detail;
    if (classification.failed) return "请求已受理，但服务返回失败。";
    if (classification.rejected) return "请求被拒绝；请查看详情或后端状态。";
    return classification.label;
  }

  function freshRequest(prefix = "grasp") {
    const now = new Date();
    const pad = (value, width = 2) => String(value).padStart(width, "0");
    const stamp = `${now.getFullYear()}${pad(now.getMonth() + 1)}${pad(now.getDate())}`
      + `_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`
      + `_${pad(now.getMilliseconds(), 3)}`;
    return `${prefix}_${stamp}`;
  }

  root.PeachDebug = Object.freeze({
    isCancelRequest, isMotionRequest, requestKey, blockReason, classify,
    messageFor, freshRequest,
  });
})(typeof window !== "undefined" ? window : globalThis);
