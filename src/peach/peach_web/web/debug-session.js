"use strict";

// Step-debug session state machine: one-shot scene, single-pick queue and
// N-round runs. It owns request identity, epoch guards and the stop rules;
// debug.js owns DOM, parameters and rendering. No ROS action is invented
// here: every call goes through the existing /api/debug endpoints.
(function (root) {
  const freshRequest = root.PeachDebug.freshRequest;
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

  function create(options) {
    const {
      api,
      getState,
      isFresh,
      pushSettings,
      poll,
      setPriority,
      confirm,
      log,
      notify,
      onStatus,
    } = options;
    let stopRounds = false;
    let roundsBusy = false;
    let singleArmed = false;
    let singleKey = "";
    let selectionEpoch = null;
    let sessionGeneration = 0;
    let unknownRequest = "";
    const clickQueue = [];

    function observations() {
      const list = getState()?.perception?.targets?.observations || [];
      return list.filter((item) => item && item.target_id);
    }

    function snapshot() {
      return {
        stopRounds, roundsBusy, singleArmed, singleKey, selectionEpoch,
        sessionGeneration, unknownRequest,
        queued: clickQueue.map((item) => ({ids: [...item.ids], epoch: item.epoch})),
      };
    }

    function changed() {
      if (typeof onStatus === "function") onStatus(snapshot());
    }

    function assertReady() {
      if (unknownRequest) {
        throw new Error(`请求 ${unknownRequest} 结果未知；先核对状态并取消在途动作`);
      }
      if (typeof isFresh === "function" && !isFresh()) {
        throw new Error("状态已陈旧，请等待连接恢复");
      }
      const state = getState() || {};
      if (state.debug?.enabled !== true) throw new Error("调试入口未开启");
      if (state.task_executor?.state?.recovery_required === true) {
        throw new Error("等待人工恢复确认");
      }
    }

    function assertContext(epoch, ids = []) {
      assertReady();
      const targets = getState()?.perception?.targets;
      if (!Number.isInteger(epoch) || epoch <= 0 || targets?.scene_epoch !== epoch
        || targets.target_set_locked !== true) {
        throw new Error("场景已变化或未锁定，请重新选择");
      }
      const current = new Set(observations().map((item) => item.target_id));
      if (ids.some((id) => !current.has(id))) {
        throw new Error("目标已不属于当前场景，请重新选择");
      }
    }

    function assertSession(generation) {
      if (generation !== sessionGeneration || stopRounds) {
        throw new Error("本次会话已停止");
      }
    }

    async function submit(action, payload) {
      let response;
      let body;
      try {
        ({response, body} = await api.debug(action, payload));
      } catch (error) {
        if (action === "run_harvest_action"
          && (!error.status || error.status === 202 || error.status >= 500)) {
          unknownRequest = payload?.request_id || "在途批次";
          changed();
        }
        throw error;
      }
      if (!response.ok) {
        const message = body?.message || body?.error || `HTTP ${response.status}`;
        const error = new Error(message);
        error.status = response.status;
        error.body = body;
        if (action === "run_harvest_action"
          && (response.status === 202 || response.status >= 500)) {
          unknownRequest = payload?.request_id || "在途批次";
          changed();
        }
        throw error;
      }
      if (!body || typeof body !== "object") throw new Error("响应格式无效");
      if (response.status === 202 || response.status === 207 || body.accepted === false) {
        const detail = (body.results || []).filter((row) => row.accepted !== true)
          .map((row) => `${row.name || ""}: ${row.message || "未写入"}`).join("；");
        const message = detail || body.result?.message || body.message || "请求未接受";
        const error = new Error(message);
        error.status = response.status;
        error.body = body;
        if (action === "run_harvest_action") {
          unknownRequest = payload?.request_id || "在途批次";
          changed();
        }
        throw error;
      }
      if (body.accepted !== true || body.result?.accepted === false
        || body.result?.success === false) {
        throw new Error(body.result?.message || body.result?.reason
          || body.message || "操作失败");
      }
      if (["set_enables_service", "control_service", "begin_scene_service"].includes(action)
        && body.result?.accepted !== true) {
        throw new Error("服务未确认接受本次请求");
      }
      if (action.endsWith("_action") && Number(body.result?.action_status) !== 4) {
        throw new Error("动作未成功结束");
      }
      return body;
    }

    function requireNoRecovery() {
      const task = getState()?.task_executor?.state || {};
      // FULL sets the contact latch while still RUNNING; only the FSM's
      // RECOVERY_REQUIRED state means execution has stopped for operator ACK.
      if (task.batch_state === 7 && task.recovery_required === true) {
        throw new Error("等待人工恢复确认，后续目标已停止");
      }
    }

    async function finishHarvest(pending, request) {
      let settled = null;
      pending.then(
        (result) => { settled = {ok: true, result}; },
        (error) => { settled = {ok: false, error}; },
      );
      while (settled == null) {
        await poll();
        try {
          requireNoRecovery();
        } catch (error) {
          unknownRequest = request;
          changed();
          throw error;
        }
        await sleep(400);
      }
      if (!settled.ok) throw settled.error;
      // Action results and the projected state travel independently. The result
      // may arrive while the cached state still carries FULL's contact latch.
      const deadline = Date.now() + 5000;
      try {
        while (Date.now() < deadline) {
          await poll();
          const state = getState() || {};
          const task = state.task_executor?.state || {};
          const arm = state.manipulation?.status || {};
          if (task.run_id === request && task.batch_state === 6
            && task.action_active === false && task.recovery_required === false
            && arm.running === false && arm.contact_recovery_required === false
            && (typeof isFresh !== "function" || isFresh())) {
            return settled.result;
          }
          requireNoRecovery();
          await sleep(100);
        }
        throw new Error("动作已返回，尚未收到同请求的终态快照；先核对状态，后续已停止");
      } catch (error) {
        unknownRequest = request;
        changed();
        throw error;
      }
    }

    async function refreshScene(request, wanted) {
      const opened = await submit("begin_scene_service", {
        request_id: request, scene_key: request,
      });
      if (opened.result && opened.result.accepted === false) {
        throw new Error(opened.result.message || "开场景被拒绝");
      }
      const epoch = opened.result && opened.result.scene_epoch;
      if (!Number.isInteger(epoch) || epoch <= 0) {
        throw new Error("开场景响应缺少有效 epoch");
      }
      const deadline = Date.now() + 45000;
      while (Date.now() < deadline) {
        await poll();
        const targets = getState()?.perception?.targets;
        const sameEpoch = targets?.scene_epoch === epoch;
        const ids = new Set(observations().map((item) => item.target_id));
        const ready = !wanted || wanted.every((id) => ids.has(id));
        if (sameEpoch && targets.target_set_locked === true && ready) return epoch;
        await sleep(500);
      }
      throw new Error("场景锁定超时；请重新核对当前场景");
    }

    function requireHarvestSuccess(result) {
      const summary = result?.result;
      if (summary?.success !== true || Number(summary.action_status) !== 4
        || Number(summary.summary_attempted) < 1
        || Number(summary.summary_succeeded) !== Number(summary.summary_attempted)
        || Number(summary.summary_failed) > 0) {
        throw new Error(summary?.message || "批次未全部成功，后续已停止");
      }
    }

    async function harvestRound(index, mode) {
      const request = freshRequest();
      const generation = sessionGeneration;
      assertReady();
      let ordinal = 0;
      if (mode === "rotate" || mode === "first") {
        await refreshScene(request);
        const count = observations().length;
        if (!count) throw new Error("本场没有锁定目标");
        ordinal = mode === "first" ? 1 : (index % count) + 1;
      } else if (mode !== "all") {
        throw new Error("目标选择模式无效");
      }
      await setPriority(ordinal);
      assertSession(generation);
      assertReady();
      log(`第 ${index + 1} 轮 · ${ordinal ? `顺序 ${ordinal}` : "全部"}`);
      const result = await finishHarvest(submit("run_harvest_action", {
        request_id: request, scene_key: request,
        intent: mode === "all" ? "PICK_ALL" : "PICK_SELECTED",
        selection_mode: "AUTO", target_ids: [],
      }), request);
      requireHarvestSuccess(result);
      log(`第 ${index + 1} 轮结束 · 全部成功`);
    }

    async function harvestNamed(ids) {
      const request = freshRequest();
      const epoch = selectionEpoch;
      const generation = sessionGeneration;
      assertReady();
      assertContext(epoch, ids);
      await setPriority(0);
      assertContext(epoch, ids);
      assertSession(generation);
      log(`单轮 · ${ids.join(", ")}`);
      const result = await finishHarvest(submit("run_harvest_action", {
        request_id: request,
        scene_key: singleKey,
        intent: "PICK_SELECTED",
        selection_mode: "MANUAL",
        target_ids: ids,
      }), request);
      requireHarvestSuccess(result);
      log(`单轮结束 · ${ids.join(", ")} · 成功`);
    }

    async function drainSingle() {
      if (roundsBusy || !singleArmed || !clickQueue.length) return;
      const queued = clickQueue.shift();
      roundsBusy = true;
      changed();
      try {
        if (queued.epoch !== selectionEpoch) throw new Error("场景已变化，请重新点选");
        await harvestNamed(queued.ids);
      } catch (error) {
        singleArmed = false;
        clickQueue.length = 0;
        log(`单轮未完成：${error.message}`);
        notify?.(`单轮未完成：${error.message}`, "err");
      } finally {
        roundsBusy = false;
        if (!singleArmed) {
          clickQueue.length = 0;
          notify?.("单轮已停止", "warn");
        } else {
          changed();
          drainSingle();
        }
      }
    }

    function enqueue(ids) {
      if (stopRounds && roundsBusy) {
        notify?.("几轮正在跑，单轮先不接", "warn");
        return;
      }
      if (!singleArmed) {
        notify?.("先启动单轮点选。启动后点哪颗抓哪颗。", "warn");
        return;
      }
      if (!ids.length) {
        notify?.("先勾选要抓的顺序", "warn");
        return;
      }
      try {
        assertReady();
      } catch (error) {
        notify?.(error.message, "warn");
        return;
      }
      clickQueue.push({ids: [...ids], epoch: selectionEpoch});
      changed();
      if (!roundsBusy) drainSingle();
    }

    async function startSingle() {
      if (singleArmed) return;
      if (roundsBusy) {
        notify?.("几轮还在跑，等它停或先停止后续轮", "warn");
        return;
      }
      const confirmed = await confirm({
        title: "启动单轮点选",
        message: "只确认这一次。之后点顺序号就抓那一颗，抓完继续等，直到停止单轮。",
        confirmText: "启动单轮",
      });
      if (!confirmed) return;
      const generation = ++sessionGeneration;
      stopRounds = false;
      roundsBusy = true;
      changed();
      try {
        await pushSettings();
        await setPriority(0);
        singleKey = `single_${Date.now().toString(36)}`;
        await refreshScene(singleKey);
        if (generation !== sessionGeneration) return;
        singleArmed = true;
        clickQueue.length = 0;
        notify?.("单轮已启动。点顺序号，或勾选后抓已勾选。", "ok");
      } catch (error) {
        singleArmed = false;
        notify?.(`未启动：${error.message}`, "err");
      } finally {
        roundsBusy = false;
        changed();
      }
    }

    async function stopSingle() {
      sessionGeneration += 1;
      singleArmed = false;
      clickQueue.length = 0;
      changed();
      notify?.("单轮已停止", "warn");
      const batch = getState()?.task_executor?.state?.batch_state;
      if (!roundsBusy && !unknownRequest && ![1, 2, 3, 4, 5, 7].includes(batch)) return;
      try {
        await submit("control_service", {
          command: "CANCEL_NOW", expected_state_seq: 0, reason: "停止单轮",
        });
      } catch (error) {
        notify?.(`停止请求失败：${error.message}`, "err");
      }
    }

    async function startRounds(count, mode) {
      if (singleArmed) {
        notify?.("单轮还开着。先停止单轮，再启动这几轮。", "warn");
        return;
      }
      if (roundsBusy) return;
      if (mode === "selected") {
        notify?.("已勾选目标只属于当前场景；请用单轮点选，换场后重新选择。", "warn");
        return;
      }
      const label = {rotate: "每轮按顺序换一颗", first: "每轮第一颗", all: "每轮全部"}[mode] || mode;
      const confirmed = await confirm({
        title: `启动 ${count} 轮`,
        message: `${label}。每轮重新开场景。真机与仿真用当前这些开关。`,
        confirmText: `启动 ${count} 轮`,
      });
      if (!confirmed) return;
      const generation = ++sessionGeneration;
      roundsBusy = true;
      stopRounds = false;
      changed();
      try {
        await pushSettings();
        for (let index = 0; index < count; index += 1) {
          if (stopRounds || generation !== sessionGeneration) {
            log("后续轮已停止");
            break;
          }
          notify?.(`第 ${index + 1} / ${count} 轮`, "info");
          await harvestRound(index, mode);
        }
        notify?.(stopRounds ? "已停止" : `${count} 轮已跑完`, stopRounds ? "warn" : "ok");
      } catch (error) {
        notify?.(`未完成：${error.message}`, "err");
        log(`未完成：${error.message}`);
      } finally {
        roundsBusy = false;
        changed();
        setPriority(0).catch(() => {});
      }
    }

    async function stopRoundsRun() {
      stopRounds = true;
      sessionGeneration += 1;
      changed();
      notify?.("正在停止后续轮", "warn");
      try {
        await submit("control_service", {
          command: "CANCEL_NOW", expected_state_seq: 0, reason: "停止后续轮",
        });
      } catch (error) {
        notify?.(`停止请求失败：${error.message}`, "err");
      }
    }

    async function acknowledgeRecovery() {
      const task = getState()?.task_executor?.state || {};
      await submit("control_service", {
        command: "ACKNOWLEDGE_RECOVERY",
        expected_state_seq: Number(task.state_seq) || 0,
        request_id: freshRequest(),
        reason: "调试页确认恢复",
      });
      notify?.("已确认恢复", "ok");
    }

    function setEpoch(epoch) {
      if (epoch === selectionEpoch) return;
      selectionEpoch = epoch;
      clickQueue.length = 0;
      changed();
    }

    function syncState() {
      const task = getState()?.task_executor?.state || {};
      const arm = getState()?.manipulation?.status || {};
      if (unknownRequest && task.run_id === unknownRequest
        && [6, 8].includes(task.batch_state)
        && task.action_active === false && task.recovery_required === false
        && arm.running === false && arm.contact_recovery_required === false
        && (typeof isFresh !== "function" || isFresh())) {
        unknownRequest = "";
        changed();
      }
    }

    return {
      snapshot, setEpoch, syncState, assertReady, assertContext, submit,
      startSingle, stopSingle, enqueue, startRounds,
      stopRounds: stopRoundsRun, acknowledgeRecovery,
    };
  }

  root.PeachDebugSession = Object.freeze({create});
})(typeof window !== "undefined" ? window : globalThis);
