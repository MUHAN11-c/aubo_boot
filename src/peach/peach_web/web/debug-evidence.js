"use strict";

// Test intent is a local draft. Evidence is a read-only projection of the
// current request; selecting a recipe never writes parameters or enables.
(function (root) {
  const LEVELS = ["未到达预抓取", "预抓取到位", "预抓取已核验", "套入完成",
    "刀指令已受理", "切断已确认", "撤退已确认", "采摘已确认"];
  const RECIPES = {
    observe: {
      values: {motion: false, full: false, ex: false, gr: false, tl: false},
      criteria: "只验证相机、场景锁定和参数接线。执行、抓取、工具均关闭，不下发抓取。",
      steps: ["载入草稿，点击本轮写入并核对两侧使能关闭", "启动单轮，等待当前场景与目标锁定", "在过程监控核对图像、目标和TF；停止单轮"],
    },
    pregrasp: {
      values: {motion: true, full: false, ex: true, gr: true, tl: false},
      criteria: "PREGRASP_ONLY：期望当前请求账本L2。到位后保持，进入人工恢复等待；不套入、不切刀。",
      steps: ["选择本轮是否使用重建；写入并核对使能、重建实际模式", "启动单轮，按本场抓取顺序选择一颗", "核对L2与到位几何；停止后由现场确认恢复，不自动继续下一颗"],
    },
    full: {
      values: {motion: true, full: true, ex: true, gr: true, tl: false},
      criteria: "FULL干跑：期望同一请求动作成功且账本L6，套入并撤退。工具关闭，不作为L7真实采摘验收。",
      steps: ["写入并核对两侧执行/抓取开启、工具关闭", "单轮选果，或选择多轮数量与目标策略", "核对本次请求终态、L6和无恢复锁；结束后关闭使能并停栈"],
    },
  };

  function reconstructionMode(params, facts = {}) {
    const groups = params?.groups || [];
    const read = (node, name) => groups.find((group) => group.node === node)
      ?.params?.find((item) => item.name === name);
    const skip = read("peach_supervisor", "skip_reconstruction");
    const allow = read("peach_arm", "quality.allow_unrefined_geometry");
    const capture = read("peach_target_reconstruction_node", "capture.auto_mode");
    const deployed = ![false, "false"].includes(facts.reconstruction_enabled);
    const available = deployed && capture?.node_running === true && capture?.present === true;
    const known = skip?.present === true && allow?.present === true
      && skip.node_running === true && allow.node_running === true
      && typeof skip.value === "boolean" && typeof allow.value === "boolean";
    const enabled = known ? !skip.value : null;
    const consistent = known && allow.value === skip.value
      && (available ? capture.value === !skip.value : skip.value === true);
    const text = !known ? "模式未知 · 先重新读取"
      : !consistent ? "配置不一致 · 先写入并核对"
        : enabled ? "开启 · Build/精化 + 自动采集"
          : available ? "关闭 · 不调用Build，使用感知几何" : "关闭 · 重建节点未部署或未运行";
    return {available, enabled, consistent, text};
  }

  function create({onDraft}) {
    const $ = (id) => document.getElementById(id);
    function renderRecipe() {
      const recipe = RECIPES[$("test-profile").value];
      $("test-criteria").textContent = recipe.criteria;
      $("test-steps").replaceChildren(...recipe.steps.map((text) => {
        const item = document.createElement("li"); item.textContent = text; return item;
      }));
    }
    $("test-profile").addEventListener("change", renderRecipe);
    $("test-load").addEventListener("click", () => onDraft({...RECIPES[$("test-profile").value].values}));
    renderRecipe();

    const enables = (value) => ["execution", "grasp", "tool"].map((name) => {
      const flag = value[`${name}_enabled`];
      return flag === true ? "开" : flag === false ? "关" : "未知";
    }).join(" / ");
    function update(state, status, params) {
      const task = state.task_executor?.state || {};
      const arm = state.manipulation?.status || {};
      const ledger = state.ledger?.live || {};
      const diag = state.reconstruction?.diagnostics || {};
      const decision = diag.grasp_decision || {};
      const fresh = status?.state === "online";
      $("evidence-freshness").textContent = fresh ? "状态在线 · 调度意图与臂侧实际值分别核对"
        : "连接中断或数据陈旧 · 下列保留值不可用于确认当前执行状态";
      $("evidence-freshness").classList.toggle("evidence-stale", !fresh);
      $("evidence-mode").textContent = state.startup?.facts?.hardware_mode || "未知";
      const waiting = task.batch_state === 7 && task.recovery_required === true;
      $("evidence-action").textContent = waiting ? "等待人工恢复确认"
        : task.action_active === true ? (arm.contact_recovery_required ? "接触保护中 · 动作仍在执行" : "动作执行中")
          : task.action_active === false
            ? (arm.contact_recovery_required ? "臂侧恢复锁 · 请核对现场" : "动作空闲") : "动作状态未知";
      $("evidence-intent").textContent = enables(task);
      $("evidence-arm").textContent = enables(arm);
      const mode = reconstructionMode(params, state.startup?.facts);
      $("evidence-reconstruction").textContent = mode.text;
      $("evidence-model").textContent = mode.enabled === false
        ? "本轮不使用重建模型 · 旧模型不作当前证据"
        : `${state.reconstruction?.status?.state || "未知"} · allowed=${typeof decision.allowed === "boolean" ? decision.allowed : "未知"}（执行前复检）`;
      const sameRequest = Boolean(task.run_id) && ledger.request_id === task.run_id;
      const rows = sameRequest ? (ledger.rows || []) : [];
      const row = task.target_id ? rows.find((item) => item.target_id === task.target_id) : rows[rows.length - 1];
      const level = row?.completion_level;
      $("evidence-level").textContent = Number.isInteger(level)
        ? `L${level} · ${LEVELS[level] || "未知级别"} · ${row.outcome_name || "终态待核对"}`
        : sameRequest ? "当前请求尚未入账" : "无匹配本次请求的账本";
      $("evidence-request").textContent = `${task.run_id || "无请求"} / ${task.target_id || row?.target_id || "无目标"}`;
    }
    return {update};
  }
  root.PeachDebugEvidence = Object.freeze({create, reconstructionMode});
})(typeof window !== "undefined" ? window : globalThis);
