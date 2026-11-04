#!/usr/bin/env node
/*
 * Peach Web console end-to-end check.
 *
 * Prerequisite: observability HTTP is live (default 8090) and Playwright is
 * resolvable. Example on this host:
 *
 *   NODE_PATH=$HOME/.npm/_npx/<hash>/node_modules \
 *     node src/peach/peach_web/scripts/e2e-console.js \
 *       --base-url http://127.0.0.1:8090 \
 *       --out runs/web_e2e_$(date +%Y%m%d_%H%M%S)
 *
 * Mock-only: the default suite starts one explicitly confirmed single-debug
 * session (BeginScene + enable readback) which never opens execution.
 * --harvest adds a FULL dry run, requires L6, then closes all enable gates.
 */
"use strict";

const fs = require("fs");
const path = require("path");

function argValue(name, fallback = "") {
  const index = process.argv.indexOf(name);
  return index >= 0 && process.argv[index + 1] ? process.argv[index + 1] : fallback;
}
function hasFlag(name) {
  return process.argv.includes(name);
}

const BASE = (argValue("--base-url", "http://127.0.0.1:8090")).replace(/\/$/, "");
const STAMP = new Date().toISOString().replace(/[-:]/g, "").replace(/\..+/, "").replace("T", "_");
const OUT = path.resolve(argValue("--out", `runs/web_e2e_${STAMP}`));
const BROWSER = argValue("--browser", process.env.CHROME_PATH || "/usr/bin/google-chrome");
const HEADED = hasFlag("--headed");
const ONLY = argValue("--only", "");
const RUN_HARVEST = hasFlag("--harvest");
const TRANSPORT = argValue("--transport", "http");
const RECONSTRUCTION = argValue("--reconstruction", "off");
const results = [];
const pageErrors = new Map();
const PAGE_PATHS = {
  overview: "/overview.html",
  monitor: "/monitor.html",
  debug: "/debug.html",
  analysis: "/analysis.html",
  force: "/force.html",
};

fs.mkdirSync(OUT, {recursive: true});

function now() {
  return new Date().toISOString();
}
function record(name, ok, detail = "") {
  results.push({name, ok, detail, at: now()});
  const mark = ok ? "PASS" : "FAIL";
  console.log(`[${mark}] ${name}${detail ? ` — ${detail}` : ""}`);
}
async function test(name, fn) {
  const started = Date.now();
  try {
    const detail = await fn();
    record(name, true, typeof detail === "string" ? detail : `${Date.now() - started} ms`);
    return true;
  } catch (error) {
    record(name, false, error.stack || error.message || String(error));
    return false;
  }
}
function assert(condition, message) {
  if (!condition) throw new Error(message);
}
async function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function attach(page, name) {
  const errors = [];
  const consoleErrors = [];
  const expectedHttpErrors = new Set();
  pageErrors.set(name, {errors, consoleErrors, expectedHttpErrors});
  page.on("pageerror", (error) => errors.push(error.stack || error.message));
  page.on("console", (message) => {
    if (message.type() !== "error") return;
    const text = message.text();
    const status = text.match(/server responded with a status of (\d+)/)?.[1];
    const url = message.location().url;
    if (status && url && expectedHttpErrors.has(`${new URL(url).pathname}:${status}`)) return;
    // Intentional offline tests produce resource errors; page-level code must
    // still stay clean. Keep those out of the failure set but record them.
    if (/net::ERR_|Failed to load resource: net::ERR_/.test(text)) return;
    consoleErrors.push(text);
  });
}
function assertPageClean(name) {
  const entry = pageErrors.get(name);
  assert(entry, `no console capture for ${name}`);
  assert(entry.errors.length === 0, `${name} pageerror: ${entry.errors.join(" | ")}`);
  assert(entry.consoleErrors.length === 0, `${name} console.error: ${entry.consoleErrors.join(" | ")}`);
}

async function shot(page, name) {
  await page.evaluate(() => window.scrollTo({top: 0, behavior: "instant"}));
  await page.screenshot({path: path.join(OUT, `${name}.png`), fullPage: true});
  return `${name}.png`;
}

async function goto(page, name) {
  await page.goto(BASE + PAGE_PATHS[name] + `?transport=${TRANSPORT}`, {waitUntil: "domcontentloaded", timeout: 30000});
  await page.waitForTimeout(600);
}

async function waitOnline(page, timeout = 15000) {
  await page.waitForFunction(() => document.querySelector("#app-connection span")?.textContent === "已连接",
    null, {timeout});
}
async function waitConnection(page, label, timeout = 15000) {
  await page.waitForFunction((expected) => document.querySelector("#app-connection span")?.textContent === expected,
    label, {timeout});
}
async function overflow(page) {
  return page.evaluate(() => document.documentElement.scrollWidth > innerWidth + 1);
}
async function clickConfirm(page, label) {
  const dialog = page.locator("dialog.app-dialog");
  await dialog.waitFor({state: "visible", timeout: 5000});
  await dialog.getByRole("button", {name: label, exact: true}).click();
  await dialog.waitFor({state: "detached", timeout: 5000}).catch(() => {});
}
async function cancelConfirm(page) {
  const dialog = page.locator("dialog.app-dialog");
  await dialog.waitFor({state: "visible", timeout: 5000});
  await dialog.getByRole("button", {name: "取消", exact: true}).click();
  await dialog.waitFor({state: "detached", timeout: 5000}).catch(() => {});
}

async function testStatic(request) {
  const required = ["/", "/overview.html", "/monitor.html", "/index.html", "/debug.html",
    "/analysis.html", "/force.html", "/api.js", "/shell.js", "/shell.css", "/debug-evidence.js",
    "/favicon.svg", "/rosbridge.js", "/vendor/ros2-web-link.js", "/vendor/peach-three.js"];
  for (const route of required) {
    const response = await request.get(BASE + route);
    assert(response.status() === 200, `${route} -> ${response.status()}`);
    const headers = response.headers();
    assert(headers["x-frame-options"] === "DENY", `${route} missing X-Frame-Options`);
    assert(/frame-ancestors 'none'/.test(headers["content-security-policy"] || ""),
      `${route} CSP missing frame-ancestors`);
    if (route.endsWith(".html") || route === "/") {
      assert(/text\/html/.test(headers["content-type"] || ""), `${route} content-type ${headers["content-type"]}`);
    }
  }
  const html = await (await request.get(BASE + "/monitor.html")).text();
  assert(html.includes("/api.js") && html.includes("/shell.css"), "monitor shell scripts missing");
  for (const route of ["/nope.js", "/payload.exe", "/.git/config", "/%2e%2e/etc/passwd"]) {
    const response = await request.get(BASE + route);
    assert(response.status() === 404, `${route} should be 404, got ${response.status()}`);
  }
  const staticResponse = await request.get(BASE + "/shell.css");
  assert(/no-cache/.test(staticResponse.headers()["cache-control"] || ""), "static cache-control");
}

async function testOverview(page) {
  await goto(page, "overview");
  await waitOnline(page);
  assert(await page.locator(".app-nav a[aria-current='page']").innerText() === "总览\n00",
    "overview nav active");
  assert((await page.locator("#mode").innerText()).trim() !== "—", "mode not reported");
  assert((await page.locator(".chain .node").count()) === 4, "chain nodes");
  const preview = page.locator("#preview");
  await page.waitForFunction(() => {
    const img = document.getElementById("preview");
    return img && !img.hidden && img.complete && img.naturalWidth > 0;
  }, null, {timeout: 10000});
  assert(await page.locator("#previewEmpty").isHidden(), "empty camera overlay still visible");
  assert((await page.locator("#imageAge").innerText()).trim() !== "无帧", "image age missing");
  assert((await page.locator("#runId").innerText()).length > 0, "run id");
  assert(await page.locator("#overview-alert").isHidden(), "online alert visible");
  await shot(page, "overview_1440");
  const snapshot = await (await page.request.get(BASE + "/api/state")).json();
  snapshot.ledger = {live: {rows: [{target_id: "overview-check", completion_level: 2}]}};
  snapshot.diagnostics = {nodes: {healthy: {level: 0}, warning: {level: 1}}};
  await page.route("**/api/state*", (route) => route.fulfill({status: 200, json: snapshot}));
  await page.waitForFunction(() => document.querySelector("#completion")?.textContent === "L2（预抓取已核验）",
    null, {timeout: 8000});
  assert(await page.locator("#diagState").innerText() === "异常 1", "diagnostics count includes healthy nodes");
  await page.unroute("**/api/state*");
  return "camera preview + chain + summary + completion semantics";
}

async function testMonitor(page) {
  await goto(page, "monitor");
  await waitOnline(page);
  await page.waitForFunction(() => document.querySelectorAll("#target-list tr").length > 0, null, {timeout: 10000});
  assert((await page.locator("#pipeline .step").count()) === 9, "pipeline steps");
  assert((await page.locator("#log-body tr").count()) > 0, "logs empty");
  assert((await page.locator("#target-list tr").count()) > 0, "targets empty");
  const traj = await page.locator("#traj-status").innerText();
  assert(!/等待/.test(traj), `trajectory status: ${traj}`);

  // Camera/3D controls.
  const select = page.locator("[data-image-source]");
  await select.selectOption("raw");
  await page.waitForFunction(() => document.querySelector("[data-image-status]")?.textContent.includes("即时检测图"),
    null, {timeout: 8000});
  await select.selectOption("perception");
  const pause = page.locator("[data-image-pause]");
  await pause.click();
  assert((await pause.innerText()).includes("恢复图像"), "image pause label");
  await pause.click();
  const zoom = page.locator("[data-image-zoom]");
  await zoom.click();
  assert(await page.locator("dialog.viz-image-dialog").isVisible(), "image dialog not open");
  await page.locator("[data-close-image]").click();
  await page.locator("dialog.viz-image-dialog").waitFor({state: "hidden"});
  for (const view of ["center", "top", "side", "reset"]) {
    await page.locator(`[data-scene-view="${view}"]`).click();
  }
  const webgl = await page.locator(".viz-scene-box canvas").count();
  assert(webgl > 0, "3D canvas missing");

  // Log filters.
  await page.locator("#log-node").fill("peach_scene_perception_node");
  await page.waitForTimeout(300);
  const rows = await page.locator("#log-body tr").count();
  assert(rows > 0, "node filter empty");
  const nodes = await page.locator("#log-body tr td:nth-child(3)").allInnerTexts();
  assert(nodes.every((text) => text.includes("peach_scene_perception_node")), "node filter leaks");
  await page.locator("#log-node").fill("");
  await page.locator("#log-keyword").fill("TF");
  await page.waitForTimeout(300);
  assert((await page.locator("#log-body tr").count()) > 0, "keyword filter empty");
  await page.locator("#log-keyword").fill("");
  await page.locator("#log-pause").click();
  const paused = await page.locator("#log-body tr").count();
  await page.waitForTimeout(1600);
  assert((await page.locator("#log-body tr").count()) === paused, "paused log changed");
  await page.locator("#log-pause").click();

  // Operation gating.
  const photo = page.locator('[data-debug="photo_pose_service"]');
  assert(await photo.isDisabled(), "motion action not gated while motion disabled");
  const cancel = page.locator('[data-debug="skill_cancel_service"]');
  assert(await cancel.isEnabled(), "cancel must stay available");
  await cancel.click();
  await cancelConfirm(page);
  await shot(page, "monitor_1440");

  // Stale/offline recovery.
  await page.route("**/api/state*", (route) => route.abort("failed"));
  await waitConnection(page, "连接中断", 15000);
  assert(await page.evaluate(() => document.body.classList.contains("data-stale")), "data-stale missing");
  assert(await page.locator("#feed-alert").isVisible(), "stale alert missing");
  assert(await photo.isDisabled(), "ordinary control enabled offline");
  assert(await cancel.isEnabled(), "cancel disabled offline");
  await page.unroute("**/api/state*");
  await waitOnline(page, 15000);
  assert(!(await page.evaluate(() => document.body.classList.contains("data-stale"))), "stale class stuck");

  // POST result classification and duplicate-submit guard.
  let posts = 0;
  let mode = "202";
  pageErrors.get("monitor").expectedHttpErrors.add("/api/debug/skill_cancel_service:500");
  pageErrors.get("monitor").expectedHttpErrors.add("/api/debug/skill_cancel_service:423");
  let release = null;
  await page.route("**/api/debug/skill_cancel_service", async (route) => {
    posts += 1;
    if (mode === "202") await new Promise((resolve) => setTimeout(resolve, 800));
    if (mode === "202") {
      await route.fulfill({status: 202, json: {accepted: true, result: {
        goal_handle_kept: true, message: "仍在执行，可稍后查询状态或取消"}}});
    } else if (mode === "500") {
      await route.fulfill({status: 500, json: {accepted: false, message: "后端内部错误"}});
    } else {
      await route.fulfill({status: 423, json: {accepted: false, result: {message: "动臂未放行"}}});
    }
  });
  await page.evaluate(() => {
    const el = document.querySelector('[data-debug="skill_cancel_service"]');
    el.click();
  });
  await clickConfirm(page, "取消执行");
  await page.waitForTimeout(250);
  const busy = page.locator('[data-debug="skill_cancel_service"]');
  assert(await busy.getAttribute("aria-busy") === "true", "pending lock missing");
  assert(await busy.isDisabled(), "pending cancel still enabled");
  assert(posts === 1, `cancel POST count ${posts}`);
  await page.waitForFunction(() => document.querySelector("#debug-notice")?.textContent.includes("仍在执行"),
    null, {timeout: 8000});
  mode = "500";
  await page.locator('[data-debug="skill_cancel_service"]').click();
  await clickConfirm(page, "取消执行");
  await page.waitForFunction(() => document.querySelector("#debug-notice")?.textContent.includes("结果未知"),
    null, {timeout: 8000});
  mode = "423";
  await page.locator('[data-debug="skill_cancel_service"]').click();
  await clickConfirm(page, "取消执行");
  await page.waitForFunction(() => document.querySelector("#debug-notice")?.textContent.includes("请求被拒"),
    null, {timeout: 8000});
  await page.unroute("**/api/debug/skill_cancel_service");

  return "filters, visual controls, stale gate, POST classes";
}

async function testDebug(page) {
  await goto(page, "debug");
  await waitOnline(page);
  await page.waitForFunction(() => document.querySelectorAll(".param-row").length > 0, null, {timeout: 10000});
  assert((await page.locator(".debug-chip").count()) >= 5, "gate chips missing");
  assert((await page.locator(".target-pick button[data-target]").count()) > 0, "targets missing");
  let draftPosts = 0;
  const onDraftRequest = (request) => { if (request.method() === "POST") draftPosts += 1; };
  page.on("request", onDraftRequest);
  const reconstructionBefore = await page.locator("#reconstruction").isChecked();
  await page.locator("#test-profile").selectOption("full");
  await page.locator("#test-load").click();
  assert(await page.locator("#full").isChecked() && await page.locator("#ex").isChecked(), "FULL recipe draft");
  assert(!(await page.locator("#tl").isChecked())
    && await page.locator("#reconstruction").isChecked() === reconstructionBefore,
  "recipe changed tool/reconstruction selection");
  assert(draftPosts === 0, "recipe selection wrote to backend");
  await page.locator("#test-profile").selectOption("observe");
  await page.locator("#test-load").click();
  assert(!(await page.locator("#ex").isChecked()), "observation recipe enabled execution");
  page.off("request", onDraftRequest);

  // Parameters read, no-op apply.
  await page.locator("#params-panel > summary").click();
  await page.locator("#params-search").fill("selection_priority");
  assert(await page.locator(".param-row:visible").count() === 1, "parameter search filter");
  const ordinal = page.locator('.param-row[data-name="selection_priority"] input');
  await ordinal.fill("1.5");
  await page.locator("#params-apply").click();
  await page.waitForFunction(() => document.querySelector("#params-note")?.textContent.includes("必须是整数"));
  const original = await ordinal.locator("..").locator("..").getAttribute("data-orig");
  await ordinal.fill(original);
  await page.locator("#params-changed").check();
  assert(await page.locator(".param-row:visible").count() === 0, "reverted parameter remained dirty");
  await page.locator("#params-changed").uncheck();
  await page.locator("#params-search").fill("");
  await page.locator("#params-apply").click();
  await page.waitForFunction(() => document.querySelector("#params-note")?.textContent.includes("没有修改"),
    null, {timeout: 5000});

  // Enable dependency chain is UI-only here.
  await page.locator("#ex").check();
  assert(await page.locator("#motion").isChecked(), "execution did not force motion");
  await page.locator("#motion").uncheck();
  assert(!(await page.locator("#ex").isChecked()), "motion off did not clear execution");

  // Confirm dialog can be cancelled without a POST.
  await page.locator("#start-single").click();
  await cancelConfirm(page);
  assert(!(await page.locator("#single-panel").evaluate((el) => el.classList.contains("live"))),
    "single armed after cancel");

  // One safe session: BeginScene + enable readback, no execution motion.
  await page.locator("#start-single").click();
  await clickConfirm(page, "启动单轮");
  await page.waitForFunction(() => document.querySelector("#single-panel")?.classList.contains("live"),
    null, {timeout: 60000});
  assert((await page.locator(".debug-chip").allInnerTexts()).some((text) => text.includes("保持中")),
    "single gate chip not live");
  await page.locator("#stop-single").click();
  await page.waitForFunction(() => document.querySelector("#session-status")?.textContent.includes("单轮已停止"),
    null, {timeout: 10000});
  await shot(page, "debug_1440");
  const protocol = await page.evaluate(async () => {
    const checks = [];
    for (const mode of ["first", "rotate", "all", "recovery", "pending", "network", "rejected"]) {
      const recovery = mode === "recovery";
      const state = {debug: {enabled: true}, task_executor: {state: {}},
        manipulation: {status: {running: false, contact_recovery_required: false}}, perception: {targets: {
        scene_epoch: 1, target_set_locked: true,
        observations: [{target_id: "a"}, {target_id: "b"}],
      }}};
      const logs = [];
      const priorities = [];
      let calls = 0;
      const session = PeachDebugSession.create({
        api: {debug: async (action, payload) => {
          if (action === "begin_scene_service") return {response: {ok: true, status: 200},
            body: {accepted: true, result: {accepted: true, scene_epoch: 1}}};
          calls += 1;
          if (mode === "network") throw new Error("fixture network loss");
          if (mode === "pending" || mode === "rejected") return {
            response: {ok: mode === "pending", status: mode === "pending" ? 202 : 423},
            body: {accepted: mode === "pending", result: {message: "fixture status"}},
          };
          state.task_executor.state = {batch_state: recovery ? 7 : 2,
            action_active: !recovery, recovery_required: true};
          await new Promise((resolve) => setTimeout(resolve, 50));
          if (!recovery) state.task_executor.state = {run_id: payload.request_id, batch_state: 6,
            action_active: false, recovery_required: false};
          return {response: {ok: true, status: 200}, body: {accepted: true, result: {
            success: true, action_status: 4, summary_attempted: 1,
            summary_succeeded: 1, summary_failed: 0,
          }}};
        }},
        getState: () => state, isFresh: () => true,
        pushSettings: async () => {}, poll: async () => {},
        setPriority: async (value) => { priorities.push(value); },
        confirm: async () => true, log: (message) => logs.push(message),
      });
      await session.startRounds(2, ["recovery", "pending", "network", "rejected"].includes(mode) ? "first" : mode);
      checks.push({mode, calls, priorities, logs, unknown: session.snapshot().unknownRequest});
    }
    return checks;
  });
  for (const check of protocol) {
    if (["recovery", "pending", "network"].includes(check.mode)) {
      assert(check.calls === 1 && Boolean(check.unknown), "recovery did not retain request and stop next round");
    } else if (check.mode === "rejected") {
      assert(check.calls === 1 && !check.unknown && check.logs.some((text) => text.includes("未完成")),
        "known rejection did not stop next round");
    } else {
      assert(check.calls === 2 && check.logs.filter((text) => text.includes("全部成功")).length === 2,
        `${check.mode}: transient contact latch stopped FULL: ${check.logs}`);
      const expected = check.mode === "rotate" ? [1, 2, 0] : check.mode === "first" ? [1, 1, 0] : [0, 0, 0];
      assert(JSON.stringify(check.priorities) === JSON.stringify(expected), `${check.mode} priority selection`);
    }
  }
  return "recipes, parameter validation/filter, enable readback, single session, rounds, recovery/202/network/rejection";
}

async function testAnalysis(page, request) {
  await goto(page, "analysis");
  await page.waitForFunction(() => document.querySelectorAll("#records option").length > 0, null, {timeout: 10000});
  const records = await (await request.get(BASE + "/api/analysis/records")).json();
  assert(records.records.length > 0, "no analysis records");
  const recordPath = records.records[0].path;
  await page.locator("#path").fill(recordPath);
  await page.locator("#pick-submit").click();
  await page.locator("#result").waitFor({state: "visible", timeout: 30000});
  const status = await page.locator("#status").innerText();
  assert(!/解析失败/.test(status), `analysis failed: ${status}`);
  assert((await page.locator("#rates article").count()) === 6, "rate cards");
  assert((await page.locator("#cycles tr").count()) > 0, "cycle rows absent");
  assert(new URL(page.url()).searchParams.get("path") === recordPath, "URL not updated");
  await shot(page, "analysis_1440");

  // Bad path is an explicit error, not a stale result.
  pageErrors.get("analysis").expectedHttpErrors.add("/api/analysis:400");
  await page.locator("#path").fill("e2e_missing_record");
  await page.locator("#pick-submit").click();
  await page.waitForFunction(() => document.querySelector("#status")?.classList.contains("error"),
    null, {timeout: 10000});
  assert(await page.locator("#result").isHidden(), "stale result still visible on error");
  return `record ${recordPath}`;
}

async function testForce(browser) {
  const context = await browser.newContext({viewport: {width: 1440, height: 1000}});
  const page = await context.newPage();
  const name = "force";
  attach(page, name);
  await page.route("**/api/force", (route) => route.fulfill({status: 200, json: {
    available: true, age_s: 0.02, source_uuid: "e2e", sequence: 42,
    channel_ids: [5, 1, 2, 3, 7], force_n: [1, 2, 3, 4, 5],
    travel_mm: [0.1, 0.2, 0.3, 0.4, 0.5], ad: [10, 20, 30, 40, 50],
    status: [0, 0, 0, 0, 0], stamp_ns: 1793069000000000000, frame_id: "tcp",
  }}));
  await goto(page, "force");
  await page.waitForFunction(() => document.querySelector("#status")?.textContent === "在线", null, {timeout: 10000});
  assert((await page.locator("#total").innerText()).includes("15.00"), "force sum");
  assert((await page.locator("#rows tr").count()) === 5, "force rows");
  assert(await page.locator("#copmark").isVisible(), "CoP marker hidden");
  assert((await page.locator(".app-conn span").innerText()) === "已连接", "force connection");
  const fov = page.locator('[data-fov="disc"]');
  await fov.click();
  assert(await fov.getAttribute("aria-pressed") === "true", "FOV aria-pressed");
  assert(await page.locator("#face").evaluate((el) => el.classList.contains("disc")), "disc class");
  await page.locator('[data-fov="full"]').click();
  await shot(page, "force_1440");

  await page.unroute("**/api/force");
  await page.route("**/api/force", (route) => route.abort("failed"));
  await page.waitForFunction(() => document.querySelector("#status")?.textContent === "连接失败",
    null, {timeout: 10000});
  assert(await page.locator("#copmark").isHidden(), "CoP visible on failure");
  await page.locator('[data-fov="disc"]').click();
  assert(await page.locator("#copmark").isHidden(), "FOV revived stale CoP after disconnect");
  assert(!(await page.locator("#channels").innerText()).includes("1.00 N"),
    "live channel readings retained after disconnect");
  await page.unroute("**/api/force");
  await page.route("**/api/force", (route) => route.fulfill({status: 200, json: {
    available: false, error: "stale", age_s: 3.5,
  }}));
  await page.waitForFunction(() => document.querySelector("#status")?.textContent === "数据陈旧",
    null, {timeout: 10000});
  assert(await page.locator("#copmark").isHidden(), "CoP visible on stale source");
  await page.unroute("**/api/force");
  await page.route("**/api/force", (route) => route.fulfill({status: 200, json: {
    available: false, error: "no_frame",
  }}));
  await page.waitForFunction(() => document.querySelector("#status")?.textContent === "力源未启用",
    null, {timeout: 10000});
  assertPageClean(name);
  await context.close();
  return "force data, CoP, FOV, disconnect, stale source";
}

async function testResponsive(browser) {
  const pages = Object.keys(PAGE_PATHS);
  for (const width of [320, 390, 768, 1440]) {
    const context = await browser.newContext({viewport: {width, height: 900}});
    for (const name of pages) {
      const page = await context.newPage();
      const ok = await test(`responsive ${width} ${name}`, async () => {
        attach(page, `resp-${width}-${name}`);
        await goto(page, name);
        await page.waitForTimeout(900);
        assert(!(await overflow(page)), `horizontal overflow at ${width}`);
        assert(await page.locator(".app-header").isVisible(), "header missing");
        assert(await page.locator("main").isVisible(), "main missing");
        const nav = await page.locator(".app-nav a").count();
        assert(nav === 5, `nav count ${nav}`);
        if (width === 390) await shot(page, `${name}_390`);
        assertPageClean(`resp-${width}-${name}`);
      });
      if (!ok) await shot(page, `fail_${name}_${width}`).catch(() => {});
      await page.close();
    }
    await context.close();
  }
}

async function testA11y(page) {
  await goto(page, "monitor");
  await waitOnline(page);
  await page.keyboard.press("Tab");
  const skipFocused = await page.evaluate(() => document.activeElement?.classList.contains("skip-link"));
  assert(skipFocused, "skip link not first focus target");
  const navCurrent = await page.locator(".app-nav a[aria-current='page']").count();
  assert(navCurrent === 1, "aria-current count");
  const cancel = page.locator('[data-debug="skill_cancel_service"]');
  await cancel.click();
  await page.locator("dialog.app-dialog").waitFor({state: "visible"});
  await page.keyboard.press("Escape");
  await page.locator("dialog.app-dialog").waitFor({state: "detached"});
  return "skip link, aria-current, Escape dialog";
}

async function runHarvest(browser) {
  const context = await browser.newContext({viewport: {width: 1440, height: 1000}});
  const debugPage = await context.newPage();
  const monitorPage = await context.newPage();
  let wsSubmitted, wsReply;
  const wsSent = new Promise((resolve) => { wsSubmitted = resolve; });
  const wsResult = new Promise((resolve) => { wsReply = resolve; });
  let wsGoal = "";
  debugPage.on("websocket", (socket) => {
    socket.on("framesent", ({payload}) => {
      const message = JSON.parse(String(payload));
      if (message.op === "send_action_goal" && message.action.endsWith("/run_harvest")) {
        wsGoal = message.id;
        wsSubmitted({postDataJSON: () => message.args, response: () => wsResult});
      }
    });
    socket.on("framereceived", ({payload}) => {
      const message = JSON.parse(String(payload));
      if (message.op === "action_result" && message.id === wsGoal) {
        wsReply({ok: () => message.result === true, json: async () => ({accepted: message.result === true,
          result: {...message.values, action_status: message.status}})});
      }
    });
  });
  attach(debugPage, "harvest-debug");
  attach(monitorPage, "harvest-monitor");
  await goto(debugPage, "debug");
  await waitOnline(debugPage);
  await debugPage.waitForFunction(() => document.querySelectorAll(".param-row").length > 0, null, {timeout: 10000});

  // FULL dry run: execution + grasp, no tool. The UI ties motion.
  await debugPage.locator("#motion").check();
  await debugPage.locator("#full").check();
  await debugPage.locator("#reconstruction").setChecked(RECONSTRUCTION === "on");
  await debugPage.locator("#ex").check();
  await debugPage.locator("#gr").check();
  await debugPage.locator("#tl").uncheck();
  await debugPage.locator("#round-apply").click();
  await clickConfirm(debugPage, "写入并应用");
  await debugPage.waitForFunction(() => document.querySelector("#round-note")?.textContent.includes("已写入并读回核对"),
    null, {timeout: 60000});
  assert((await debugPage.locator("#debug-chips").innerText()).includes("已放行"), "motion not applied");
  const paramsReply = await context.request.get(BASE + "/api/params");
  assert(paramsReply.ok(), "harvest parameter readback failed");
  const effectiveParams = await paramsReply.json();
  const reconstructionSettings = [
    {node: "peach_supervisor", name: "execute_pregrasp_only", expected: false},
    {node: "peach_supervisor", name: "skip_reconstruction", expected: RECONSTRUCTION === "off"},
    {node: "peach_target_reconstruction_node", name: "capture.auto_mode", expected: RECONSTRUCTION === "on"},
    {node: "peach_arm", name: "quality.allow_unrefined_geometry", expected: RECONSTRUCTION === "off"},
  ].map((setting) => {
    const param = effectiveParams.groups?.find((group) => group.node === setting.node)
      ?.params?.find((item) => item.name === setting.name);
    return {...setting, present: param?.present === true, actual: param?.value ?? null};
  });
  fs.writeFileSync(path.join(OUT, "harvest_settings.json"), JSON.stringify(reconstructionSettings, null, 2));
  assert(reconstructionSettings.every((setting) => setting.present && setting.actual === setting.expected),
    `FULL reconstruction mode ${RECONSTRUCTION} mismatch: ${JSON.stringify(reconstructionSettings)}`);
  const timeouts = [
    {node: "peach_supervisor", name: "action_timeout_s"},
    {node: "peach_arm", name: "moveit.execute_timeout_s"},
  ].map((setting) => {
    const param = effectiveParams.groups?.find((group) => group.node === setting.node)
      ?.params?.find((item) => item.name === setting.name);
    return {...setting, present: param?.present === true, seconds: param?.value ?? null};
  });
  fs.writeFileSync(path.join(OUT, "harvest_timeouts.json"), JSON.stringify(timeouts, null, 2));
  assert(timeouts.every((setting) => setting.present && Number.isFinite(setting.seconds) && setting.seconds > 0),
    `harvest timeout readback missing/invalid: ${JSON.stringify(timeouts)}`);

  await debugPage.locator("#start-single").click();
  await clickConfirm(debugPage, "启动单轮");
  await debugPage.waitForFunction(() => document.querySelector("#single-panel")?.classList.contains("live"),
    null, {timeout: 60000});
  const target = debugPage.getByRole("button", {name: /^抓取 顺序 1 /});
  const targetId = await target.getAttribute("data-target");
  console.log(`harvest target ${targetId}`);
  const sent = TRANSPORT === "rosbridge" ? wsSent : debugPage.waitForRequest((request) => request.method() === "POST"
    && new URL(request.url()).pathname === "/api/debug/run_harvest_action");
  await target.click();
  const submitted = await sent;
  const requestId = submitted.postDataJSON()?.request_id;
  assert(typeof requestId === "string" && requestId.length > 0, "harvest request identity missing");

  await goto(monitorPage, "monitor");
  await waitOnline(monitorPage);
  const deadline = Date.now() + 240000;
  let last = null;
  while (Date.now() < deadline) {
    const response = await fetch(`${BASE}/api/state?t=${Date.now()}`);
    const state = await response.json();
    last = state;
    const task = state.task_executor?.state || {};
    const ledger = state.ledger?.live || {};
    if (task.run_id !== requestId || ledger.request_id !== requestId) {
      await sleep(1000);
      continue;
    }
    const row = (state.ledger?.live?.rows || []).find((item) => item.target_id === targetId);
    if (task.batch_state === 7 && task.recovery_required) break;
    if (task.action_active === false && task.batch_state >= 6 && row) break;
    if (debugPage && (await debugPage.locator("#round-log li").count()) > 0) {
      const text = await debugPage.locator("#round-log li").first().innerText();
      if (/单轮未完成/.test(text) && task.action_active === false) break;
    }
    await sleep(1000);
  }
  await shot(monitorPage, "harvest_monitor_final");
  await shot(debugPage, "harvest_debug_final");
  const task = last?.task_executor?.state || {};
  const ledger = last?.ledger?.live || {};
  const row = (ledger.rows || []).find((item) => item.target_id === targetId);
  const summary = {
    request_id: requestId,
    target_id: targetId,
    batch_state: task.batch_state,
    recovery_required: task.recovery_required === true,
    completion_level: row?.completion_level ?? null,
    outcome_name: row?.outcome_name ?? null,
    reason: row?.reason ?? "",
    elapsed_s: row?.elapsed_s ?? null,
    ledger_request: ledger.request_id || "",
    reconstruction_settings: reconstructionSettings,
    reconstruction_mode: RECONSTRUCTION,
    timeout_settings: timeouts,
    round_log: await debugPage.locator("#round-log li").allInnerTexts(),
  };
  fs.writeFileSync(path.join(OUT, "harvest.json"), JSON.stringify(summary, null, 2));
  await debugPage.locator("#stop-single").click().catch(() => {});
  assertPageClean("harvest-debug");
  assertPageClean("harvest-monitor");
  assert(summary.completion_level != null, `no ledger row for ${targetId}`);
  assert(task.run_id === requestId && ledger.request_id === requestId,
    "harvest evidence belongs to another request");
  assert(Number(summary.completion_level) >= 6 && !summary.recovery_required,
    `completion_level ${summary.completion_level}: ${summary.reason}`);
  assert(summary.outcome_name === "SUCCEEDED", `harvest outcome ${summary.outcome_name}`);
  const reply = await submitted.response();
  const result = reply ? await reply.json() : null;
  assert(reply?.ok() && result?.accepted === true && result.result?.success === true
    && result.result?.action_status === 4, `harvest action did not succeed: ${JSON.stringify(result)}`);
  await context.close();
  console.log("harvest summary", JSON.stringify(summary));
  return summary;
}

async function main() {
  let browser;
  let requestContext;
  let request;
  let context;
  let harvestStarted = false;

  try {
    assert(["http", "rosbridge"].includes(TRANSPORT), "--transport must be http or rosbridge");
    assert(TRANSPORT !== "rosbridge" || ["harvest", "responsive", "a11y"].includes(ONLY),
      "Use --only harvest --harvest, --only responsive or --only a11y with ROSbridge; HTTP fixtures require --transport http");
    assert(!ONLY || [...Object.keys(PAGE_PATHS), "a11y", "responsive", "harvest"].includes(ONLY),
      `Unknown --only target: ${ONLY}`);
    assert(ONLY !== "harvest" || RUN_HARVEST, "--only harvest requires --harvest");
    assert(["on", "off"].includes(RECONSTRUCTION), "--reconstruction must be on or off");
    const {chromium} = require("playwright");
    browser = await chromium.launch({
      headless: !HEADED,
      executablePath: BROWSER,
      args: ["--no-sandbox", "--disable-dev-shm-usage", "--disable-crash-reporter", "--disable-breakpad"],
    });
    requestContext = await browser.newContext();
    request = requestContext.request;
    context = await browser.newContext({viewport: {width: 1440, height: 1000}});
    const state = await (await request.get(BASE + "/api/state")).json();
    assert(state.startup?.facts?.hardware_mode === "mock",
      "Web E2E requires hardware_mode=mock; refusing state-changing checks");
    await test("static assets and headers", () => testStatic(request));
    if (!ONLY || ONLY === "overview") {
      const page = await context.newPage();
      attach(page, "overview");
      await test("overview", async () => { const detail = await testOverview(page); assertPageClean("overview"); return detail; });
      await page.close();
    }
    if (!ONLY || ONLY === "monitor") {
      const page = await context.newPage();
      attach(page, "monitor");
      await test("monitor", async () => { const detail = await testMonitor(page); assertPageClean("monitor"); return detail; });
      await page.close();
    }
    if (!ONLY || ONLY === "debug") {
      const page = await context.newPage();
      attach(page, "debug");
      await test("debug", async () => { const detail = await testDebug(page); assertPageClean("debug"); return detail; });
      await page.close();
    }
    if (!ONLY || ONLY === "analysis") {
      const page = await context.newPage();
      attach(page, "analysis");
      await test("analysis", async () => { const detail = await testAnalysis(page, request); assertPageClean("analysis"); return detail; });
      await page.close();
    }
    if (!ONLY || ONLY === "force") {
      await test("force", () => testForce(browser));
    }
    if (!ONLY || ONLY === "a11y") {
      const page = await context.newPage();
      attach(page, "a11y");
      await test("accessibility", async () => { const detail = await testA11y(page); assertPageClean("a11y"); return detail; });
      await page.close();
    }
    if (!ONLY || ONLY === "responsive") await test("responsive", () => testResponsive(browser));
    if (RUN_HARVEST) {
      harvestStarted = true;
      await test("harvest full flow", () => runHarvest(browser));
    }
  } catch (error) {
    record("suite setup/execution", false, error.stack || error.message || String(error));
  } finally {
    if (harvestStarted) await test("harvest cleanup", async () => {
      const failures = [];
      let cleanupPage = null;
      if (TRANSPORT === "rosbridge") {
        cleanupPage = await context.newPage();
        await goto(cleanupPage, "debug");
        await waitOnline(cleanupPage);
      }
      const post = async (path, payload) => {
        if (cleanupPage) return cleanupPage.evaluate(async ({path, payload}) => {
          const result = path === "/api/params"
            ? await PeachApi.api.setParams(payload.changes, {timeoutMs: 20000})
            : await PeachApi.api.debug(path.split("/").pop(), payload, {timeoutMs: 20000});
          return {ok: result.response.ok, body: result.body};
        }, {path, payload});
        const response = await request.post(BASE + path, {data: payload, timeout: 20000});
        return {ok: response.ok(), body: await response.json()};
      };
      let before = null;
      try { before = await (await request.get(BASE + "/api/state")).json(); }
      catch (error) { failures.push(`cleanup state: ${error.message}`); }
      for (const [action, payload] of [
        ["control_service", {command: "CANCEL_NOW", expected_state_seq: 0, reason: "Web E2E cleanup"}],
        ["set_enables_service", {execution: false, grasp: false, tool: false, reason: "Web E2E cleanup"}],
      ]) {
        if (action === "control_service" && before?.task_executor?.state?.action_active === false) continue;
        try {
          const {ok, body} = await post("/api/debug/" + action, payload);
          assert(ok && body.accepted === true && body.result?.accepted === true,
            `cleanup ${action}: ${JSON.stringify(body)}`);
        } catch (error) { failures.push(error.message); }
      }
      const response = await post("/api/params", {changes: [
        {node: "peach_observability", name: "debug.motion_enabled", value: false},
      ]});
      assert(response.ok, "cleanup motion gate failed");
      let closed = false;
      const deadline = Date.now() + 10000;
      while (Date.now() < deadline) {
        const state = await (await request.get(BASE + "/api/state")).json();
        const task = state.task_executor?.state || {};
        const arm = state.manipulation?.status || {};
        closed = [task, arm].every((value) => ["execution", "grasp", "tool"]
          .every((key) => value[`${key}_enabled`] === false))
          && state.debug?.motion_enabled === false && task.action_active === false
          && arm.running === false;
        if (closed) break;
        await sleep(150);
      }
      assert(closed, "cleanup gates/actions still active or missing on supervisor/arm");
      assert(failures.length === 0, failures.join("; "));
      await cleanupPage?.close();
      return "cancel + disable enables + close motion gate, readback verified";
    });
    try {
      await context?.close();
      await requestContext?.close();
    } catch (error) {
      record("browser context cleanup", false, error.message);
    }
    try { await browser?.close(); }
    catch (error) { record("browser cleanup", false, error.message); }
    const report = {
      base_url: BASE, transport: TRANSPORT, reconstruction: RECONSTRUCTION, out: OUT, started: STAMP, finished: now(),
      passed: results.filter((item) => item.ok).length,
      failed: results.filter((item) => !item.ok).length,
      results,
    };
    fs.writeFileSync(path.join(OUT, "report.json"), JSON.stringify(report, null, 2));
    console.log(`\n${report.passed} passed, ${report.failed} failed -> ${OUT}/report.json`);
    if (report.failed) process.exitCode = 1;
  }
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
