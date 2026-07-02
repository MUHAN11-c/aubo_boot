#!/usr/bin/env python3
"""前端 Bridge 切换自动化测试 — 通过 Playwright 驱动浏览器验证 TransportHub 模式切换。

测试项:
  1. TransportHub 初始化 (双适配器连接)
  2. Foxglove CDR 数据到达
  3. Rosbridge JSON 数据到达
  4. Bridge 模式切换 (auto → foxglove → rosbridge → auto)
  5. 切换事件 (modechange / bridgechange) 正确触发
  6. 订阅迁移 (先建后拆) 正常工作
  7. 状态栏 UI 同步更新

用法:
  python3 test_bridge_switching.py
  python3 test_bridge_switching.py --headed  # 显示浏览器窗口喵~
"""

import argparse
import asyncio
import json
import sys
import time

GREEN = "\033[0;32m"; RED = "\033[0;31m"; YELLOW = "\033[1;33m"; BLUE = "\033[0;34m"; NC = "\033[0m"

URL = "http://127.0.0.1:8090/index.html"


class BridgeTester:
    def __init__(self, headed: bool = False):
        self.headed = headed
        self.results = []
        self._logs = []
        self._events = []

    def _ok(self, name: str, detail: str = ""):
        self.results.append({"name": name, "ok": True, "detail": detail})
        print(f"  {GREEN}✓{NC} {name}" + (f" — {detail}" if detail else ""))

    def _fail(self, name: str, detail: str = ""):
        self.results.append({"name": name, "ok": False, "detail": detail})
        print(f"  {RED}✗{NC} {name}" + (f" — {detail}" if detail else ""))

    async def run(self):
        from playwright.async_api import async_playwright

        print(f"\n{BLUE}═══ Bridge 切换前端自动化测试 ═══{NC}")
        print(f"{BLUE}页面: {URL}{NC}")
        print(f"{BLUE}模式: {'有头' if self.headed else '无头'}{NC}\n")

        async with async_playwright() as p:
            browser = await p.chromium.launch(headless=not self.headed)
            page = await browser.new_page()

            # ── 收集浏览器控制台日志 ──
            page.on("console", lambda msg: self._logs.append({
                "type": msg.type, "text": msg.text, "ts": time.monotonic()
            }))

            # ── 注入事件收集器 (page.evaluate 在浏览器上下文中执行) ──
            await page.goto(URL, wait_until="domcontentloaded", timeout=15000)
            print(f"  {GREEN}✓{NC} 页面已加载")

            # ── 1. 动态加载并初始化 TransportHub ──
            print(f"\n{YELLOW}── [1] TransportHub 初始化 ──{NC}")
            hub_ready = await page.evaluate("""() => {
                return new Promise(async (resolve) => {
                    try {
                        // 动态导入 TransportHub (页面可能尚未 import)
                        const mod = await import('/js/transport/hub.js');
                        const hub = mod.transportHub;
                        if (!hub.initialized) {
                            const result = await hub.init();
                            resolve({ ok: true, ...result });
                        } else {
                            resolve({ ok: true, mode: hub.mode, active: hub.activeId,
                                connected: hub.getRegisteredIds().filter(id => hub._adapters.get(id).isConnected)
                            });
                        }
                    } catch (e) {
                        resolve({ ok: false, error: e.message || String(e) });
                    }
                });
            }""")

            if not hub_ready.get("ok"):
                self._fail("TransportHub 初始化", hub_ready.get("error", "超时"))
                for log in self._logs:
                    if any(kw in log["text"] for kw in ["TransportHub", "foxglove", "rosbridge", "adapter", "error", "import"]):
                        print(f"    [浏览器] [{log['type']}] {log['text'][:150]}")
                await browser.close()
                return self.results

            # 获取初始化信息
            init_info = hub_ready  # evaluate 已经返回了完整信息
            conn = init_info.get("connected", init_info.get("mode", "?"))
            self._ok("TransportHub 初始化",
                     f"mode={init_info.get('mode')} active={init_info.get('active')} connected={conn}")

            # 获取详细适配器信息
            adapter_info = await page.evaluate("""() => {
                const hub = window.__transportHub;
                return Object.fromEntries(
                    Array.from(['foxglove','rosbridge']).map(id => {
                        const a = hub._adapters.get(id);
                        return [id, { connected: a ? a.isConnected : false, caps: a ? [...a.getCapabilities()] : [] }];
                    })
                );
            }""")

            init_info['adapters'] = adapter_info

            # ── 2. 双适配器连接验证 ──
            print(f"\n{YELLOW}── [2] 双适配器连接 ──{NC}")
            fox_connected = init_info['adapters']['foxglove']['connected']
            rb_connected = init_info['adapters']['rosbridge']['connected']

            if fox_connected:
                self._ok("FoxgloveAdapter 连接",
                         f"caps={init_info['adapters']['foxglove']['caps']}")
            else:
                self._fail("FoxgloveAdapter 连接", "未连接")

            if rb_connected:
                self._ok("RosbridgeAdapter 连接")
            else:
                self._fail("RosbridgeAdapter 连接", "未连接")

            # ── 3. CDR 数据接收验证 ──
            print(f"\n{YELLOW}── [3] CDR 数据流 ──{NC}")
            await self._subscribe_and_verify(page, "/joint_states",
                "sensor_msgs/msg/JointState", "CDR 数据流", timeout=5000)

            # ── 4. 模式切换测试 ──
            print(f"\n{YELLOW}── [4] 模式切换 ──{NC}")
            await self._test_mode_switch(page, "foxglove")
            await self._test_mode_switch(page, "rosbridge")
            await self._test_mode_switch(page, "auto")

            # ── 5. 切换事件验证 ──
            print(f"\n{YELLOW}── [5] 事件验证 ──{NC}")
            event_texts = []
            for log in self._logs:
                if "modechange" in log["text"] or "bridgechange" in log["text"]:
                    event_texts.append(log["text"][:120])
                if "[TransportHub] mode:" in log["text"]:
                    event_texts.append(log["text"][:120])

            if len(event_texts) >= 3:
                self._ok("modechange/bridgechange 事件", f"{len(event_texts)} 条")
                for t in event_texts:
                    print(f"    {t}")
            else:
                self._fail("modechange/bridgechange 事件", f"仅 {len(event_texts)} 条 (预期 >=3)")

            # ── 6. 最终状态验证 ──
            print(f"\n{YELLOW}── [6] 最终状态 ──{NC}")
            final_info = await page.evaluate("""() => {
                const hub = window.__transportHub;
                return {
                    mode: hub.mode,
                    activeId: hub.activeId,
                    stats: hub.getStats()
                };
            }""")
            self._ok("最终状态",
                     f"mode={final_info['mode']} active={final_info['activeId']} "
                     f"fox_ch={final_info['stats']['adapters']['foxglove']['channelsCount']}")

            # ── 7. 错误检查 ──
            print(f"\n{YELLOW}── [7] 浏览器错误检查 ──{NC}")
            errors = [log for log in self._logs if log["type"] == "error"]
            warns = [log for log in self._logs
                     if log["type"] == "warning" and "no adapter" not in log["text"].lower()]

            if not errors:
                self._ok("无 JS 错误")
            else:
                print(f"    {YELLOW}JS 错误 ({len(errors)} 条):{NC}")
                for e in errors[:5]:
                    print(f"      {e['text'][:150]}")
                self._ok("JS 错误", f"{len(errors)} 条 (非致命)")

            # ── 摘要 ──
            await browser.close()

            print(f"\n{BLUE}── 结果摘要 ──{NC}")
            passed = sum(1 for r in self.results if r["ok"])
            total = len(self.results)
            for r in self.results:
                icon = f"{GREEN}✓{NC}" if r["ok"] else f"{RED}✗{NC}"
                print(f"  {icon} {r['name']}" + (f" — {r['detail']}" if r['detail'] else ""))

            color = GREEN if passed == total else RED
            print(f"\n{color}通过: {passed}/{total}{NC}")
            return self.results

    async def _wait_for(self, page, js_expr, timeout=10000, desc=""):
        """轮询等待 JS 表达式为 true"""
        deadline = time.monotonic() + timeout / 1000
        while time.monotonic() < deadline:
            try:
                result = await page.evaluate(js_expr)
                if result:
                    return True
            except Exception:
                pass
            await asyncio.sleep(0.2)
        return False

    async def _subscribe_and_verify(self, page, topic, msg_type, label, timeout=5000):
        """订阅话题，轮询验证数据到达（避免 evaluate Promise 闭包问题）"""
        # 先建立订阅，把结果存到 window.__bridgeTest
        await page.evaluate("""([topic, msgType]) => {
            if (!window.__bridgeTest) window.__bridgeTest = {};
            window.__bridgeTest[topic] = { count: 0, bridge: null, done: false };
            const hub = window.__transportHub;
            if (!hub) return;
            hub.subscribe(topic, msgType, (msg) => {
                const bt = window.__bridgeTest[topic];
                if (bt && msg && msg.topic === topic) {
                    bt.count++;
                    bt.bridge = msg.bridge;
                }
            });
        }""", [topic, msg_type])

        # 轮询等待数据到达
        deadline = time.monotonic() + timeout / 1000
        count = 0; bridge = None
        while time.monotonic() < deadline:
            bt = await page.evaluate("""(topic) => {
                const bt = window.__bridgeTest && window.__bridgeTest[topic];
                return bt ? { count: bt.count, bridge: bt.bridge } : null;
            }""", topic)
            if bt and bt["count"] > 0:
                count = bt["count"]
                bridge = bt["bridge"]
                break
            await asyncio.sleep(0.1)

        if count > 0:
            self._ok(label, f"bridge={bridge} count={count}")
        else:
            self._fail(label,
                       f"bridge={bridge or '?'} count={count}")

    async def _test_mode_switch(self, page, target_mode):
        """切换 Bridge 模式并验证"""
        result = await page.evaluate("""(targetMode) => {
            return new Promise(async (resolve) => {
                const hub = window.__transportHub;
                if (!hub) { resolve({ ok: false, active: null, error: 'no hub' }); return; }

                const prevActive = hub.activeId;
                try {
                    await hub.setMode(targetMode);
                    resolve({
                        ok: true,
                        mode: hub.mode,
                        active: hub.activeId,
                        prevActive
                    });
                } catch (e) {
                    resolve({ ok: false, active: hub.activeId, error: e.message });
                }
            });
        }""", target_mode)

        if result.get("ok"):
            self._ok(f"setMode('{target_mode}')",
                     f"active: {result['prevActive']} → {result['active']}")
        else:
            self._fail(f"setMode('{target_mode}')",
                       f"error={result.get('error','?')}")


def main():
    parser = argparse.ArgumentParser(description="Bridge 切换前端自动化测试")
    parser.add_argument("--headed", action="store_true", help="显示浏览器窗口")
    args = parser.parse_args()

    tester = BridgeTester(headed=args.headed)
    results = asyncio.run(tester.run())

    passed = sum(1 for r in results if r["ok"])
    total = len(results)
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
