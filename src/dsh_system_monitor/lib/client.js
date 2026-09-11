/**
 * dsh-system-monitor — browser half.
 *
 * Registers the right-sidebar "System" tab:
 *   1. `ctx.sidebarRightTabs.register(...)` declares the page kind (no
 *      patterns: it is a page, opened by kind).
 *   2. `ctx.slots.register({ name: "sidebar.right.pane.tab", key: <same id> })`
 *      binds the body that renders inside the tab.
 *   3. `ctx.slots.inject("sidebar.right.pane.tab.title", ...)` supplies the chip
 *      label, and the `guide` entry makes the tab discoverable from the guide
 *      page's entry list.
 *
 * The panel polls the host half's loopback route once per second and draws
 * nvtop/htop-style meters with app theme variables. It is authored as a plain
 * prebuilt bundle (`window.__ModuleLoader__.load`), exactly like the shipped
 * client bundles, so the package needs no build step:
 *
 *   - `require` resolves the shell's frozen module table (react,
 *     react/jsx-runtime, @deepseek-ai/dsh-client-ui-primitives), plus whatever
 *     `dsh.client.inject` names.
 *   - `react.createElement` is used instead of JSX to keep the file valid
 *     JavaScript as-is.
 *
 * Everything degrades gracefully: no GPU / no hwmon / older host simply means
 * the corresponding section shows a hint instead of breaking the panel.
 */

window.__ModuleLoader__.load({
  id: "dsh-system-monitor",
  factory: (require) => {
    const module = { exports: {} };
    const exports = module.exports;
    Object.defineProperty(exports, Symbol.toStringTag, { value: "Module" });

    const react = require("react");
    const h = react.createElement;

    /** Stable ids: the tab type registry, the slot key and the chip all agree. */
    const TYPE_ID = "dsh-system-monitor/system";
    const TAB_KIND = "system-monitor";
    const NS = "dsh-system-monitor";

    /**
     * Client context captured in `apply`, for handlers outside the effect scope
     * (the sidebar chip's click). Failures are NOT swallowed: every distinct
     * step that can go wrong gets its own message, it is logged to the console,
     * and the chip surfaces it — a silently dead click is undebuggable.
     */
    let pluginCtx = null;
    function openSystemTab() {
      if (pluginCtx === null) {
        return { ok: false, error: "插件上下文未捕获（bundle 未执行 apply）" };
      }
      const controller = pluginCtx.sidebarRight;
      if (controller === void 0 || controller === null) {
        return { ok: false, error: "ctx.sidebarRight 不可用（右侧栏服务未注入）" };
      }
      const attempt = () => {
        try {
          controller.openTab(TAB_KIND);
          return { ok: true, error: null };
        } catch (error) {
          const message = error instanceof Error ? error.message : String(error);
          pluginCtx.logger?.warn?.(`[${NS}] openTab(${TAB_KIND}) failed: ${message}`);
          return { ok: false, error: message };
        }
      };
      const first = attempt();
      if (first.ok) return { ok: true, error: null };
      // `openTab` needs a mounted session surface. If the right sidebar was never
      // opened in this view, expand it first (the frame's layout service) and
      // retry once on the next tick, so a click from the always-visible chip
      // works even before the user has ever opened the right column.
      const layout = pluginCtx.layout;
      if (layout !== void 0 && layout !== null && typeof layout.openRightbar === "function") {
        try {
          layout.openRightbar(false, false);
          setTimeout(() => {
            const retry = attempt();
            if (!retry.ok) pluginCtx.logger?.warn?.(`[${NS}] retry after openRightbar failed: ${retry.error}`);
          }, 0);
          return { ok: true, error: null, deferred: true };
        } catch (error) {
          const message = error instanceof Error ? error.message : String(error);
          pluginCtx.logger?.warn?.(`[${NS}] openRightbar failed: ${message}`);
          return { ok: false, error: `展开右侧栏失败：${message}` };
        }
      }
      return { ok: false, error: `openTab 失败：${first.error}` };
    }

    /** Host route registered by the node half (same path spelled there). */
    const SNAPSHOT_URL = "/api/dsh-system-monitor/snapshot";
    /** Panel poll cadence; the host additionally rate-limits its own sampling. */
    const POLL_MS = 1000;
    /** Chip poll cadence: an idle GUI should not be woken every second. */
    const CHIP_POLL_MS = 3000;

    /* ---------------------------------------------------------------- *
     * formatting helpers
     * ---------------------------------------------------------------- */

    function clampPercent(value) {
      if (typeof value !== "number" || !Number.isFinite(value)) return null;
      return Math.max(0, Math.min(100, value));
    }

    function formatBytes(bytes) {
      if (typeof bytes !== "number" || !Number.isFinite(bytes)) return "—";
      const units = ["B", "KiB", "MiB", "GiB", "TiB"];
      let value = bytes;
      let unit = 0;
      while (value >= 1024 && unit < units.length - 1) {
        value /= 1024;
        unit += 1;
      }
      return `${value >= 100 || unit === 0 ? Math.round(value) : value.toFixed(1)} ${units[unit]}`;
    }

    function formatPercent(value, digits = 0) {
      if (typeof value !== "number" || !Number.isFinite(value)) return "—";
      return `${value.toFixed(digits)}%`;
    }

    function formatNumber(value, digits = 0) {
      if (typeof value !== "number" || !Number.isFinite(value)) return "—";
      return value.toFixed(digits);
    }

    /** Utilisation → theme colour step: normal / warn / hot. */
    function levelOf(percent) {
      const value = clampPercent(percent);
      if (value === null) return 0;
      if (value >= 90) return 2;
      if (value >= 75) return 1;
      return 0;
    }

    const LEVEL_COLOR = [
      "var(--dsw-alias-state-business-primary, #4d6bfe)",
      "var(--dsw-alias-state-warn-primary, #d98b0b)",
      "var(--dsw-alias-state-error-primary, #d94a4a)"
    ];

    function formatUptime(seconds) {
      if (typeof seconds !== "number" || !Number.isFinite(seconds)) return "—";
      const days = Math.floor(seconds / 86400);
      const hours = Math.floor((seconds % 86400) / 3600);
      const minutes = Math.floor((seconds % 3600) / 60);
      if (days > 0) return `${days}d ${hours}h`;
      if (hours > 0) return `${hours}h ${minutes}m`;
      return `${minutes}m`;
    }

    /* ---------------------------------------------------------------- *
     * primitives
     * ---------------------------------------------------------------- */

    const S = {
      root: {
        display: "flex",
        flexDirection: "column",
        gap: "14px",
        padding: "12px 14px 18px",
        fontSize: "12px",
        lineHeight: "18px",
        color: "var(--dsw-alias-label-primary, inherit)",
        overflowY: "auto",
        height: "100%",
        boxSizing: "border-box"
      },
      header: {
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        gap: "8px"
      },
      title: { fontWeight: 600, fontSize: "13px" },
      meta: {
        color: "var(--dsw-alias-label-tertiary, #8a8f98)",
        fontSize: "11px",
        display: "flex",
        gap: "10px",
        flexWrap: "wrap"
      },
      card: {
        border: "1px solid var(--dsw-alias-border-l1, rgba(127,127,127,.22))",
        borderRadius: "10px",
        padding: "10px 12px",
        display: "flex",
        flexDirection: "column",
        gap: "8px",
        background: "var(--dsw-alias-bg-layer-2, transparent)"
      },
      cardHead: {
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        gap: "8px"
      },
      cardTitle: { fontWeight: 600 },
      row: {
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        gap: "10px"
      },
      label: {
        color: "var(--dsw-alias-label-secondary, #6b7280)",
        whiteSpace: "nowrap",
        overflow: "hidden",
        textOverflow: "ellipsis"
      },
      value: { fontVariantNumeric: "tabular-nums", fontWeight: 600 },
      bar: {
        position: "relative",
        height: "6px",
        borderRadius: "3px",
        background: "var(--dsw-alias-border-l2, rgba(127,127,127,.22))",
        overflow: "hidden"
      },
      hint: {
        color: "var(--dsw-alias-label-tertiary, #8a8f98)",
        fontSize: "11px",
        lineHeight: "16px"
      },
      grid: {
        display: "grid",
        gridTemplateColumns: "repeat(auto-fill, minmax(58px, 1fr))",
        gap: "4px 8px"
      },
      coreChip: {
        display: "flex",
        alignItems: "center",
        gap: "5px",
        fontSize: "10px",
        fontVariantNumeric: "tabular-nums",
        color: "var(--dsw-alias-label-secondary, #6b7280)"
      },
      coreTrack: {
        position: "relative",
        flex: "1 1 auto",
        height: "4px",
        minWidth: "14px",
        borderRadius: "2px",
        background: "var(--dsw-alias-border-l2, rgba(127,127,127,.22))",
        overflow: "hidden"
      },
      refresh: {
        background: "none",
        border: "1px solid var(--dsw-alias-border-l2, rgba(127,127,127,.28))",
        borderRadius: "6px",
        color: "var(--dsw-alias-label-secondary, #6b7280)",
        cursor: "pointer",
        font: "inherit",
        padding: "1px 7px"
      },
      chip: {
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        gap: "8px",
        margin: "0 8px 4px",
        padding: "5px 8px",
        border: "1px solid var(--dsw-alias-border-l1, rgba(127,127,127,.22))",
        borderRadius: "8px",
        background: "var(--dsw-alias-bg-layer-2, transparent)",
        color: "var(--dsw-alias-label-primary, inherit)",
        cursor: "pointer",
        font: "inherit",
        fontSize: "11px",
        lineHeight: "16px",
        textAlign: "left"
      }
    };

    function Meter(props) {
      const percent = clampPercent(props.percent);
      const level = levelOf(percent);
      return h(
        "div",
        { style: S.bar, title: percent === null ? "无数据" : `${percent.toFixed(1)}%` },
        h("div", {
          style: {
            position: "absolute",
            inset: "0 auto 0 0",
            width: `${percent === null ? 0 : percent}%`,
            background: props.color ?? LEVEL_COLOR[level],
            transition: "width .35s ease"
          }
        })
      );
    }

    function StatRow(props) {
      return h(
        "div",
        { style: S.row },
        h("span", { style: S.label }, props.label),
        h(
          "span",
          { style: { ...S.value, color: props.color } },
          props.value,
          props.detail === void 0 ? null : h(
            "span",
            { style: { ...S.hint, marginLeft: "6px", fontWeight: 400 } },
            props.detail
          )
        )
      );
    }

    /** Sparkline built from the host's history ring (no chart dependency). */
    function Sparkline(props) {
      const points = (props.points ?? []).filter((value) => typeof value === "number" && Number.isFinite(value));
      const width = 220;
      const height = 34;
      if (points.length < 2) {
        return h("div", { style: { ...S.hint, height: `${height}px` } }, "累积样本中…");
      }
      const step = width / (points.length - 1);
      const path = points
        .map((value, index) => {
          const x = index * step;
          const y = height - (Math.max(0, Math.min(100, value)) / 100) * height;
          return `${index === 0 ? "M" : "L"}${x.toFixed(1)},${y.toFixed(1)}`;
        })
        .join(" ");
      const area = `${path} L${width},${height} L0,${height} Z`;
      const color = LEVEL_COLOR[levelOf(points[points.length - 1])];
      return h(
        "svg",
        {
          viewBox: `0 0 ${width} ${height}`,
          preserveAspectRatio: "none",
          style: { width: "100%", height: `${height}px`, display: "block" }
        },
        h("path", { d: area, fill: color, opacity: 0.16 }),
        h("path", {
          d: path,
          fill: "none",
          stroke: color,
          strokeWidth: 1.5,
          vectorEffect: "non-scaling-stroke"
        })
      );
    }

    function Section(props) {
      return h(
        "section",
        { style: S.card },
        h(
          "div",
          { style: S.cardHead },
          h("span", { style: S.cardTitle }, props.title),
          props.aside === void 0 ? null : h("span", { style: S.hint }, props.aside)
        ),
        props.children
      );
    }

    /* ---------------------------------------------------------------- *
     * sections
     * ---------------------------------------------------------------- */

    function CpuSection(props) {
      const cpu = props.snapshot.cpu ?? {};
      const perCore = Array.isArray(cpu.perCore) ? cpu.perCore : [];
      const frequency = cpu.frequency ?? {};
      const load = cpu.load ?? {};
      return h(
        Section,
        {
          title: "CPU",
          aside: `${cpu.cores ?? "—"} 核${
            frequency.avg === null || frequency.avg === void 0 ? "" : ` · 平均 ${formatNumber(frequency.avg)} MHz`
          }`
        },
        h(StatRow, {
          label: "总占用",
          value: formatPercent(cpu.usage, 1),
          color: LEVEL_COLOR[levelOf(cpu.usage)]
        }),
        h(Meter, { percent: cpu.usage }),
        h(Sparkline, {
          points: (props.snapshot.history ?? []).map((entry) => entry.cpu)
        }),
        h(
          "div",
          { style: S.grid },
          perCore.map((core) =>
            h(
              "div",
              { key: core.core, style: S.coreChip, title: `core ${core.core}: ${formatPercent(core.usage, 1)}` },
              h("span", { style: { minWidth: "18px" } }, `c${core.core}`),
              h(
                "span",
                { style: S.coreTrack },
                h("span", {
                  style: {
                    position: "absolute",
                    inset: "0 auto 0 0",
                    width: `${clampPercent(core.usage) ?? 0}%`,
                    background: LEVEL_COLOR[levelOf(core.usage)]
                  }
                })
              )
            )
          )
        ),
        h(
          "div",
          { style: S.meta },
          h("span", null, `负载 ${formatNumber(load.one, 2)} / ${formatNumber(load.five, 2)} / ${formatNumber(load.fifteen, 2)}`),
          frequency.min === null || frequency.min === void 0 ?
            null : h("span", null, `频率 ${formatNumber(frequency.min)}–${formatNumber(frequency.max)} MHz`)
        )
      );
    }

    function MemorySection(props) {
      const memory = props.snapshot.memory ?? {};
      const swap = memory.swap ?? {};
      return h(
        Section,
        { title: "内存", aside: formatBytes(memory.total) },
        h(StatRow, {
          label: "已用",
          value: formatPercent(memory.usedPercent, 1),
          detail: `${formatBytes(memory.used)} / ${formatBytes(memory.total)}`,
          color: LEVEL_COLOR[levelOf(memory.usedPercent)]
        }),
        h(Meter, { percent: memory.usedPercent }),
        h(Sparkline, { points: (props.snapshot.history ?? []).map((entry) => entry.mem) }),
        h(
          "div",
          { style: S.meta },
          h("span", null, `可用 ${formatBytes(memory.available)}`),
          h("span", null, `缓存 ${formatBytes(memory.cached)}`),
          h("span", null, `Swap ${formatPercent(swap.usedPercent, 1)}（${formatBytes(swap.used)}）`)
        )
      );
    }

    function GpuSection(props) {
      const gpu = props.snapshot.gpu ?? { available: false, devices: [], processes: [] };
      if (!gpu.available) {
        return h(
          Section,
          { title: "GPU" },
          h("div", { style: S.hint }, "无可用 NVIDIA GPU 遥测"),
          gpu.reason === null || gpu.reason === void 0 ? null : h(
            "div",
            { style: S.hint, wordBreak: "break-word" },
            String(gpu.reason).slice(0, 200)
          )
        );
      }
      const primary = gpu.devices[0];
      return h(
        Section,
        {
          title: "GPU",
          aside: gpu.devices.length > 1 ? `${gpu.devices.length} 张` : primary.name
        },
        h(StatRow, {
          label: "利用率",
          value: formatPercent(primary.utilizationGpu, 0),
          color: LEVEL_COLOR[levelOf(primary.utilizationGpu)]
        }),
        h(Meter, { percent: primary.utilizationGpu }),
        h(Sparkline, { points: (props.snapshot.history ?? []).map((entry) => entry.gpu) }),
        h(StatRow, {
          label: "显存",
          value: formatPercent(primary.memoryPercent, 1),
          detail: `${formatNumber(primary.memoryUsedMiB)} / ${formatNumber(primary.memoryTotalMiB)} MiB`,
          color: LEVEL_COLOR[levelOf(primary.memoryPercent)]
        }),
        h(Meter, { percent: primary.memoryPercent }),
        h(
          "div",
          { style: S.meta },
          h("span", null, `温度 ${formatNumber(primary.temperatureC)}°C`),
          h("span", null, `功耗 ${formatNumber(primary.powerW)}/${formatNumber(primary.powerLimitW)} W`),
          primary.fanPercent === null ? null : h("span", null, `风扇 ${formatNumber(primary.fanPercent)}%`),
          h("span", null, `SM ${formatNumber(primary.clockSmMHz)} MHz`)
        ),
        gpu.devices.length > 1 ? h(
          "div",
          { style: S.grid },
          gpu.devices.slice(1).map((device) =>
            h(
              "div",
              { key: device.index, style: S.coreChip, title: device.name },
              h("span", { style: { minWidth: "18px" } }, `g${device.index}`),
              h(
                "span",
                { style: S.coreTrack },
                h("span", {
                  style: {
                    position: "absolute",
                    inset: "0 auto 0 0",
                    width: `${clampPercent(device.utilizationGpu) ?? 0}%`,
                    background: LEVEL_COLOR[levelOf(device.utilizationGpu)]
                  }
                })
              )
            )
          )
        ) : null,
        (gpu.processes ?? []).length === 0 ? null : h(
          "div",
          { style: { display: "flex", flexDirection: "column", gap: "2px" } },
          gpu.processes.slice(0, 5).map((process) =>
            h(
              "div",
              { key: `${process.pid}-${process.name}`, style: S.row },
              h("span", { style: S.label }, `${process.name} #${process.pid}`),
              h("span", { style: { ...S.hint, fontVariantNumeric: "tabular-nums" } }, `${formatNumber(process.memoryMiB)} MiB`)
            )
          )
        )
      );
    }

    function ThermalSection(props) {
      const thermals = props.snapshot.thermals ?? {};
      const temperatures = Array.isArray(thermals.temperatures) ? thermals.temperatures : [];
      if (temperatures.length === 0) {
        return h(
          Section,
          { title: "温度" },
          h("div", { style: S.hint }, "未发现 hwmon 温度传感器（lm-sensors/coretemp 未加载？）")
        );
      }
      const hottest = temperatures.reduce(
        (best, item) => (best === null || (item.celsius ?? -Infinity) > (best.celsius ?? -Infinity) ? item : best),
        null
      );
      return h(
        Section,
        { title: "温度", aside: `最高 ${formatNumber(hottest?.celsius, 1)}°C` },
        temperatures.slice(0, 12).map((item) =>
          h(
            "div",
            { key: `${item.chip}/${item.label}`, style: S.row },
            h("span", { style: S.label, title: `${item.chip} · ${item.label}` }, `${item.label}`),
            h(
              "span",
              {
                style: {
                  ...S.value,
                  color: item.celsius >= 85 ? LEVEL_COLOR[2] : item.celsius >= 70 ? LEVEL_COLOR[1] : void 0
                }
              },
              `${formatNumber(item.celsius, 1)}°C`
            )
          )
        ),
        (thermals.fans ?? []).length === 0 ? null : h(
          "div",
          { style: S.meta },
          thermals.fans.map((fan) =>
            h("span", { key: `${fan.chip}/${fan.label}` }, `${fan.chip} ${fan.label} ${fan.rpm} rpm`)
          )
        ),
        (thermals.power ?? []).length === 0 ? null : h(
          "div",
          { style: S.meta },
          thermals.power.map((entry) =>
            h("span", { key: `${entry.chip}/${entry.label}` }, `${entry.chip} ${formatNumber(entry.watts, 1)} W`)
          )
        )
      );
    }

    function ProcessSection(props) {
      const processes = props.snapshot.processes ?? {};
      const byCpu = Array.isArray(processes.byCpu) ? processes.byCpu : [];
      const byMemory = Array.isArray(processes.byMemory) ? processes.byMemory : [];
      if (byCpu.length === 0 && byMemory.length === 0) return null;
      const line = (item, right, key) =>
        h(
          "div",
          { key, style: S.row },
          h("span", { style: S.label, title: `${item.name} #${item.pid}` }, `${item.name}`),
          h("span", { style: { ...S.hint, fontVariantNumeric: "tabular-nums" } }, right)
        );
      return h(
        Section,
        { title: "进程", aside: processes.total === null ? void 0 : `${processes.total} 个` },
        byCpu.length === 0 ? null : h(
          "div",
          { style: { display: "flex", flexDirection: "column", gap: "2px" } },
          h("div", { style: S.hint }, "CPU 占用最高"),
          byCpu.map((item, index) => line(item, formatPercent(item.cpuPercent, 1), `cpu-${item.pid}-${index}`))
        ),
        byMemory.length === 0 ? null : h(
          "div",
          { style: { display: "flex", flexDirection: "column", gap: "2px" } },
          h("div", { style: S.hint }, "内存占用最高"),
          byMemory.map((item, index) => line(item, formatBytes(item.rssBytes), `mem-${item.pid}-${index}`))
        )
      );
    }

    /* ---------------------------------------------------------------- *
     * sidebar footer chip (always-visible summary + entry point)
     * ---------------------------------------------------------------- */

    /**
     * Compact CPU / memory / GPU summary in the left sidebar footer. It is the
     * discoverable entry point to the full panel: clicking opens the
     * right-sidebar tab. Polls slower than the panel, which refreshes on open.
     */
    function SystemChip() {
      const [snapshot, setSnapshot] = react.useState(null);
      const [error, setError] = react.useState(null);
      const [navError, setNavError] = react.useState(null);
      react.useEffect(() => {
        let cancelled = false;
        let timer = null;
        const pull = async () => {
          try {
            const response = await fetch(SNAPSHOT_URL, { headers: { accept: "application/json" } });
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            const payload = await response.json();
            if (!cancelled) {
              setSnapshot(payload);
              setError(null);
            }
          } catch (cause) {
            if (!cancelled) setError(cause instanceof Error ? cause.message : String(cause));
          } finally {
            if (!cancelled) timer = setTimeout(pull, CHIP_POLL_MS);
          }
        };
        pull();
        return () => {
          cancelled = true;
          if (timer !== null) clearTimeout(timer);
        };
      }, []);

      const gpuPercent = snapshot?.gpu?.available ? snapshot.gpu.devices?.[0]?.utilizationGpu ?? null : null;
      const hottest = (snapshot?.thermals?.temperatures ?? []).reduce(
        (best, item) => (item.celsius !== null && item.celsius !== void 0 && item.celsius > best ? item.celsius : best),
        -Infinity
      );
      const title = snapshot === null ?
        (error === null ? "系统监控：采样中…" : `系统监控不可用：${error}`) :
        [
          `CPU ${formatPercent(snapshot.cpu?.usage, 0)}`,
          `内存 ${formatPercent(snapshot.memory?.usedPercent, 0)}`,
          snapshot.gpu?.available ? `GPU ${formatPercent(gpuPercent, 0)}` : "GPU 无遥测",
          Number.isFinite(hottest) ? `最高温 ${formatNumber(hottest, 1)}°C` : null,
          navError === null ? "点击打开系统面板" : `打不开面板：${navError}`
        ].filter(Boolean).join(" · ");

      const cell = (label, value, percent) =>
        h(
          "span",
          { style: { display: "inline-flex", alignItems: "center", gap: "3px" } },
          h("span", { style: { color: "var(--dsw-alias-label-tertiary, #8a8f98)" } }, label),
          h("span", { style: { fontVariantNumeric: "tabular-nums", color: LEVEL_COLOR[levelOf(percent)] } }, value)
        );

      return h(
        "button",
        {
          type: "button",
          onClick: () => {
            const outcome = openSystemTab();
            setNavError(outcome.ok ? null : outcome.error);
          },
          title,
          style: { ...S.chip, flexDirection: "column", alignItems: "stretch", gap: "2px" }
        },
        h(
          "span",
          { style: { display: "flex", alignItems: "center", justifyContent: "space-between", gap: "8px" } },
          h("span", { style: { fontWeight: 600 } }, "系统"),
          cell("CPU", formatPercent(snapshot?.cpu?.usage, 0), snapshot?.cpu?.usage),
          cell("MEM", formatPercent(snapshot?.memory?.usedPercent, 0), snapshot?.memory?.usedPercent),
          snapshot?.gpu?.available ?
            cell("GPU", formatPercent(gpuPercent, 0), gpuPercent) :
            h("span", { style: { color: "var(--dsw-alias-label-tertiary, #8a8f98)" } }, "GPU —")
        ),
        navError === null ? null : h(
          "span",
          {
            style: {
              color: LEVEL_COLOR[2],
              fontSize: "10px",
              lineHeight: "14px",
              whiteSpace: "normal",
              wordBreak: "break-word"
            }
          },
          navError
        )
      );
    }

    /* ---------------------------------------------------------------- *
     * panel
     * ---------------------------------------------------------------- */

    function SystemPanel() {
      const [snapshot, setSnapshot] = react.useState(null);
      const [error, setError] = react.useState(null);
      const [paused, setPaused] = react.useState(false);
      const [tick, setTick] = react.useState(0);

      react.useEffect(() => {
        if (paused) return void 0;
        let cancelled = false;
        let timer = null;
        const pull = async () => {
          try {
            const response = await fetch(SNAPSHOT_URL, { headers: { accept: "application/json" } });
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            const payload = await response.json();
            if (!cancelled) {
              setSnapshot(payload);
              setError(null);
            }
          } catch (cause) {
            if (!cancelled) setError(cause instanceof Error ? cause.message : String(cause));
          } finally {
            if (!cancelled) timer = setTimeout(pull, POLL_MS);
          }
        };
        pull();
        return () => {
          cancelled = true;
          if (timer !== null) clearTimeout(timer);
        };
      }, [paused, tick]);

      const body = [];
      if (snapshot === null) {
        body.push(h("div", { key: "boot", style: S.hint }, error === null ? "采样中…" : `无法读取：${error}`));
      } else {
        if (error !== null) body.push(h("div", { key: "err", style: S.hint }, `最近一次读取失败：${error}`));
        body.push(h(CpuSection, { key: "cpu", snapshot }));
        body.push(h(MemorySection, { key: "mem", snapshot }));
        body.push(h(GpuSection, { key: "gpu", snapshot }));
        body.push(h(ThermalSection, { key: "thermal", snapshot }));
        body.push(h(ProcessSection, { key: "proc", snapshot }));
      }

      return h(
        "div",
        { style: S.root },
        h(
          "div",
          { style: S.header },
          h("span", { style: S.title }, "系统负载"),
          h(
            "span",
            { style: { display: "flex", gap: "6px", alignItems: "center" } },
            h("span", { style: S.hint }, snapshot === null ? "" : `运行 ${formatUptime(snapshot.uptimeSeconds)}`),
            h(
              "button",
              {
                type: "button",
                style: S.refresh,
                title: paused ? "继续每秒刷新" : "暂停刷新",
                onClick: () => setPaused((value) => !value)
              },
              paused ? "继续" : "暂停"
            ),
            h(
              "button",
              {
                type: "button",
                style: S.refresh,
                title: "立即刷新",
                onClick: () => setTick((value) => value + 1)
              },
              "刷新"
            )
          )
        ),
        body
      );
    }

    /* ---------------------------------------------------------------- *
     * cordis client plugin
     * ---------------------------------------------------------------- */

    /**
     * Cordis service dependencies: the slot seat registry, the right sidebar's
     * tab registry and controller, and the frame layout (used only as the
     * expand-then-retry fallback when no session surface is mounted yet).
     */
    const inject = ["slots", "sidebarRightTabs", "sidebarRight", "layout"];

    function apply(ctx) {
      pluginCtx = ctx;
      ctx.effect(
        () =>
          ctx.sidebarRightTabs.register({
            id: TYPE_ID,
            kind: TAB_KIND,
            title: () => "系统",
            guide: [{ kind: TAB_KIND, title: "系统监控", description: "CPU / 内存 / GPU / 温度", order: 50 }]
          }),
        "dsh-system-monitor: tab type"
      );
      // Always-visible chip: the entry point that does not require the user to
      // discover the right sidebar first.
      ctx.effect(
        () =>
          ctx.slots.inject("sidebar.footer.action", () =>
            ctx.slots.register({ name: "sidebar.footer.action", id: "system-monitor", order: 20 }, SystemChip)
          ),
        "dsh-system-monitor: sidebar chip"
      );
      ctx.effect(
        () =>
          ctx.slots.inject("sidebar.right.pane.tab", () =>
            ctx.slots.register({ name: "sidebar.right.pane.tab", key: TYPE_ID }, SystemPanel)
          ),
        "dsh-system-monitor: tab body"
      );
      ctx.effect(
        () =>
          ctx.slots.inject("sidebar.right.pane.tab.title", () =>
            ctx.slots.register({ name: "sidebar.right.pane.tab.title", key: TYPE_ID }, () => h("span", null, "系统"))
          ),
        "dsh-system-monitor: tab title"
      );
    }

    exports.apply = apply;
    exports.inject = inject;
    return module.exports;
  }
});
