/**
 * dsh-system-monitor — host half.
 *
 * Serves one loopback-only JSON route over `ctx.webServer`:
 *
 *   GET /api/dsh-system-monitor/snapshot
 *     → CPU (aggregate + per-core + frequency), memory/swap, load average,
 *       GPU (utilisation, VRAM, temperature, power, fan, clocks, processes),
 *       temperatures / fans / package power, top CPU processes,
 *       plus a small server-side history ring for the sparklines.
 *
 * Everything is sampled from the kernel's own interfaces:
 *   /proc/stat, /proc/meminfo, /proc/loadavg, /proc/cpuinfo, /proc/<pid>/{stat,statm}
 *   /sys/class/hwmon/**, /sys/devices/system/cpu/**
 * plus `nvidia-smi` when an NVIDIA GPU is present. No external npm dependency
 * is imported on purpose (same rule as dsh-balance-widget): the host half must
 * resolve from any profile layout.
 *
 * CPU percentages are deltas between consecutive samples, so the first poll
 * after start reports `null` for utilisation and fills in from the second poll
 * onwards (the panel shows "—" in the meantime).
 */

/** Stable cordis plugin name (also the client bundle mount id). */
export const name = "dsh-system-monitor";

/** Services required before the monitor surface can mount. */
export const inject = ["webServer"];

/** Route path shared with the browser half (spelled here, not imported). */
export const API = {
  snapshot: "/api/dsh-system-monitor/snapshot"
};

/** How many samples the server keeps for the sparklines. */
const DEFAULT_HISTORY = 180;
/** Minimum spacing between two kernel samples, in ms. */
const DEFAULT_MIN_SAMPLE_MS = 200;
/** nvidia-smi query timeout, in ms. */
const DEFAULT_GPU_TIMEOUT_MS = 1500;
/** How many processes to report per list. */
const DEFAULT_TOP_PROCESSES = 5;

/**
 * Config schema exposed to the cordis loader. Cordis calls
 * `Config["~standard"].validate(config)` and expects a Standard Schema
 * (version 1) result, so we hand-build one and keep zero external imports.
 */
const CONFIG_FIELDS = {
  history: "number",
  minSampleMs: "number",
  gpuTimeoutMs: "number",
  topProcesses: "number",
  showProcesses: "boolean"
};
const CONFIG_DEFAULTS = {
  history: DEFAULT_HISTORY,
  minSampleMs: DEFAULT_MIN_SAMPLE_MS,
  gpuTimeoutMs: DEFAULT_GPU_TIMEOUT_MS,
  topProcesses: DEFAULT_TOP_PROCESSES,
  showProcesses: true
};

export const Config = {
  "~standard": {
    version: 1,
    vendor: "dsh-system-monitor",
    validate(value) {
      const source = value !== null && typeof value === "object" ? value : {};
      const out = { ...CONFIG_DEFAULTS };
      for (const [key, type] of Object.entries(CONFIG_FIELDS)) {
        const raw = source[key];
        if (raw === void 0 || raw === null) continue;
        if (typeof raw !== type) {
          return { issues: [{ message: `${key} must be a ${type}`, path: [key] }] };
        }
        out[key] = raw;
      }
      out.history = Math.max(8, Math.min(1200, Math.floor(out.history)));
      out.minSampleMs = Math.max(50, Math.min(5000, Math.floor(out.minSampleMs)));
      out.gpuTimeoutMs = Math.max(200, Math.min(10000, Math.floor(out.gpuTimeoutMs)));
      out.topProcesses = Math.max(0, Math.min(20, Math.floor(out.topProcesses)));
      return { value: out };
    }
  }
};

/* ------------------------------------------------------------------ *
 * generic helpers
 * ------------------------------------------------------------------ */

function writeJson(res, status, body) {
  const payload = JSON.stringify(body);
  res.writeHead(status, {
    "content-type": "application/json; charset=utf-8",
    "content-length": Buffer.byteLength(payload),
    "cache-control": "no-store"
  });
  res.end(payload);
}

/** Whether the request comes from the loopback interface. */
function isLoopbackRequest(req) {
  const address = req.socket?.remoteAddress ?? "";
  return address === "127.0.0.1" || address === "::1" || address === "::ffff:127.0.0.1";
}

function guard(req, res, method) {
  if (!isLoopbackRequest(req)) {
    writeJson(res, 403, { error: "forbidden: loopback-only" });
    return false;
  }
  if ((req.method ?? "GET") !== method) {
    writeJson(res, 405, { error: `method not allowed: ${req.method}` });
    return false;
  }
  return true;
}

function round(value, digits = 1) {
  if (!Number.isFinite(value)) return null;
  const factor = 10 ** digits;
  return Math.round(value * factor) / factor;
}

function clampPercent(value) {
  if (!Number.isFinite(value)) return null;
  return Math.max(0, Math.min(100, round(value, 1)));
}

/* ------------------------------------------------------------------ *
 * /proc sampling
 * ------------------------------------------------------------------ */

/** Split a `/proc` file into lines; missing/unreadable files yield []. */
function procLines(path) {
  try {
    return require$readFileSync(path).split("\n");
  } catch {
    return [];
  }
}

/**
 * Node's fs, resolved without an ESM import so this module keeps zero
 * package-level imports (the plugin may be loaded from a workspace link).
 */
let fsModule;
function require$readFileSync(path) {
  fsModule ??= globalThis.process?.getBuiltinModule?.("node:fs");
  if (fsModule === void 0) throw new Error("node:fs unavailable");
  return fsModule.readFileSync(path, "utf8");
}

/** One `/proc/stat` cpu line → counters. */
function parseCpuLine(line) {
  const parts = line.trim().split(/\s+/);
  if (parts.length < 5) return null;
  const counters = parts.slice(1, 9).map((value) => {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : 0;
  });
  while (counters.length < 8) counters.push(0);
  const [user, nice, system, idle, iowait, irq, softirq, steal] = counters;
  const total = user + nice + system + idle + iowait + irq + softirq + steal;
  return { id: parts[0], total, idle: idle + iowait, busy: total - idle - iowait };
}

function readCpuCounters() {
  const lines = procLines("/proc/stat");
  const aggregate = [];
  const cores = [];
  for (const line of lines) {
    if (!line.startsWith("cpu")) break;
    const parsed = parseCpuLine(line);
    if (parsed === null) continue;
    if (parsed.id === "cpu") aggregate.push(parsed);
    else cores.push(parsed);
  }
  return { aggregate: aggregate[0] ?? null, cores };
}

function utilisation(previous, current) {
  if (previous === null || current === null) return null;
  const total = current.total - previous.total;
  const busy = current.busy - previous.busy;
  if (!(total > 0)) return null;
  return clampPercent((busy / total) * 100);
}

/** Per-core and average MHz from /proc/cpuinfo (no cpufreq driver needed). */
function readCpuFrequency() {
  const mhz = [];
  for (const line of procLines("/proc/cpuinfo")) {
    const match = /^cpu MHz\s*:\s*([0-9.]+)/.exec(line);
    if (match !== null) mhz.push(Number(match[1]));
  }
  if (mhz.length === 0) {
    const freqs = [];
    for (let index = 0; ; index += 1) {
      try {
        const raw = require$readFileSync(`/sys/devices/system/cpu/cpu${index}/cpufreq/scaling_cur_freq`);
        freqs.push(Number(raw.trim()) / 1000);
      } catch {
        break;
      }
    }
    if (freqs.length === 0) return { perCore: [], avg: null, max: null, min: null };
    return {
      perCore: freqs.map((value) => round(value, 0)),
      avg: round(freqs.reduce((sum, value) => sum + value, 0) / freqs.length, 0),
      max: round(Math.max(...freqs), 0),
      min: round(Math.min(...freqs), 0)
    };
  }
  return {
    perCore: mhz.map((value) => round(value, 0)),
    avg: round(mhz.reduce((sum, value) => sum + value, 0) / mhz.length, 0),
    max: round(Math.max(...mhz), 0),
    min: round(Math.min(...mhz), 0)
  };
}

function readMemory() {
  const info = new Map();
  for (const line of procLines("/proc/meminfo")) {
    const match = /^([A-Za-z_()]+):\s+(\d+)/.exec(line);
    if (match !== null) info.set(match[1], Number(match[2]) * 1024);
  }
  const total = info.get("MemTotal") ?? 0;
  const available = info.get("MemAvailable") ?? info.get("MemFree") ?? 0;
  const used = Math.max(0, total - available);
  const swapTotal = info.get("SwapTotal") ?? 0;
  const swapFree = info.get("SwapFree") ?? 0;
  return {
    total,
    available,
    used,
    free: info.get("MemFree") ?? 0,
    cached: (info.get("Cached") ?? 0) + (info.get("SReclaimable") ?? 0),
    buffers: info.get("Buffers") ?? 0,
    dirty: info.get("Dirty") ?? 0,
    usedPercent: total > 0 ? clampPercent((used / total) * 100) : null,
    swap: {
      total: swapTotal,
      used: Math.max(0, swapTotal - swapFree),
      usedPercent: swapTotal > 0 ? clampPercent(((swapTotal - swapFree) / swapTotal) * 100) : null
    }
  };
}

function readLoadAverage() {
  const parts = (procLines("/proc/loadavg")[0] ?? "").trim().split(/\s+/);
  const load = parts.slice(0, 3).map((value) => {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : null;
  });
  return { one: load[0] ?? null, five: load[1] ?? null, fifteen: load[2] ?? null };
}

function readUptimeSeconds() {
  const raw = (procLines("/proc/uptime")[0] ?? "").trim().split(/\s+/);
  const seconds = Number(raw[0]);
  return Number.isFinite(seconds) ? Math.round(seconds) : null;
}

/* ------------------------------------------------------------------ *
 * /sys/class/hwmon sampling
 * ------------------------------------------------------------------ */

function readSysfsText(path) {
  try {
    return require$readFileSync(path).trim();
  } catch {
    return null;
  }
}

function hwmonDirectories() {
  const root = "/sys/class/hwmon";
  const fs = (fsModule ??= globalThis.process?.getBuiltinModule?.("node:fs"));
  if (fs === void 0) return [];
  try {
    return fs
      .readdirSync(root)
      .filter((entry) => entry.startsWith("hwmon"))
      .map((entry) => `${root}/${entry}`)
      .sort();
  } catch {
    return [];
  }
}

/**
 * Temperature / fan / power readings from hwmon. Labels are prefixed with the
 * chip name (coretemp, nvme, …) so identical "Composite" labels stay tellable
 * apart in the panel.
 */
function readThermals() {
  const temperatures = [];
  const fans = [];
  const power = [];
  for (const directory of hwmonDirectories()) {
    const chip = readSysfsText(`${directory}/name`) ?? "hwmon";
    const fs = (fsModule ??= globalThis.process?.getBuiltinModule?.("node:fs"));
    if (fs === void 0) break;
    let entries = [];
    try {
      entries = fs.readdirSync(directory);
    } catch {
      continue;
    }
    for (const entry of entries) {
      const tempMatch = /^temp(\d+)_input$/.exec(entry);
      const fanMatch = /^fan(\d+)_input$/.exec(entry);
      const powerMatch = /^power(\d+)_input$/.exec(entry);
      if (tempMatch !== null) {
        const raw = Number(readSysfsText(`${directory}/${entry}`));
        if (!Number.isFinite(raw)) continue;
        const label = readSysfsText(`${directory}/temp${tempMatch[1]}_label`) ?? `temp${tempMatch[1]}`;
        temperatures.push({ chip, label, celsius: round(raw / 1000, 1) });
      } else if (fanMatch !== null) {
        const raw = Number(readSysfsText(`${directory}/${entry}`));
        if (!Number.isFinite(raw)) continue;
        const label = readSysfsText(`${directory}/fan${fanMatch[1]}_label`) ?? `fan${fanMatch[1]}`;
        fans.push({ chip, label, rpm: Math.round(raw) });
      } else if (powerMatch !== null) {
        const raw = Number(readSysfsText(`${directory}/${entry}`));
        if (!Number.isFinite(raw)) continue;
        const label = readSysfsText(`${directory}/power${powerMatch[1]}_label`) ?? `power${powerMatch[1]}`;
        power.push({ chip, label, watts: round(raw / 1e6, 1) });
      }
    }
  }
  // Keep the panel readable: CPU-package/core style sensors first, then a few
  // extras (board, NVMe, GPU if it reports through hwmon).
  const interesting = /package|core|tctl|tdie|composite|edge|junction|board|acpitz|soc/i;
  temperatures.sort((left, right) => {
    const leftRank = interesting.test(left.label) ? 0 : 1;
    const rightRank = interesting.test(right.label) ? 0 : 1;
    if (leftRank !== rightRank) return leftRank - rightRank;
    if (left.chip !== right.chip) return left.chip.localeCompare(right.chip);
    return left.label.localeCompare(right.label);
  });
  return {
    temperatures: temperatures.slice(0, 24),
    fans,
    power,
    hasSensors: temperatures.length > 0 || fans.length > 0
  };
}

/* ------------------------------------------------------------------ *
 * GPU sampling (nvidia-smi)
 * ------------------------------------------------------------------ */

const GPU_FIELDS = [
  "index",
  "name",
  "utilization.gpu",
  "utilization.memory",
  "memory.used",
  "memory.total",
  "temperature.gpu",
  "power.draw",
  "power.limit",
  "fan.speed",
  "clocks.sm",
  "clocks.mem"
];

function parseNumber(value) {
  if (value === void 0) return null;
  const cleaned = String(value).trim().replace(/[^0-9.+-]/g, "");
  if (cleaned === "" || cleaned === "N/A") return null;
  const parsed = Number(cleaned);
  return Number.isFinite(parsed) ? parsed : null;
}

/** Split a CSV line, honouring the double quotes nvidia-smi uses for names. */
function splitCsvLine(line) {
  const cells = [];
  let current = "";
  let quoted = false;
  for (const character of line) {
    if (character === '"') {
      quoted = !quoted;
    } else if (character === "," && !quoted) {
      cells.push(current);
      current = "";
    } else {
      current += character;
    }
  }
  cells.push(current);
  return cells.map((cell) => cell.trim());
}

function runNvidiaSmi(args, timeoutMs) {
  const childProcess = globalThis.process?.getBuiltinModule?.("node:child_process");
  if (childProcess === void 0) {
    return Promise.resolve({ ok: false, error: "node:child_process unavailable" });
  }
  return new Promise((resolve) => {
    let settled = false;
    const finish = (value) => {
      if (!settled) {
        settled = true;
        resolve(value);
      }
    };
    let child;
    try {
      child = childProcess.execFile(
        "nvidia-smi",
        args,
        { timeout: timeoutMs, maxBuffer: 4 * 1024 * 1024, windowsHide: true },
        (error, stdout, stderr) => {
          if (error !== null && error !== void 0) {
            // nvidia-smi reports NVML failures on stderr ("Failed to initialize
            // NVML: Unknown Error") while execFile's own message only repeats the
            // command line; prefer the tool's own words for the panel hint.
            const detail = String(stderr ?? "").trim().split("\n").filter(Boolean).pop() ??
              error.message ?? "nvidia-smi failed";
            finish({ ok: false, error: detail });
            return;
          }
          finish({ ok: true, stdout: String(stdout ?? "") });
        }
      );
    } catch (error) {
      finish({ ok: false, error: error instanceof Error ? error.message : String(error) });
      return;
    }
    if (child !== void 0 && typeof child.on === "function") {
      child.on("error", (error) => {
        finish({ ok: false, error: error instanceof Error ? error.message : String(error) });
      });
    }
  });
}

async function readGpu(config) {
  const query = await runNvidiaSmi(
    [`--query-gpu=${GPU_FIELDS.join(",")}`, "--format=csv,noheader,nounits"],
    config.gpuTimeoutMs
  );
  if (!query.ok) {
    return { available: false, reason: query.error, devices: [], processes: [] };
  }
  const devices = [];
  for (const line of query.stdout.split("\n")) {
    if (line.trim() === "") continue;
    const cells = splitCsvLine(line);
    if (cells.length < GPU_FIELDS.length) continue;
    const memoryUsed = parseNumber(cells[4]);
    const memoryTotal = parseNumber(cells[5]);
    devices.push({
      index: parseNumber(cells[0]),
      name: cells[1],
      utilizationGpu: clampPercent(parseNumber(cells[2])),
      utilizationMemory: clampPercent(parseNumber(cells[3])),
      memoryUsedMiB: memoryUsed,
      memoryTotalMiB: memoryTotal,
      memoryPercent: memoryTotal !== null && memoryTotal > 0 && memoryUsed !== null ?
        clampPercent((memoryUsed / memoryTotal) * 100) : null,
      temperatureC: parseNumber(cells[6]),
      powerW: parseNumber(cells[7]),
      powerLimitW: parseNumber(cells[8]),
      fanPercent: parseNumber(cells[9]),
      clockSmMHz: parseNumber(cells[10]),
      clockMemMHz: parseNumber(cells[11])
    });
  }
  let processes = [];
  const processQuery = await runNvidiaSmi(
    [
      "--query-compute-apps=gpu_uuid,pid,process_name,used_memory",
      "--format=csv,noheader,nounits"
    ],
    config.gpuTimeoutMs
  );
  if (processQuery.ok) {
    processes = processQuery.stdout
      .split("\n")
      .filter((line) => line.trim() !== "")
      .map((line) => {
        const cells = splitCsvLine(line);
        return {
          gpuUuid: cells[0] ?? "",
          pid: parseNumber(cells[1]),
          name: cells[2] ?? "",
          memoryMiB: parseNumber(cells[3])
        };
      })
      .filter((entry) => entry.pid !== null)
      .sort((left, right) => (right.memoryMiB ?? 0) - (left.memoryMiB ?? 0))
      .slice(0, 12);
  }
  return { available: devices.length > 0, reason: null, devices, processes };
}

/* ------------------------------------------------------------------ *
 * process sampling
 * ------------------------------------------------------------------ */

function readProcesses(previous, config) {
  if (!config.showProcesses || config.topProcesses === 0) {
    return { cpu: [], memory: [], total: null, previous: new Map() };
  }
  const fs = (fsModule ??= globalThis.process?.getBuiltinModule?.("node:fs"));
  if (fs === void 0) return { cpu: [], memory: [], total: null, previous: previous ?? new Map() };
  let entries = [];
  try {
    entries = fs.readdirSync("/proc").filter((entry) => /^\d+$/.test(entry));
  } catch {
    return { cpu: [], memory: [], total: null, previous: previous ?? new Map() };
  }
  const pageSize = 4096;
  const clockTicks = 100;
  const samples = new Map();
  const memory = [];
  for (const entry of entries) {
    const pid = Number(entry);
    let stat;
    let statm;
    try {
      stat = fs.readFileSync(`/proc/${entry}/stat`, "utf8");
      statm = fs.readFileSync(`/proc/${entry}/statm`, "utf8");
    } catch {
      continue; // process vanished mid-scan
    }
    const open = stat.lastIndexOf(")");
    if (open < 0) continue;
    const name = stat.slice(stat.indexOf("(") + 1, open);
    const rest = stat.slice(open + 2).trim().split(/\s+/);
    const utime = Number(rest[11]);
    const stime = Number(rest[12]);
    const rssPages = Number((statm.split(/\s+/)[1] ?? "").trim());
    if (!Number.isFinite(utime) || !Number.isFinite(stime)) continue;
    const cpuSeconds = (utime + stime) / clockTicks;
    const resident = Number.isFinite(rssPages) ? rssPages * pageSize : null;
    samples.set(pid, cpuSeconds);
    memory.push({ pid, name, rssBytes: resident });
  }
  const totalCpu = samples.size;
  // Percentage needs the same sampling window as the CPU counters; express the
  // delta over the wall time between the two /proc scans when we have one.
  const now = Date.now();
  const elapsed = previous?.at !== void 0 ? (now - previous.at) / 1000 : null;
  const cpu = [];
  if (elapsed !== null && elapsed > 0.05) {
    for (const [pid, cpuSeconds] of samples) {
      const before = previous.totals.get(pid);
      if (before === void 0) continue;
      const delta = cpuSeconds - before;
      if (!(delta > 0)) continue;
      cpu.push({ pid, cpuPercent: clampPercent((delta / elapsed) * 100) });
    }
  }
  const named = new Map();
  for (const item of memory) named.set(item.pid, item.name);
  for (const item of cpu) item.name = named.get(item.pid) ?? "";
  memory.sort((left, right) => (right.rssBytes ?? 0) - (left.rssBytes ?? 0));
  cpu.sort((left, right) => (right.cpuPercent ?? 0) - (left.cpuPercent ?? 0));
  return {
    cpu: cpu.slice(0, config.topProcesses),
    memory: memory.slice(0, config.topProcesses),
    total: totalCpu,
    previous: { at: now, totals: samples }
  };
}

/* ------------------------------------------------------------------ *
 * snapshot assembly
 * ------------------------------------------------------------------ */

function createMonitor(ctx, config) {
  const history = [];
  let last = null;
  let processState = null;
  let lastSampleAt = 0;
  let lastSnapshot = null;
  let inFlight = null;

  function sampleOnce() {
    const previous = last;
    const current = readCpuCounters();
    last = current;
    const perCore = current.cores.map((core, index) => ({
      core: index,
      usage: utilisation(previous?.cores?.[index] ?? null, core)
    }));
    const usage = utilisation(previous?.aggregate ?? null, current.aggregate);
    const frequency = readCpuFrequency();
    const memory = readMemory();
    const thermals = readThermals();
    const processResult = readProcesses(processState, config);
    processState = processResult.previous;
    return {
      cpu: {
        usage,
        perCore,
        cores: current.cores.length,
        frequency,
        load: readLoadAverage()
      },
      memory,
      thermals,
      processes: {
        byCpu: processResult.cpu,
        byMemory: processResult.memory,
        total: processResult.total
      }
    };
  }

  function pushHistory(entry) {
    history.push(entry);
    while (history.length > config.history) history.shift();
  }

  async function snapshot() {
    const now = Date.now();
    if (lastSnapshot !== null && now - lastSampleAt < config.minSampleMs) {
      return lastSnapshot;
    }
    if (inFlight !== null) return inFlight;
    inFlight = (async () => {
      const core = sampleOnce();
      const gpu = await readGpu(config);
      const payload = {
        ts: now,
        uptimeSeconds: readUptimeSeconds(),
        cpu: core.cpu,
        memory: core.memory,
        thermals: core.thermals,
        gpu,
        processes: core.processes,
        history: []
      };
      pushHistory({
        t: now,
        cpu: core.cpu.usage,
        mem: core.memory.usedPercent,
        gpu: gpu.devices[0]?.utilizationGpu ?? null,
        vram: gpu.devices[0]?.memoryPercent ?? null
      });
      payload.history = history.slice();
      lastSnapshot = payload;
      lastSampleAt = now;
      return payload;
    })();
    try {
      return await inFlight;
    } finally {
      inFlight = null;
    }
  }

  return { snapshot, history };
}

/** Cordis plugin apply: register the snapshot route. */
export function apply(ctx, config) {
  const resolved = { ...CONFIG_DEFAULTS };
  const validated = Config["~standard"].validate(config ?? {});
  if (validated.issues === void 0) Object.assign(resolved, validated.value);
  else ctx.logger?.warn?.("[dsh-system-monitor] invalid config, using defaults: %o", validated.issues);
  const monitor = createMonitor(ctx, resolved);
  const dispose = ctx.webServer.register({
    kind: "exact",
    path: API.snapshot,
    handler: async (req, res) => {
      if (!guard(req, res, "GET")) return;
      try {
        writeJson(res, 200, await monitor.snapshot());
      } catch (error) {
        ctx.logger?.warn?.("[dsh-system-monitor] snapshot failed: %s", error instanceof Error ? error.message : String(error));
        writeJson(res, 500, { error: error instanceof Error ? error.message : String(error) });
      }
    }
  });
  ctx.effect(() => () => dispose(), "dsh-system-monitor: routes");
  ctx.logger?.info?.("[dsh-system-monitor] serving %s", API.snapshot);
}
