# dsh-system-monitor

DeepSeek Harness Web GUI 的系统监控插件：在**右侧栏**新增一个「系统」tab，实时显示
CPU（总占用 + 每核 + 频率）、内存/Swap、GPU（利用率、显存、温度、功耗、风扇、进程）、
温度/风扇/封装功耗，以及 CPU/内存占用最高的进程。形态参考 `htop` / `nvtop`：条形表 +
历史迷你折线（sparkline），每秒刷新。

> 这个目录是 dsh 插件，**不是 ROS 2 包**（因此放了 `COLCON_IGNORE`）。它放在本仓是为了
> 让插件源码有版本管理归宿；运行时安装位置是 `~/.dsh/plugins/dsh-system-monitor`。

## 它由两半组成

| 半边 | 文件 | 作用 |
|---|---|---|
| host（Node，跑在 dsh 宿主进程里） | `lib/index.js` | 采样 `/proc/stat`、`/proc/meminfo`、`/proc/loadavg`、`/proc/cpuinfo`、`/proc/<pid>/{stat,statm}`、`/sys/class/hwmon/**`，并在有 NVIDIA GPU 时调用 `nvidia-smi`；通过 `ctx.webServer.register` 暴露唯一一条**仅回环**路由 `GET /api/dsh-system-monitor/snapshot`；维护一份样本环供折线使用。**零外部 import**（只用 `process.getBuiltinModule`），因此从任意 profile 布局（npm / workspace link / flat fallback）都能解析。 |
| client（浏览器，预构建 bundle） | `lib/client.js` | 用 `window.__ModuleLoader__.load` 声明 `dsh.system-monitor` bundle，向 `ctx.sidebarRightTabs` 注册页类型、向 `sidebar.right.pane.tab` / `.title` 两个席位注册正文与标题，每秒轮询上面那条路由并渲染面板。**不需要构建步骤**：文件本身就是可加载的 JS（用 `react.createElement`，不写 JSX），`require` 只取 shell 平台表里的 `react` / `react/jsx-runtime`。 |

### 为什么 host 侧用 `process.getBuiltinModule`

插件可能以 pnpm workspace link 形式安装，其自身路径下解析不到 `@deepseek-ai/*`。为了让
host 半边在任何布局下都能加载，它**不 import 任何包**，Node 内置模块按需取：

```js
fsModule ??= globalThis.process?.getBuiltinModule?.("node:fs");
```

需要 Node ≥ 22.3（本机 22.22.3 已验证）。旧版本上 `getBuiltinModule` 不存在，采样会走空数据
分支而不会崩。

## 安装

```bash
# 1) 让插件源码出现在 ~/.dsh/plugins 下（运行位置）
mkdir -p ~/.dsh/plugins
cp -r src/dsh_system_monitor ~/.dsh/plugins/dsh-system-monitor

# 2) 装进 web profile：写入 dsh.profile.bundles 并让 cordis 加载它
dsh plugin --profile web add ~/.dsh/plugins/dsh-system-monitor
```

装完后重启 web profile（或依赖 `patchReload: live` 热加载），刷新浏览器页面即可在右侧栏
看到「系统」tab；也可以在右侧栏的引导页/添加控件里找到「系统监控」入口。

卸载：

```bash
dsh plugin --profile web remove dsh-system-monitor
```

## 自检（不起 GUI 也能验）

host 半边可以单独跑通，无需 dsh：

```bash
node -e '
const { apply } = await import("file://" + process.cwd() + "/lib/index.js");
const routes = [];
apply({ logger: console, effect: (f) => f(), webServer: { register: (r) => { routes.push(r); return () => {}; } } }, {});
const res = { writeHead(s) { this.status = s; }, end(b) { console.log(this.status, JSON.parse(b).cpu.usage, JSON.parse(b).gpu.available); } };
await routes[0].handler({ method: "GET", socket: { remoteAddress: "127.0.0.1" } }, res);
' --input-type=module
```

期望：第一次 `cpu.usage = null`（需要两次采样求差），第二次给出百分比；`gpu.available`
取决于 `nvidia-smi` 是否可用。

安装到 profile 之后，可以只读地验证路由已挂上：

```bash
curl -s http://127.0.0.1:3080/api/dsh-system-monitor/snapshot | head -c 400
```

## 排障：装了但看不到 / 点不开

1. **界面上什么都没有** → 客户端 bundle 没被执行（客户端 bundle 是惰性的：脚本只注册工厂，
   `apply()` 要等启动图 import 它才跑）。检查启动图里那条记录是否带 `inject`：

   ```bash
   curl -s "http://127.0.0.1:3080/?token=<你的 token>" | grep -o '{"id":"dsh-system-monitor"[^}]*}'
   # 期望：...,"inject":["@deepseek-ai/dsh-client-ui-slots","@deepseek-ai/dsh-client-ui-sidebar-right"]}
   ```

   没有 `inject` 字段就不会被物化。**改完 manifest 必须重启 `dsh web`**，刷新浏览器不够。

2. **小卡在、点击打不开面板** → 小卡把原因直接显示在第二行（红字），同时 `console.warn`
   一条 `[dsh-system-monitor] …`。三种已知情况：
   - `ctx.sidebarRight 不可用` → 服务未注入，检查 `dsh.client.inject` 与插件 `inject` 服务列表；
   - `openTab 失败：sidebarRight: no session surface is mounted` → 右侧栏从未挂载过。插件会自动
     `layout.openRightbar()` 后重试一次；仍失败说明当前视图没有会话停靠面（先选中一个会话）；
   - `插件上下文未捕获` → bundle 被物化但 `apply()` 没跑完，看控制台里更早的报错。

3. **面板开了但全是「—」** → 宿主路由没通：

   ```bash
   curl -s "http://127.0.0.1:3080/api/dsh-system-monitor/snapshot?token=<你的 token>" | head -c 200
   ```

   200 且带 `"cpu"` / `"memory"` 即正常；401/404 说明宿主半边没加载（同样先重启 `dsh web`）。

## 配置

host 半边暴露 cordis 配置（`Config`，标准 schema 校验）：

| 键 | 默认 | 含义 |
|---|---|---|
| `history` | 180 | 服务端保留的样本数（客户端折线用），8–1200 |
| `minSampleMs` | 200 | 两次内核采样之间的最小间隔（毫秒），50–5000 |
| `gpuTimeoutMs` | 1500 | `nvidia-smi` 超时（毫秒），200–10000 |
| `topProcesses` | 5 | 每个进程榜的条数，0–20 |
| `showProcesses` | true | 关掉则跳过 `/proc/<pid>` 扫描（省 CPU） |

在 profile 的 `cordis.patch.yml` 里按行 id 覆盖，例如：

```yaml
- id: system-monitor
  config:
    topProcesses: 8
    minSampleMs: 500
```

## 边界与已知限制

- **GPU 遥测依赖 `nvidia-smi` 可用**。沙箱/容器里若 NVML 初始化失败（`nvidia-smi` 报
  `Failed to initialize NVML: Unknown Error`），host 会如实把该信息返回，面板显示「无可用
  NVIDIA GPU 遥测 + 原因」，其余部分照常工作。
- **CPU 百分比是两次采样的差**，所以进程启动后的第一帧是 `null`（面板显示「—」），第二帧
  起才有值。
- 进程 CPU% 是相对**两次扫描之间墙钟时间**的单核算力占比（可能 >100%，与 htop 同语义）。
- 温度/风扇/功耗只读 `hwmon`，不解析 `sensors` 文本输出；未加载 `coretemp` 等驱动时该段
  显示提示。
- 面板是**每会话**的右侧栏 tab（右侧栏按会话持有停靠面），不做全局常驻浮窗。
