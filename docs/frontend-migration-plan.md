# IVG Web Dashboard — Vue 3 前端迁移方案

> **决策日期**: 2026-05-15 | **完成日期**: 2026-05-20 | **状态**: ✅ 已完成
>
> 从原生 JS + Web Components (8,660 行) 渐进式迁移到 Vue 3 + TypeScript + 前沿工具链，

---

## 一、最终技术栈

```
Vite 6 + TypeScript 5 + Vue 3.5
Tailwind CSS v4       ← 全球最热原子 CSS (Rust 核心，87k Stars)
Element Plus          ← Vue 3 最广泛的 UI 组件库 (28k Stars，中文母语)
VueUse                ← Vue 3 事实标准工具库 (20k+ Stars, Anthony Fu)
Pinia                 ← Vue 官方状态管理 (80% 采用率)
vue-echarts           ← 百度 ECharts Vue 3 封装 (64k Stars，中文最完善)
unplugin-auto-import  ← 自动导入 Vue API (Anthony Fu)
unplugin-vue-components ← 自动导入 Element Plus 组件 (Anthony Fu)
```

---

## 二、每个工具的决策理由

### Tailwind CSS v4 × UnoCSS

| | Tailwind CSS v4 | UnoCSS |
|--|:--:|:--:|
| GitHub Stars | **87,000** | 17,000 |
| npm 周下载 | **数百万** | 数十万 |
| 构建速度 (Vite) | **268ms** (Rust 核心反超) | 362ms |
| AI 代码生成 | **100%** 默认输出 | 几乎不生成 |
| Vue 3 适配 | Vite 插件原生支持 | Attributify 对 Vue 模板更优 |

> **选 Tailwind CSS v4**：Claude Code 生成的代码 100% 用 Tailwind 类名，无需翻译。全球事实标准，社区问题覆盖率最高喵~

### Element Plus × TDesign × Naive UI

| | Element Plus | TDesign | Naive UI |
|--|:--:|:--:|:--:|
| npm 月下载 | **119 万** | — | 11.4 万 |
| 最近更新 | **1 天前** | 活跃 | 3 个月前 |
| 社区中文教程 | **最多** | 中 | 少 |

> **选 Element Plus**：中文社区最大（119万月下载），零基础搜问题有答案，维护最活跃喵~

### VueUse — 不可替代

`useWebSocket`、`useLocalStorage`、`useResizeObserver`、`useDebounceFn` 各一行代码，直接消灭现有 5 个手写文件 (~500 行) 喵~

### Pinia — 仅用于跨页面全局状态

| 状态 | 存哪里 | 理由 |
|------|--------|------|
| ROS 连接状态 / 运行时配置 | **Pinia** | 所有页面都要知道 |
| 当前工具 ID | **Pinia** | 导航栏 + 3D 查看器 + 控制面板共用 |
| 话题设置值 | VueUse `useLocalStorage` | 不需要全局响应式 |
| 关节数据 / 位姿数据 | 组件内 `ref` | 收到 → 显示，不需要共享 |

---

## 三、现有代码映射

### 保留不动（框架无关的纯逻辑/数学）

| 文件 | 行数 | 去向 | 原因 |
|------|------|------|------|
| `view3d/tf_clients.js` | 212 | `src/lib/tf_math.ts` | 四元数/变换数学，框架无关 |
| `view3d/patches.js` | 71 | `src/lib/ros3d_patches.ts` | Three.js monkey-patch，框架无关 |
| `vision_grasp/projection_overlay.js` | 322 | `src/lib/projection.ts` | Canvas 2D 绘制，框架无关 |
| `vision_grasp/pose_card.ts` | 155 | `src/lib/pose_card.ts` | 纯数据格式化，框架无关 |

### 用工具直接消灭

| 文件 | 行数 | Vue 3 替代 | 节省 |
|------|------|-----------|------|
| `vision_grasp/ui_settings.js` | 97 | `useLocalStorage(key, defaults)` | **1 行** |
| `vision_grasp/mode_controller.js` | 43 | `v-if` / `v-show` | **3 行** |
| `vision_grasp/ui_binder.js` | 61 | `@click` / `@change` | 模板内直接写 |
| `core/dom_cache.js` | 19 | Vue `ref` 模板引用 | **0 行**（删除） |
| `core/ros_connector.js` | 114 | 合并到 `useRos` composable | **0 行**（合并） |
| `view3d/hints.js` | 20 | `v-if` | **1 行** |

### 用 composable 重写

| 文件 | 行数 | Vue 3 composable | 新行数 |
|------|------|-----------------|--------|
| `ivg_transport.js` | 298 | `useRos.ts` (VueUse `useWebSocket`) | ~80 |
| `ivg_runtime.js` | 172 | `useRuntime.ts` | ~50 |
| `ivg_web_video.js` | 124 | `useMJPEGStream.ts` | ~40 |
| `vision_grasp/services.js` | 300 | `useRosService.ts` | ~80 |
| `vision_grasp/subscription_binder.js` | 172 | 各 composable 内 `onMounted`/`onUnmounted` | 分散到各 composable |
| `view3d/session.js` | 457 | `useUrdfViewer.ts` | ~150 |
| `vision_grasp/urdf_panel.js` | 95 | 合并到 `useUrdfViewer.ts` | — |

### 用 Vue 组件重写

| 文件 | 行数 | Vue 组件 |
|------|------|---------|
| `ivg_site_nav.js` | 122 | `SiteNav.vue` (Element Plus 导航) |
| `core/ivg_status_bar.js` | 202 | `RobotStatusBar.vue` (Element Plus `n-tag`) |
| `vision_grasp/joint_chart.js` | 231 | `JointChart.vue` (vue-echarts) |
| `vision_grasp_panel.js` | 656 | `VisionGraspView.vue` (拆分为多个子组件) |

### CSS

| 文件 | 行数 | 去向 |
|------|------|------|
| 全部 7 个 CSS 文件 | ~3,650 行 | Tailwind 原子类 (~60%) + Vue `<style scoped>` (~40%) |

---

## 四、项目目录结构

```
aubo_ros2_ws/src/aubo_ros2_web_dashboard/
│
├── web/                                     # 新前端根目录（Vite 项目）
│   ├── index.html                           # Vite 入口 HTML
│   ├── package.json                         # npm 依赖
│   ├── vite.config.ts                       # Vite + Vue + Tailwind + unplugin
│   ├── tailwind.config.ts                   # Tailwind 配置
│   ├── tsconfig.json                        # TypeScript 配置
│   │
│   ├── public/
│   │   └── favicon.ico
│   │
│   └── src/
│       ├── main.ts                          # 入口
│       ├── App.vue                          # 根布局
│       ├── types/                           # TS 类型定义
│       ├── router/                          # Vue Router
│       ├── stores/                          # Pinia
│       ├── composables/                     # 可复用逻辑
│       ├── components/                      # Vue 组件
│       ├── views/                           # 页面
│       └── lib/                             # 框架无关纯逻辑
│
├── web/public/                              # 旧前端（保留至迁移完成）
│   ├── index.html
│   ├── js/
│   ├── css/
│   └── *.html
│
├── aubo_ros2_web_dashboard/                 # Python 网关（不变）
│   ├── gateway/
│   │   ├── app.py                           # FastAPI 应用
│   │   ├── cli.py
│   │   ├── routes/
│   │   └── ...
│   └── ...
│
├── config/defaults.yaml                     # 配置（不变）
├── launch/web_dashboard.launch.py           # ROS 2 启动（更新静态目录路径）
├── setup.py
└── package.xml
```

---

## 五、迁移阶段

### Phase 1: 骨架搭建（1-2 天）

```
目标: Vite + Vue 3 + TS 跑通，能替换一个最简单的页面 (index.html → DashboardView.vue)

步骤:
1. cd aubo_ros2_ws/src/aubo_ros2_web_dashboard
2. npm create vite@latest web -- --template vue-ts
3. cd web && npm install
4. npm install tailwindcss @tailwindcss/vite
5. npm install element-plus @element-plus/icons-vue
6. npm install @vueuse/core
7. npm install pinia vue-router
8. npm install -D unplugin-auto-import unplugin-vue-components
9. 配置 vite.config.ts (Tailwind v4 + unplugin + Element Plus 解析器)
10. 创建 src/views/DashboardView.vue 替代 index.html
11. npm run dev → 浏览器验证
```

### Phase 2: 核心 composables（2-3 天）

```
目标: 三个核心 composable 跑通，能连接 rosbridge 并订阅话题

1. useRuntime.ts    — 替代 ivg_runtime.js + runtime_provider.js
   └─ 从 /api/v1/runtime 获取运行时配置 (VueUse useFetch)

2. useRos.ts        — 替代 ivg_transport.js + ros_connector.js
   └─ ROSLIB.Ros 单例 + 自动重连 (VueUse useWebSocket)
   └─ subscribe/unsubscribe/onRosJson/callService API 不变

3. useRosTopic.ts   — 替代 subscription_binder.js
   └─ onMounted 订阅 / onUnmounted 取消订阅（自动管理生命周期）

验证: 在 DashboardView 中订阅 /aubo_driver/robot_status，打印消息
```

### Phase 3: 逐个页面迁移（1 周）

```
顺序: 简单 → 复杂，每迁移一个就验证一个

1. DashboardView.vue    (门户首页，最简单，纯展示)
2. SettingsView.vue     (设置页，表单 + localStorage)
3. LogView.vue          (日志页，简单列表)
4. TfMonitorView.vue    (TF 监控，ros3d 集成)
5. VisionGraspView.vue  (视觉抓取，最复杂)
6. CoffeeLatteView.vue  (咖啡拉花，中复杂度)
```

### Phase 4: 清理（1 天）

```
1. 更新 launch/web_dashboard.launch.py 静态目录指向 → web/dist/
2. 删除 web/public/ 旧文件
3. 更新 README.md
4. 全量测试
```

---

## 六、部署流程

### 开发环境

```bash
cd aubo_ros2_ws/src/aubo_ros2_web_dashboard/web
npm install
npm run dev       # Vite 开发服务器 :5173，HMR 热更新
```

### 生产构建

```bash
cd aubo_ros2_ws/src/aubo_ros2_web_dashboard/web
npm run build                   # 生成 web/dist/
```

构建产物由 FastAPI 网关直接服务：

```python
# app.py — 网关静态文件 mount 指向 Vite 构建产物
app.mount("/", StaticFiles(directory="web/dist", html=True), name="static")
```

### ROS 2 启动集成

```python
# launch/web_dashboard.launch.py 修改
static_dir = os.path.join(pkg_share, 'web', 'dist')  # 指向 Vite 构建产物
gateway_cmd = ['python3', '-m', 'aubo_ros2_web_dashboard.fastapi_static_gateway',
               '--web-root', static_dir]
```

### 一键启动

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aubo_ros2_web_dashboard
source install/setup.bash
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
# 浏览器打开 http://<robot_ip>:8090
```

---

## 七、npm 依赖清单

```json
{
  "dependencies": {
    "vue": "^3.5",
    "vue-router": "^4.5",
    "pinia": "^2.3",
    "element-plus": "^2.9",
    "@element-plus/icons-vue": "^2.3",
    "@vueuse/core": "^12",
    "roslib": "^1.5",
    "ros3d": "^1.2",
    "echarts": "^5.6",
    "vue-echarts": "^8.0"
  },
  "devDependencies": {
    "@vitejs/plugin-vue": "^5.2",
    "vite": "^6.0",
    "typescript": "~5.7",
    "tailwindcss": "^4.0",
    "@tailwindcss/vite": "^4.0",
    "unplugin-auto-import": "^19",
    "unplugin-vue-components": "^28",
    "vue-tsc": "^2.2"
  }
}
```

---

## 八、通信架构不变

```
浏览器 (Vue 3 SPA)
  │
  ├── WebSocket /ws/rosbridge → FastAPI 网关 → rosbridge (9090) → ROS 2
  │   └─ useRos.ts (composable) 单例管理连接+订阅+服务调用
  │
  ├── HTTP /api/ivg/proxy/web-video/* → FastAPI → web_video_server (8089)
  │   └─ useMJPEGStream.ts (composable) 自动重连
  │
  ├── HTTP /api/v1/runtime → FastAPI BFF
  │   └─ useRuntime.ts (composable)
  │
  └── HTTP /api/ivg/robot-mesh/* → FastAPI → 3D 模型文件
      └─ useUrdfViewer.ts (composable) ros3d 加载
```

rosbridge 协议不变，roslib/ros3d 库不变，仅封装方式从 `IvgTransport` 构造函数 → `useRos` composable 喵~

---

## 九、ros3d 集成策略

ros3d 管理自己的 Three.js 场景和渲染循环，与 Vue 3 的集成点是 **一个 `<div>` 宿主容器**：

```vue
<template>
  <div ref="urdfHost" class="w-full h-full min-h-[400px]" />
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import { useRos } from '@/composables/useRos'

const urdfHost = ref<HTMLDivElement>()
const { ros } = useRos()  // 共享的 ROSLIB.Ros 实例

let viewer: ROS3D.Viewer
let urdfClient: ROS3D.UrdfClient

onMounted(() => {
  viewer = new ROS3D.Viewer({ divID: urdfHost.value!.id, ... })
  urdfClient = new ROS3D.UrdfClient({ ros, ... })
})

onUnmounted(() => {
  viewer?.stop()
  urdfClient?.unsubscribeTf()
})
</script>
```

与现有 `session.js` 的逻辑完全一致，只是改为 Vue 3 生命周期管理喵~

---

*最后更新: 2026-05-15*
*基于：GitHub Stars、npm 下载量、ROS 2 机器人 Web 可视化社区调查、Claude Code AI 代码生成兼容性分析*
