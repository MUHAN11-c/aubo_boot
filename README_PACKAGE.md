# 工作区迁移包说明（README_PACKAGE）

本文件随包提供（由打包流程维护），用于说明本压缩包**装了什么、没装什么、怎么恢复**。
打包入口见 [`scripts/pack_workspace.sh`](scripts/pack_workspace.sh)，参数与排除规则以脚本为准。

## 一、本包范围

- 工作区：`aubo_e5_jazzy_ws`
- 分支/提交：`test/20260924-full-sync` @ `f489b5a1`（内容基线 `d2324947`）
- 打包日期：2026-10-23
- 内容：源码（`src/` 全部 ROS 2 包与资产）、`docs/`、`wiki/`、`reports/`、`weights/`、
  `peach_autonomous_harvest_design/`、根文档、`scripts/`、构建配置（`.clangd`/`.colcon`）、
  `.github`/`.agents`/`.cursor/rules`、`requirements.txt`、`.git/` 完整历史。
- 额外入包（本次要求）：
  - `_archive/`：厂商资料、历史快照与归档；
  - `_tools/`：Blender 发行版、`cadenv`、`shear_pipeline` 等工具与产物；
  - `runs/` 最近 2 天（2026-10-21 16:20 起）有改动的**整条**运行记录，
    含 rosbag2 `*.mcap` 回放包、截图、JSONL/CSV 日志与报告。

## 二、没有入包的内容（目标环境可自动再生成）

| 类别 | 路径 | 恢复方式 |
| --- | --- | --- |
| colcon 编译产物 | `build/` `install/` `log/` | `colcon build` 重新编译 |
| Python 虚拟环境 | `aubo_py3.12/`、任意 `.venv*`、`__pycache__`、`*.pyc` | 见下方 venv 步骤 |
| 前端依赖 | `node_modules/` | `npm ci` / `npm install` |
| IDE/工具缓存 | `.mimosa/` `.serena/` `.codegraph/` `.cache/` `.playwright-mcp/` `.zcode/` `.codex/` 等 | 工具自行重建 |
| IntelliSense 数据库 | `.vscode/browse.vc.db*`（约 2.2G） | 编辑器重新索引 |
| 旧打包产物 | `out/` | 无需恢复 |
| 可重下模型 | `ivg_pose_estimation/models/*.onnx/*.pt`、YCB meshes、`mobileclip2_b.ts` | 见各 README 与 `fetch_*.sh` |
| 可重导出场景 | `field_no_grass.glb` | 见 `sim_worlds/README.md` 重导出 |
| 未跟踪重渲大件 | `sim_worlds/reconstruction/output/` 内未跟踪的 `*.blend`/`*.exr` 等 | Blender 重渲 |

> 说明：`runs/` 中最近 2 天以外的运行记录（含更早的 mcap）不在包内，工作区本地仍保留。
> 按仓库约定（见 `.gitignore`），结构化文本记录才会入库，图像/mcap/点云等二进制只留本地。

## 三、目标环境恢复

```bash
tar -xf aubo_e5_jazzy_ws-src-*.tar.zst
cd aubo_e5_jazzy_ws

# 1) ROS 依赖（需已 source /opt/ros/jazzy/setup.bash）
rosdep install --from-paths src --ignore-src -r -y

# 2) Python 环境
python3 -m venv aubo_py3.12
aubo_py3.12/bin/pip install -r requirements.txt

# 3) 编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

大件与工具的重取入口：

- ivg 权重：`src/ivg/ivg_pose_estimation/models/README.md`
- YCB meshes：`src/ivg/ivg_sim/models/fetch_ycb.sh`
- Blender：本次已随包（`_tools/blender-4.5.14-linux-x64/`）；下次可用 `--include-tools` 决定是否携带

## 四、校验

```bash
zstd -t aubo_e5_jazzy_ws-src-*.tar.zst   # 压缩流完整性
tar -tf aubo_e5_jazzy_ws-src-*.tar.zst | wc -l
```

包内不含 `build/` `install/` `log/` `aubo_py3.12/` 与 `node_modules/`，可用下面的命令抽查：

```bash
tar -tf aubo_e5_jazzy_ws-src-*.tar.zst \
  | grep -E '^aubo_e5_jazzy_ws/(build|install|log|aubo_py3\.12|node_modules)/' | head
```

> `_archive/` 与 `runs/` 内部自带的 `build/`、`log/` 目录属于被归档/被记录的历史现场，随包保留。
