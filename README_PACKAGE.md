# 工作区打包与恢复

本页说明当前 [pack_workspace.sh](scripts/pack_workspace.sh) 的行为，不记录某一次压缩包的提交号或日期。生成物的实际版本、时间和可选内容应随该次交付单独记录。2026-10-23 迁移包原说明已保存于 本机归档（仅本机；公开历史见 Git 提交记录）。

## 包含范围

默认包含当前存在的 `src/` 源码与资产、`docs/`、`wiki/` 导航、`reports/`、`weights/`、`peach_autonomous_harvest_design/`、根文档、`scripts/`、`requirements.txt`、构建配置、`.github`、`.agents`、规则和 `.git/` 历史。`--no-git` 取消历史。

- `--include-archive`：附带 `_archive/` 历史原件；默认不含。
- `--include-tools`：附带 `_tools/`，包含 Blender、cadenv 等本机工具；默认不含。
- `--records-days N`：附带 `runs/` 最近 N 天内修改过的完整顶层会话，包括对应 bag/截图；默认不含运行证据。

脚本保留前端 vendor 发行文件及当前感知模型 `src/peach/peach_scene_perception/model/`。colcon 的任意层级 `build/`、`install/`、`log/`、venv、node_modules、工具缓存和可重下/重渲大件按脚本排除；前端 vendor 的 `build/` 发行目录有专门保留规则。`reconstruction/output/` 按 Git 索引区分已跟踪交付件与未跟踪生成物。

## 预览与生成

在工作区根运行：

```bash
scripts/pack_workspace.sh --dry-run --out /tmp
scripts/pack_workspace.sh
# 示例：附带归档、工具和最近两天的整条证据
scripts/pack_workspace.sh --include-archive --include-tools --records-days 2
```

默认输出在工作区上一级，文件名含实际打包时间。`--out DIR` 改目录，`--zstd` / `--gz` 指定压缩格式；`--help` 查看完整选项。先检查磁盘空间和 dry-run 的包含/排除范围。

## 目标环境恢复

先安装与工作区匹配的 Jazzy apt 依赖。以下只恢复环境并构建，不启动相机、串口、机械臂或 Gazebo：

```bash
tar -xf aubo_e5_jazzy_ws-src-*.tar.zst
cd aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash
mapfile -t peach_package_paths < <(colcon list --packages-up-to peach_system_tests peach_web peach_stereo peach_vegetation --paths-only)
rosdep install --from-paths "${peach_package_paths[@]}" --ignore-src --rosdistro jazzy -y
python3 -m venv --system-site-packages aubo_py3.12
PYTHONNOUSERSITE=1 aubo_py3.12/bin/pip install --ignore-installed -r requirements-peach.txt
bash scripts/build_workspace.sh
```

Python 固定 `numpy==1.26.4`。浏览器资源构建按 [ros2_web_link](src/ros2_web_link/README.md) 和 [peach_web](src/peach/peach_web/README.md) README 执行。`src/peach_sim` 采用其独立运行器与子工作区；恢复环境不表示恢复 Gazebo 运行授权。

可重下资产入口：IVG 权重见 [models/README.md](src/ivg/ivg_pose_estimation/models/README.md)，YCB meshes 见 `src/ivg/ivg_sim/models/fetch_ycb.sh`；仿真重建资产见 [sim_worlds](src/peach_sim/sim_worlds/README.md)。

## 压缩包校验

```bash
zstd -t aubo_e5_jazzy_ws-src-*.tar.zst
tar -tf aubo_e5_jazzy_ws-src-*.tar.zst | head
```

核对交付版本、配置、模型及所需证据后再按 [测试指导](src/peach/peach_system_tests/doc/testing.md) 启动 mock。硬件运行仍遵循 [AGENTS.md](AGENTS.md) 的授权与进程清理约束。

GitHub 新克隆、完整构建与随仓 RGB-D 回放详见 [部署说明](docs/deployment.md)。

标准 `peach_system_tests/data` 样例是部署资产，默认打包保留其 MCAP/metadata/SHA；`--records-days` 仅控制运行证据。当前验收限 Peach，IVG 源码保留但暂不处理。Git worktree 的迁移包使用 `--no-git`，避免携带本机 .git 指针。
