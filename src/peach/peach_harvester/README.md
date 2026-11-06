# peach_harvester

兼容用 metapackage。当前仅有 `package.xml` 与 ament 导出配置，无 Python/C++
实现、无 launch、无节点；不再提供旧 `brain` 一进程组合。依赖的能力包及系统入口如下：

- `peach_bringup/launch/harvest_system.launch.py`
- `peach_scene_perception`
- `peach_target_reconstruction`
- `peach_supervisor`
- `peach_scene_obstacles`

系统架构见 [docs/architecture](../../../docs/architecture/00-overview.md)；能力包文档在各包 `doc/`。
批次实现与接口见 [peach_supervisor 架构](../peach_supervisor/doc/architecture.md) /
[IO](../peach_supervisor/doc/io.md)。本包承担依赖汇总的兼容角色，归档前须核对外部
包选择/安装脚本，不能仅凭没有可执行文件判断已无用途。
