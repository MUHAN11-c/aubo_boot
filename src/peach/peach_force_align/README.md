# peach_force_align（规划，不是现行 ROS 2 包）

本目录只有 2026-10-21 的 `ForceSeat` 分析与分步实施计划，**没有**
`package.xml` / `CMakeLists.txt` / 源码，`colcon` 不会把它当成包构建。

- 现行五点力驱动在 [`serial_force`](../../serial_force/README.md)；
- 现行臂侧 CONTACT/TOOL 阶段与授权门在 [`peach_arm`](../peach_arm/doc/architecture.md)；
- 力/CoP 数据契约与标定限制见 `serial_force/doc/{io,validation,hardware}.md`。

若启动实现，按 [doc/architecture.md](doc/architecture.md) 与
[doc/steps.md](doc/steps.md) 建包；在此之前不要把本目录加入 launch 或依赖。
