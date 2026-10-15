# BUNKER Pro 2 外观资产来源

- 作者/来源：[AgileX Robotics 官方 UGV 模型仓库](https://github.com/agilexrobotics/ugv_gazebo_sim)。
- 固定提交：`27633a956c845903ee630538afeb17fe70afdd84`。
- 原文件：`bunker/bunker_pro2/meshes/base_link.STL`。
- 本地原件：`bunker_pro2_source.stl`，3,119,484 字节、62,388 个三角面。
- SHA256：`aa9f81c3d75c28e6720fda247bc01d052d17d9343b1e050d13ea2f7a5e60ce9d`。
- 上游 `bunker/bunker_pro2/package.xml` 声明许可为 `BSD`；原声明保留在
  `bunker_pro2_source.package.xml`。上游仓库未提供独立 LICENSE 文本或更具体
  的 BSD 版本，不将资产的许可声明改写为本包源码的 BSD-3-Clause。

`bunker_chassis.glb` 是本轮修改产物：保持来源网格拓扑（导入器移除两重复
三角面），将 CAD 包络重新居中并缩放到 1.05 × 0.71 × 0.54 m，最低点 z=0。
增加黑色橡胶、深灰车体、橙色面板和金属灰轮毂四种 PBR 材质。此空间区域
着色为查看用途，不是厂商的零件/材料标签。URDF 仅在固定工位模式使用它。

在工作区根目录重导出：

```bash
_tools/blender-4.5.14-linux-x64/blender -b --factory-startup --python-exit-code 1 \
  -P src/sim/sim_worlds/blender/color_bunker.py -- \
  --source src/sim/sim_description/meshes/bunker_pro2_source.stl \
  --out src/sim/sim_description/meshes/bunker_chassis.glb
```
