# ivg_sim models（YCB 对象库）

16 个 YCB 对象的 Gazebo SDF 模型（apple / banana / bleach_cleanser / bowl /
chips_can / cracker_box / gelatin_box / master_chef_can / mustard_bottle /
pitcher_base / potted_meat_can / pudding_box / sugar_box / tomato_soup_can /
tuna_fish_can / windex_bottle）。

## 分发口径

- **入库**：每对象的 `model.sdf` + `model.config` + `LICENSE_YCB`（CC BY 4.0）。
- **不入库**：`meshes/`（textured.dae / collision.dae / texture_map.png，
  全库约 118MB）——git 忽略，clone 后执行 `./fetch_ycb.sh` 恢复（或从
  备份拷贝）。缺失 meshes 时 gz 报 `Unable to find uri` / 渲染失败。
- 来源：YCB 对象集 SDF 转换（model.config 署名 Markus Vieth）；上游镜像
  如 <https://github.com/CentralLabFacilities/gazebo_ycb>（meshes 与本
  库一致，fetch 脚本默认从该仓库取）。

## 裁剪记录（2026-09-29 入库轮）

原始克隆含 docs/ examples/ tools/ uniform_quaternions/ 与全量 test_data
（>400MB）；已裁至推理/场景所需子集（checkpoints 无关，本目录无模型权重），
`test_data` 仅保留评分工具不依赖（ivg_sim 自产 GT manifest）。
