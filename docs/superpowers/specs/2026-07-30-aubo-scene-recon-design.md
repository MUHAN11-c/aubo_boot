# aubo_scene_recon — 眼在手上连续慢扫三维重建（设计）

日期：2026-07-30  
状态：已定稿，Phase 1 实现中

## 目标

机械臂**不间断缓慢移动**时，将 Percipio 彩色点云按 TF 拼到 `base_link`，在 RViz 实时看到累积场景点云（「地图」= 场景融合结果，非导航 costmap）。包**不驱动机械臂**。

约束：开启深度/点云后相机约 0.8 FPS；位姿完全信任手眼 TF，不做 ICP/SLAM。

## 方案

分两阶段同一节点：

1. **Phase 1（本期）**：TF 对齐彩色点云累加 + voxel 下采样 + `/recon/map_cloud` + reset/save  
2. **Phase 2（后）**：`backend:=tsdf` 体素融合出网格

## 架构

```
/camera/depth_registered/points
        → recon_fusion_node
            tf2: base_link ← cloud.frame_id @ stamp
            cloud backend: transform → range filter → voxel merge
        → /recon/map_cloud, /recon/status
        → ~/reset, ~/save
```

前置：bringup + RGB-D + `extrinsics_publisher`。

## 参数（摘要）

| 参数 | 默认 | 说明 |
|---|---|---|
| backend | cloud | cloud \| tsdf |
| map_frame | base_link | 地图坐标系 |
| pointcloud_topic | /camera/depth_registered/points | 输入 |
| voxel_size | 0.005 | m |
| min/max_range | 0.2 / 1.5 | 相机系深度裁剪 |
| tf_timeout_sec | 0.1 | TF 查不到丢帧 |
| publish_period_sec | 0.5 | 发布周期 |
| max_map_points | 2000000 | 超限再 voxel |
| save_dir | `<ws>/recon_maps` | PLY 输出 |

## 验收（Phase 1）

- 静帧目视对齐；慢扫地图连续长大、无明显整层错位  
- reset/save 可用；单元测试覆盖变换/裁剪/voxel/PLY  

## 非目标

抬 FPS、自动扫轨、ICP、扣机器人自遮挡、本期 TSDF 实现。
