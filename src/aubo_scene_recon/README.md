# aubo_scene_recon

Eye-in-hand 场景重建。默认用 **Open3D** 对彩色点云做 TF 对齐累加 + voxel + 统计滤波；
可选 `backend:=tsdf` 做 RGB-D TSDF。不驱动机械臂。

节点通过 `scripts/recon_fusion_node` 包装脚本用工作区 `aubo_py3.12` 运行（需已装 open3d）。

## 依赖

```bash
aubo_py3.12/bin/pip install 'open3d==0.19.0'
# 或: aubo_py3.12/bin/pip install -r requirements.txt
```

## 运行

```bash
# 前置：bringup(camera_enabled:=false) + percipio_rgbd + extrinsics_publisher
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws && source install/setup.bash
ros2 launch aubo_scene_recon recon.launch.py
```

TSDF（更干净，订阅 color+depth）：

```bash
ros2 launch aubo_scene_recon recon.launch.py backend:=tsdf
```

```bash
ros2 service call /recon_fusion_node/save std_srvs/srv/Trigger {}
ros2 service call /recon_fusion_node/reset std_srvs/srv/Trigger {}
```
