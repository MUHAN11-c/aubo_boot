# GraspNet demo.py 函数用法总结

以 `demo.py` 为基础整理，涵盖命令行参数、各函数参数含义、输入输出及数据类型格式。

---

## 一、概述与入口

**脚本路径**：`graspnet-baseline/demo.py`  
**功能**：加载 RGB-D 数据 → 用预训练 GraspNet 模型预测抓取 → 可选碰撞检测 → NMS 与可视化。

**依赖**：需 CUDA；PointNet2 扩展仅支持 GPU。

---

## 二、命令行参数（argparse）

```bash
python demo.py --checkpoint_path <path> [--data_dir ...] [--num_point ...] ...
```

| 参数 | 类型 | 必填 | 默认 | 含义 |
|------|------|------|------|------|
| `--checkpoint_path` | str | 是 | - | 模型权重文件路径，如 `logs/log_kn/checkpoint-rs.tar`；内含 `model_state_dict`、`epoch` |
| `--data_dir` | str | 否 | `'doc/pose_1'` | 数据目录，需包含 `color.png`、`depth.png`、`workspace_mask.png`、`meta.mat` |
| `--num_point` | int | 否 | 20000 | 送入网络的点云采样点数；越大推理越慢 |
| `--num_view` | int | 否 | 300 | 多视角编码视角数，需与训练时一致 |
| `--collision_thresh` | float | 否 | 0.01 | 碰撞检测阈值（米）；>0 启用碰撞检测，=0 关闭 |
| `--voxel_size` | float | 否 | 0.01 | 碰撞检测前体素下采样的边长（米） |
| `--gpu` | int | 否 | 0 | GPU 设备编号 |

**输入**：命令行参数经 `parser.parse_args()` 得到 `cfgs` 对象。  
**输出**：`cfgs`，属性为上述参数名；`DEVICE = torch.device("cuda:%d" % cfgs.gpu)`。

---

## 三、数据目录要求

`data_dir` 下需包含：

| 文件 | 类型 | 形状/格式 | 说明 |
|------|------|-----------|------|
| `color.png` | RGB 图像 | (H, W, 3)，uint8 | 与 depth 对齐 |
| `depth.png` | 深度图 | (H, W) | 与 color 对齐；单位由 `meta.mat` 的 `factor_depth` 决定 |
| `workspace_mask.png` | 二值掩码 | (H, W)，0/255 或 bool | 有效区域为真 |
| `meta.mat` | MATLAB 文件 | - | 含 `intrinsic_matrix` (3×3)、`factor_depth`（深度缩放因子） |

---

## 四、`get_net()`

**功能**：创建 GraspNet 模型、加载权重并设为 eval 模式。

**输入**：无参数；使用 `cfgs.checkpoint_path`、`cfgs.num_view`、`DEVICE`。

**输出**：
- **类型**：`torch.nn.Module`（GraspNet 实例）
- **位置**：已 `net.to(DEVICE)`
- **状态**：`net.eval()`，权重已加载

**调用的 GraspNet 构造函数**（`models.graspnet.GraspNet`）：

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `input_feature_dim` | int | 0 | 点云特征维数，0 表示仅 xyz |
| `num_view` | int | 300 | 多视角编码视角数 |
| `num_angle` | int | 12 | 抓取角度离散化档位 |
| `num_depth` | int | 4 | 抓取深度档位 |
| `cylinder_radius` | float | 0.05 | 圆柱裁剪半径（米） |
| `hmin` | float | -0.02 | 抓取空间下界（米） |
| `hmax_list` | list of float | [0.01,0.02,0.03,0.04] | 抓取深度候选（米） |
| `is_training` | bool | False | demo 中为 False |

**checkpoint 格式**：
- `torch.load(path)` 得到 dict
- 必有键：`model_state_dict`（state_dict）、`epoch`（int）
- `net.load_state_dict(checkpoint['model_state_dict'])`

---

## 四（续）GraspNet 模型参数详细说明

以下为 `models.graspnet.GraspNet` 构造函数各参数在模型中的**作用**、**使用位置**与**几何/物理含义**。推理或训练时若修改这些参数，必须与 checkpoint 训练时的配置一致，否则需重新训练。

### 1. `input_feature_dim`（默认 0）

- **类型**：int  
- **含义**：点云每个点除 xyz 外附带的**额外特征通道数**。  
- **使用位置**：`GraspNetStage1` → `Pointnet2Backbone(input_feature_dim)`。Backbone 的输入为 `[B, N, 3+input_feature_dim]`，仅当 `input_feature_dim=0` 时输入为纯 xyz。  
- **作用**：0 表示只用坐标；若训练时用过颜色/法向等，则需与训练时一致。demo 中固定为 0。

### 2. `num_view`（默认 300）

- **类型**：int  
- **含义**：**approach 方向（夹爪接近方向）的离散化数量**，即在单位球面上采样的视角个数。  
- **使用位置**：  
  - `GraspNetStage1` → `ApproachNet(num_view, 256)`  
  - `ApproachNet` 中：输出通道含 `2+num_view`（2 为 objectness，num_view 为各视角得分）；`generate_grasp_views(num_view)` 在球面上用 Fibonacci 格点采样得到 `(num_view, 3)` 的单位方向向量。  
- **作用**：每个种子点会预测“最可能的 approach 方向”在哪个视角类上；类别数 = num_view。**必须与训练时一致**，否则权重维度不匹配。  
- **几何**：approach 向量为夹爪从远处接近物体时的方向；300 表示在球面上较密采样，提高朝向分辨率。

### 3. `num_angle`（默认 12）

- **类型**：int  
- **含义**：**绕 approach 轴的平面内旋转角度**的离散类别数，即夹爪在“夹取平面”内的旋转档位。  
- **使用位置**：  
  - `GraspNetStage2` → `CloudCrop`、`OperationNet(num_angle, num_depth)`、`ToleranceNet(num_angle, num_depth)`  
  - `OperationNet` 输出：score / angle_class / width 均为 `num_angle` 维；`pred_decode` 中角度解码为 `grasp_angle_class / 12 * π`（代码写死 12，即默认 num_angle=12）。  
- **作用**：第 i 类对应角度 `i * π/num_angle`（i=0,…,num_angle-1），即 [0°, 15°, …, 165°]（12 类时）。网络为每个 (种子点, 深度) 预测 num_angle 个角度的分数、宽度、容差等。  
- **注意**：`pred_decode` 里用 `12` 做角度解码，若改 num_angle 需同步改 pred_decode。

### 4. `num_depth`（默认 4）

- **类型**：int  
- **含义**：**抓取深度**的离散类别数，即夹爪沿 approach 方向“伸入”的档位。  
- **使用位置**：  
  - `GraspNetStage2` → `CloudCrop`、`OperationNet`、`ToleranceNet`  
  - `CloudCrop`：`hmax_list` 长度必须为 num_depth，为每个深度建一个 `CylinderQueryAndGroup`，在圆柱内采点得到 `(B, C, num_seed, num_depth)` 的特征。  
  - `pred_decode`：深度解码为 `(grasp_depth_class + 1) * 0.01`，即 0.01、0.02、0.03、0.04 米（4 类时）。  
- **作用**：与 `hmax_list` 一一对应，表示“夹爪闭合平面”相对种子点沿 approach 方向的 4 个候选位置；网络为每个深度预测分数并取最高分深度作为最终深度。

### 5. `cylinder_radius`（默认 0.05）

- **类型**：float，单位：米  
- **含义**：**圆柱裁剪区域的半径**。以种子点为轴、沿 approach 方向为轴向的圆柱内点被采入，用于预测该点的抓取。  
- **使用位置**：`GraspNetStage2` → `CloudCrop` → 每个 `CylinderQueryAndGroup(cylinder_radius, hmin, hmax, nsample)`。  
- **作用**：半径越大，参与预测的邻域点越多，感受野越大；过大会包含无关物体，过小则局部几何不足。0.05 m 约 5 cm，与常见夹爪尺寸匹配。

### 6. `hmin`（默认 -0.02）

- **类型**：float，单位：米  
- **含义**：圆柱**下底面**在“圆柱局部 x 轴”（approach 方向）上的坐标。  
- **使用位置**：`CloudCrop` → `CylinderQueryAndGroup(cylinder_radius, hmin, hmax, …)`。圆柱范围在局部坐标系下为 [hmin, hmax] 沿 x 轴。  
- **几何**：在夹爪坐标系下，x 轴通常为 approach 方向；hmin=-0.02 表示从种子点向“远离物体”方向 2 cm 处为圆柱下界，即抓取点后方。

### 7. `hmax_list`（默认 [0.01, 0.02, 0.03, 0.04]）

- **类型**：list of float，单位：米  
- **含义**：每个**抓取深度类别**对应的圆柱**上底面**在局部 x 轴上的坐标；长度必须等于 `num_depth`。  
- **使用位置**：`CloudCrop` 为每个 hmax 创建一个 `CylinderQueryAndGroup(..., hmin, hmax, ...)`，得到 num_depth 个不同深度的裁剪区域。  
- **几何**：hmax=0.01 表示夹爪闭合面在种子点前方 1 cm，0.04 表示 4 cm；即 4 个深度档位对应 1 cm、2 cm、3 cm、4 cm 的“伸入深度”。  
- **与 pred_decode 的关系**：解码时 `grasp_depth = (depth_class + 1) * 0.01`，与 hmax_list 的 0.01/0.02/0.03/0.04 一一对应。

### 8. `is_training`（默认 True，demo 中 False）

- **类型**：bool  
- **含义**：是否为**训练模式**。  
- **使用位置**：  
  - `GraspNet.forward`：若 True 会调用 `process_grasp_labels(end_points)` 生成训练标签（view、angle、depth 等），再交给 Stage2；若 False 直接用 Stage1 的预测作为 Stage2 的输入。  
  - `GraspNetStage2.forward`：True 时用标签里的 `grasp_top_view_rot`、`batch_grasp_point`；False 时用 `end_points['grasp_top_view_rot']`、`end_points['fp2_xyz']`（即 backbone 的种子点与预测的 approach）。  
- **作用**：推理时必须为 False，否则会依赖不存在的标签并改变前向逻辑。

---

### 参数与子模块对应关系小结

| 参数 | 主要影响模块 | 影响内容 |
|------|--------------|----------|
| input_feature_dim | Pointnet2Backbone | 输入点特征维度 |
| num_view | ApproachNet | 视角类别数、approach 方向采样 |
| num_angle | OperationNet, ToleranceNet, CloudCrop 下游 | 平面内角度类别数、输出通道 |
| num_depth | CloudCrop（groupers 个数）, OperationNet, ToleranceNet | 深度类别数、圆柱个数 |
| cylinder_radius, hmin, hmax_list | CloudCrop → CylinderQueryAndGroup | 圆柱裁剪几何与深度档位 |

**训练/推理一致性**：`num_view`、`num_angle`、`num_depth` 及 `hmax_list` 长度直接决定网络各层维度；更换 checkpoint 或修改这些参数时，必须与训练配置一致或重新训练。

---

## 五、`get_and_process_data(data_dir)`

**功能**：从 `data_dir` 读取图像与 meta，生成点云并整理为模型输入。

| 参数 | 类型 | 含义 |
|------|------|------|
| `data_dir` | str | 数据目录路径 |

**输入**：`data_dir` 为 str，指向包含 `color.png`、`depth.png`、`workspace_mask.png`、`meta.mat` 的目录。

**输出**：
- **类型**：`tuple (end_points, cloud)`
- **end_points**：dict
  - `'point_clouds'`：`torch.Tensor`，shape `(1, N, 3)`，dtype float32，在 DEVICE 上；N = `num_point`
  - `'cloud_colors'`：`np.ndarray`，shape `(N, 3)`，dtype float32，0–1
- **cloud**：`o3d.geometry.PointCloud`，全部有效点（未采样），用于碰撞检测与可视化

**中间数据格式**：

| 变量 | 类型 | 形状 | dtype | 说明 |
|------|------|------|-------|------|
| color | np.ndarray | (H, W, 3) | float32 | 0–1 |
| depth | np.ndarray | (H, W) | 由 Image 决定 | 原始深度值 |
| workspace_mask | np.ndarray | (H, W) | 由 Image 决定 | 二值 |
| intrinsic | np.ndarray | (3, 3) | - | meta['intrinsic_matrix'] |
| factor_depth | float | - | - | meta['factor_depth'] |
| cloud（反投影后） | np.ndarray | (H, W, 3) 或 (H*W, 3) | float32 | 单位米 |
| cloud_masked | np.ndarray | (M, 3) | float32 | 有效点，M 不定 |
| cloud_sampled | np.ndarray | (N, 3) | float32 | N = num_point |

---

## 六、`CameraInfo`（utils.data_utils）

**功能**：封装相机内参，供 `create_point_cloud_from_depth_image` 使用。

```python
camera = CameraInfo(width, height, fx, fy, cx, cy, scale)
```

| 参数 | 类型 | 含义 |
|------|------|------|
| width | int | 图像宽度 |
| height | int | 图像高度 |
| fx | float | 焦距 x |
| fy | float | 焦距 y |
| cx | float | 主点 x |
| cy | float | 主点 y |
| scale | float | 深度缩放因子（depth/scale 得到米） |

**属性**：`width`, `height`, `fx`, `fy`, `cx`, `cy`, `scale`。

---

## 七、`create_point_cloud_from_depth_image(depth, camera, organized=True)`

**功能**：由深度图与相机内参反投影为 3D 点云。

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| depth | np.ndarray | 必填 | 深度图，形状 (H, W)，与 camera 尺寸一致 |
| camera | CameraInfo | 必填 | 相机内参 |
| organized | bool | True | True 保持 (H,W,3)，False 为 (H*W,3) |

**输入**：`depth` 形状需为 `(camera.height, camera.width)`。

**输出**：
- **类型**：`np.ndarray`
- **形状**：`organized=True` 时为 `(H, W, 3)`，否则 `(H*W, 3)`
- **dtype**：float32
- **单位**：米（depth 除以 camera.scale）

---

## 八、`get_grasps(net, end_points)`

**功能**：执行前向推理，将输出解码为 GraspGroup。

| 参数 | 类型 | 含义 |
|------|------|------|
| net | GraspNet (nn.Module) | 已加载权重的模型 |
| end_points | dict | 含 `point_clouds` 等的输入字典 |

**输入**：
- `end_points` 至少含 `point_clouds`：`(B, N, 3)` tensor，在 net 所在设备上。

**输出**：
- **类型**：`GraspGroup`（graspnetAPI）
- **内部**：`grasp_group_array`，shape `(M, 17)`，dtype float64
- **17 维格式**：`[score, width, height, depth, R(9), t(3), object_id]`，位姿在相机系，单位米

**流程**：`end_points = net(end_points)` → `grasp_preds = pred_decode(end_points)` → 取 `grasp_preds[0]` 转 numpy → `GraspGroup(gg_array)`。

---

## 九、`pred_decode(end_points)`（models.graspnet）

**功能**：将模型原始输出解码为 17 维抓取数组列表。

**输入**：
- **类型**：dict（`net(end_points)` 的返回值）
- **主要键**：`point_clouds`, `objectness_score`, `grasp_score_pred`, `fp2_xyz`, `grasp_top_view_xyz`, `grasp_angle_cls_pred`, `grasp_width_pred`, `grasp_tolerance_pred`

**输出**：
- **类型**：`list of torch.Tensor`
- **长度**：batch_size（通常 1）
- **每个元素**：shape `(M, 17)`，dtype float32；格式同 GraspGroup 的 `grasp_group_array`（score, width, height, depth, rotation(9), translation(3), object_id）。

---

## 十、`collision_detection(gg, cloud)`

**功能**：无模型碰撞检测，过滤与场景碰撞的抓取。

| 参数 | 类型 | 含义 |
|------|------|------|
| gg | GraspGroup | 待检测的抓取组 |
| cloud | np.ndarray 或 o3d.PointCloud | 场景点云；若为 PointCloud 则用 `np.array(cloud.points)` |

**输入**：
- `gg`：GraspGroup，M 条抓取
- `cloud`：shape `(N, 3)`，dtype float32，单位米；demo 中为 `np.array(cloud.points)`，N 为有效点数

**输出**：
- **类型**：`GraspGroup`
- **含义**：过滤掉 `collision_mask==True` 的抓取后的子集；条数 ≤ M

**内部调用**：`ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)`，再 `detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)`，最后 `gg = gg[~collision_mask]`。

---

## 十一、`ModelFreeCollisionDetector`（utils.collision_detector）

### 11.1 构造函数

```python
mfcdetector = ModelFreeCollisionDetector(scene_points, voxel_size=0.005)
```

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| scene_points | np.ndarray | 必填 | 场景点云，shape `(N, 3)`，dtype float32，单位米 |
| voxel_size | float | 0.005 | 体素下采样边长（米） |

**输出**：实例；内部将点云体素下采样并保存 `self.scene_points`。

### 11.2 `detect(grasp_group, approach_dist=0.03, collision_thresh=0.05, return_empty_grasp=False, empty_thresh=0.01, return_ious=False)`

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| grasp_group | GraspGroup | 必填 | 待检测抓取，M 条 |
| approach_dist | float | 0.03 | 沿 approach 方向检查距离（米） |
| collision_thresh | float | 0.05 | 全局碰撞 IoU 阈值，超过则判为碰撞 |
| return_empty_grasp | bool | False | 是否返回 empty_mask |
| empty_thresh | float | 0.01 | 内部空间 IoU 低于此值判为空抓取 |
| return_ious | bool | False | 是否返回 iou_list |

**输入**：`grasp_group` 为 GraspGroup（M 条）；其余为标量。

**输出**：
- **默认**：`collision_mask`，`np.ndarray`，shape `(M,)`，dtype bool；True 表示碰撞
- **return_empty_grasp=True**：`(collision_mask, empty_mask)`，empty_mask shape `(M,)`
- **return_ious=True**：额外返回 `[global_iou, left_iou, right_iou, bottom_iou, shifting_iou]`，均为 shape `(M,)`

**夹爪参数（类内固定）**：`finger_width=0.01`，`finger_length=0.06`（米）。

---

## 十二、`vis_grasps(gg, cloud)`

**功能**：对抓取做 NMS、按分数排序，取最优一条并用 Open3D 可视化。

| 参数 | 类型 | 含义 |
|------|------|------|
| gg | GraspGroup | 抓取组 |
| cloud | o3d.geometry.PointCloud | 场景点云 |

**输入**：无额外约束；会原地修改 `gg`（nms、sort_by_score、切片）。

**输出**：无返回值；调用 `o3d.visualization.draw_geometries([cloud, *grippers, frame])` 显示。

**流程**：`gg.nms()` → `gg.sort_by_score()` → `gg = gg[:1]` → `gg.to_open3d_geometry_list()` → 与点云、坐标轴一起显示。

---

## 十三、`demo(data_dir)`

**功能**：串联加载模型、处理数据、预测抓取、可选碰撞检测与可视化。

| 参数 | 类型 | 含义 |
|------|------|------|
| data_dir | str | 数据目录，同 `--data_dir` |

**输入**：`data_dir` 为 str。  
**输出**：无返回值。

**流程**：`get_net()` → `get_and_process_data(data_dir)` → `get_grasps(net, end_points)` → 若 `cfgs.collision_thresh > 0` 则 `collision_detection(gg, ...)` → `vis_grasps(gg, cloud)`。

---

## 十四、GraspGroup（graspnetAPI）——demo 中用到的接口

demo 仅使用 GraspGroup 的以下接口：

### 14.1 构造

```python
gg = GraspGroup(gg_array)
```

- **输入**：`gg_array`，`np.ndarray`，shape `(N, 17)`，dtype float64
- **输出**：GraspGroup 实例，内部 `grasp_group_array` 同上

### 14.2 `gg.nms()`

**功能**：非极大值抑制，去除重叠抓取。

**输入**：可选用默认参数 `translation_thresh=0.03`，`rotation_thresh=30/180*π`；demo 中无参调用。

**输出**：返回新 GraspGroup；demo 中未接收返回值，可能使用 `gg.nms()` 的原地版本需查看 graspnetAPI 实现。实际 grasp.py 中 `nms` 返回新 GraspGroup，不修改 self；若 demo 写 `gg.nms()` 未赋值，则 gg 不变。查看 demo：`gg.nms()` 无赋值，因此若 nms 返回新对象，demo 中 gg 不会被更新。需核对 graspnetAPI 的 nms 是否为原地操作。

根据之前阅读的 grasp.py：`nms` 返回 `GraspGroup(nms_grasp(...))`，是新的 GraspGroup，不修改 self。所以 demo 的 `gg.nms()` 未接收返回值，gg 实际未做 NMS。此处按文档说明接口行为，使用时建议写 `gg = gg.nms()`。

### 14.3 `gg.sort_by_score(reverse=False)`

**功能**：按 score 排序；`reverse=False` 为降序（高分在前）。

**输入**：`reverse` 可选，默认 False。  
**输出**：返回 self（原地排序）。

### 14.4 切片 `gg[:1]`

**输入**：切片索引。  
**输出**：GraspGroup，只含前 1 条抓取。

### 14.5 `gg.to_open3d_geometry_list()`

**输入**：无。  
**输出**：`list of open3d.geometry.Geometry`，每条抓取对应夹爪几何体，用于可视化。

### 14.6 布尔索引 `gg[~collision_mask]`

**输入**：`collision_mask`，`np.ndarray`，shape `(M,)`，dtype bool。  
**输出**：GraspGroup，保留 `mask==False` 的抓取。

---

## 十五、6D 抓取数组格式（GraspGroup 内部）

- **形状**：`(N, 17)`，dtype float64
- **列含义**：

| 索引 | 名称 | 含义 | 单位 |
|------|------|------|------|
| 0 | score | 抓取质量分数 | 无量纲 |
| 1 | width | 夹爪开口宽度 | 米 |
| 2 | height | 夹爪高度 | 米 |
| 3 | depth | 抓取深度 | 米 |
| 4–12 | rotation_matrix | 3×3 旋转矩阵行优先 | 无量纲 |
| 13–15 | translation | 抓取中心 (tx, ty, tz) | 米 |
| 16 | object_id | 物体 ID | 整数 |

---

## 十六、运行示例

```bash
python demo.py --checkpoint_path logs/log_kn/checkpoint-rs.tar --data_dir doc/pose_1 --num_point 20000 --collision_thresh 0.01 --voxel_size 0.01 --gpu 0
```

---

## 十七、示例说明

下面用**具体例子**说明命令行用法、数据准备、GraspNet 参数含义以及代码调用后的数据形状。

### 17.1 命令行示例

**例 1：使用默认数据目录，开启碰撞检测**

```bash
python demo.py --checkpoint_path logs/log_kn/checkpoint-rs.tar
# 等价于：data_dir=doc/pose_1, num_point=20000, num_view=300, collision_thresh=0.01, voxel_size=0.01, gpu=0
```

**例 2：使用自己的数据目录**

```bash
python demo.py --checkpoint_path logs/log_kn/checkpoint-rs.tar --data_dir /home/user/my_scene
# 要求 my_scene 下存在 color.png, depth.png, workspace_mask.png, meta.mat
```

**例 3：关闭碰撞检测、减少点云数以加快推理**

```bash
python demo.py --checkpoint_path logs/log_kn/checkpoint-rs.tar --data_dir doc/pose_1 --collision_thresh 0 --num_point 10000
# collision_thresh=0 表示不做碰撞过滤；num_point 越小推理越快、但可能漏检
```

**例 4：指定第二块 GPU**

```bash
CUDA_VISIBLE_DEVICES=1 python demo.py --checkpoint_path logs/log_kn/checkpoint-rs.tar --gpu 0
# 或在多卡环境下：--gpu 1 表示用 cuda:1
```

---

### 17.2 数据目录示例

**目录结构**（例如 `doc/pose_1`）：

```
doc/pose_1/
├── color.png           # 例如 640×480 RGB
├── depth.png           # 640×480，像素值如 0–5000（毫米）
├── workspace_mask.png  # 640×480，有效区域 255，其余 0
└── meta.mat            # 见下
```

**meta.mat 示例**（深度单位毫米，分辨率 640×480）：

```python
import scipy.io as scio
import numpy as np

# 假设相机内参：fx=615.0, fy=615.0, cx=320.0, cy=240.0
intrinsic_matrix = np.array([
    [615.0,    0, 320.0],
    [   0, 615.0, 240.0],
    [   0,    0,    1 ]
])
# 深度图单位毫米，转米需除以 1000
factor_depth = 1000.0

scio.savemat('doc/pose_1/meta.mat', {
    'intrinsic_matrix': intrinsic_matrix,
    'factor_depth': np.array([factor_depth])
})
```

则 `depth[i,j] / 1000` 得到该像素处的深度（米）；反投影时用 `points_z = depth / factor_depth`。

---

### 17.3 GraspNet 参数举例（几何含义）

**num_view = 300**

- 在单位球面上用 Fibonacci 格点采样 300 个方向，每个方向是一个“approach 候选”。
- 每个种子点会得到 300 个得分，取 argmax 得到当前点的最佳 approach 方向。
- 例如第 0 个视角可能是 (0.05, 0.86, 0.51)，第 149 个是另一方向；推理时只保留得分最高的那个方向。

**num_angle = 12**

- 绕 approach 轴的平面内角度分成 12 类：0°, 15°, 30°, …, 165°。
- 例如类别 0 → 0°，类别 3 → 45°，类别 6 → 90°。
- 网络为每个 (种子点, 深度) 预测 12 个角度的分数、宽度、容差；解码时取分数最高的角度。

**num_depth = 4 与 hmax_list = [0.01, 0.02, 0.03, 0.04]**

- 深度类别 0 → 0.01 m（夹爪闭合面在种子点前方 1 cm）
- 深度类别 1 → 0.02 m（2 cm）
- 深度类别 2 → 0.03 m（3 cm）
- 深度类别 3 → 0.04 m（4 cm）
- 圆柱裁剪：对每个深度在 [hmin=-0.02, hmax=0.01/0.02/0.03/0.04] 的圆柱内各采 64 个点，得到 4 组特征，再预测 4 个深度的分数并取最优。

**cylinder_radius = 0.05**

- 以种子点为轴、approach 为 x 轴，半径 5 cm 的圆柱内的点参与预测。
- 例如种子点在 (0.3, 0.1, 0.5)，approach 沿 z 负方向，则圆柱覆盖大致 x∈[0.25,0.35], y∈[0.05,0.15], z∈[0.46,0.54] 的局部范围（示意）。

---

### 17.4 代码调用与数据形状示例

**完整流程**（与 demo 一致）：

```python
# 假设已 parse 得到 cfgs，DEVICE 已设置
net = get_net()                                    # net: GraspNet, 在 DEVICE 上
end_points, cloud = get_and_process_data('doc/pose_1')

# end_points['point_clouds'].shape = (1, 20000, 3), 在 GPU 上
# cloud: Open3D PointCloud, 点数 M 不定（有效点总数）

gg = get_grasps(net, end_points)                   # gg: GraspGroup
# gg.grasp_group_array.shape = (K, 17), K 为预测抓取数（通常几百到几千）

gg = collision_detection(gg, np.array(cloud.points))  # 若 cfgs.collision_thresh > 0
# 过滤后 gg 条数 ≤ K

gg = gg.nms()                                      # 建议显式接收返回值
gg.sort_by_score()
gg = gg[:1]                                        # 只留最高分一条
grippers = gg.to_open3d_geometry_list()            # list of 1 个 Geometry
# 与 cloud、坐标系一起送入 draw_geometries
```

**单步形状小结**：

| 步骤 | 变量 | 形状/类型示例 |
|------|------|----------------|
| get_and_process_data | point_clouds | (1, 20000, 3) float32 tensor |
| get_and_process_data | cloud.points | (M, 3) 有效点 |
| get_grasps | gg.grasp_group_array | (K, 17) float64，K 不定 |
| collision_detection 后 | gg.grasp_group_array | (K', 17)，K' ≤ K |
| gg[:1] | gg.grasp_group_array | (1, 17) |
| to_open3d_geometry_list | 返回值 | list of 1 个 o3d.geometry.Geometry |

---

### 17.5 单条抓取数组示例（17 维）

解码后一条抓取的 `grasp_array` 示例（单位：米，相机坐标系）：

```
score=0.82, width=0.04, height=0.02, depth=0.02,
rotation_matrix=[[...],[...],[...]],   # 3×3
translation=[0.31, 0.08, 0.52],       # 抓取中心
object_id=-1                            # 推理时常为 -1
```

表示：在 (0.31, 0.08, 0.52) 处、以给定旋转为姿态、开口 4 cm、夹爪高 2 cm、深度 2 cm 的抓取，质量分数 0.82。

---

以上为 demo.py 及相关模块的函数用法总结与示例；实现细节以源码为准。
