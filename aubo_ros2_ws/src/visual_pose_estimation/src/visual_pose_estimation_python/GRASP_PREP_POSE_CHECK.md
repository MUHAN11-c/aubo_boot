# 准备 / 抓取位姿计算详细检查

本文档梳理**准备姿态**（preparation）与**抓取姿态**（grasp）的完整计算链路、公式与潜在问题。

---

## 1. 数据流概览

```
模板加载 (pose_estimator)
  ├─ grab_position.json / preparation_position.json → T_B_E_grasp, T_B_E_prep
  ├─ camera_pose.json → T_B_E_camera
  ├─ T_E_C (手眼标定)
  └─ T_C_E_grasp = inv(T_B_C_template) @ T_B_E_grasp
     T_C_E_prep   = inv(T_B_C_template) @ T_B_E_prep
     (T_B_C_template = T_B_E_camera @ T_E_C)

估计请求 (ros2_communication → pose_estimator.estimate_pose)
  ├─ T_B_C: 当前相机位姿 (基座→相机)
  ├─ T_B_C_template: 模板相机位姿 (来自 camera_pose + T_E_C)
  ├─ 目标特征: workpiece_center (u,v), 深度图 or 模板深度
  ├─ dtheta: 角度差 (暴力匹配 or standardized_angle 差)
  └─ 输出: result.T_B_E_grasp, result.T_B_E_prep
```

---

## 2. 抓取姿态 (T_B_E_grasp) 计算步骤

**代码位置**: `pose_estimator.py` → `estimate_pose`，约 1358–1504 行。

### 2.1 输入

| 符号 | 含义 | 来源 |
|------|------|------|
| `T_C_E_grasp_template` | 模板抓取位姿（相机→末端） | `best_template.T_C_E_grasp` |
| `T_B_C_template` | 模板拍摄时基座→相机 | `camera_pose.json` → `T_B_E_camera @ T_E_C`，缺省时用 `T_B_C` |
| `T_B_C` | **当前**拍摄时基座→相机 | `_load_camera_pose`，**当前恒为 `np.eye(4)`** |
| `target_feature` | 当前检测到的工件特征 | `workpiece_center` (u,v)、`workpiece_radius` 等 |
| `dtheta` | 绕基座 Z 轴的角度差 (rad) | 暴力匹配 `best_angle_deg` 或 `target_angle - template_angle` |
| `depth_image` | 深度图 | 请求传入，可选 |
| `camera_matrix` | 相机内参 fx, fy, cx, cy | 配置 |

### 2.2 步骤 1：模板抓取深度

- `z_template_camera = T_C_E_grasp_template[2, 3]`（相机系 Z）
- 若 `z_template_camera <= 0`：退化为直接用模板位姿转基座，不进行目标相对变换。

### 2.3 步骤 2：模板抓取转基座

- `T_B_E_grasp_template = T_B_C_template @ T_C_E_grasp_template`
- `template_grasp_pos = T_B_E_grasp_template[:3, 3]`
- 模板抓取**位置、姿态**均在基座系。

### 2.4 步骤 3：模板工件中心（基座系）

- 像素：`(template_center_u, template_center_v) = best_template.feature.workpiece_center`
- 相机系：  
  `x = (u - cx) * z_template / fx`，  
  `y = (v - cy) * z_template / fy`，  
  `z = z_template`  
  其中 `z_template = z_template_camera`（即抓取深度）。
- `template_center_base = T_B_C_template @ [x,y,z,1]`，取平移部分 `template_center_base_pos`。

### 2.5 步骤 4：模板抓取相对模板中心的偏移（基座系）

- `offset_template_base = template_grasp_pos - template_center_base_pos`
- XY 偏移：`template_grasp_offset_xy = [offset[0], offset[1], 0]`（Z 置 0）

### 2.6 步骤 5：目标工件中心（基座系）

- 像素：`(target_center_u, target_center_v) = target_feature.workpiece_center`
- 深度：优先从 `depth_image` 在 `(u,v)` 邻域取深度；失败则用 `z_template_camera`。
- 相机系：  
  `x = (u - cx) * z_target / fx`，  
  `y = (v - cy) * z_target / fy`，  
  `z = z_target`
- **`target_center_base = T_B_C @ target_center_camera`**  
  当前实现中 **`T_B_C = I`**，故 `target_center_base = target_center_camera`（目标中心被当作在“基座”系，实际未做相机→基座变换）。

### 2.7 步骤 6：目标抓取位置与姿态

- 绕基座 Z 轴旋转矩阵：`R_z_base = Rz(dtheta)`（标准 2D 旋转）。
- 中间偏移：  
  `intermediate_offset = template_grasp_offset_xy`  
  然后 `rotated_offset = R_z_base @ intermediate_offset`。
- 目标抓取位置：  
  `target_grasp_pos_base = target_center_base + rotated_offset`，  
  再 **强制 `target_grasp_pos_base[2] = template_grasp_pos[2]`**（Z 始终用模板抓取高度）。
- 目标抓取旋转：  
  `R_target_grasp = R_z_base @ R_template_grasp`，  
  其中 `R_template_grasp = T_B_E_grasp_template[:3,:3]`。
- 最终：  
  `T_B_E_grasp_current[:3,:3] = R_target_grasp`，  
  `T_B_E_grasp_current[:3,3] = target_grasp_pos_base`，  
  即 `result.T_B_E_grasp`。

### 2.8 小结（抓取）

- 逻辑：用模板“抓取相对工件中心的 XY 偏移”+ 模板 Z，经 `dtheta` 旋转后平移到**当前目标中心**，得到目标抓取位姿。
- 前提：**目标中心**、**模板中心**均在**同一坐标系**（基座）下表示。当前 **`T_B_C = I`** 破坏了这一点（见下文）。

---

## 3. 准备姿态 (T_B_E_prep) 计算步骤

**代码位置**: 同上，约 1512–1552 行。

### 3.1 输入

- 与抓取共用 `T_B_C`、`T_B_C_template`、`target_feature`、`dtheta`、`R_z_base`、`target_center_base_pos` 等。
- 额外：`best_template.T_C_E_prep`（模板准备位姿，相机→末端）。

### 3.2 步骤

- `T_B_E_prep_template = T_B_C_template @ T_C_E_prep`  
  `prep_pos_template = T_B_E_prep_template[:3, 3]`
- `offset_prep_base = prep_pos_template - template_center_base_pos`（**含 Z**）
- `intermediate_prep_pos = target_center_base + offset_prep_base`  
  `intermediate_prep_offset = intermediate_prep_pos - target_center_base = offset_prep_base`  
  `rotated_prep_offset = R_z_base @ intermediate_prep_offset`
- `target_prep_pos_base = target_center_base + rotated_prep_offset`  
  再 **`target_prep_pos_base[2] = prep_pos_template[2]`**（Z 用模板准备高度）。
- `R_target_prep = R_z_base @ R_template_prep`，  
  `T_B_E_prep_current` 由旋转 + 平移构成，即 `result.T_B_E_prep`。

### 3.3 小结（准备）

- 与抓取类似：用模板“准备相对模板中心的偏移”（含 Z，但最终 Z 被覆盖为模板值），绕 Z 旋转 `dtheta` 后平移到目标中心。
- 同样依赖 **目标中心** 在 **基座系** 下的正确表示。

---

## 4. 潜在问题与检查结论

### 4.1 【严重】当前 T_B_C 恒为单位阵

**位置**: `ros2_communication.py` → `_load_camera_pose`。

```python
T_B_C = np.eye(4)  # 当前使用单位矩阵作为相机位姿（未来可从机器人获取）
```

- **影响**：  
  `target_center_base = T_B_C @ target_center_camera` 实际等于 `target_center_camera`。  
  即目标工件中心**未被变换到基座**，而是直接当作基座系坐标使用。
- **后果**：  
  模板相关量（中心、抓取、准备）均在**真实基座系**（由 `T_B_C_template` 定义）；目标中心却在**相机系**。两者混用，导致抓取/准备位姿在基座系下**系统性错误**。
- **正确做法**：  
  眼在手上时，应用 **当前** 机器人位姿 + 手眼标定得到 **当前** `T_B_C = T_B_E_current @ T_E_C`，再对 `target_center_camera` 做变换。  
- **建议**：  
  从机器人/控制器获取当前 `T_B_E`（或等效的相机位姿），在 `_load_camera_pose` 中填充 `T_B_C`，不再使用单位阵。

### 4.2 目标中心深度回退到模板深度

- 深度图在 `(u,v)` 邻域无有效深度时，使用 `z_template_camera`。  
- 若目标高度/倾斜与模板差异大，会带来误差；属设计上的回退策略，需知悉。

### 4.3 抓取 / 准备 Z 完全沿用模板

- 抓取：`target_grasp_pos_base[2] = template_grasp_pos[2]`  
- 准备：`target_prep_pos_base[2] = prep_pos_template[2]`  
- 即 **高度** 不随当前场景变化。若工作面或工件高度与模板不一致，可能不合适。  
- 若希望随深度或工作面变化，需额外设计（例如用目标中心 Z、或固定平面约束等）。

### 4.4 角度差 dtheta 的两种来源

- **暴力匹配**：  
  `dtheta_rad = np.deg2rad(best_angle_deg)`，`best_angle_deg` 为匹配得到的最优角度。  
  若启用暴力匹配，则使用该值。
- **特征角度差**：  
  `dtheta = target_feature.standardized_angle - template_angle`。  
  若未启用暴力匹配或未提供 `dtheta_rad`，则用此式。  
- 两者需与模板匹配策略一致，避免混用导致朝向偏差。

### 4.5 四元数取反与笛卡尔输出

- `_convert_transform_to_cartesian_position` 中，对旋转矩阵得到的四元数 **整体取反** 再写入 `CartesianPosition.orientation`，与 C++ 对齐。
- `q` 与 `-q` 表示同一旋转，但若下游（如 MoveIt、机械臂接口）对四元数符号有约定，需保持一致。

### 4.6 抓取/准备 关节角为 0

- 估计结果仅输出笛卡尔位姿（位置 + 四元数 / RPY），**不做逆解**。  
- 故 `joint_position_deg` / `joint_position_rad` 对抓取、准备**恒为 0**，属当前设计；preplace/place 若来自模板 JSON，可保留模板关节角（见前序修改）。

---

## 5. 公式速查

- **模板抓取转基座**：  
  `T_B_E_grasp_template = T_B_C_template @ T_C_E_grasp_template`
- **模板工件中心（基座）**：  
  `p_cam = [(u-cx)*z/fx, (v-cy)*z/fy, z]` → `p_base = T_B_C_template @ p_cam`
- **目标工件中心（基座）**：  
  `p_cam` 同理，深度可用深度图或模板 → **`p_base = T_B_C @ p_cam`**（当前 `T_B_C=I` 即未变换）
- **抓取 XY**：  
  `offset_xy = (template_grasp - template_center)_xy`，  
  `target_grasp_xy = target_center_xy + Rz(dtheta) @ offset_xy`  
  **抓取 Z**：`template_grasp_z`
- **准备**：  
  同思路，偏移用 `prep - template_center`，旋转后加至 `target_center`，**Z 用 `prep_pos_template[2]`**

---

## 6. 建议修改优先级

1. **高**：  
   实现并接入 **当前** `T_B_C`（如从机器人读取 `T_B_E`，再 `T_B_C = T_B_E @ T_E_C`），替换 `_load_camera_pose` 中的单位阵。  
   否则准备/抓取位姿在基座系下始终存在坐标系错误。
2. **中**：  
   评估抓取/准备 Z 是否必须随场景变化；若需要，再设计基于深度或工作面的 Z 修正。
3. **低**：  
   统一并显式约定 dtheta 来源（暴力匹配 vs 特征角）、四元数符号与下游接口的一致性。

---

*文档版本：基于当前 `pose_estimator` 与 `ros2_communication` 实现整理。*
