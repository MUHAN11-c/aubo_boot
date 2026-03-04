# OpenCV模式自动手眼标定 - 详细执行流程文档

## 目录
1. [概述](#概述)
2. [前端执行流程](#前端执行流程)
3. [后端API处理流程](#后端api处理流程)
4. [OpenCV标定模块内部流程](#opencv标定模块内部流程)
5. [数据格式和单位转换](#数据格式和单位转换)
6. [函数调用顺序图](#函数调用顺序图)
7. [关键时间点和延迟](#关键时间点和延迟)

---

## 概述

OpenCV模式自动手眼标定采用**姿态法（AX=XB约束）**，使用OpenCV的`calibrateHandEye`函数（TSAI算法）进行标定计算。

**核心特点：**
- 前端：JavaScript控制机器人运动和图像采集
- 后端：Python Flask API处理请求，调用OpenCV标定模块
- 算法：OpenCV `calibrateHandEye` (TSAI方法)
- 误差验证：AX=XB约束验证

---

## 前端执行流程

### 1. 入口函数：`handleAutoCalibStart()`
**文件：** `script_v2_auto_calib_addon.js:1197`

**执行步骤：**

```javascript
1. 检查状态
   - 验证是否已在运行
   - 检查记录的位姿数量（至少3个）
   - 检查相机参数是否已加载

2. 初始化状态
   - autoCalibState.isRunning = true
   - autoCalibState.currentStep = 0
   - autoCalibState.totalSteps = recordedPoses.length
   - autoCalibState.collectedCalibrationData = []

3. 清空collect_data目录
   - POST /api/hand_eye/clear_collect_data

4. 循环处理每个位姿（for循环）
   - 对每个recordedPose执行以下步骤
```

### 2. 位姿处理循环：`handleAutoCalibStart()` 内部循环
**文件：** `script_v2_auto_calib_addon.js:1257-1401`

**对每个位姿执行：**

#### 步骤1：移动机器人到目标位姿
```javascript
// 调用函数：moveRobotToPose(targetPose, false, 0.2, 0.1)
// 文件：script_v2_auto_calib_addon.js:950
1. POST /api/robot/move_to_pose
   - 参数：targetPose (position + orientation)
   - 等待机器人到达目标位姿

2. OpenCV模式特殊延迟（仅OpenCV模式）
   - 位姿到位后等待3秒
   - 代码：await sleep(3000)
   - 目的：确保机器人稳定
```

#### 步骤2：采集图像和角点
```javascript
// 调用函数：captureImageAndExtractCorners(poseIndex)
// 文件：script_v2_auto_calib_addon.js:1015

2.1 触发相机拍照
   - POST /api/camera/capture
   - 等待200ms

2.2 获取图像数据
   - GET /api/current_image
   - 轮询最多5次，每次间隔200ms
   - 显示图像到UI

2.3 提取标定板位姿
   - POST /api/camera/get_board_pose
   - 参数：{square_size: squareSize}
   - 返回：board_pose (position, orientation, rvec, tvec, corners等)
   - 显示带角点的图像

2.4 获取机器人当前位姿
   - GET /api/robot_status
   - 提取cartesian_position

2.5 返回采集的数据结构
   {
     pose_index: poseIndex,
     robot_pose: {position, orientation},
     board_pose: {position, orientation, rvec, tvec, ...},
     corners: [...],
     corners_3d: [...],
     corners_3d_filtered: [...]
   }
```

#### 步骤3：保存图像和位姿
```javascript
// 文件：script_v2_auto_calib_addon.js:1337-1358
- POST /api/hand_eye/save_image_and_pose
  - 参数：
    {
      pose_index: i,  // 从0开始
      robot_pose: calibrationData.robot_pose,
      square_size: squareSize
    }
  - 后端保存到collect_data/pose_{i}/目录
```

#### 步骤4：保存到内存
```javascript
// 文件：script_v2_auto_calib_addon.js:1367
- autoCalibState.collectedCalibrationData.push(calibrationData)
- 更新UI显示
```

#### OpenCV模式特殊延迟（仅OpenCV模式）
```javascript
// 文件：script_v2_auto_calib_addon.js:1391-1394
- 数据采集完成后等待2秒
- 代码：await sleep(2000)
- 目的：确保数据稳定后再移动到下一个位姿
```

### 3. 标定计算：`performAutoCalibrationFromCollectedData()`
**文件：** `script_v2_auto_calib_addon.js:1458`

**执行步骤：**

```javascript
1. 数据格式转换
   - 将collectedCalibrationData转换为poses_list格式
   - 每个元素包含robot_pose和board_pose

2. 构建请求数据
   {
     calibration_type: 'eye-in-hand',
     calibration_method: 'pose-based',
     method: 'opencv',  // 或 'custom'
     poses_list: posesList
   }

3. 发送标定请求
   - POST /api/hand_eye/calibrate
   - 等待后端处理

4. 处理返回结果
   - 显示标定结果
   - 显示误差统计
   - 更新UI
```

---

## 后端API处理流程

### 1. API路由：`/api/hand_eye/calibrate`
**文件：** `hand_eye_calibration_node.py:1532`

**处理流程：**

```python
1. 接收请求数据
   - 解析JSON：calibration_type, calibration_method, method, poses_list

2. 验证数据
   - 检查calibration_type是否为'eye-in-hand'
   - 检查calibration_method是否为'pose-based'
   - 检查poses_list长度（至少3个）

3. 检查相机参数
   - 验证camera_matrix和dist_coeffs是否已加载

4. 根据method选择标定方法
   - if method == 'opencv':
      调用OpenCV模式标定
   - else:
      调用Custom模式标定
```

### 2. OpenCV模式标定调用
**文件：** `hand_eye_calibration_node.py:1599-1643`

```python
1. 创建OpenCV标定对象
   opencv_calib = OpenCVHandEyeCalibration(logger=self.get_logger())

2. 调用标定方法
   result = opencv_calib.calibrate(input_data)
   # input_data = poses_list

3. 提取结果
   - T_camera2gripper: 4x4变换矩阵（单位：毫米）
   - translation_errors: 平移误差列表
   - rotation_errors: 旋转误差列表
   - error_statistics: 误差统计信息

4. 返回JSON响应
   {
     success: true,
     T_camera2gripper: [...],
     translation_errors: [...],
     rotation_errors: [...],
     error_statistics: {...},
     ...
   }
```

### 3. 其他相关API

#### `/api/hand_eye/clear_collect_data`
**文件：** `hand_eye_calibration_node.py:5260`
- 清空collect_data目录

#### `/api/hand_eye/save_image_and_pose`
**文件：** `hand_eye_calibration_node.py:5300`
- 保存图像和位姿到collect_data/pose_{index}/目录
- 保存文件：
  - image.jpg（灰度图）
  - rgb.jpg（RGB备份）
  - robot_pose.json
  - board_pose.json
  - corners.csv
  - image_with_features.jpg

#### `/api/camera/get_board_pose`
**文件：** `hand_eye_calibration_node.py:5170`
- 检测标定板角点
- 使用solvePnP估计标定板位姿
- 返回rvec, tvec, position, orientation等

---

## OpenCV标定模块内部流程

### 1. 主入口：`calibrate(poses_data)`
**文件：** `opencv_hand_eye_calibration.py:1372`

**执行流程：**

```python
def calibrate(self, poses_data):
    """
    完整标定流程
    """
    # 步骤0：Z值一致性验证（可选）
    - 验证机器人Z值与标定板Z值的物理一致性
    
    # 步骤1：数据准备
    R_gripper2base, t_gripper2base, rvecs, tvecs, T_gripper_list, T_board_list = \
        self.prepare_data(poses_data)
    
    # 步骤2：算法计算
    T_camera2gripper = self.solve_hand_eye(
        R_gripper2base, t_gripper2base, rvecs, tvecs
    )
    
    # 步骤3：误差计算
    translation_errors, rotation_errors, error_statistics = \
        self.calculate_errors(T_gripper_list, T_board_list, T_camera2gripper)
    
    # 返回结果
    return {
        'T_camera2gripper': T_camera2gripper,
        'translation_errors': translation_errors,
        'rotation_errors': rotation_errors,
        'error_statistics': error_statistics,
        'T_gripper_list': T_gripper_list,
        'T_board_list': T_board_list,
        ...
    }
```

### 2. 数据准备：`prepare_data(poses_data)`
**文件：** `opencv_hand_eye_calibration.py:129`

**详细流程：**

```python
def prepare_data(self, poses_data):
    """
    准备OpenCV格式的数据
    """
    # 1. 初始化列表
    T_gripper_list = []  # Base->Gripper（单位：毫米）
    T_board_list = []    # Board->Camera（单位：毫米）
    board_rvec_tvec_map = {}  # 保存原始rvec/tvec
    seen_poses = set()   # 用于去重
    
    # 2. 遍历每个位姿数据
    for idx, pose_data in enumerate(poses_data):
        # 2.1 提取机器人位姿
        robot_pose_data = pose_data['robot_pose']
        robot_pos_m = {
            'x': robot_pose_data['robot_pos_x'],  # 单位：米
            'y': robot_pose_data['robot_pos_y'],
            'z': robot_pose_data['robot_pos_z']
        }
        robot_ori = {
            'x': robot_pose_data['robot_ori_x'],
            'y': robot_pose_data['robot_ori_y'],
            'z': robot_pose_data['robot_ori_z'],
            'w': robot_pose_data['robot_ori_w']
        }
        
        # 2.2 构建T_gripper（Base->Gripper，单位：毫米）
        T_gripper = self._pose_to_transform_matrix(
            {'x': robot_pos_m['x'] * 1000.0,  # 米→毫米
             'y': robot_pos_m['y'] * 1000.0,
             'z': robot_pos_m['z'] * 1000.0},
            robot_ori
        )
        
        # 2.3 提取标定板位姿
        board_pose_data = pose_data['board_pose']
        board_pos_mm = {
            'x': board_pose_data['position']['x'],  # 单位：毫米
            'y': board_pose_data['position']['y'],
            'z': board_pose_data['position']['z']
        }
        board_ori = board_pose_data['orientation']
        
        # 2.4 构建T_board（Board->Camera，单位：毫米）
        T_board = self._pose_to_transform_matrix(
            board_pos_mm, board_ori
        )
        
        # 2.5 保存原始rvec/tvec（如果存在）
        if 'rvec' in board_pose_data and 'tvec' in board_pose_data:
            board_rvec_tvec_map[pose_key(T_board)] = {
                'rvec': board_pose_data['rvec'],
                'tvec': board_pose_data['tvec']  # 单位：毫米
            }
        
        # 2.6 去重处理（精度：0.1mm）
        key_combined = (pose_key(T_gripper), pose_key(T_board))
        if key_combined not in seen_poses:
            seen_poses.add(key_combined)
            T_gripper_list.append(T_gripper)
            T_board_list.append(T_board)
    
    # 3. 提取数据用于OpenCV
    R_gripper2base_list = []  # 实际存储Base->Gripper
    t_gripper2base_list = []  # 实际存储Base->Gripper（单位：毫米）
    rvecs_list = []
    tvecs_list = []
    
    for T_gripper, T_board in zip(T_gripper_list, T_board_list):
        # 3.1 提取机器人旋转和平移（Base->Gripper）
        R_base2gripper = T_gripper[:3, :3]
        t_base2gripper_mm = T_gripper[:3, 3]  # 单位：毫米
        
        R_gripper2base_list.append(R_base2gripper)
        t_gripper2base_list.append(t_base2gripper_mm)
        
        # 3.2 提取标定板rvec和tvec
        # 优先使用保存的原始数据
        board_key = pose_key(T_board)
        if board_key in board_rvec_tvec_map:
            rvec_data = board_rvec_tvec_map[board_key]['rvec']
            tvec_data = board_rvec_tvec_map[board_key]['tvec']
            rvec = np.array([[rvec_data['x']], [rvec_data['y']], [rvec_data['z']]])
            tvec = np.array([[tvec_data['x']], [tvec_data['y']], [tvec_data['z']]])
        else:
            # 从T_board转换
            R_board = T_board[:3, :3]
            rvec, _ = cv2.Rodrigues(R_board)
            tvec = T_board[:3, 3].reshape(3, 1)
        
        rvecs_list.append(rvec)
        tvecs_list.append(tvec)  # 单位：毫米
    
    # 4. 单位转换：毫米 → 米（OpenCV要求）
    R_gripper2base = np.array(R_gripper2base_list, dtype=np.float64)
    t_gripper2base = [t.reshape(3, 1) / 1000.0 for t in t_gripper2base_list]
    rvecs = rvecs_list  # 弧度，无需转换
    tvecs = [t / 1000.0 for t in tvecs_list]  # 毫米 → 米
    
    # 5. 返回数据
    return R_gripper2base, t_gripper2base, rvecs, tvecs, T_gripper_list, T_board_list
```

### 3. 算法计算：`solve_hand_eye(R_gripper2base, t_gripper2base, rvecs, tvecs)`
**文件：** `opencv_hand_eye_calibration.py:920`

**详细流程：**

```python
def solve_hand_eye(self, R_gripper2base, t_gripper2base, rvecs, tvecs):
    """
    使用OpenCV calibrateHandEye进行标定计算
    """
    # 1. 调用OpenCV函数
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,  # (N, 3, 3) numpy数组，Base->Gripper旋转矩阵
        t_gripper2base,  # 列表，每个元素(3,1)数组，Base->Gripper平移（单位：米）
        rvecs,          # 列表，每个元素(3,1)数组，Board->Camera旋转向量（弧度）
        tvecs,          # 列表，每个元素(3,1)数组，Board->Camera平移（单位：米）
        method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    # 2. 构建变换矩阵（单位：毫米）
    T_camera2gripper = np.eye(4)
    T_camera2gripper[:3, :3] = R_cam2gripper
    T_camera2gripper[:3, 3] = t_cam2gripper.flatten() * 1000.0  # 米 → 毫米
    
    # 3. 验证旋转矩阵
    det_R = np.linalg.det(R_cam2gripper)
    # 应该接近1.0（正交矩阵）
    
    # 4. 返回结果
    return T_camera2gripper
```

### 4. 误差计算：`calculate_errors(T_gripper_list, T_board_list, T_camera2gripper)`
**文件：** `opencv_hand_eye_calibration.py:1055`

**详细流程：**

```python
def calculate_errors(self, T_gripper_list, T_board_list, T_camera2gripper):
    """
    计算AX=XB约束的误差
    """
    # 1. 初始化列表
    A_list_for_error = []  # 末端运动前后的相对变换（A矩阵）
    B_list_for_error = []  # 相机观测标定板的相对变换（B矩阵）
    
    # 2. 构建相邻姿态对
    num_motions = len(T_gripper_list) - 1
    for i in range(num_motions):
        # 2.1 计算A矩阵：Gripper1->Gripper2
        T_base2gripper1 = T_gripper_list[i]  # Base->Gripper（单位：毫米）
        T_base2gripper2 = T_gripper_list[i + 1]
        A = np.linalg.inv(T_base2gripper1) @ T_base2gripper2
        A_list_for_error.append(A)
        
        # 2.2 计算B矩阵：Board1->Board2（通过Camera）
        T_board1 = T_board_list[i]  # Board->Camera（单位：毫米）
        T_board2 = T_board_list[i + 1]
        T_board2_inv = np.linalg.inv(T_board2)
        B = T_board1 @ T_board2_inv
        B_list_for_error.append(B)
    
    # 3. 验证AX=XB约束
    translation_errors = []
    rotation_errors = []
    
    for i in range(len(A_list_for_error)):
        A = A_list_for_error[i]
        B = B_list_for_error[i]
        X = T_camera2gripper  # 单位：毫米
        
        # 3.1 计算AX和XB
        AX = A @ X
        XB = X @ B
        
        # 3.2 计算误差矩阵
        error_matrix = AX - XB
        
        # 3.3 平移误差（单位：毫米）
        translation_error = float(np.linalg.norm(error_matrix[:3, 3]))
        translation_errors.append(translation_error)
        
        # 3.4 旋转误差（单位：弧度）
        R_AX = AX[:3, :3]
        R_XB = XB[:3, :3]
        R_diff = R_AX @ R_XB.T
        trace = np.trace(R_diff)
        cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        rotation_error_angle = float(np.arccos(cos_angle))
        rotation_errors.append(rotation_error_angle)
    
    # 4. 计算统计信息
    mean_translation_error = float(np.sqrt(np.mean([e**2 for e in translation_errors])))
    mean_rotation_error = float(np.sqrt(np.mean([e**2 for e in rotation_errors])))
    max_translation_error = float(np.max(translation_errors))
    min_translation_error = float(np.min(translation_errors))
    std_translation_error = float(np.std(translation_errors))
    max_rotation_error = float(np.max(rotation_errors))
    min_rotation_error = float(np.min(rotation_errors))
    
    error_statistics = {
        'mean_translation_error': mean_translation_error,
        'max_translation_error': max_translation_error,
        'min_translation_error': min_translation_error,
        'std_translation_error': std_translation_error,
        'mean_rotation_error_rad': mean_rotation_error,
        'mean_rotation_error_deg': float(np.degrees(mean_rotation_error)),
        'max_rotation_error_rad': max_rotation_error,
        'max_rotation_error_deg': float(np.degrees(max_rotation_error)),
        'min_rotation_error_rad': min_rotation_error,
        'min_rotation_error_deg': float(np.degrees(min_rotation_error)),
        'translation_errors': translation_errors,
        'rotation_errors_rad': rotation_errors,
        'rotation_errors_deg': [float(np.degrees(e)) for e in rotation_errors]
    }
    
    # 5. 返回结果
    return translation_errors, rotation_errors, error_statistics
```

---

## 数据格式和单位转换

### 1. 前端数据格式

#### 采集的数据结构（`collectedCalibrationData`）
```javascript
{
  pose_index: 1,
  robot_pose: {
    position: {x, y, z},  // 单位：米
    orientation: {x, y, z, w}  // 四元数
  },
  board_pose: {
    position: {x, y, z},  // 单位：毫米
    orientation: {x, y, z, w},  // 四元数
    rvec: {x, y, z},  // 旋转向量（弧度）
    tvec: {x, y, z}   // 平移向量（单位：毫米）
  },
  corners: [...],
  corners_3d: [...],
  corners_3d_filtered: [...]
}
```

#### 提交给后端的格式（`poses_list`）
```javascript
[
  {
    robot_pose: {
      robot_pos_x: 0.5,  // 单位：米
      robot_pos_y: 0.3,
      robot_pos_z: 0.4,
      robot_ori_x: 0.0,
      robot_ori_y: 0.0,
      robot_ori_z: 0.0,
      robot_ori_w: 1.0
    },
    board_pose: {
      position: {x: 100, y: 200, z: 500},  // 单位：毫米
      orientation: {x, y, z, w},
      rvec: {x, y, z},  // 弧度
      tvec: {x, y, z}   // 单位：毫米
    }
  },
  ...
]
```

### 2. 后端数据格式

#### OpenCV标定模块输入（`poses_data`）
```python
[
  {
    'robot_pose': {
      'robot_pos_x': 0.5,  # 单位：米
      'robot_pos_y': 0.3,
      'robot_pos_z': 0.4,
      'robot_ori_x': 0.0,
      'robot_ori_y': 0.0,
      'robot_ori_z': 0.0,
      'robot_ori_w': 1.0
    },
    'board_pose': {
      'position': {'x': 100, 'y': 200, 'z': 500},  # 单位：毫米
      'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
      'rvec': {'x': 0.1, 'y': 0.2, 'z': 0.3},  # 弧度
      'tvec': {'x': 100, 'y': 200, 'z': 500}   # 单位：毫米
    }
  },
  ...
]
```

### 3. 单位转换流程

```
前端 → 后端API → OpenCV模块 → OpenCV函数 → 结果返回

机器人位姿：
  米 (前端) → 米 (API) → 毫米 (prepare_data内部) → 米 (OpenCV) → 毫米 (结果)

标定板位姿：
  毫米 (前端) → 毫米 (API) → 毫米 (prepare_data内部) → 米 (OpenCV) → 毫米 (结果)
```

**详细转换点：**

1. **prepare_data内部**：
   - 机器人位姿：米 → 毫米（构建T_gripper）
   - 标定板位姿：保持毫米（构建T_board）
   - 提取数据时：毫米 → 米（传递给OpenCV）

2. **solve_hand_eye**：
   - 输入：米（OpenCV要求）
   - 输出：米（OpenCV返回）
   - 转换为毫米（构建T_camera2gripper）

3. **calculate_errors**：
   - 输入：毫米（T_gripper_list, T_board_list, T_camera2gripper）
   - 输出：毫米（平移误差），弧度（旋转误差）

---

## 函数调用顺序图

```
前端 (JavaScript)
│
├─ handleAutoCalibStart()
│  │
│  ├─ clear_collect_data (API)
│  │
│  └─ for each recordedPose:
│     │
│     ├─ moveRobotToPose()
│     │  └─ POST /api/robot/move_to_pose
│     │
│     ├─ sleep(3000) [仅OpenCV模式]
│     │
│     ├─ captureImageAndExtractCorners()
│     │  ├─ POST /api/camera/capture
│     │  ├─ GET /api/current_image
│     │  ├─ POST /api/camera/get_board_pose
│     │  └─ GET /api/robot_status
│     │
│     ├─ POST /api/hand_eye/save_image_and_pose
│     │
│     ├─ collectedCalibrationData.push()
│     │
│     └─ sleep(2000) [仅OpenCV模式]
│
└─ performAutoCalibrationFromCollectedData()
   │
   └─ POST /api/hand_eye/calibrate
      │
      └─ 后端 (Python)
         │
         └─ _perform_eye_in_hand_pose_based_calibration()
            │
            └─ OpenCVHandEyeCalibration.calibrate()
               │
               ├─ prepare_data()
               │  ├─ 构建T_gripper (Base→Gripper, 毫米)
               │  ├─ 构建T_board (Board→Camera, 毫米)
               │  ├─ 去重处理
               │  ├─ 提取rvec/tvec
               │  └─ 单位转换：毫米→米
               │
               ├─ solve_hand_eye()
               │  ├─ cv2.calibrateHandEye()
               │  │  └─ method=cv2.CALIB_HAND_EYE_TSAI
               │  └─ 构建T_camera2gripper (毫米)
               │
               └─ calculate_errors()
                  ├─ 构建A矩阵 (Gripper1→Gripper2)
                  ├─ 构建B矩阵 (Board1→Board2)
                  ├─ 验证AX=XB约束
                  ├─ 计算平移误差 (毫米)
                  ├─ 计算旋转误差 (弧度)
                  └─ 计算统计信息
```

---

## 关键时间点和延迟

### OpenCV模式特殊延迟

1. **位姿到位后延迟：3秒**
   - 位置：`script_v2_auto_calib_addon.js:1322`
   - 条件：`autoCalibState.calibrationAlgorithm === 'opencv'`
   - 目的：确保机器人稳定

2. **数据采集后延迟：2秒**
   - 位置：`script_v2_auto_calib_addon.js:1392`
   - 条件：`autoCalibState.calibrationAlgorithm === 'opencv'`
   - 目的：确保数据稳定后再移动到下一个位姿

### 其他时间延迟

1. **拍照后等待：200ms**
   - 位置：`script_v2_auto_calib_addon.js:1034`

2. **图像获取轮询：200ms间隔，最多5次**
   - 位置：`script_v2_auto_calib_addon.js:1039`

3. **相机参数加载等待：1000ms**
   - 位置：`script_v2_auto_calib_addon.js:1213`

---

## 关键文件清单

### 前端文件
- `aubo_ros2_ws/src/hand_eye_calibration/web/static/script_v2_auto_calib_addon.js`
  - `handleAutoCalibStart()`: 主入口
  - `moveRobotToPose()`: 移动机器人
  - `captureImageAndExtractCorners()`: 采集图像和角点
  - `performAutoCalibrationFromCollectedData()`: 执行标定计算

### 后端文件
- `aubo_ros2_ws/src/hand_eye_calibration/hand_eye_calibration/hand_eye_calibration_node.py`
  - `_perform_eye_in_hand_pose_based_calibration()`: API处理函数
  - `/api/hand_eye/calibrate`: 标定API路由
  - `/api/hand_eye/save_image_and_pose`: 保存数据API
  - `/api/camera/get_board_pose`: 获取标定板位姿API

- `aubo_ros2_ws/src/hand_eye_calibration/hand_eye_calibration/opencv_hand_eye_calibration.py`
  - `OpenCVHandEyeCalibration.calibrate()`: 主入口
  - `prepare_data()`: 数据准备
  - `solve_hand_eye()`: 算法计算
  - `calculate_errors()`: 误差计算

---

## 总结

OpenCV模式自动手眼标定的完整流程包括：

1. **前端控制**：JavaScript控制机器人运动和图像采集
2. **数据采集**：每个位姿采集机器人位姿和标定板位姿
3. **数据保存**：保存到collect_data目录和内存
4. **标定计算**：后端调用OpenCV标定模块
5. **误差验证**：使用AX=XB约束验证标定结果

**关键特点：**
- OpenCV模式有特殊的延迟要求（位姿到位后3秒，采集后2秒）
- 数据单位在多个环节进行转换（米↔毫米）
- 使用OpenCV的TSAI算法进行标定计算
- 误差计算使用AX=XB约束验证

---

**文档版本：** 1.0  
**最后更新：** 2025-01-XX  
**维护者：** 手眼标定系统开发团队
