# API 调用流程文档

本文档描述前端 (`web_ui/`) → FastAPI Web Server → ROS2 节点的完整调用链路，包含每个 API 调用的 ROS2 服务/话题、请求参数、响应结构。

---

## 调用关系总图

```mermaid
graph TB
    subgraph Browser["浏览器 (app.js)"]
        F1["captureImage()"]
        F2["estimatePose()"]
        F3["estimatePose2D()"]
        F4["autoGrasp()"]
        F5["loopAutoGrasp()"]
        F6["openGripper() / closeGripper()"]
        F7["executeRobotPose()"]
        F8["getRobotPose()"]
        F9["standardizeTemplate()"]
        F10["refreshTemplateList()"]
        F11["debugCaptureImage()"]
        F12["debugRefreshImages()"]
        F13["toggleLoopAutoGrasp()"]
    end

    subgraph FastAPI["FastAPI Web Server (port 8088)"]
        subgraph Routers["routers/"]
            R1["camera.py<br>/api/capture_image<br>/api/capture_template_image"]
            R2["pose.py<br>/api/estimate_pose<br>/api/estimate_pose_2d"]
            R3["robot.py<br>/api/get_robot_status<br>/api/set_robot_pose<br>/api/set_robot_io<br>/api/execute_pose_sequence"]
            R4["grasp.py<br>/api/execute_single_grasp<br>/api/loop_grasp_control<br>/api/publish_grasps_loop_control<br>/api/run_gripper_swap"]
            R5["templates.py<br>/api/list_templates<br>/api/standardize_template<br>/api/read_template_pose<br>/api/save_template_pose"]
            R6["debug.py<br>/api/debug/capture<br>/api/debug/get_images<br>/api/debug/update_params<br>/api/debug/get_params<br>/api/debug/save_thresholds"]
            R7["system.py<br>GET /health /status<br>POST /exit<br>WS /ws"]
        end
        SVC["NativeWebService<br>(services/native_api.py)"]
    end

    subgraph ROS2Node["ROS2Node (node_runtime.py)<br>algorithm_http_server_node"]
        subgraph Persist["持久订阅 (后台持续)"]
            PS1["/aubo_driver/robot_status<br>→ latest_robot_status"]
            PS2["/image_data<br>→ latest_image_data"]
        end
        subgraph Clients["ROS2 客户端 (按需调用)"]
            CL1["capture_image()"]
            CL2["estimate_pose()"]
            CL3["estimate_pose_2d()"]
            CL4["list_templates()"]
            CL5["standardize_template()"]
            CL6["move_to_pose()"]
            CL7["set_robot_io()"]
            CL8["call_execute_single_grasp()"]
            CL9["call_loop_grasp_control()"]
            CL10["call_publish_grasps_loop_control()"]
            CL11["call_run_gripper_swap()"]
            CL12["notify_params_updated()"]
        end
        Local["本地算法模块<br>Preprocessor<br>FeatureExtractor<br>ConfigReader<br>ParamsManager"]
    end

    subgraph SpinThread["后台 spin 线程 (manager.py)"]
        SPIN["rclpy.spin_once(node, 0.1s)<br>处理回调 + 服务响应"]
    end

    subgraph ROS2Services["ROS2 服务端"]
        subgraph VPE["visual_pose_estimation_python 节点"]
            S1["/estimate_pose<br>(EstimatePose)"]
            S2["/estimate_pose_2d<br>(EstimatePose2D)"]
            S3["/list_templates<br>(ListTemplates)"]
            S4["/standardize_template<br>(StandardizeTemplate)"]
            S5["/update_params<br>(UpdateParams)"]
        end
        subgraph Aubo["aubo_driver 节点"]
            S6["/move_to_pose<br>(MoveToPose)"]
            S7["/aubo_driver/set_io<br>(SetRobotIO)"]
            T1["/aubo_driver/robot_status<br>(RobotStatus) 话题"]
        end
        subgraph Demo["demo_driver 节点"]
            S8["/execute_single_grasp<br>(ExecuteGraspPose)"]
            S9["/loop_grasp_control<br>(SetBool)"]
            S10["/publish_grasps_worker_loop_control<br>(SetBool)"]
            S11["/run_gripper_swap<br>(RunGripperSwap)"]
        end
        subgraph Camera["camera_control 节点"]
            S12["/software_trigger<br>(SoftwareTrigger)"]
            T2["/camera/depth/image_raw<br>(Image) 话题"]
            T3["/camera/color/image_raw<br>(Image) 话题"]
            T4["/image_data<br>(ImageData) 话题"]
        end
    end

    F1 & F2 & F3 & F4 & F5 & F6 & F7 & F8 & F9 & F10 & F11 & F12 & F13 --> Routers
    Routers --> SVC
    SVC --> Clients
    SVC --> Persist
    SVC --> Local

    CL1 --> S12
    CL1 -.临时订阅.-> T2
    CL1 -.临时订阅.-> T3
    CL2 --> S1
    CL3 --> S2
    CL4 --> S3
    CL5 --> S4
    CL6 --> S6
    CL7 --> S7
    CL8 --> S8
    CL9 --> S9
    CL10 --> S10
    CL11 --> S11
    CL12 --> S5

    Clients -.call_async + _spin_future.-> SPIN
    Spin -.event.set().-> Clients

    PS1 -.订阅.-> T1
    PS2 -.订阅.-> T4

    S1 --> VPE_ALGO["Preprocessor → FeatureExtractor<br>→ PoseEstimator<br>→ 3D 位姿"]
    S2 --> VPE_ALGO2["读缓存深度图<br>→ Preprocessor → FeatureExtractor<br>→ 2D 结果"]
    S4 --> VPE_ALGO3["Preprocessor → FeatureExtractor<br>→ TemplateStandardizer"]
```

---

## 调度机制

### threading.Event + 后台 spin 线程

```mermaid
sequenceDiagram
    participant FT as FastAPI 线程
    participant F as ROS2 Future
    participant E as threading.Event
    participant ST as 后台 spin 线程
    participant RS as ROS2 服务端

    FT->>F: call_async(request)
    FT->>F: add_done_callback(fn)
    FT->>E: event.wait(timeout)
    Note over FT: 阻塞于此，不干扰 ROS2

    loop spin_once(0.1s)
        ST->>ST: rclpy.spin_once(node)
        ST->>RS: 处理服务响应
    end

    RS-->>ST: 响应到达
    ST->>F: 触发 done callback
    F->>E: event.set()
    E-->>FT: 解除阻塞
    FT->>F: future.result()
    FT-->>FT: return response
```

---

## 按功能分组的调用关系图

### 1. 图像采集链

```mermaid
sequenceDiagram
    participant FE as app.js<br>captureImage()
    participant SVC as NativeWebService<br>capture_image()
    participant N as ROS2Node<br>capture_image()
    participant SW as /software_trigger
    participant DT as /camera/depth/image_raw
    participant CT as /camera/color/image_raw

    FE->>SVC: POST /api/capture_image {camera_id}
    SVC->>N: capture_image(camera_id, timeout=10s)

    N->>DT: create_subscription (临时)
    N->>CT: create_subscription (临时)
    Note over N: sleep(0.2s) 等待订阅建立

    N->>SW: call_async({camera_id})
    Note over N: _spin_future(future, 10s)
    SW-->>N: {success, message}

    alt 失败且 message 含 "Expected: camera"
        N->>SW: call_async({camera_id: "camera"})
        SW-->>N: {success, message}
    end

    loop 轮询 (最多 timeout 秒)
        N->>N: 检查 depth_image_received && color_image_received
    end

    DT-->>N: depth Image → depth_image_callback → latest_depth_image
    CT-->>N: color Image → color_image_callback → latest_color_image

    N->>N: destroy_subscription ×2 (清理)
    N-->>SVC: (depth_image, color_image, None)
    SVC->>SVC: cv2.imencode → base64
    SVC-->>FE: {success, depth_image_base64, color_image_base64}
```

### 2. 姿态估计链 (3D)

```mermaid
sequenceDiagram
    participant FE as app.js<br>estimatePose()
    participant SVC as NativeWebService<br>estimate_pose()
    participant N as ROS2Node<br>estimate_pose()
    participant S as /estimate_pose<br>(VPE 节点)

    FE->>SVC: POST /api/estimate_pose {depth_image, color_image, object_id}
    SVC->>SVC: split base64 prefix
    SVC->>N: estimate_pose(depth_b64, color_b64, object_id)

    N->>N: base64.b64decode → cv2.imdecode → cv_bridge → ROS Image
    N->>S: call_async({image: depth, color_image: color, object_id})
    Note over N: _spin_future(30s)

    rect rgb(240, 248, 255)
        Note over S: _handle_estimate_pose()
        S->>S: 1. _get_images_from_request() → cv_bridge 解码
        S->>S: 2. _validate_images() → 尺寸检查
        S->>S: 3. _preprocess_images()
        Note over S: Preprocessor.preprocess(depth,color,min,max)<br>→ components[], preprocessed_color<br>[可选 rembg]
        S->>S: 4. _extract_features()
        Note over S: FeatureExtractor.extract_features()<br>→ ComponentFeature[]
        S->>S: 5. _ensure_template_library_loaded(object_id)
        Note over S: PoseEstimator.load_template_library()<br>加载 templates/{id}/pose_*/
        loop 每个 feature
            S->>S: 6. _process_single_feature()
            Note over S: .select_best_template()<br>.estimate_pose()<br>→ T_B_E_grasp/prep/preplace/place
        end
        S->>S: 7. _fill_estimate_pose_response()
    end

    S-->>N: {success_num, confidence[], position[],<br>grab_position[], preparation_position[],<br>preplace_position[], place_position[],<br>matched_pose_ids[], pose_image[], processing_time_sec}

    N->>N: convert_cartesian_position ×N
    N->>N: vis_image → cv2.imencode → base64
    N-->>SVC: (result, None)
    SVC-->>FE: JSON {success, success_num, confidence[], ...}

    FE->>FE: updateResultsList(data)<br>updateResultCount(successNum)<br>workflowState.lastEstimateResult = data
```

### 3. 2D 姿态估计链 (新增)

```mermaid
sequenceDiagram
    participant FE as app.js<br>estimatePose2D()
    participant SVC as NativeWebService<br>estimate_pose_2d()
    participant N as ROS2Node<br>estimate_pose_2d()
    participant S as /estimate_pose_2d<br>(VPE 节点)
    participant CACHE as 持久订阅缓存<br>/camera/depth/image_raw

    FE->>SVC: POST /api/estimate_pose_2d {color_image, object_id}
    SVC->>SVC: split base64 prefix
    SVC->>N: estimate_pose_2d(color_b64, object_id)

    N->>N: base64.b64decode → cv2.imdecode → cv_bridge → ROS Image
    N->>S: call_async({image: color, object_id})
    Note over N: _spin_future(30s)

    rect rgb(240, 248, 255)
        Note over S: _handle_estimate_pose_2d()
        S->>S: 1. cv_bridge 解码 RGB
        S->>CACHE: 2. 读取 current_depth_image
        CACHE-->>S: depth_image (或 None)
        opt 缓存为空
            S->>S: _capture_images_on_trigger(10s)
        end
        S->>S: 3. cv2.resize 尺寸对齐
        S->>S: 4. Preprocessor.preprocess() → components[]
        S->>S: 5. FeatureExtractor.extract_features() → ComponentFeature[]
        Note over S: 无模板匹配，无手眼标定
        S->>S: 6. 填充响应<br>center_x = wp_center[0]<br>center_y = wp_center[1]<br>rotation_angle = standardized_angle_deg<br>confidence = 启发式
    end

    S-->>N: {success_num, center_x[], center_y[],<br>rotation_angle[], confidence[],<br>vis_image, message}

    N->>N: vis_image → cv2.imencode → base64
    N-->>SVC: (result, None)
    SVC-->>FE: JSON {success, success_num, center_x[], ...}

    FE->>FE: 显示 vis_image 到 main-image<br>updateResultsList2D(data)<br>addLogEntry 逐目标输出
```

### 4. 机器人控制链

```mermaid
sequenceDiagram
    participant FE as app.js
    participant SVC as NativeWebService
    participant N as ROS2Node
    participant RS as aubo_driver 节点

    Note over FE,RS: ─── 获取状态 ───
    FE->>SVC: POST /api/get_robot_status
    SVC->>N: get_robot_status()
    N->>N: 读缓存 latest_robot_status<br>(来自 /aubo_driver/robot_status 持久订阅)
    N-->>SVC: {is_online, enable, in_motion, joint_position_deg[6], cartesian_position}
    SVC-->>FE: {success, robot_status}

    Note over FE,RS: ─── 移动到位姿 ───
    FE->>SVC: POST /api/set_robot_pose {target_pose, use_joints, velocity_factor, acceleration_factor}
    SVC->>SVC: 验证格式 (数组≥6 或 dict含position+orientation)
    SVC->>N: move_to_pose(target_pose, use_joints, velocity_factor, acceleration_factor, timeout=180s)
    N->>RS: call_async({target_pose, use_joints, velocity_factor, acceleration_factor})
    Note over N: _spin_future(180s)
    RS-->>N: {success, error_code, message}
    N-->>SVC: (result, error)
    SVC-->>FE: {success, error_code, message}

    Note over FE,RS: ─── 设置 IO ───
    FE->>SVC: POST /api/set_robot_io {io_type, io_index, value}
    SVC->>N: set_robot_io(io_type, io_index, value, timeout=10s)
    N->>RS: call_async({io_type, io_index, value})
    RS-->>N: {success, error_code, message}
    N-->>SVC: (result, error)
    SVC-->>FE: {success, error_code, message}

    Note over FE,RS: ─── 姿态序列 ───
    FE->>SVC: POST /api/execute_pose_sequence {folder_name}
    SVC->>SVC: 读取 pose_list/{folder_name}/*.json<br>检查是否已在最后点位
    SVC->>SVC: 启动 daemon 线程
    SVC-->>FE: {success, skipped, message}
    loop 每个姿态文件 (后台线程)
        SVC->>N: move_to_pose(joints, use_joints=True)
        N->>RS: /move_to_pose
        SVC->>N: get_robot_status() 轮询 in_motion
    end
```

### 5. 抓取操作链

```mermaid
sequenceDiagram
    participant FE as app.js
    participant SVC as NativeWebService
    participant N as ROS2Node
    participant DD as demo_driver 节点

    Note over FE,DD: ─── 单次抓取 ───
    FE->>SVC: POST /api/execute_single_grasp {object_id, use_visual_estimation}
    SVC->>N: call_execute_single_grasp(object_id, use_visual_estimation, timeout=300s)
    N->>DD: call_async({object_id, use_visual_estimation})

    rect rgb(255, 248, 240)
        Note over DD: runOneCycle() 内部串行
        DD->>DD: /software_trigger (拍照)
        DD->>DD: /estimate_pose (视觉估计)
        DD->>DD: /move_to_pose (准备位姿)
        DD->>DD: /move_to_pose (抓取位姿)
        DD->>DD: /aubo_driver/set_io (关闭夹爪)
        DD->>DD: /move_to_pose (提起)
        DD->>DD: /move_to_pose (放置位姿)
        DD->>DD: /aubo_driver/set_io (打开夹爪)
    end

    DD-->>N: {success, message, final_position, final_orientation}
    N-->>SVC: result
    SVC-->>FE: {success, message, final_position, final_orientation}

    Note over FE,DD: ─── 循环控制 ───
    FE->>SVC: POST /api/loop_grasp_control {data: true/false}
    SVC->>N: call_loop_grasp_control(start)
    N->>DD: /loop_grasp_control (SetBool)
    DD-->>FE: {success, message}

    Note over FE,DD: ─── 夹爪切换 ───
    FE->>SVC: POST /api/run_gripper_swap {direction}
    SVC->>N: call_run_gripper_swap(direction)
    N->>DD: /run_gripper_swap (RunGripperSwap)
    DD-->>FE: {success, message, direction}

    Note over FE,DD: ─── 循环自动抓取 ───

    FE->>FE: ensureVisualGraspMode()
    FE->>N: /loop_grasp_control {data: false}
    FE->>N: /publish_grasps_loop_control {data: false}
    loop while(!stopFlag)
        FE->>FE: captureImageAsync() → /capture_image
        FE->>FE: await 1000ms
        FE->>FE: estimatePoseAsync() → /estimate_pose
        alt success_num > 0
            FE->>FE: callExecuteSingleGrasp() → /execute_single_grasp
        end
    end
```

### 6. 模板管理链

```mermaid
sequenceDiagram
    participant FE as app.js
    participant SVC as NativeWebService
    participant N as ROS2Node
    participant VPE as VPE 节点
    participant FS as 文件系统

    Note over FE,FS: ─── 列出模板 ───
    FE->>SVC: POST /api/list_templates {workpiece_id}
    SVC->>N: list_templates(workpiece_id, timeout=10s)
    N->>VPE: /list_templates {workpiece_id}
    VPE-->>N: {success, template_ids[], workpiece_ids[]}
    loop 每个 template
        SVC->>FS: 读取 templates/{id}/pose_{pose}/image.jpg
        FS-->>SVC: 缩略图 → base64
    end
    SVC-->>FE: {success, templates: [{id, pose_id, image_base64}], count}
    FE->>FE: 按 pose_id 数字排序
    loop 每个 template (异步)
        FE->>SVC: POST /api/read_template_pose ×4<br>(preparation, grab, preplace, place)
        SVC->>FS: 读取 {pose_type}.json
        FS-->>FE: pose_data
    end

    Note over FE,FS: ─── 标准化模板 ───
    FE->>SVC: POST /api/standardize_template {workpiece_id}
    SVC->>N: standardize_template(workpiece_id, timeout=120s)
    N->>VPE: /standardize_template {workpiece_id}
    rect rgb(240, 248, 255)
        Note over VPE: 遍历 templates/{id}/pose_*/
        VPE->>VPE: Preprocessor.preprocess()
        VPE->>VPE: FeatureExtractor.extract_features()
        VPE->>VPE: TemplateStandardizer<br>.standardize() .compute_pose()<br>.draw_gripper() .save()
    end
    VPE-->>N: {success, processed_count, skipped_count, processed_pose_ids[]}
    N-->>FE: {success, processed_count, skipped_count}
    FE->>FE: refreshTemplateList()

    Note over FE,FS: ─── 保存模板姿态 ───
    FE->>SVC: POST /api/get_robot_status
    SVC->>N: get_robot_status() → 读缓存
    N-->>FE: robot_status
    FE->>SVC: POST /api/save_template_pose {workpiece_id, pose_id, pose_type, robot_status}
    SVC->>SVC: normalize_pose_rotation() + 可选固定 orientation
    SVC->>FS: 写入 templates/{id}/pose_{id}/{pose_type}.json
    SVC-->>FE: {success, file_path}
```

### 7. 调试链

```mermaid
sequenceDiagram
    participant FE as app.js
    participant SVC as NativeWebService
    participant N as ROS2Node
    participant FS as 文件系统
    participant VPE as VPE 节点

    Note over FE,VPE: ─── 加载参数 ───
    FE->>SVC: POST /api/debug/get_params
    SVC->>N: config_reader.load_debug_thresholds()
    N->>FS: 读取 configs/debug_thresholds.json
    FS-->>FE: {params: {binary_threshold_min, ...}}
    FE->>FE: 更新 12 个 slider 值和显示

    Note over FE,VPE: ─── 自动刷新 (500ms 间隔) ───
    loop setInterval 500ms
        FE->>SVC: POST /api/debug/get_images
        SVC->>N: 读取缓存 depth_image + color_image
        Note over N: 本地算法 (不走 ROS2!)
        N->>N: Preprocessor.set_parameters(params)
        N->>N: FeatureExtractor.set_parameters(params)
        N->>N: Preprocessor.preprocess() → components[]
        N->>N: FeatureExtractor.extract_features() → features[]
        opt use_rembg
            N->>N: rembg_processor.process_roi()
        end
        N->>N: DebugVisualizer.create_debug_panel()<br>→ depth/color/binary/preprocessed 4路图像
        N->>N: cv2.imencode → base64 ×4
        N-->>FE: {depth_image, color_image, binary_image, preprocessed_image,<br>stats: {component_count, feature_count}, features[]}
        FE->>FE: updateDebugImage ×4
    end

    Note over FE,VPE: ─── 更新参数 ───
    FE->>SVC: POST /api/debug/update_params {param_name, param_value}
    SVC->>SVC: param_name 映射: 连字符→下划线
    SVC->>N: params_manager.update(key, value)
    N->>FS: 写入 debug_thresholds.json
    N->>VPE: /update_params {section: "all", params_json: ""}
    Note over VPE: 通知算法节点重新加载参数
    SVC-->>FE: {success}

    Note over FE,VPE: ─── 保存阈值 ───
    FE->>SVC: POST /api/debug/save_thresholds
    SVC->>N: params_manager.save()
    SVC->>N: config_reader.load_debug_thresholds()
    SVC->>N: notify_params_updated() → /update_params
    SVC-->>FE: {success, message}
```

### 8. 系统链

```mermaid
sequenceDiagram
    participant FE as app.js (浏览器)
    participant WS as WebSocket /ws
    participant SVC as FastAPI
    participant RB as RosBridgeManager

    Note over FE,RB: ─── WebSocket 生命周期 ───
    FE->>WS: connect
    WS->>WS: ws_manager.connect(ws)
    WS-->>FE: {type: "status", payload: {service, ros_bridge}}
    loop 保持连接
        FE->>WS: {action: "ping"}
        WS-->>FE: {type: "pong"}
        FE->>WS: {action: "status"}
        WS-->>FE: {type: "status", payload: {ros_bridge}}
    end
    FE->>WS: disconnect
    WS->>WS: ws_manager.disconnect(ws)

    Note over FE,RB: ─── 健康检查 / 状态 ───
    FE->>SVC: GET /health
    SVC->>RB: ros_bridge.status()
    RB-->>FE: {ok, ros_bridge_ready, startup_error}

    FE->>SVC: GET /status
    SVC->>RB: ros_bridge.status()
    RB-->>FE: {status, port, service, timestamp, ros_bridge}

    Note over FE,RB: ─── 退出 ───
    FE->>SVC: POST /exit
    SVC->>SVC: schedule_exit(1.0s)
    Note over SVC: 1秒后 os.kill(SIGTERM)
    SVC-->>FE: {status: "success", message: "服务正在退出..."}

    Note over FE,RB: ─── 选项卡切换 ───
    FE->>FE: switchTab('workflow')
    FE->>SVC: /api/list_templates + /api/list_workpiece_ids
    FE->>FE: switchTab('template')
    FE->>SVC: /api/list_workpiece_ids
    FE->>FE: switchTab('debug')
    FE->>SVC: /api/debug/get_params
    FE->>FE: startDebugAutoRefresh() → setInterval(500ms)
```

---

## 完整 API 清单

### 相机

| 方法 | 端点 | 超时 | 调用的 ROS2 服务/话题 | 说明 |
|------|------|------|----------------------|------|
| POST | `/api/capture_image` | 20s | `/software_trigger` + 临时订阅 `/camera/depth/image_raw`, `/camera/color/image_raw` | 触发拍照，返回深度图+彩色图 base64 |
| POST | `/api/capture_template_image` | 20s | 同上 | 拍照并保存为模板原始图像 |

### 姿态估计

| 方法 | 端点 | 超时 | 调用的 ROS2 服务/话题 | 说明 |
|------|------|------|----------------------|------|
| POST | `/api/estimate_pose` | 30s | `/estimate_pose` | 3D 姿态估计，含模板匹配和手眼标定 |
| POST | `/api/estimate_pose_2d` | 30s | `/estimate_pose_2d` + 读缓存 `/camera/depth/image_raw` | 2D 姿态估计，仅像素坐标和旋转角度 |

### 机器人控制

| 方法 | 端点 | 超时 | 调用的 ROS2 服务/话题 | 说明 |
|------|------|------|----------------------|------|
| POST | `/api/set_robot_pose` | 180s | `/move_to_pose` | 机械臂运动到指定位姿 |
| POST | `/api/set_robot_io` | 10s | `/aubo_driver/set_io` | 设置数字/模拟 IO 输出 |
| POST | `/api/get_robot_status` | 立即 | 读缓存 `/aubo_driver/robot_status` | 获取当前机器人状态 |
| POST | `/api/execute_pose_sequence` | 300s | `/move_to_pose` ×N + 轮询 `/aubo_driver/robot_status` | 执行位姿序列（后台线程） |

### 抓取操作

| 方法 | 端点 | 超时 | 调用的 ROS2 服务/话题 | 说明 |
|------|------|------|----------------------|------|
| POST | `/api/execute_single_grasp` | 300s | `/execute_single_grasp` (worker 内部串行调用全部) | 单次完整抓取流程 |
| POST | `/api/loop_grasp_control` | 10s | `/loop_grasp_control` (SetBool) | 启停循环抓取 |
| POST | `/api/publish_grasps_loop_control` | 10s | `/publish_grasps_worker_loop_control` (SetBool) | 启停 GraspNet 循环 |
| POST | `/api/run_gripper_swap` | 可配置 | `/run_gripper_swap` | 快换操作 |

### 模板管理

| 方法 | 端点 | 超时 | 调用的 ROS2 服务/话题 | 说明 |
|------|------|------|----------------------|------|
| POST | `/api/list_templates` | 5s | `/list_templates` + 文件读取缩略图 | 列出模板 |
| POST | `/api/list_workpiece_ids` | 5s | 无 (纯文件系统) | 列出工件 ID |
| POST | `/api/standardize_template` | 60s | `/standardize_template` | 模板标准化 |
| POST | `/api/read_template_pose` | 5s | 无 (纯文件读取) | 读取模板位姿 |
| POST | `/api/save_template_pose` | 5s | 无 (纯文件写入) | 保存模板位姿 |
| GET | `/api/get_template_image` | 立即 | 无 (纯静态文件) | 下载模板图像 |

### 调试

| 方法 | 端点 | 超时 | 调用的 ROS2 服务/话题 | 说明 |
|------|------|------|----------------------|------|
| POST | `/api/debug/capture` | 20s | `/software_trigger` + 临时订阅 | 调试拍照 |
| POST | `/api/debug/get_images` | 5s | 无 (本地 Preprocessor/FeatureExtractor/DebugVisualizer) | 获取 4 路调试图像 |
| POST | `/api/debug/update_params` | 5s | `/update_params` (通知) + 文件写入 | 更新参数 |
| POST | `/api/debug/get_params` | 5s | 无 (纯文件读取) | 获取参数 |
| POST | `/api/debug/save_thresholds` | 5s | `/update_params` (通知) + 文件写入 | 保存阈值 |
| POST | `/api/save_debug_features` | 5s | 无 (纯文件写入 CSV) | 保存调试特征 |

### 系统

| 方法 | 端点 | 说明 |
|------|------|------|
| GET | `/health` | 健康检查 |
| GET | `/status` | 系统状态 |
| POST | `/exit` | 关闭服务 (SIGTERM) |
| WS | `/ws` | WebSocket 状态推送 |

---

## API 参数速查

### POST `/api/set_robot_pose`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `target_pose` | `float[6]` 或 `float[7]` 或 `{position, orientation}` | 必填 | `[x,y,z, qx,qy,qz(,qw)]` 或 `{position: {x,y,z}, orientation: {x,y,z,w}}` |
| `use_joints` | bool | false | true=关节空间, false=笛卡尔空间 |
| `velocity_factor` | float64 | 0.08 | 速度缩放 (0-1) |
| `acceleration_factor` | float64 | 0.05 | 加速度缩放 (0-1) |
| `timeout_sec` | float64 | 180.0 | 超时秒数 (限制 5-600) |

### POST `/api/set_robot_io`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `io_type` | string | 必填 | `"digital_output"` 等 |
| `io_index` | int32 | 必填 | 夹爪: `6` |
| `value` | float64 | 必填 | 打开: `0.0`, 关闭: `1.0` |

### POST `/api/estimate_pose`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `depth_image` | string (base64) | 必填 | 深度图 (PNG) |
| `color_image` | string (base64) | 必填 | 彩色图 (JPEG) |
| `object_id` | string | 必填 | 工件ID |

### POST `/api/estimate_pose_2d`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `color_image` | string (base64) | 必填 | 彩色RGB图 (JPEG) |
| `object_id` | string | 必填 | 工件ID |

### POST `/api/run_gripper_swap`

| direction | 含义 |
|-----------|------|
| `"gripper0_to_gripper2"` | 完整快换: 视觉夹爪0 → AI夹爪2 |
| `"gripper2_to_gripper0"` | 完整快换: AI夹爪2 → 视觉夹爪0 |
| `"gripper2"` | 简程: 当前位置 → AI夹爪2 |

### 调试参数 (debug_thresholds.json)

| 参数名 | 前端 slider ID | 默认值 | 范围 | 说明 |
|--------|---------------|--------|------|------|
| `binary_threshold_min` | `min-depth-slider` | 0 | 0-65535 | 深度二值化下限 |
| `binary_threshold_max` | `max-depth-slider` | 2149 | 0-65535 | 深度二值化上限 |
| `component_min_area` | `contour-min-area-slider` | 10 | 1-100000 | 连通域最小面积 |
| `component_max_area` | `contour-max-area-slider` | 100000 | 1-1000000 | 连通域最大面积 |
| `component_min_aspect_ratio` | `min-aspect-slider` | 0.3 | 0.1-10.0 | 最小宽高比 (前端值÷10) |
| `component_max_aspect_ratio` | `max-aspect-slider` | 4.0 | 0.1-10.0 | 最大宽高比 (前端值÷10) |
| `component_min_width` | `min-width-slider` | 60 | 1-500 | 最小宽度 (像素) |
| `component_min_height` | `min-height-slider` | 60 | 1-500 | 最小高度 (像素) |
| `component_max_count` | `max-count-slider` | 3 | 1-20 | 最大连通域数量 |
| `enable_smooth_edges` | `enable-smooth-edges-slider` | True | 0/1 | 启用边缘平滑 |
| `smooth_edges_blur_sigma` | `smooth-edges-blur-sigma-slider` | 0 | 0-30 | 平滑高斯σ (前端值÷10, 0=关闭) |
| `use_rembg` | `use-rembg-slider` | False | 0/1 | 启用背景去除 |
| `enable_zero_interp` | (无前端 slider) | True | bool | 启用零值插值 |

---

## 消息结构速查

### EstimatePose.Response (3D)

```
success_num: int32
confidence: float64[]
position: geometry_msgs/Point[]         ← 图像坐标系中心点
grab_position: CartesianPosition[]       ← 抓取位姿
preparation_position: CartesianPosition[] ← 准备位姿
preplace_position: CartesianPosition[]  ← 预放置位姿
place_position: CartesianPosition[]     ← 放置位姿
matched_pose_ids: string[]              ← 匹配的模板 pose_id
pose_image: sensor_msgs/Image[]         ← 可视化图像
processing_time_sec: float32
```

### EstimatePose2D.Response (2D)

```
success_num: int32
center_x: float32[]        ← 目标中心 X 像素坐标
center_y: float32[]        ← 目标中心 Y 像素坐标
rotation_angle: float32[]  ← 旋转角度（度）
confidence: float32[]      ← 置信度（0-1.0）
vis_image: sensor_msgs/Image
message: string
```

### CartesianPosition

```
position: {x, y, z}
orientation: {x, y, z, w}
euler_orientation_rpy_rad: float64[3]
euler_orientation_rpy_deg: float64[3]
joint_position_rad: float64[6]
joint_position_deg: float64[6]
```

### RobotStatus (来自 `/aubo_driver/robot_status`)

```
header: {stamp, frame_id}
is_online: bool
enable: bool
in_motion: bool
joint_position_deg: float64[6]
joint_position_rad: float64[6]
cartesian_position: CartesianPosition
```

---

## 并发安全

| 组件 | 保护机制 | 锁粒度 |
|------|---------|--------|
| 图像缓存 | `threading.Lock` (image_lock) | latest_depth_image, latest_color_image, image_received |
| 机器人状态 | `threading.Lock` (robot_status_lock) | latest_robot_status |
| ROS2 Future 等待 | `threading.Event` (回调驱动) | 每个 Future 独立 Event |
| 前端机器人移动请求 | `_setRobotPoseInFlight` + `_moveToPoseLock` | 同一时刻只允许一个 /api/set_robot_pose |
| 前端夹爪切换 | `_switchFlowInProgress` | 同一时刻只允许一个切换流程 |
| 前端循环抓取 | `_loopAutoGraspRunning` + `_loopAutoGraspStopFlag` | 唯一循环实例 |
| WebSocket 广播 | `asyncio.Lock` | 广播期间保护连接集合 |

### 已知限制

- 长时操作（move_to_pose 180s、execute_single_grasp 300s）期间，FastAPI 线程被 `Event.wait()` 阻塞，但不干扰 ROS2 调度。
- `POST /exit` 在长时操作进行中可能延迟响应，直到当前 Future 完成或超时。
- 建议实际部署时使用 uvicorn `--workers 2` 提高并发处理能力。
- `/api/debug/get_images` 在 HTTP 线程内直接调用 Preprocessor/FeatureExtractor (同步 OpenCV)，避免在 debug 自动刷新期间同时执行 `/api/estimate_pose`。
- 前端 `captureImage()` 默认 camera_id="207000152740"，若相机驱动期望 "camera" 会自动重试，增加约 10s 延迟。


