# 函数调用检查报告

检查范围：`visual_pose_estimation_python` 包内主要 Python 文件的函数定义与调用关系。

---

## 1. http_bridge_server.py

### 1.1 顶层函数
| 函数 | 调用处 | 状态 |
|------|--------|------|
| `quaternion_to_euler_rpy` | estimate_pose 内多处（约 234、244、552 行） | ✅ 已调用 |
| `get_templates_dir` | handle_* 内多处（1164、1300、1565、1714、1955、2191 行） | ✅ 已调用 |
| `start_server` | `if __name__ == "__main__"` | ✅ 已调用 |

### 1.2 ROS2Node 方法
| 方法 | 调用处 | 状态 |
|------|--------|------|
| `capture_image` | handle_capture_image, handle_capture_template_image, handle_debug_capture | ✅ 已调用 |
| `estimate_pose` | handle_estimate_pose | ✅ 已调用 |
| `move_to_pose` | handle_execute_pose_sequence, handle_set_robot_pose | ✅ 已调用 |
| `set_robot_io` | handle_set_robot_io | ✅ 已调用 |
| `standardize_template` | handle_standardize_template | ✅ 已调用 |
| `write_plc_register` | handle_write_plc_register | ✅ 已调用 |
| `get_robot_status` | handle_get_robot_status, handle_execute_pose_sequence | ✅ 已调用 |
| `notify_params_updated` | handle_debug_update_params, handle_debug_save_thresholds | ✅ 已调用 |
| `list_templates` | handle_list_templates（先调 ROS2 服务再按工件扫 pose） | ✅ 已调用（符合 ROS2） |
| `_cleanup_temp_subscriptions` | capture_image 内多处 | ✅ 已调用 |

### 1.3 AlgorithmHandler 方法
| 方法 | 调用处 | 状态 |
|------|--------|------|
| `end_headers` | 各 handle_* 内（继承重写后由 self.end_headers() 调用） | ✅ 已调用 |
| `do_OPTIONS` | HTTP 框架 | ✅ 已调用 |
| `do_GET` | HTTP 框架 | ✅ 已调用 |
| `do_POST` | HTTP 框架，内部分发到各 handle_* | ✅ 已调用 |
| 各 `handle_*` | 均由 do_GET/do_POST 按 path 分发调用 | ✅ 已调用 |
| `save_geometric_features` | **无** | ❌ 未调用（已移除该死代码） |

---

## 2. ros2_communication.py

### 2.1 服务处理
| 方法 | 调用处 | 状态 |
|------|--------|------|
| `_handle_estimate_pose` | create_service 注册 | ✅ 已调用 |
| `_handle_list_templates` | create_service 注册 | ✅ 已调用 |
| `_handle_standardize_template` | create_service 注册 | ✅ 已调用 |
| `_handle_update_params` | create_service 注册 | ✅ 已调用 |
| ~~process_debug_images~~ | **无** | 已移除 |

### 2.2 核心模块调用链
- `PoseEstimator`: load_template_library, select_best_template, estimate_pose 等均由 _handle_estimate_pose → _process_single_feature 等调用 ✅
- `TemplateStandardizer`: standardize_template, save_standardized_template, compute_standardized_pose, draw_gripper 等均由 _handle_standardize_template 及相关逻辑调用 ✅

---

## 3. 核心包 (visual_pose_estimation_python/*.py)

### 3.1 main.py
- `main` 为 entry_point ✅；`_delayed_initialize` 由定时器调用 ✅

### 3.2 pose_estimator.py
| 方法 | 调用处 | 状态 |
|------|--------|------|
| load_template_library, select_best_template, estimate_pose | ros2_communication | ✅ 已调用 |
| _load_pose_json, _calculate_feature_distance, calculate_mask_iou | 本文件内（load_template_library、select_best_template、_process_template_for_matching） | ✅ 已调用 |
| _brute_force_template_matching, _process_template_for_matching | select_best_template 分支 | ✅ 已调用 |
| _get_depth_from_image | estimate_pose 内 | ✅ 已调用 |
| ~~find_best_mask_alignment~~ | 已移除 | 逻辑已内联到 _process_template_for_matching |
| ~~compute_2d_alignment~~ | 已移除 | 无调用 |

### 3.3 template_standardizer.py
| 方法 | 调用处 | 状态 |
|------|--------|------|
| standardize_template, save_standardized_template, compute_standardized_pose, draw_gripper, _build_feature_params_from_feature | ros2_communication | ✅ 已调用 |
| ~~list_template_poses~~ | 已移除 | 无调用 |
| ~~load_standardized_template~~ | 已移除 | pose_estimator 直接读 JSON |

### 3.4 preprocessor.py / feature_extractor.py / debug_visualizer.py
- 主要方法均被 pose_estimator、template_standardizer、ros2_communication 或 http_bridge_server 调用 ✅

### 3.5 config_reader.py
| 方法 | 调用处 | 状态 |
|------|--------|------|
| get_section, load_debug_thresholds | ros2_communication | ✅ 已调用 |
| ~~set~~ | 已移除 | 无调用 |

### 3.6 params_manager.py (web_ui/scripts)
| 方法 | 调用处 | 状态 |
|------|--------|------|
| load, update, save, config_path | http_bridge_server | ✅ 已调用 |
| ~~get_shared_config_path, get_root_dir, get_config_dir~~ | 已移除 | 静态方法无调用 |

---

## 4. 已删除的未使用项

| 文件 | 已删除函数/方法 |
|------|-----------------|
| http_bridge_server.py | save_geometric_features |
| ros2_communication.py | process_debug_images |
| pose_estimator.py | find_best_mask_alignment, compute_2d_alignment |
| template_standardizer.py | list_template_poses, load_standardized_template |
| config_reader.py | set |
| params_manager.py | get_shared_config_path, get_root_dir, get_config_dir（静态方法） |

**说明**：handle_list_templates 已改为先调用 ROS2 `/list_templates` 服务（符合 ROS2）。
