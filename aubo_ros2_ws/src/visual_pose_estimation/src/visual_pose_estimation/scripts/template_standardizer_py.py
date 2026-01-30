#!/usr/bin/env python3
"""
模板标准化器（Python版本）
复刻 template_standardizer.cpp 的功能，使用 trigger_depth.py 中的已有函数

功能：
1. 标准化模板图像（裁剪、标准化）
2. 提取工件特征（工件外接圆、阀体外接圆）
3. 计算标准化后的抓取姿态
4. 保存标准化结果

使用方法：
    python3 template_standardizer_py.py
"""

import cv2
import numpy as np
import json
import yaml
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import sys
import os

# 导入 trigger_depth.py 中的 ImageProcessor 类
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trigger_depth import ImageProcessor

# 尝试导入 PreprocessorDebugger（如果可用）
try:
    from debug_preprocess_feature import PreprocessorDebugger
    HAS_PREPROCESSOR = True
except ImportError:
    HAS_PREPROCESSOR = False
    PreprocessorDebugger = None


class TemplateStandardizer:
    """模板标准化器：将模板图像标准化并计算标准化后的抓取姿态"""
    
    def __init__(self, logger=None):
        """
        初始化模板标准化器
        
        参数:
            logger: 日志记录器（可选）
        """
        self.logger = logger
        # 创建 ImageProcessor 实例以复用其方法
        self.image_processor = ImageProcessor(
            camera_name='template_standardizer',
            depth_scale=0.25,
            logger=logger,
            enable_depth_processing=False  # 不需要深度图处理
        )
        # 创建预处理器（如果可用）
        self.preprocessor = None
        if HAS_PREPROCESSOR:
            try:
                # 使用默认配置创建预处理器
                self.preprocessor = PreprocessorDebugger({})
            except Exception as e:
                self._log('warn', f'无法创建预处理器，将使用简化预处理: {e}')
                self.preprocessor = None
    
    def _log(self, level, message):
        """记录日志"""
        if self.logger:
            if level == 'info':
                self.logger.info(message)
            elif level == 'warn':
                self.logger.warn(message)
            elif level == 'error':
                self.logger.error(message)
            elif level == 'debug':
                self.logger.debug(message)
        else:
            print(f'[{level.upper()}] {message}')
    
    def standardize_template(self, template_root: str, workpiece_id: str, 
                           calib_file: str = "", debug: bool = False) -> List[Dict]:
        """
        标准化模板：处理指定工件的所有姿态模板
        
        参数:
            template_root: 模板根目录
            workpiece_id: 工件ID
            calib_file: 标定文件路径（可选）
            debug: 是否启用调试模式
        
        返回:
            标准化结果列表，每个元素包含 success, pose_id, error_message
        """
        results = []
        workpiece_dir = Path(template_root) / workpiece_id
        
        if not workpiece_dir.exists():
            results.append({
                'success': False,
                'pose_id': '',
                'error_message': f'工件文件夹不存在: {workpiece_dir}'
            })
            return results
        
        # 遍历所有pose_*目录
        for entry in workpiece_dir.iterdir():
            if not entry.is_dir():
                continue
            
            pose_dir_name = entry.name
            if not pose_dir_name.startswith('pose_'):
                continue
            
            pose_id = pose_dir_name[5:]  # 去掉"pose_"前缀
            pose_dir = entry
            
            # 标准化单个姿态
            result = self.standardize_pose(pose_dir, pose_id, calib_file, debug)
            results.append(result)
        
        return results
    
    def standardize_pose(self, pose_dir: Path, pose_id: str, 
                        calib_file: str = "", debug: bool = False) -> Dict:
        """
        标准化单个姿态：处理单个pose目录的模板
        
        参数:
            pose_dir: 姿态目录路径
            pose_id: 姿态ID
            calib_file: 标定文件路径（可选）
            debug: 是否启用调试模式
        
        返回:
            标准化结果字典，包含 success, pose_id, error_message
        """
        result = {
            'success': False,
            'pose_id': pose_id,
            'error_message': ''
        }
        
        try:
            # 1. 检查必要文件
            original_image_path = pose_dir / "original_image.jpg"
            grab_position_path = pose_dir / "grab_position.json"
            
            if not original_image_path.exists():
                result['error_message'] = "缺少original_image.jpg"
                return result
            
            if not grab_position_path.exists():
                result['error_message'] = "缺少grab_position.json"
                return result
            
            # 2. 读取原始图像
            original_image = cv2.imread(str(original_image_path), cv2.IMREAD_COLOR)
            if original_image is None or original_image.size == 0:
                result['error_message'] = "无法读取原始图像"
                return result
            
            # 创建debug目录（如果启用debug模式）
            debug_dir = None
            if debug:
                debug_dir = pose_dir / "debug"
                debug_dir.mkdir(exist_ok=True)
                cv2.imwrite(str(debug_dir / "00_original_image.jpg"), original_image)
            
            # 3. 加载手眼标定参数
            calibration_path = None
            if calib_file and Path(calib_file).exists():
                calibration_path = Path(calib_file)
            else:
                calibration_path = pose_dir.parent / "hand_eye_calibration.xml"
            
            if not calibration_path.exists():
                result['error_message'] = "缺少hand_eye_calibration.xml"
                return result
            
            camera_matrix, dist_coeffs, T_E_C = self.load_hand_eye_calibration(str(calibration_path))
            if camera_matrix is None:
                result['error_message'] = "无法加载手眼标定参数"
                return result
            
            # 4. 预处理和特征提取
            # 使用预处理器去除绿色背景并提取连通域
            components = []
            debug_images = {} if debug else None
            
            if self.preprocessor is not None:
                # 使用 PreprocessorDebugger 进行预处理（去除绿色背景）
                try:
                    components = self.preprocessor.preprocess(original_image, debug_images)
                except Exception as e:
                    self._log('warn', f'预处理器失败，使用简化方法: {e}')
                    components = []
            
            # 如果预处理器不可用或失败，使用简化方法
            if not components:
                # 简化方法：使用灰度阈值
                gray = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)
                # 使用OTSU阈值
                _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                # 提取连通域
                components_data = self.image_processor._extract_connected_components(binary)
                components = [comp[0] for comp in components_data]  # 只取掩膜
            
            if not components:
                result['error_message'] = "预处理失败：未找到连通域"
                return result
            
            # 保存所有连通域（debug模式）
            if debug and debug_dir:
                for i, (component_mask, area, bbox) in enumerate(components):
                    cv2.imwrite(str(debug_dir / f"01_component_{i}.jpg"), component_mask)
            
            # 选择最大的连通域（通常是工件）
            largest_component = components[0][0]
            max_area = components[0][1]
            largest_idx = 0
            for i, (component_mask, area, bbox) in enumerate(components):
                if area > max_area:
                    max_area = area
                    largest_component = component_mask
                    largest_idx = i
            
            # 保存选中的最大连通域（debug模式）
            if debug and debug_dir:
                cv2.imwrite(str(debug_dir / "02_selected_largest_component.jpg"), largest_component)
            
            # 5. 提取特征（使用 ImageProcessor 的方法）
            feature = self.image_processor._extract_features_from_mask(largest_component)
            if feature is None:
                result['error_message'] = "特征提取失败"
                return result
            
            # 保存特征提取可视化（debug模式）
            if debug and debug_dir:
                feature_vis = original_image.copy()
                if feature.get('workpiece_center'):
                    center = feature['workpiece_center']
                    radius = feature.get('workpiece_radius', 0)
                    cv2.circle(feature_vis, (int(center[0]), int(center[1])), 
                              int(radius), (0, 255, 0), 3)
                    cv2.circle(feature_vis, (int(center[0]), int(center[1])), 
                              5, (0, 255, 0), -1)
                
                if feature.get('valve_center') and feature.get('valve_radius', 0) > 0:
                    valve_center = feature['valve_center']
                    valve_radius = feature['valve_radius']
                    cv2.circle(feature_vis, (int(valve_center[0]), int(valve_center[1])), 
                              int(valve_radius), (255, 0, 0), 3)
                    cv2.circle(feature_vis, (int(valve_center[0]), int(valve_center[1])), 
                              5, (255, 0, 0), -1)
                    cv2.line(feature_vis, 
                            (int(center[0]), int(center[1])), 
                            (int(valve_center[0]), int(valve_center[1])), 
                            (0, 255, 255), 2)
                
                cv2.imwrite(str(debug_dir / "03_feature_extraction_visualization.jpg"), feature_vis)
            
            # 6. 裁剪和旋转图像（生成标准化图像）
            workpiece_center = feature['workpiece_center']
            radius = feature['workpiece_radius']
            
            crop_size = int(np.ceil(radius * 2.0))
            crop_x = int(max(0, workpiece_center[0] - radius))
            crop_y = int(max(0, workpiece_center[1] - radius))
            crop_w = min(crop_size, original_image.shape[1] - crop_x)
            crop_h = min(crop_size, original_image.shape[0] - crop_y)
            
            crop_rect = (crop_x, crop_y, crop_w, crop_h)
            cropped_mask = largest_component[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w].copy()
            
            # 使用掩膜抠图工件，背景用白色填充
            cropped_image = np.full((crop_h, crop_w, 3), (255, 255, 255), dtype=np.uint8)
            cropped_original = original_image[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w].copy()
            # 将掩膜转换为3通道以便与彩色图像匹配
            if len(cropped_mask.shape) == 2:
                mask_3ch = cv2.cvtColor(cropped_mask, cv2.COLOR_GRAY2BGR)
            else:
                mask_3ch = cropped_mask
            # 使用掩膜复制工件部分
            cropped_image = np.where(mask_3ch > 0, cropped_original, cropped_image)
            
            # 保存裁剪后的图像（debug模式）
            if debug and debug_dir:
                cv2.imwrite(str(debug_dir / "04_cropped_image.jpg"), cropped_image)
                cv2.imwrite(str(debug_dir / "04_cropped_mask.jpg"), cropped_mask)
            
            # 不再旋转图像，保持原始角度
            rotated_image = cropped_image.copy()
            rotated_mask = cropped_mask.copy()
            rotation_angle = 0.0
            
            # 保存标准化后的图像（debug模式）
            if debug and debug_dir:
                cv2.imwrite(str(debug_dir / "05_standardized_image.jpg"), rotated_image)
                cv2.imwrite(str(debug_dir / "05_standardized_mask.jpg"), rotated_mask)
            
            # 7. 读取姿态JSON文件
            T_B_E_grasp = self.load_pose_json(str(grab_position_path))
            if T_B_E_grasp is None:
                result['error_message'] = "无法加载抓取姿态"
                return result
            
            # 读取准备姿态（如果存在）
            T_B_E_preparation = None
            has_preparation = False
            prep_pose_path = pose_dir / "preparation_position.json"
            if prep_pose_path.exists():
                T_B_E_preparation = self.load_pose_json(str(prep_pose_path))
                if T_B_E_preparation is not None:
                    has_preparation = True
            
            # 读取相机姿态
            T_B_E_camera = None
            camera_pose_path = pose_dir / "camera_pose.json"
            if camera_pose_path.exists():
                T_B_E_camera = self.load_pose_json(str(camera_pose_path))
            elif has_preparation:
                T_B_E_camera = T_B_E_preparation.copy()
            else:
                T_B_E_camera = T_B_E_grasp.copy()
            
            # 8. 构建特征参数字典
            feature_params = {
                'workpiece_center_x_original': float(workpiece_center[0]),
                'workpiece_center_y_original': float(workpiece_center[1]),
                'workpiece_radius_original': float(radius),
                'workpiece_area': float(feature.get('workpiece_area', 0)),
                'valve_center_x': float(feature.get('valve_center', (0, 0))[0]),
                'valve_center_y': float(feature.get('valve_center', (0, 0))[1]),
                'valve_radius': float(feature.get('valve_radius', 0)),
                'valve_area': float(feature.get('valve_area', 0)),
                'rotation_angle_deg': 0.0,
                'rotation_angle_rad': 0.0,
                'crop_x': crop_x,
                'crop_y': crop_y,
                'crop_width': crop_w,
                'crop_height': crop_h,
            }
            
            # 计算裁剪后的阀体中心
            valve_center_cropped = (
                feature.get('valve_center', (0, 0))[0] - crop_x,
                feature.get('valve_center', (0, 0))[1] - crop_y
            )
            feature_params['final_valve_center_x'] = float(valve_center_cropped[0])
            feature_params['final_valve_center_y'] = float(valve_center_cropped[1])
            feature_params['final_workpiece_radius'] = float(radius)
            feature_params['standardized_angle_rad'] = float(feature.get('standardized_angle', 0))
            feature_params['standardized_angle_deg'] = float(feature.get('standardized_angle_deg', 0))
            
            # 9. 计算标准化姿态
            T_B_E_standardized_grasp = self.compute_standardized_pose(
                T_B_E_grasp, T_B_E_camera, T_E_C, camera_matrix, feature_params)
            
            T_B_E_standardized_preparation = None
            if has_preparation:
                T_B_E_standardized_preparation = self.compute_standardized_pose(
                    T_B_E_preparation, T_B_E_camera, T_E_C, camera_matrix, feature_params)
            
            # 10. 绘制抓取姿态可视化
            gripper_vis_image = original_image.copy()
            T_B_C_template = T_B_E_camera @ T_E_C
            T_C_template_B = np.linalg.inv(T_B_C_template)
            T_C_template_E_grasp = T_C_template_B @ T_B_E_grasp
            
            self.draw_gripper(gripper_vis_image, T_C_template_E_grasp, 
                            camera_matrix, dist_coeffs, 50.0, 100.0)
            
            gripper_vis_path = pose_dir / "gripper_visualization.jpg"
            cv2.imwrite(str(gripper_vis_path), gripper_vis_image)
            
            # 保存最终可视化图（debug模式）
            if debug and debug_dir:
                final_vis = rotated_image.copy()
                cropped_center = (workpiece_center[0] - crop_x, workpiece_center[1] - crop_y)
                cv2.circle(final_vis, (int(cropped_center[0]), int(cropped_center[1])), 
                          int(radius), (0, 255, 0), 3)
                cv2.circle(final_vis, (int(cropped_center[0]), int(cropped_center[1])), 
                          5, (0, 255, 0), -1)
                
                if feature.get('valve_center') and feature.get('valve_radius', 0) > 0:
                    final_valve_center = valve_center_cropped
                    cv2.circle(final_vis, (int(final_valve_center[0]), int(final_valve_center[1])), 
                              int(feature['valve_radius']), (255, 0, 0), 3)
                    cv2.circle(final_vis, (int(final_valve_center[0]), int(final_valve_center[1])), 
                              5, (255, 0, 0), -1)
                    cv2.line(final_vis, 
                            (int(cropped_center[0]), int(cropped_center[1])), 
                            (int(final_valve_center[0]), int(final_valve_center[1])), 
                            (0, 255, 255), 2)
                
                cv2.imwrite(str(debug_dir / "06_final_standardized_visualization.jpg"), final_vis)
                cv2.imwrite(str(debug_dir / "07_gripper_visualization_on_original.jpg"), gripper_vis_image)
            
            # 11. 保存标准化结果
            if not self.save_standardization_results(
                    pose_dir, feature_params,
                    T_B_E_grasp, T_B_E_standardized_grasp,
                    T_B_E_preparation, T_B_E_standardized_preparation,
                    rotated_image, rotated_mask):
                result['error_message'] = "保存标准化结果失败"
                return result
            
            result['success'] = True
            return result
            
        except Exception as e:
            result['error_message'] = f"标准化异常: {str(e)}"
            import traceback
            traceback.print_exc()
            return result
    
    def load_hand_eye_calibration(self, calibration_path: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        加载手眼标定参数：支持YAML和XML格式
        
        参数:
            calibration_path: 标定文件路径
        
        返回:
            (camera_matrix, dist_coeffs, T_E_C) 或 (None, None, None) 如果失败
        """
        try:
            path = Path(calibration_path)
            ext = path.suffix.lower()
            
            if ext in ['.yaml', '.yml']:
                # 解析 YAML 格式
                with open(calibration_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                # 读取相机内参矩阵
                if 'camera_matrix' not in config:
                    return None, None, None
                
                cam_matrix_data = config['camera_matrix']
                camera_matrix = np.array(cam_matrix_data, dtype=np.float64)
                
                # 读取畸变系数
                if 'distortion_coefficients' not in config:
                    return None, None, None
                
                dist_data = config['distortion_coefficients']
                dist_coeffs = np.array(dist_data, dtype=np.float64).reshape(-1, 1)
                
                # 读取手眼标定变换矩阵
                if 'hand_eye_calibration' not in config or 'transformation_matrix' not in config['hand_eye_calibration']:
                    return None, None, None
                
                transform_data = config['hand_eye_calibration']['transformation_matrix']
                T_E_C = np.array(transform_data, dtype=np.float64)
                
                return camera_matrix, dist_coeffs, T_E_C
                
            else:
                # 解析 XML 格式
                import xml.etree.ElementTree as ET
                tree = ET.parse(calibration_path)
                root = tree.getroot()
                
                # 提取CameraMatrix
                camera_matrix_node = root.find('CameraMatrix')
                if camera_matrix_node is None:
                    return None, None, None
                
                data_node = camera_matrix_node.find('data')
                if data_node is None:
                    return None, None, None
                
                camera_matrix_data = [float(x) for x in data_node.text.split()]
                if len(camera_matrix_data) != 9:
                    return None, None, None
                
                camera_matrix = np.array(camera_matrix_data, dtype=np.float64).reshape(3, 3)
                
                # 提取DistortionCoefficients
                dist_node = root.find('DistortionCoefficients')
                if dist_node is None:
                    return None, None, None
                
                dist_data_node = dist_node.find('data')
                if dist_data_node is None:
                    return None, None, None
                
                dist_data = [float(x) for x in dist_data_node.text.split()]
                dist_coeffs = np.array(dist_data, dtype=np.float64).reshape(-1, 1)
                
                # 提取TransformationMatrix
                transform_node = root.find('TransformationMatrix')
                if transform_node is None:
                    return None, None, None
                
                transform_data_node = transform_node.find('data')
                if transform_data_node is None:
                    return None, None, None
                
                transform_data = [float(x) for x in transform_data_node.text.split()]
                if len(transform_data) != 16:
                    return None, None, None
                
                T_E_C = np.array(transform_data, dtype=np.float64).reshape(4, 4)
                
                return camera_matrix, dist_coeffs, T_E_C
                
        except Exception as e:
            self._log('error', f'加载手眼标定参数失败: {e}')
            return None, None, None
    
    def load_pose_json(self, pose_path: str) -> Optional[np.ndarray]:
        """
        加载姿态JSON文件：从JSON文件读取机器人姿态（位置和四元数）
        
        参数:
            pose_path: 姿态JSON文件路径
        
        返回:
            4x4齐次变换矩阵（基座坐标系到末端执行器），失败返回None
        """
        try:
            with open(pose_path, 'r') as f:
                json_data = json.load(f)
            
            # 读取位置
            pos = json_data['cartesian_position']['position']
            x = float(pos['x'])
            y = float(pos['y'])
            z = float(pos['z'])
            
            # 读取四元数
            orient = json_data['cartesian_position']['orientation']
            qx = float(orient['x'])
            qy = float(orient['y'])
            qz = float(orient['z'])
            qw = float(orient['w'])
            
            # 将四元数转换为旋转矩阵
            qx2 = qx * qx
            qy2 = qy * qy
            qz2 = qz * qz
            qw2 = qw * qw
            
            R = np.array([
                [qw2 + qx2 - qy2 - qz2, 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
                [2.0 * (qx * qy + qw * qz), qw2 - qx2 + qy2 - qz2, 2.0 * (qy * qz - qw * qx)],
                [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), qw2 - qx2 - qy2 + qz2]
            ], dtype=np.float64)
            
            # 构建4x4齐次变换矩阵
            T_B_E = np.eye(4, dtype=np.float64)
            T_B_E[:3, :3] = R
            T_B_E[0, 3] = x
            T_B_E[1, 3] = y
            T_B_E[2, 3] = z
            
            return T_B_E
            
        except Exception as e:
            self._log('error', f'加载姿态JSON文件失败: {e}')
            return None
    
    def compute_standardized_pose(self, T_B_E_pose: np.ndarray, T_B_E_camera: np.ndarray,
                                  T_E_C: np.ndarray, camera_matrix: np.ndarray,
                                  feature_params: Dict) -> np.ndarray:
        """
        计算标准化姿态：将原始姿态转换为标准化坐标系下的姿态
        
        参数:
            T_B_E_pose: 原始姿态（基座坐标系）
            T_B_E_camera: 相机姿态（基座坐标系）
            T_E_C: 手眼变换矩阵
            camera_matrix: 相机内参矩阵
            feature_params: 特征参数字典
        
        返回:
            标准化后的姿态（基座坐标系）
        """
        # 计算相机坐标系下的姿态
        T_B_C_camera = T_B_E_camera @ T_E_C
        T_C_camera_B = np.linalg.inv(T_B_C_camera)
        T_C_camera_E_pose = T_C_camera_B @ T_B_E_pose
        
        R_pose_camera = T_C_camera_E_pose[:3, :3]
        t_pose_camera = T_C_camera_E_pose[:3, 3]
        
        # 获取旋转角度
        rotation_angle_rad = feature_params.get('rotation_angle_rad', 0.0)
        
        # 构建绕Z轴旋转矩阵
        rotation_z_camera = np.array([
            [np.cos(rotation_angle_rad), -np.sin(rotation_angle_rad), 0],
            [np.sin(rotation_angle_rad), np.cos(rotation_angle_rad), 0],
            [0, 0, 1]
        ], dtype=np.float64)
        
        # 计算工件中心在相机坐标系下的位置
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        workpiece_center_x_original = feature_params['workpiece_center_x_original']
        workpiece_center_y_original = feature_params['workpiece_center_y_original']
        workpiece_center_z_camera = t_pose_camera[2]
        
        workpiece_center_camera = np.array([
            (workpiece_center_x_original - cx) * workpiece_center_z_camera / fx,
            (workpiece_center_y_original - cy) * workpiece_center_z_camera / fy,
            workpiece_center_z_camera
        ])
        
        # 计算从工件中心到姿态位置的偏移
        offset_from_center = t_pose_camera - workpiece_center_camera
        offset_x = offset_from_center[0]
        offset_y = offset_from_center[1]
        
        # 旋转偏移
        offset_rotated = rotation_z_camera @ np.array([offset_x, offset_y, 0])
        offset_x_rotated = offset_rotated[0]
        offset_y_rotated = offset_rotated[1]
        
        # 计算标准化后的位置
        t_standardized_camera = np.array([
            workpiece_center_camera[0] + offset_x_rotated,
            workpiece_center_camera[1] + offset_y_rotated,
            t_pose_camera[2]
        ])
        
        # 计算标准化后的旋转
        R_standardized_camera = rotation_z_camera @ R_pose_camera
        
        # 构建标准化变换矩阵
        T_C_camera_E_standardized = np.eye(4, dtype=np.float64)
        T_C_camera_E_standardized[:3, :3] = R_standardized_camera
        T_C_camera_E_standardized[:3, 3] = t_standardized_camera
        
        # 转换回基座坐标系
        T_B_E_standardized = T_B_C_camera @ T_C_camera_E_standardized
        
        return T_B_E_standardized
    
    def save_standardization_results(self, pose_dir: Path, feature_params: Dict,
                                    T_B_E_grasp: np.ndarray, T_B_E_standardized_grasp: np.ndarray,
                                    T_B_E_preparation: Optional[np.ndarray],
                                    T_B_E_standardized_preparation: Optional[np.ndarray],
                                    rotated_image: np.ndarray, rotated_mask: np.ndarray) -> bool:
        """
        保存标准化结果：将标准化后的图像、掩码和姿态信息保存到文件
        
        参数:
            pose_dir: 姿态目录路径
            feature_params: 特征参数字典
            T_B_E_grasp: 原始抓取姿态
            T_B_E_standardized_grasp: 标准化抓取姿态
            T_B_E_preparation: 原始准备姿态（可选）
            T_B_E_standardized_preparation: 标准化准备姿态（可选）
            rotated_image: 标准化后的图像
            rotated_mask: 标准化后的掩码
        
        返回:
            成功返回True，失败返回False
        """
        try:
            # 保存标准化图像和掩膜
            image_path = pose_dir / "image.jpg"
            mask_path = pose_dir / "mask.jpg"
            cv2.imwrite(str(image_path), rotated_image)
            cv2.imwrite(str(mask_path), rotated_mask)
            
            # 辅助函数：将旋转矩阵转换为四元数
            def rotation_matrix_to_quaternion(R):
                trace = np.trace(R)
                if trace > 0:
                    s = np.sqrt(trace + 1.0) * 2.0
                    w = 0.25 * s
                    x = (R[2, 1] - R[1, 2]) / s
                    y = (R[0, 2] - R[2, 0]) / s
                    z = (R[1, 0] - R[0, 1]) / s
                elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                    s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                    w = (R[2, 1] - R[1, 2]) / s
                    x = 0.25 * s
                    y = (R[0, 1] + R[1, 0]) / s
                    z = (R[0, 2] + R[2, 0]) / s
                elif R[1, 1] > R[2, 2]:
                    s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                    w = (R[0, 2] - R[2, 0]) / s
                    x = (R[0, 1] + R[1, 0]) / s
                    y = 0.25 * s
                    z = (R[1, 2] + R[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                    w = (R[1, 0] - R[0, 1]) / s
                    x = (R[0, 2] + R[2, 0]) / s
                    y = (R[1, 2] + R[2, 1]) / s
                    z = 0.25 * s
                return np.array([x, y, z, w])
            
            # 辅助函数：将姿态矩阵转换为JSON
            def pose_to_json(T_B_E):
                position = {
                    'x': float(T_B_E[0, 3]),
                    'y': float(T_B_E[1, 3]),
                    'z': float(T_B_E[2, 3])
                }
                
                R = T_B_E[:3, :3]
                quat = rotation_matrix_to_quaternion(R)
                
                orientation = {
                    'x': float(quat[0]),
                    'y': float(quat[1]),
                    'z': float(quat[2]),
                    'w': float(quat[3])
                }
                
                return {
                    'cartesian_position': {
                        'position': position,
                        'orientation': orientation
                    }
                }
            
            # 构建template_info.json
            template_info = {
                'feature_parameters': feature_params,
                'original_grasp_pose': pose_to_json(T_B_E_grasp),
                'standardized_grasp_pose': pose_to_json(T_B_E_standardized_grasp)
            }
            
            if T_B_E_preparation is not None:
                template_info['original_preparation_pose'] = pose_to_json(T_B_E_preparation)
                template_info['standardized_preparation_pose'] = pose_to_json(T_B_E_standardized_preparation)
            
            template_info['rotation_angle_deg'] = feature_params.get('rotation_angle_deg', 0.0)
            template_info['rotation_angle_rad'] = feature_params.get('rotation_angle_rad', 0.0)
            
            # 保存JSON文件
            template_info_path = pose_dir / "template_info.json"
            with open(template_info_path, 'w') as f:
                json.dump(template_info, f, indent=2)
            
            return True
            
        except Exception as e:
            self._log('error', f'保存标准化结果失败: {e}')
            import traceback
            traceback.print_exc()
            return False
    
    def draw_gripper(self, image: np.ndarray, T_C_E: np.ndarray,
                    camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                    gripper_opening_mm: float, gripper_length_mm: float):
        """
        绘制夹爪：在图像上绘制夹爪的可视化（用于调试）
        
        参数:
            image: 输入图像（会被修改）
            T_C_E: 末端执行器在相机坐标系下的变换矩阵
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数
            gripper_opening_mm: 夹爪开口宽度（毫米）
            gripper_length_mm: 夹爪长度（毫米）
        """
        if T_C_E is None or camera_matrix is None:
            return
        
        # 提取末端执行器原点在相机坐标系下的位置
        ee_pos_camera = T_C_E[:3, 3]
        
        # 提取末端执行器坐标系的三个轴在相机坐标系下的方向
        gripper_x_axis = T_C_E[:3, 0]
        gripper_y_axis = T_C_E[:3, 1]
        gripper_z_axis = T_C_E[:3, 2]
        
        # 归一化
        gripper_x_axis = gripper_x_axis / np.linalg.norm(gripper_x_axis) if np.linalg.norm(gripper_x_axis) > 1e-6 else gripper_x_axis
        gripper_y_axis = gripper_y_axis / np.linalg.norm(gripper_y_axis) if np.linalg.norm(gripper_y_axis) > 1e-6 else gripper_y_axis
        gripper_z_axis = gripper_z_axis / np.linalg.norm(gripper_z_axis) if np.linalg.norm(gripper_z_axis) > 1e-6 else gripper_z_axis
        
        # 夹爪坐标系定义：Z轴=指向方向，Y轴=开口方向
        pointing_direction = gripper_z_axis
        opening_direction = gripper_y_axis
        
        # 计算夹爪的两个手指位置
        finger_center = ee_pos_camera
        
        # 两个手指分别沿着开口方向的正负方向偏移
        finger1_center = finger_center + opening_direction * (gripper_opening_mm / 2.0)
        finger2_center = finger_center - opening_direction * (gripper_opening_mm / 2.0)
        
        # 手指沿着指向方向延伸
        finger1_tip = finger1_center + pointing_direction * gripper_length_mm
        finger2_tip = finger2_center + pointing_direction * gripper_length_mm
        
        # 将3D点投影到2D像素坐标
        object_points = np.array([
            finger_center, finger1_center, finger1_tip, finger2_center, finger2_tip
        ], dtype=np.float64)
        
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        image_points = []
        for pt_3d in object_points:
            if pt_3d[2] > 0:  # 确保点在相机前方
                u = fx * pt_3d[0] / pt_3d[2] + cx
                v = fy * pt_3d[1] / pt_3d[2] + cy
                image_points.append((u, v))
            else:
                image_points.append((-1, -1))  # 无效点
        
        # 检查点是否有效（在图像范围内）
        def is_valid_point(pt):
            return pt[0] >= 0 and pt[1] >= 0 and pt[0] < image.shape[1] and pt[1] < image.shape[0]
        
        # 绘制夹爪（红色）
        red_color = (0, 0, 255)  # BGR格式，红色
        thickness = 3
        circle_radius = 5
        
        # 绘制中心点
        if len(image_points) > 0 and is_valid_point(image_points[0]):
            cv2.circle(image, (int(image_points[0][0]), int(image_points[0][1])), 
                      circle_radius, red_color, -1)
        
        # 绘制第一个手指（从中心到指尖）
        if len(image_points) > 2 and is_valid_point(image_points[1]) and is_valid_point(image_points[2]):
            cv2.line(image, 
                    (int(image_points[1][0]), int(image_points[1][1])), 
                    (int(image_points[2][0]), int(image_points[2][1])), 
                    red_color, thickness)
            cv2.circle(image, (int(image_points[2][0]), int(image_points[2][1])), 
                      circle_radius, red_color, -1)
        
        # 绘制第二个手指（从中心到指尖）
        if len(image_points) > 4 and is_valid_point(image_points[3]) and is_valid_point(image_points[4]):
            cv2.line(image, 
                    (int(image_points[3][0]), int(image_points[3][1])), 
                    (int(image_points[4][0]), int(image_points[4][1])), 
                    red_color, thickness)
            cv2.circle(image, (int(image_points[4][0]), int(image_points[4][1])), 
                      circle_radius, red_color, -1)
        
        # 绘制连接两个手指中心的线
        if len(image_points) > 3 and is_valid_point(image_points[1]) and is_valid_point(image_points[3]):
            cv2.line(image, 
                    (int(image_points[1][0]), int(image_points[1][1])), 
                    (int(image_points[3][0]), int(image_points[3][1])), 
                    red_color, thickness)


def main():
    """主函数：示例用法"""
    import argparse
    
    parser = argparse.ArgumentParser(description='模板标准化器（Python版本）')
    parser.add_argument('--template_root', type=str, required=True,
                       help='模板根目录路径')
    parser.add_argument('--workpiece_id', type=str, required=True,
                       help='工件ID')
    parser.add_argument('--calib_file', type=str, default='',
                       help='标定文件路径（可选）')
    parser.add_argument('--debug', action='store_true',
                       help='启用调试模式')
    
    args = parser.parse_args()
    
    standardizer = TemplateStandardizer()
    results = standardizer.standardize_template(
        args.template_root,
        args.workpiece_id,
        args.calib_file,
        args.debug
    )
    
    # 输出结果
    success_count = sum(1 for r in results if r['success'])
    print(f'\n标准化完成: {success_count}/{len(results)} 个姿态成功')
    
    for result in results:
        if result['success']:
            print(f"✓ 姿态 {result['pose_id']}: 标准化成功")
        else:
            print(f"✗ 姿态 {result['pose_id']}: 标准化失败 - {result['error_message']}")


if __name__ == '__main__':
    main()
