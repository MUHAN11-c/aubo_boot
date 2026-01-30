#!/usr/bin/env python3
"""
使用示例：展示如何直接使用各个模块（不通过ROS2）

该示例展示了如何独立使用预处理器、特征提取器和模板标准化器
"""

import cv2
import numpy as np
import logging
from pathlib import Path

# 导入模块
from visual_pose_estimation_python.preprocessor import Preprocessor
from visual_pose_estimation_python.feature_extractor import FeatureExtractor
from visual_pose_estimation_python.template_standardizer import TemplateStandardizer
from visual_pose_estimation_python.config_reader import ConfigReader


def example_preprocess_and_extract_features():
    """示例：深度图预处理和特征提取"""
    
    # 配置日志
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    # 1. 创建预处理器
    logger.info("=" * 60)
    logger.info("示例1: 深度图预处理和特征提取")
    logger.info("=" * 60)
    
    preprocessor = Preprocessor()
    
    # 可选：设置自定义参数
    preprocessor.set_parameters({
        'binary_threshold_min': 0.0,        # 最小深度阈值
        'binary_threshold_max': 65535.0,    # 最大深度阈值
        'enable_zero_interp': 1.0,          # 启用0值插值
        'component_min_area': 1000.0,
        'component_max_area': 1000000.0
    })
    
    # 2. 加载深度图像
    depth_image_path = "/path/to/your/depth_image.png"  # 替换为实际深度图路径
    
    if not Path(depth_image_path).exists():
        logger.warning(f"示例深度图不存在: {depth_image_path}")
        logger.info("请替换为实际的深度图路径（16位PNG或其他格式）")
        return
    
    # 读取深度图（保持原始深度值）
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)
    logger.info(f"加载深度图: {depth_image_path}, 尺寸: {depth_image.shape}, 类型: {depth_image.dtype}")
    
    # 3. 预处理深度图像（如果有彩色图，也可以一起处理）
    color_image = None  # 如果有彩色图，在这里加载
    
    components, preprocessed_color = preprocessor.preprocess(
        depth_image,
        color_image,
        binary_threshold_min=1000,   # 示例：最小深度1000
        binary_threshold_max=50000   # 示例：最大深度50000
    )
    logger.info(f"深度图预处理完成，提取到 {len(components)} 个连通域")
    
    # 4. 创建特征提取器
    feature_extractor = FeatureExtractor()
    
    # 5. 提取特征
    features = feature_extractor.extract_features(components)
    logger.info(f"特征提取完成，成功提取 {len(features)} 个特征")
    
    # 6. 打印特征信息并可视化
    for idx, feature in enumerate(features):
        logger.info(f"\n特征 {idx}:")
        logger.info(f"  工件中心: {feature.workpiece_center}")
        logger.info(f"  工件半径: {feature.workpiece_radius:.2f} 像素")
        logger.info(f"  工件面积: {feature.workpiece_area:.2f} 像素²")
        logger.info(f"  阀体中心: {feature.valve_center}")
        logger.info(f"  阀体半径: {feature.valve_radius:.2f} 像素")
        logger.info(f"  阀体面积: {feature.valve_area:.2f} 像素²")
        logger.info(f"  标准化角度: {feature.standardized_angle_deg:.2f}°")
        
        # 可视化特征（如果有彩色图）
        if preprocessed_color is not None:
            vis_image = feature.draw_features()
            # cv2.imshow(f'Feature {idx}', vis_image)
            # cv2.waitKey(0)
    
    return features


def example_standardize_template():
    """示例：标准化模板"""
    
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    logger.info("\n" + "=" * 60)
    logger.info("示例2: 标准化模板")
    logger.info("=" * 60)
    
    # 1. 创建模板标准化器
    standardizer = TemplateStandardizer()
    
    # 2. 加载图像（假设已经有特征）
    image_path = "/path/to/your/image.jpg"  # 替换为实际图像路径
    
    if not Path(image_path).exists():
        logger.warning(f"示例图像不存在: {image_path}")
        return
    
    image = cv2.imread(image_path)
    
    # 3. 假设我们已经有了特征（这里使用示例值）
    workpiece_center = (320, 240)
    valve_center = (350, 240)
    standardized_angle = 0.5  # 弧度
    
    # 创建一个简单的掩码（实际应该从特征提取中获得）
    mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
    workpiece_radius = 100.0
    cv2.circle(mask, workpiece_center, int(workpiece_radius), 255, -1)
    
    # 4. 标准化模板（参考C++版本：裁剪而不是旋转）
    standardized_image, standardized_mask, crop_params = standardizer.standardize_template(
        image,
        mask,
        workpiece_center,
        valve_center,
        standardized_angle,
        workpiece_radius
    )
    
    logger.info("模板标准化完成")
    
    # 5. 保存标准化后的模板
    template_dir = Path("/tmp/test_templates/3211242785")
    feature_params = {
        "workpiece_center_x_original": float(workpiece_center[0]),
        "workpiece_center_y_original": float(workpiece_center[1]),
        "workpiece_radius_original": float(workpiece_radius),
        "valve_center_x": float(valve_center[0]),
        "valve_center_y": float(valve_center[1]),
        "valve_radius": 20.0,
        "standardized_angle_rad": float(standardized_angle),
        "standardized_angle_deg": float(np.degrees(standardized_angle)),
    }
    feature_params.update(crop_params)

    success = standardizer.save_standardized_template(
        template_dir,
        "pose_1",
        standardized_image,
        standardized_mask,
        feature_params,
        T_B_E_grasp=np.eye(4),              # 示例占位
        T_B_E_standardized_grasp=np.eye(4), # 示例占位
        save_metadata=False
    )
    
    if success:
        logger.info(f"模板已保存到: {template_dir / 'pose_1'}")
    else:
        logger.error("模板保存失败")


def example_load_config():
    """示例：加载配置文件"""
    
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    logger.info("\n" + "=" * 60)
    logger.info("示例3: 加载配置文件")
    logger.info("=" * 60)
    
    # 1. 创建配置读取器
    config_reader = ConfigReader()
    
    # 2. 加载配置文件
    config_file = "configs/default.yaml"
    
    if Path(config_file).exists():
        success = config_reader.load_from_file(config_file)
        if success:
            logger.info(f"配置文件加载成功: {config_file}")
        else:
            logger.error(f"配置文件加载失败: {config_file}")
    else:
        logger.warning(f"配置文件不存在: {config_file}")
    
    # 3. 获取配置值
    scale_factor = config_reader.get('preprocessor.scale_factor', 1.0)
    logger.info(f"缩放因子: {scale_factor}")
    
    # 4. 获取配置段
    preprocessor_config = config_reader.get_section('preprocessor')
    logger.info(f"预处理器配置: {list(preprocessor_config.keys())}")
    
    feature_extractor_config = config_reader.get_section('feature_extractor')
    logger.info(f"特征提取器配置: {list(feature_extractor_config.keys())}")


if __name__ == '__main__':
    """运行所有示例"""
    
    print("\n" + "=" * 70)
    print("visual_pose_estimation_python 使用示例")
    print("=" * 70)
    
    # 运行示例
    try:
        # 示例1: 图像预处理和特征提取
        example_preprocess_and_extract_features()
        
        # 示例2: 标准化模板
        # example_standardize_template()
        
        # 示例3: 加载配置文件
        example_load_config()
        
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 70)
    print("示例运行完成")
    print("=" * 70)
    print("\n提示:")
    print("1. 请替换示例中的图像路径为实际路径")
    print("2. 确保已安装所有依赖: opencv-python, numpy, pyyaml")
    print("3. 可以通过ROS2 launch文件启动完整节点:")
    print("   ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py")
