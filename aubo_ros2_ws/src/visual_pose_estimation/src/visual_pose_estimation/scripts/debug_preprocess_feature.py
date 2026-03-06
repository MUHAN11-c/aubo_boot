#!/usr/bin/env python3
"""
预处理和特征提取参数调试脚本

该脚本用于调试视觉姿态估计系统中的图像预处理和特征提取参数。
实现与C++源代码（preprocessor.cpp和feature_extractor.cpp）相同的处理逻辑，
并提供实时参数调整和可视化功能。

使用方法:
    python3 debug_preprocess_feature.py <image_path> [config_path]

参数说明:
    image_path: 待处理的图像文件路径
    config_path: 可选，配置文件路径（YAML格式）

功能:
    - 实时调整预处理参数（背景去除、连通域提取等）
    - 实时调整特征提取参数（工件外接圆、阀体外接圆等）
    - 可视化处理结果（按照C++代码的处理顺序显示）
    - 保存优化后的参数配置到YAML文件

处理流程（与C++代码一致）:
    1. 预处理阶段（Preprocessor）:
       - 图像缩放（如果scale_factor < 1.0）
       - 去除绿色背景（HSV+Lab颜色空间）
       - 提取和筛选连通域
    2. 特征提取阶段（FeatureExtractor）:
       - 提取工件外接圆（大圆）
       - 提取阀体外接圆（小圆）
       - 计算标准化角度

作者: Visual Pose Estimation Team
日期: 2024
"""

import cv2
import numpy as np
import yaml
import sys
import os
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import math

# 尝试导入matplotlib，如果不可用则使用OpenCV显示
try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    from matplotlib.patches import Rectangle
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("警告: matplotlib不可用，将使用OpenCV显示（功能受限）")


class PreprocessorDebugger:
    """
    预处理调试器类
    
    实现与C++ Preprocessor类相同的预处理逻辑，包括：
    - 图像缩放
    - 绿色背景去除（HSV+Lab颜色空间）
    - 连通域提取和筛选
    
    该类用于调试和可视化预处理过程，允许实时调整参数。
    
    属性:
        config (Dict): 配置参数字典
        preprocess_config (Dict): 预处理配置
        bg_config (Dict): 背景去除配置
        feature_config (Dict): 特征提取配置
        scale_factor (float): 图像缩放因子
        border_ratio (float): 边缘采样区域比例
        hue_margin (float): HSV色调容差
        ... (其他参数)
    """
    
    def __init__(self, config: Dict):
        """
        初始化预处理调试器
        
        从配置字典中加载所有预处理参数，如果参数不存在则使用默认值。
        参数命名与C++代码中的参数名称保持一致。
        
        参数:
            config (Dict): 配置参数字典，应包含'preprocess'键
        """
        self.config = config
        self.preprocess_config = config.get('preprocess', {})
        self.bg_config = self.preprocess_config.get('background', {})
        self.feature_config = self.preprocess_config.get('feature_extraction', {})
        
        # 预处理参数
        self.scale_factor = self.preprocess_config.get('scale_factor', 1.0)
        self.min_area = self.preprocess_config.get('min_area', 2000)
        
        # 背景去除参数
        self.border_ratio = self.bg_config.get('border_ratio', 0.08)
        self.hue_margin = self.bg_config.get('hue_margin', 12.0)
        self.hue_std_mul = self.bg_config.get('hue_std_mul', 3.0)
        self.sat_margin = self.bg_config.get('sat_margin', 25.0)
        self.sat_std_mul = self.bg_config.get('sat_std_mul', 2.0)
        self.val_margin = self.bg_config.get('val_margin', 35.0)
        self.val_std_mul = self.bg_config.get('val_std_mul', 2.0)
        self.lab_threshold = self.bg_config.get('lab_threshold', 3.5)
        self.cleanup_kernel = self.bg_config.get('cleanup_kernel', 7)
        self.foreground_close_kernel = self.bg_config.get('foreground_close_kernel', 9)
        self.median_ksize = self.bg_config.get('median_ksize', 5)
        self.enable_classic_hsv = self.bg_config.get('enable_classic_hsv', True)
        self.use_histogram = self.bg_config.get('use_histogram', True)
        self.min_noise_area = self.bg_config.get('min_noise_area', 100)
        self.erode_before_dilate = self.bg_config.get('erode_before_dilate', False)
        self.component_min_area = self.bg_config.get('component_min_area', 1500)
        self.component_max_area = self.bg_config.get('component_max_area', 80000)
        self.component_min_aspect_ratio = self.bg_config.get('component_min_aspect_ratio', 0.3)
        self.component_max_aspect_ratio = self.bg_config.get('component_max_aspect_ratio', 4.0)
        self.component_min_width = self.bg_config.get('component_min_width', 60)
        self.component_min_height = self.bg_config.get('component_min_height', 60)
        self.component_max_count = self.bg_config.get('component_max_count', 3)
        
        # 特征提取参数
        self.feature_min_component_area = self.feature_config.get('min_component_area', 2000)
        self.feature_max_component_area = self.feature_config.get('max_component_area', 10000000)
        self.big_circle_combine_contours = self.feature_config.get('big_circle', {}).get('combine_contours', True)
        self.big_circle_min_area = self.feature_config.get('big_circle', {}).get('min_area', 100)
        self.small_circle_erode_kernel = self.feature_config.get('small_circle', {}).get('erode_kernel', 11)
        self.small_circle_erode_iterations = self.feature_config.get('small_circle', {}).get('erode_iterations', 5)
        self.small_circle_largest_cc = self.feature_config.get('small_circle', {}).get('largest_cc', True)
        self.small_circle_dilate_kernel = self.feature_config.get('small_circle', {}).get('dilate_kernel', 9)
        self.small_circle_dilate_iterations = self.feature_config.get('small_circle', {}).get('dilate_iterations', 1)
    
    def remove_green_background(self, image: np.ndarray, debug_images: dict = None) -> np.ndarray:
        """
        去除绿色背景
        
        实现与C++ Preprocessor::removeGreenBackground相同的算法：
        1. 在HSV和Lab颜色空间中进行背景检测
        2. 使用边缘区域统计信息计算自适应阈值
        3. 结合HSV掩码和Lab掩码进行背景分割
        4. 形态学处理和噪声去除
        
        函数语法:
            remove_green_background(image, debug_images=None)
        
        参数说明:
            image (np.ndarray): 
                - 类型: numpy数组，形状为(H, W, 3)
                - 格式: BGR颜色空间
                - 说明: 输入的彩色图像，需要去除绿色背景
                - 要求: 图像不能为空，尺寸必须大于0
            
            debug_images (dict, optional): 
                - 类型: 字典，默认值为None
                - 说明: 用于保存中间处理结果的字典
                - 如果提供，会保存以下键值对：
                  * 'scaled_image' (np.ndarray): 缩放后的图像，形状为(new_h, new_w, 3)
                  * 'border_mask' (np.ndarray): 边缘掩码，二值图像，255为边缘区域
                  * 'hsv_mask' (np.ndarray): HSV背景掩码，二值图像，255为背景
                  * 'lab_mask' (np.ndarray): Lab背景掩码，二值图像，255为背景
                  * 'bg_mask_after_morph' (np.ndarray): 形态学处理后的背景掩码
                  * 'non_green_before_noise' (np.ndarray): 去噪前的前景掩码
                  * 'non_green_after_noise' (np.ndarray): 去噪后的前景掩码
        
        返回:
            np.ndarray: 
                - 类型: numpy数组，形状为(original_h, original_w)
                - 格式: 二值图像，数据类型为uint8
                - 说明: 前景掩码，255表示前景（非绿色），0表示背景（绿色）
                - 异常: 如果输入图像为空或处理失败，返回空数组 np.array([])
        
        算法原理:
            1. 图像缩放: 如果scale_factor < 1.0，先缩小图像以加速处理
            2. 颜色空间转换: 转换到HSV和Lab颜色空间
            3. 边缘采样: 从图像边缘区域采样背景颜色统计信息
            4. 自适应阈值计算: 基于统计信息计算HSV和Lab的匹配阈值
            5. 背景掩码生成: 分别生成HSV和Lab背景掩码，然后合并
            6. 形态学处理: 膨胀+腐蚀填充小孔洞
            7. 噪声去除: 去除小面积连通域
            8. 前景优化: 闭运算+中值滤波平滑前景掩码
            9. 尺寸恢复: 如果之前缩小了，恢复到原始尺寸
        
        使用的关键参数（来自self）:
            - self.scale_factor (float): 图像缩放因子，默认1.0，范围[0.1, 1.0]
            - self.border_ratio (float): 边缘采样比例，默认0.08，范围[0.05, 0.30]
            - self.hue_margin (float): HSV色调容差，默认12.0，范围[0, 100]
            - self.hue_std_mul (float): 色调标准差倍数，默认3.0，范围[0, 10]
            - self.sat_margin (float): 饱和度容差，默认25.0，范围[0, 200]
            - self.sat_std_mul (float): 饱和度标准差倍数，默认2.0，范围[0, 10]
            - self.val_margin (float): 亮度容差，默认35.0，范围[0, 200]
            - self.val_std_mul (float): 亮度标准差倍数，默认2.0，范围[0, 10]
            - self.lab_threshold (float): Lab距离阈值，默认3.5，范围[2.0, 15.0]
            - self.cleanup_kernel (int): 形态学核大小，默认7，必须为奇数，范围[3, 21]
            - self.min_noise_area (int): 最小噪声面积，默认100，范围[0, 1000]
            - self.foreground_close_kernel (int): 前景闭运算核大小，默认9，范围[3, 9]
            - self.median_ksize (int): 中值滤波核大小，默认5，必须为奇数，范围[3, 5]
        
        示例:
            >>> preprocessor = PreprocessorDebugger(config)
            >>> image = cv2.imread('test.jpg')
            >>> debug_images = {}
            >>> foreground_mask = preprocessor.remove_green_background(image, debug_images)
            >>> # 现在debug_images包含了所有中间处理结果
            >>> cv2.imshow('foreground', foreground_mask)
        
        注意事项:
            - 如果scale_factor < 1.0，处理后会恢复到原始尺寸
            - 所有调试图像也会相应缩放
            - 边缘区域如果被工件污染，会自动调整统计信息
            - HSV色调匹配考虑了色环的周期性（0和180相邻）
        """
        if image is None or image.size == 0:
            return np.array([])
        
        # 保存原始图像尺寸，用于后续恢复
        original_size = image.shape[:2]
        
        # ========== 步骤1: 图像缩放 ==========
        # 如果scale_factor < 1.0，先缩小图像以加速处理
        # cv2.resize()参数说明:
        #   - image: 输入图像，BGR格式
        #   - (new_w, new_h): 目标尺寸，格式为(宽度, 高度)
        #   - interpolation: 插值方法
        #     * cv2.INTER_AREA: 区域插值，适合缩小图像，质量好
        #     * cv2.INTER_LINEAR: 双线性插值，适合放大
        #     * cv2.INTER_CUBIC: 双三次插值，质量最好但速度慢
        
        # 实际参数值（基于default.yaml）:
        #   - scale_factor = 1.0（默认值，不缩放）
        #   - 如果scale_factor = 0.5，图像缩小到50%（640x480 -> 320x240）
        #   - 如果scale_factor = 0.8，图像缩小到80%（640x480 -> 512x384）
        #   说明: scale_factor=1.0表示不缩放，直接使用原始图像，保留最高质量
        
        if self.scale_factor < 1.0:
            h, w = image.shape[:2]  # 获取原始图像高度和宽度
            # 实际计算示例（假设原始图像640x480，scale_factor=0.8）:
            #   new_w = round(640 * 0.8) = 512
            #   new_h = round(480 * 0.8) = 384
            #   缩放后图像尺寸: 512x384（原始尺寸的80%）
            new_w = int(round(w * self.scale_factor))  # 计算新宽度（向下取整到最近整数）
            new_h = int(round(h * self.scale_factor))  # 计算新高度
            # 使用INTER_AREA插值缩小图像（适合缩小操作，避免摩尔纹）
            # 实际调用示例: cv2.resize(image, (512, 384), interpolation=cv2.INTER_AREA)
            #   - 输入: 640x480 BGR图像
            #   - 输出: 512x384 BGR图像
            #   - 插值方法: INTER_AREA（区域插值，适合缩小）
            scaled_image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        else:
            scaled_image = image.copy()
        
        # ========== 步骤2: 颜色空间转换 ==========
        # cv2.cvtColor()函数语法:
        #   cv2.cvtColor(src, code) -> dst
        #   参数:
        #     - src: 输入图像，BGR格式
        #     - code: 颜色空间转换代码
        #       * cv2.COLOR_BGR2HSV: BGR转HSV
        #       * cv2.COLOR_BGR2Lab: BGR转Lab
        #   返回: 转换后的图像
        
        # HSV颜色空间说明:
        #   - H (Hue): 色调，范围[0, 180)，表示颜色类型（红、绿、蓝等）
        #   - S (Saturation): 饱和度，范围[0, 255]，表示颜色纯度（0为灰色，255为纯色）
        #   - V (Value): 亮度，范围[0, 255]，表示明暗程度
        #   优点: 对光照变化不敏感，适合颜色分割
        hsv = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2HSV)
        
        # Lab颜色空间说明:
        #   - L (Lightness): 亮度，范围[0, 100]
        #   - a: 绿-红轴，范围[-127, 127]，负值为绿，正值为红
        #   - b: 蓝-黄轴，范围[-127, 127]，负值为蓝，正值为黄
        #   优点: 感知均匀的颜色空间，两个颜色的Lab距离近似等于人眼感知的颜色差异
        lab = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2Lab)
        
        # cv2.split()函数: 将多通道图像分离为单通道图像
        #   返回: 三个单通道数组的元组 (H通道, S通道, V通道)
        hsv_channels = cv2.split(hsv)
        
        # ========== 步骤3: 边缘区域采样 ==========
        # np.clip()函数: 将值限制在指定范围内
        #   语法: np.clip(a, a_min, a_max) -> 限制后的数组
        #   参数:
        #     - a: 输入值
        #     - a_min: 最小值
        #     - a_max: 最大值
        #   说明: 确保border_ratio在合理范围内（5%到30%）
        
        # 实际参数值（基于default.yaml）:
        #   - border_ratio = 0.08（8%）
        #   - 限制范围: [0.05, 0.30]（5%到30%）
        #   说明: 0.08表示边缘采样区域是图像尺寸的8%
        border_ratio = np.clip(self.border_ratio, 0.05, 0.30)
        
        # 计算边缘宽度（margin）:
        #   - border_ratio * min(height, width): 按比例计算边缘宽度
        #   - 限制在图像尺寸的1/3以内，避免边缘区域过大
        #   - 最小值为10像素，确保边缘区域有足够采样点
        
        # 实际计算示例（假设图像640x480，border_ratio=0.08）:
        #   min(height, width) = min(480, 640) = 480
        #   border_ratio * min(height, width) = 0.08 * 480 = 38.4
        #   round(38.4) = 38
        #   min(38, 480//3) = min(38, 160) = 38
        #   max(10, 38) = 38
        #   边缘宽度: 38像素（图像宽度的6%，高度的8%）
        #   边缘区域: 上边缘38行 + 下边缘38行 + 左边缘38列 + 右边缘38列
        margin = max(10, min(int(round(border_ratio * min(scaled_image.shape[:2]))), 
                            min(scaled_image.shape[:2]) // 3))
        
        # 创建边缘掩码（二值图像）:
        #   np.zeros()函数: 创建全零数组
        #     语法: np.zeros(shape, dtype)
        #     参数:
        #       - shape: 数组形状 (height, width)
        #       - dtype: 数据类型，uint8表示8位无符号整数（0-255）
        border_mask = np.zeros(scaled_image.shape[:2], dtype=np.uint8)
        
        # 填充边缘区域为255（白色）:
        #   数组切片说明:
        #     - border_mask[:margin, :]: 上边缘（前margin行）
        #     - border_mask[-margin:, :]: 下边缘（后margin行）
        #     - border_mask[:, :margin]: 左边缘（前margin列）
        #     - border_mask[:, -margin:]: 右边缘（后margin列）
        if margin > 0:
            border_mask[:margin, :] = 255      # 上边缘
            border_mask[-margin:, :] = 255     # 下边缘
            border_mask[:, :margin] = 255      # 左边缘
            border_mask[:, -margin:] = 255     # 右边缘
        
        # cv2.countNonZero()函数: 统计非零像素数量
        #   如果边缘区域太小（小于图像面积的10%），说明边缘可能被工件占据
        #   此时使用整个图像作为采样区域
        if cv2.countNonZero(border_mask) < scaled_image.shape[0] * scaled_image.shape[1] * 0.1:
            border_mask.fill(255)  # fill()方法: 用指定值填充整个数组
        
        # ========== 步骤4: 计算背景颜色统计信息 ==========
        # cv2.meanStdDev()函数: 计算图像的均值和标准差
        #   函数语法:
        #     cv2.meanStdDev(src, mask=None) -> (mean, stddev)
        #   参数:
        #     - src: 输入图像（多通道）
        #     - mask: 可选掩码，只统计掩码中非零区域
        #   返回:
        #     - mean: 均值数组，形状为(通道数, 1)，每个通道的均值
        #     - stddev: 标准差数组，形状为(通道数, 1)，每个通道的标准差
        
        # 统计边缘区域的HSV和Lab统计信息
        #   返回的mean_hsv形状为(3, 1): [H均值, S均值, V均值]
        #   返回的std_hsv形状为(3, 1): [H标准差, S标准差, V标准差]
        mean_hsv, std_hsv = cv2.meanStdDev(hsv, mask=border_mask)
        mean_lab, std_lab = cv2.meanStdDev(lab, mask=border_mask)
        
        # 同时计算整个图像的统计信息（用于检测边缘是否被工件污染）
        #   如果边缘被工件占据，统计信息会有较大差异
        mean_hsv_full, std_hsv_full = cv2.meanStdDev(hsv)
        mean_lab_full, std_lab_full = cv2.meanStdDev(lab)
        
        # ========== 步骤5: 检测边缘污染并调整统计信息 ==========
        # 检查边缘区域是否被工件污染:
        #   如果边缘区域和整体图像差异较大，说明边缘可能被工件占据
        #   阈值说明:
        #     - 色调差异 > 30度: 颜色类型明显不同
        #     - 饱和度差异 > 50: 颜色纯度明显不同
        #     - 亮度差异 > 60: 明暗程度明显不同
        hsv_hue_diff = abs(mean_hsv[0][0] - mean_hsv_full[0][0])  # 色调差异
        hsv_sat_diff = abs(mean_hsv[1][0] - mean_hsv_full[1][0])  # 饱和度差异
        hsv_val_diff = abs(mean_hsv[2][0] - mean_hsv_full[2][0])  # 亮度差异
        
        # 如果检测到污染，使用加权平均调整统计信息:
        #   weight_edge = 0.3: 边缘权重（较小，因为可能被污染）
        #   weight_full = 0.7: 整体权重（较大，更可靠）
        #   加权平均公式: 新均值 = 0.3 * 边缘均值 + 0.7 * 整体均值
        if hsv_hue_diff > 30.0 or hsv_sat_diff > 50.0 or hsv_val_diff > 60.0:
            weight_edge, weight_full = 0.3, 0.7
            # 数组运算: numpy数组可以直接与标量相乘，逐元素运算
            mean_hsv = weight_edge * mean_hsv + weight_full * mean_hsv_full
            std_hsv = weight_edge * std_hsv + weight_full * std_hsv_full
            mean_lab = weight_edge * mean_lab + weight_full * mean_lab_full
            std_lab = weight_edge * std_lab + weight_full * std_lab_full
        
        # ========== 步骤6: 直方图分析（可选）==========
        # cv2.calcHist()函数: 计算直方图
        #   函数语法:
        #     cv2.calcHist(images, channels, mask, histSize, ranges) -> hist
        #   参数:
        #     - images: 图像列表，格式为[image]
        #     - channels: 通道索引列表，[0]表示第0通道（H通道）
        #     - mask: 掩码，只统计掩码中非零区域
        #     - histSize: 直方图大小（bin数量），[180]表示180个bin（对应H的范围0-180）
        #     - ranges: 值范围，[0, 180]表示H的值范围
        #   返回: 直方图数组，形状为(histSize, 1)
        #   说明: 使用直方图找到最频繁的色调值（峰值），更准确
        
        # cv2.bitwise_and()函数: 按位与运算
        #   函数语法:
        #     cv2.bitwise_and(src1, src2) -> dst
        #   说明: 只统计掩码区域的值
        if self.use_histogram:
            # 提取边缘区域的H通道（色调）
            border_h = cv2.bitwise_and(hsv_channels[0], border_mask)
            # 计算色调直方图，找出最频繁的色调值
            hist = cv2.calcHist([border_h], [0], border_mask, [180], [0, 180])
            # np.argmax()函数: 返回数组中最大值的索引
            max_idx = np.argmax(hist)  # 找到直方图峰值对应的色调值
            if max_idx >= 0:
                mean_hsv[0][0] = max_idx  # 使用峰值作为色调均值
            
            # 提取边缘区域的S和V通道
            border_s = cv2.bitwise_and(hsv_channels[1], border_mask)
            border_v = cv2.bitwise_and(hsv_channels[2], border_mask)
            
            # 布尔索引: border_mask > 0 返回掩码中非零位置的索引
            #   用于提取掩码区域的值
            s_values = border_s[border_mask > 0]  # 提取S通道值
            v_values = border_v[border_mask > 0]  # 提取V通道值
            
            # np.median()函数: 计算中位数
            #   说明: 中位数对异常值不敏感，比均值更稳健
            if len(s_values) > 0:
                mean_hsv[1][0] = np.median(s_values)  # 使用中位数作为S均值
            if len(v_values) > 0:
                mean_hsv[2][0] = np.median(v_values)  # 使用中位数作为V均值
        
        # ========== 步骤7: 计算自适应阈值 ==========
        # 自适应阈值公式:
        #   threshold = margin + std_mul * std_dev
        #   说明:
        #     - margin: 基础容差（固定值）
        #     - std_mul: 标准差倍数（可调参数）
        #     - std_dev: 标准差（统计值）
        #   好处: 根据背景颜色的变化程度自动调整阈值，适应不同场景
        
        # 实际参数值（基于default.yaml）:
        #   - hue_margin = 12.0: 基础色调容差12度
        #   - hue_std_mul = 3.0: 标准差倍数3.0
        #   - sat_margin = 25.0: 基础饱和度容差25
        #   - sat_std_mul = 2.0: 标准差倍数2.0
        #   - val_margin = 35.0: 基础亮度容差35
        #   - val_std_mul = 2.0: 标准差倍数2.0
        
        # HSV色调阈值:
        #   - 范围限制在[15.0, 60.0]，避免过大或过小
        #   - 说明: 色调范围是0-180，但容差不需要太大
        #   实际计算示例（假设背景H均值=60（绿色），std_H=5）:
        #     阈值 = 12.0 + 3.0 * 5 = 27.0
        #     匹配范围 = [60-27, 60+27] = [33, 87]（考虑周期性）
        #     说明: 绿色在HSV中约60度，±27度表示33-87度范围（绿色区域）
        hue_threshold = np.clip(
            self.hue_margin + self.hue_std_mul * std_hsv[0][0],  # 基础容差 + 标准差倍数
            # 实际值: 12.0 + 3.0 * std_hsv[0][0]
            # 示例: 如果std_H=5，则阈值=12.0+15.0=27.0（色调容差27度）
            15.0, 60.0  # 最小值, 最大值（防止过大或过小）
        )
        
        # HSV饱和度阈值:
        #   - 范围限制在[25.0, 120.0]
        #   实际计算示例（假设背景S均值=200，std_S=10）:
        #     阈值 = 25.0 + 2.0 * 10 = 45.0
        #     匹配范围 = [200-45, 200+45] = [155, 255]（饱和度高，接近纯色）
        sat_threshold = np.clip(
            self.sat_margin + self.sat_std_mul * std_hsv[1][0],
            # 实际值: 25.0 + 2.0 * std_hsv[1][0]
            # 示例: 如果std_S=10，则阈值=25.0+20.0=45.0（饱和度容差45）
            25.0, 120.0
        )
        
        # HSV亮度阈值:
        #   - 范围限制在[30.0, 140.0]
        #   实际计算示例（假设背景V均值=220，std_V=15）:
        #     阈值 = 35.0 + 2.0 * 15 = 65.0
        #     匹配范围 = [220-65, 220+65] = [155, 255]（亮度较高，适应光照变化）
        val_threshold = np.clip(
            self.val_margin + self.val_std_mul * std_hsv[2][0],
            # 实际值: 35.0 + 2.0 * std_hsv[2][0]
            # 示例: 如果std_V=15，则阈值=35.0+30.0=65.0（亮度容差65）
            30.0, 140.0
        )
        
        # ========== 步骤8: HSV颜色空间匹配 ==========
        # np.round()函数: 四舍五入到最近整数
        #   .astype(np.uint8): 转换为8位无符号整数（0-255）
        hue_mean = np.round(mean_hsv[0][0]).astype(np.uint8)
        
        # cv2.absdiff()函数: 计算两个数组的绝对差值
        #   函数语法:
        #     cv2.absdiff(src1, src2) -> dst
        #   参数:
        #     - src1: 第一个数组（H通道）
        #     - src2: 第二个数组（H均值，使用np.full_like创建相同形状的数组）
        #   返回: 每个像素的H值与H均值的绝对差
        hue_diff = cv2.absdiff(hsv_channels[0], np.full_like(hsv_channels[0], hue_mean))
        
        # HSV色调的周期性处理:
        #   色调是环形的，0和180相邻（红色在两端）
        #   例如: 色调10和色调170的距离应该是20，而不是160
        #   处理: 计算两种距离，取较小值
        #     距离1: abs(hue - mean_hue)
        #     距离2: 180 - abs(hue - mean_hue)  (绕环的距离)
        hue_wrap = cv2.absdiff(hue_diff, 180)  # 计算绕环的距离
        hue_diff = cv2.min(hue_diff, hue_wrap)  # 取两种距离的较小值
        
        # S和V通道直接计算差值（没有周期性）
        sat_mean = np.round(mean_hsv[1][0]).astype(np.uint8)
        sat_diff = cv2.absdiff(hsv_channels[1], np.full_like(hsv_channels[1], sat_mean))
        
        val_mean = np.round(mean_hsv[2][0]).astype(np.uint8)
        val_diff = cv2.absdiff(hsv_channels[2], np.full_like(hsv_channels[2], val_mean))
        
        # 布尔运算: 同时满足三个条件（H、S、V都在阈值内）
        #   & 运算符: 按位与，要求所有条件都为True
        #   说明: 像素必须同时满足色调、饱和度、亮度都在阈值内才被认为是背景
        hsv_mask = (hue_diff < hue_threshold) & (sat_diff < sat_threshold) & (val_diff < val_threshold)
        
        # 转换为二值图像: True -> 255（白色，背景），False -> 0（黑色，前景）
        bg_mask_hsv = (hsv_mask * 255).astype(np.uint8)
        
        # ========== 步骤9: Lab颜色空间匹配 ==========
        # cv2.split()函数: 分离Lab图像为L、a、b三个通道
        #   实际调用: cv2.split(lab)
        #   返回: (L通道, a通道, b通道) 三个单通道数组
        lab_channels = cv2.split(lab)
        
        # 初始化Lab距离数组（浮点型，用于精确计算）
        #   实际尺寸: 与缩放后图像相同（如512x384或640x480）
        lab_distance = np.zeros(scaled_image.shape[:2], dtype=np.float32)
        
        # 计算每个通道的归一化距离:
        #   normalized_distance = abs(channel - mean) / max(std, 2.0)
        #   说明:
        #     - 归一化使不同通道的距离可以相加
        #     - max(std, 2.0)避免除零，2.0是最小除数
        #     - 对三个通道分别计算，然后累加
        
        # 实际参数值（基于default.yaml）:
        #   - lab_threshold = 3.5（默认值）
        #   实际计算示例（假设背景Lab均值L=80, a=-50, b=40，标准差std_L=5, std_a=3, std_b=4）:
        #     对于像素值L=85, a=-47, b=43:
        #       L距离 = |85-80| / max(5, 2) = 5/5 = 1.0
        #       a距离 = |(-47)-(-50)| / max(3, 2) = 3/3 = 1.0
        #       b距离 = |43-40| / max(4, 2) = 3/4 = 0.75
        #       总距离 = (1.0 + 1.0 + 0.75) / 3 = 0.92 < 3.5，被认为是背景
        
        for i in range(3):  # i=0(L), 1(a), 2(b)
            channel_float = lab_channels[i].astype(np.float32)  # 转换为浮点型（精确计算）
            diff = np.abs(channel_float - mean_lab[i][0])  # 计算绝对差值
            denom = max(std_lab[i][0], 2.0)  # 分母（避免除零）
            lab_distance += diff / denom  # 累加归一化距离
        
        # 计算平均距离（三个通道的平均）
        lab_distance /= 3.0
        
        # Lab阈值限制在[2.0, 15.0]范围内
        #   实际值: lab_threshold = 3.5（默认值）
        #   说明: 阈值3.5表示平均归一化距离小于3.5的像素被认为是背景
        #   较小值（如2.0）: 更严格，只匹配非常接近背景色的像素
        #   较大值（如10.0）: 更宽松，匹配更多颜色的像素
        lab_threshold = np.clip(self.lab_threshold, 2.0, 15.0)
        
        # 生成Lab背景掩码: 距离小于阈值的是背景
        #   (lab_distance < lab_threshold): 布尔数组
        #   .astype(np.uint8): 转换为整数（True->1, False->0）
        #   * 255: 转换为二值图像（1->255白色，0->0黑色）
        #   实际调用示例: lab_mask = (lab_distance < 3.5).astype(np.uint8) * 255
        lab_mask = (lab_distance < lab_threshold).astype(np.uint8) * 255
        
        # 保存中间步骤
        if debug_images is not None:
            debug_images['hsv_mask'] = bg_mask_hsv.copy()
            debug_images['lab_mask'] = lab_mask.copy()
        
        # ========== 步骤10: 合并HSV和Lab掩码 ==========
        # cv2.bitwise_and()函数: 按位与运算
        #   函数语法:
        #     cv2.bitwise_and(src1, src2) -> dst
        #   说明: 两个掩码都认为是背景的像素才被认为是背景（取交集，更严格）
        #   优点: 结合两种颜色空间的优势，提高准确率
        bg_mask = cv2.bitwise_and(bg_mask_hsv, lab_mask)
        
        # ========== 步骤11: 经典绿色范围检测（可选）==========
        # cv2.inRange()函数: 范围检测
        #   函数语法:
        #     cv2.inRange(src, lowerb, upperb) -> dst
        #   参数:
        #     - src: 输入图像（HSV格式）
        #     - lowerb: 下界，格式为[H, S, V]
        #     - upperb: 上界，格式为[H, S, V]
        #   返回: 二值图像，在范围内的为255（白色），否则为0（黑色）
        #   说明: 这是传统的HSV绿色范围检测方法，作为补充
        
        # 实际参数值（基于default.yaml）:
        #   - enable_classic_hsv = true（默认启用）
        
        # 绿色HSV范围（硬编码值）:
        #   - H: [25, 95] 对应绿色色调范围（约60度±35度）
        #   - S: [25, 255] 饱和度范围（排除灰色，S<25为灰色）
        #   - V: [25, 255] 亮度范围（排除黑色，V<25为黑色）
        #   实际调用示例: cv2.inRange(hsv, [25, 25, 25], [95, 255, 255])
        #      - 输入: HSV格式图像，形状(H, W, 3)
        #      - 输出: 二值图像，形状(H, W)，255为绿色，0为非绿色
        #   说明: 这是一个固定的绿色范围，作为自适应检测的补充
        
        if self.enable_classic_hsv:
            lower_green = np.array([25, 25, 25])  # 下界 [H, S, V]
            upper_green = np.array([95, 255, 255])  # 上界 [H, S, V]
            # 实际调用: cv2.inRange(hsv, [25, 25, 25], [95, 255, 255])
            #   检测H在25-95范围内的像素（绿色色调）
            classic_green_mask = cv2.inRange(hsv, lower_green, upper_green)
            
            # cv2.bitwise_or()函数: 按位或运算
            #   说明: 合并经典检测结果（取并集，更宽松）
            #   好处: 捕获一些自适应方法可能遗漏的绿色区域
            bg_mask = cv2.bitwise_or(bg_mask, classic_green_mask)
        
        # ========== 步骤12: 形态学处理背景掩码 ==========
        # 形态学处理说明:
        #   1. 膨胀（Dilate）: 扩大前景区域，填充小孔洞
        #   2. 腐蚀（Erode）: 缩小前景区域，去除小噪声
        #   3. 先膨胀后腐蚀 = 闭运算（Close），用于填充内部孔洞
        
        # 实际参数值（基于default.yaml）:
        #   - cleanup_kernel = 7（默认值，7x7椭圆核）
        #   说明: 7x7的椭圆核，适合处理中等大小的噪声和孔洞
        
        # 确保核大小为奇数（形态学操作的常见要求）
        cleanup_kernel_size = int(round(self.cleanup_kernel))
        # 实际值: cleanup_kernel_size = 7（已经是奇数，不需要调整）
        if cleanup_kernel_size % 2 == 0:  # 如果是偶数
            cleanup_kernel_size += 1  # 加1变成奇数
        
        if cleanup_kernel_size >= 3:
            # cv2.getStructuringElement()函数: 创建形态学结构元素（核）
            #   函数语法:
            #     cv2.getStructuringElement(shape, ksize) -> element
            #   参数:
            #     - shape: 形状类型
            #       * cv2.MORPH_ELLIPSE: 椭圆形（适合圆形物体）
            #       * cv2.MORPH_RECT: 矩形
            #       * cv2.MORPH_CROSS: 十字形
            #     - ksize: 核大小，格式为(宽度, 高度)
            #   返回: 结构元素（核）
            #   实际调用示例: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            #      - 创建7x7的椭圆核
            #      - 形状: 椭圆形，适合圆形或类圆形物体
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cleanup_kernel_size, cleanup_kernel_size))
            
            # cv2.dilate()函数: 膨胀操作
            #   函数语法:
            #     cv2.dilate(src, kernel, iterations) -> dst
            #   参数:
            #     - src: 输入二值图像（背景掩码）
            #     - kernel: 结构元素（7x7椭圆核）
            #     - iterations: 迭代次数（执行膨胀的次数）= 1
            #   原理: 用核的最大值替换中心像素，扩大白色区域
            #   实际调用示例: cv2.dilate(bg_mask, 7x7椭圆核, iterations=1)
            #      - 输入: 背景掩码（255为背景，0为前景）
            #      - 输出: 膨胀后的背景掩码（背景区域扩大）
            bg_mask = cv2.dilate(bg_mask, kernel, iterations=1)
            
            # cv2.erode()函数: 腐蚀操作
            #   函数语法:
            #     cv2.erode(src, kernel, iterations) -> dst
            #   原理: 用核的最小值替换中心像素，缩小白色区域
            #   效果: 先膨胀后腐蚀，填充小孔洞并恢复大致边界
            #   实际调用示例: cv2.erode(bg_mask, 7x7椭圆核, iterations=1)
            #      - 输入: 膨胀后的背景掩码
            #      - 输出: 腐蚀后的背景掩码（背景区域缩小，但孔洞被填充）
            bg_mask = cv2.erode(bg_mask, kernel, iterations=1)
        
        # ========== 步骤13: 获取前景掩码 ==========
        # cv2.bitwise_not()函数: 按位非运算（取反）
        #   函数语法:
        #     cv2.bitwise_not(src) -> dst
        #   说明: 背景掩码取反，得到前景掩码
        #   结果: 背景(255) -> 前景(0), 前景(0) -> 背景(255)
        non_green = cv2.bitwise_not(bg_mask)
        
        # ========== 步骤14: 去除小噪声连通域 ==========
        # cv2.connectedComponentsWithStats()函数: 连通域分析
        #   函数语法:
        #     cv2.connectedComponentsWithStats(image, connectivity) -> (num_labels, labels, stats, centroids)
        #   参数:
        #     - image: 输入二值图像（前景掩码）
        #     - connectivity: 连通性
        #       * 4: 4邻域（上下左右）
        #       * 8: 8邻域（上下左右+对角线）
        #   返回:
        #     - num_labels: 连通域数量（包括背景0）
        #     - labels: 标签图像，每个像素标记为所属连通域的ID
        #     - stats: 统计信息，形状为(num_labels, 5)，每行包含：
        #       * stats[i, cv2.CC_STAT_LEFT]: 左边界
        #       * stats[i, cv2.CC_STAT_TOP]: 上边界
        #       * stats[i, cv2.CC_STAT_WIDTH]: 宽度
        #       * stats[i, cv2.CC_STAT_HEIGHT]: 高度
        #       * stats[i, cv2.CC_STAT_AREA]: 面积（像素数）
        #     - centroids: 质心坐标，形状为(num_labels, 2)
        #   说明: 用于找到所有连通的前景区域，并计算每个区域的面积
        
        # 实际参数值（基于default.yaml）:
        #   - min_noise_area = 100（默认值，最小噪声面积100像素²）
        #   说明: 面积小于100像素的连通域被认为是噪声，会被去除
        
        if self.min_noise_area > 0:
            # 连通域分析（8邻域）
            #   实际调用示例: cv2.connectedComponentsWithStats(non_green, connectivity=8)
            #      - 输入: 前景掩码（255为前景，0为背景）
            #      - 连通性: 8邻域（包括对角线）
            #      - 返回: 连通域数量、标签图像、统计信息、质心坐标
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(non_green, connectivity=8)
            
            # 创建清理后的前景掩码
            cleaned_fg = np.zeros_like(non_green)  # np.zeros_like(): 创建与输入相同形状的零数组
            
            # 遍历所有连通域（跳过背景标签0）
            for i in range(1, num_labels):  # i=1,2,3,...,num_labels-1
                area = stats[i, cv2.CC_STAT_AREA]  # 获取连通域i的面积
                
                # 只保留面积大于阈值的连通域（去除小噪声）
                if area >= self.min_noise_area:
                    # 布尔索引: labels == i 返回属于连通域i的像素位置
                    #   将这些像素设为255（白色，前景）
                    cleaned_fg[labels == i] = 255
            
            non_green = cleaned_fg  # 更新前景掩码
        
        # ========== 步骤15: 前景形态学处理 ==========
        # 可选的前腐蚀操作: 如果启用，先缩小前景区域（去除小突起）
        if self.erode_before_dilate:
            fg_erode_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))  # 创建5x5椭圆核
            non_green = cv2.erode(non_green, fg_erode_k, iterations=1)  # 腐蚀1次
        
        # 保存原始前景掩码（用于后续恢复）
        original_fg = non_green.copy()  # .copy(): 创建副本，避免引用问题
        
        # ========== 步骤16: 形态学闭运算 ==========
        # 闭运算（Close）: 先膨胀后腐蚀，用于填充前景内部的孔洞
        #   原理: 先膨胀扩大前景，然后腐蚀恢复大致边界，内部孔洞被填充
        #   优点: 填充前景内部的黑色孔洞，得到更完整的前景区域
        
        # 实际参数值（基于default.yaml）:
        #   - foreground_close_kernel = 9（默认值，9x9椭圆核）
        #   说明: 9x9的椭圆核，执行闭运算2次，填充中等大小的孔洞
        
        # 限制核大小在合理范围内
        fg_close_kernel_size = int(round(self.foreground_close_kernel))
        # 实际值: fg_close_kernel_size = 9（已经是奇数，不需要调整）
        if fg_close_kernel_size > 9:  # 最大限制为9
            fg_close_kernel_size = 9
        if fg_close_kernel_size % 2 == 0 and fg_close_kernel_size > 0:  # 必须为奇数
            fg_close_kernel_size += 1
        
        if fg_close_kernel_size >= 3:
            # 创建椭圆核（适合圆形或类圆形物体）
            #   实际调用示例: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            #      - 创建9x9的椭圆核
            fg_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (fg_close_kernel_size, fg_close_kernel_size))
            
            # cv2.morphologyEx()函数: 形态学操作
            #   函数语法:
            #     cv2.morphologyEx(src, op, kernel, iterations) -> dst
            #   参数:
            #     - src: 输入二值图像（前景掩码）
            #     - op: 操作类型
            #       * cv2.MORPH_CLOSE: 闭运算（先膨胀后腐蚀）
            #       * cv2.MORPH_OPEN: 开运算（先腐蚀后膨胀）
            #       * cv2.MORPH_DILATE: 膨胀
            #       * cv2.MORPH_ERODE: 腐蚀
            #     - kernel: 结构元素（9x9椭圆核）
            #     - iterations: 迭代次数 = 2
            #   说明: 执行2次闭运算，更彻底地填充孔洞
            #   实际调用示例: cv2.morphologyEx(non_green, cv2.MORPH_CLOSE, 9x9椭圆核, iterations=2)
            #      - 输入: 前景掩码（可能有孔洞）
            #      - 输出: 闭运算后的前景掩码（孔洞被填充）
            #      - 效果: 执行2次闭运算，填充最大约9像素的孔洞
            non_green = cv2.morphologyEx(non_green, cv2.MORPH_CLOSE, fg_kernel, iterations=2)
            
            # 与原始前景掩码合并: 保留原始前景，填充内部孔洞
            #   说明: 闭运算可能改变边界，与原始掩码合并确保不丢失原始前景区域
            non_green = cv2.bitwise_or(non_green, original_fg)
        
        # ========== 步骤17: 中值滤波 ==========
        # cv2.medianBlur()函数: 中值滤波（去除椒盐噪声）
        #   函数语法:
        #     cv2.medianBlur(src, ksize) -> dst
        #   参数:
        #     - src: 输入图像（单通道或多通道）
        #     - ksize: 核大小，必须是奇数（3, 5, 7, ...）
        #   原理: 用邻域中值替换中心像素，去除孤立的噪声点
        #   优点: 对椒盐噪声特别有效，不会模糊边缘
        
        # 实际参数值（基于default.yaml）:
        #   - median_ksize = 5（默认值，5x5中值滤波核）
        #   说明: 5x5的中值滤波核，适合去除中等大小的椒盐噪声
        
        median_kernel = int(round(self.median_ksize))
        # 实际值: median_kernel = 5（已经是奇数，不需要调整）
        if median_kernel > 5:  # 最大限制为5（避免过度模糊）
            median_kernel = 5
        if median_kernel % 2 == 0 and median_kernel > 0:  # 必须为奇数
            median_kernel += 1
        
        if median_kernel >= 3:
            # 中值滤波: 去除椒盐噪声，平滑前景掩码
            #   实际调用示例: cv2.medianBlur(non_green, 5)
            #      - 输入: 前景掩码（255为前景，0为背景）
            #      - 核大小: 5x5
            #      - 输出: 中值滤波后的前景掩码（去除椒盐噪声）
            non_green = cv2.medianBlur(non_green, median_kernel)
        
        # 保存中间步骤图像（用于调试）- 在恢复到原始尺寸之前保存
        if debug_images is not None:
            debug_images['scaled_image'] = scaled_image.copy()
            debug_images['border_mask'] = border_mask.copy()
            debug_images['bg_mask_after_morph'] = bg_mask.copy()  # 形态学处理后的背景掩码
            debug_images['non_green_before_noise'] = cv2.bitwise_not(bg_mask).copy()  # 去噪前的前景
            debug_images['non_green_after_noise'] = non_green.copy()  # 去噪后的前景（形态学处理前）
        
        # 恢复到原始尺寸
        if self.scale_factor < 1.0:
            non_green = cv2.resize(non_green, (original_size[1], original_size[0]), interpolation=cv2.INTER_NEAREST)
            # 同时调整调试图像到原始尺寸
            if debug_images is not None:
                resize_keys = ['scaled_image', 'border_mask', 'hsv_mask', 'lab_mask', 
                              'bg_mask_after_morph', 'non_green_before_noise', 'non_green_after_noise']
                for key in resize_keys:
                    if key in debug_images:
                        debug_images[key] = cv2.resize(debug_images[key], 
                                                       (original_size[1], original_size[0]), 
                                                       interpolation=cv2.INTER_NEAREST)
        
        return non_green
    
    def extract_connected_components(self, binary_mask: np.ndarray, debug_images: dict = None) -> List[np.ndarray]:
        """
        提取连通域
        
        实现与C++ Preprocessor::extractConnectedComponents和filterComponents相同的逻辑：
        1. 使用connectedComponentsWithStats提取所有连通域
        2. 根据面积、宽高比、尺寸等条件筛选
        3. 限制连通域数量（保留最大的N个）
        
        函数语法:
            extract_connected_components(binary_mask, debug_images=None)
        
        参数说明:
            binary_mask (np.ndarray): 
                - 类型: numpy数组，形状为(H, W)
                - 格式: 二值图像，数据类型为uint8
                - 说明: 前景掩码，255为前景，0为背景
                - 要求: 图像不能为空，必须是单通道二值图像
            
            debug_images (dict, optional): 
                - 类型: 字典，默认值为None
                - 说明: 用于保存中间处理结果的字典
                - 如果提供，会保存以下键值对：
                  * 'all_components_before_filter' (np.ndarray): 筛选前的所有连通域可视化
                  * 'components_after_filter' (np.ndarray): 筛选后的连通域可视化
        
        返回:
            List[np.ndarray]: 
                - 类型: 列表，元素为numpy数组
                - 格式: 每个元素是一个二值掩码图像（uint8），255为连通域，0为背景
                - 说明: 筛选后的连通域列表，按面积从大到小排序
                - 数量: 最多返回component_max_count个连通域
        
        算法原理:
            1. 连通域分析: 使用8邻域连接分析所有前景区域
            2. 多条件筛选: 根据面积、宽高比、尺寸筛选
            3. 数量限制: 按面积排序，保留最大的N个
        
        使用的关键参数（来自self）:
            - self.component_min_area (int): 最小面积，默认1500，范围[0, 100000]
            - self.component_max_area (int): 最大面积，默认80000，范围[0, 1000000]
            - self.component_min_aspect_ratio (float): 最小宽高比，默认0.3，范围[0.1, 1.0]
            - self.component_max_aspect_ratio (float): 最大宽高比，默认4.0，范围[1.0, 10.0]
            - self.component_min_width (int): 最小宽度，默认60，范围[10, 500]
            - self.component_min_height (int): 最小高度，默认60，范围[10, 500]
            - self.component_max_count (int): 最大数量，默认3，范围[1, 10]
        
        筛选条件说明:
            1. 面积筛选: area >= min_area and area <= max_area
            2. 宽高比筛选: min_ratio <= aspect_ratio <= max_ratio
               aspect_ratio = min(width, height) / max(width, height)
            3. 尺寸筛选: width >= min_width and height >= min_height
            4. 数量限制: 按面积排序，保留最大的N个
        
        示例:
            >>> preprocessor = PreprocessorDebugger(config)
            >>> foreground_mask = preprocessor.remove_green_background(image)
            >>> debug_images = {}
            >>> components = preprocessor.extract_connected_components(foreground_mask, debug_images)
            >>> print(f"找到 {len(components)} 个连通域")
            >>> # 现在debug_images包含了筛选前后的可视化结果
        
        注意事项:
            - 连通域分析使用8邻域（包括对角线）
            - 筛选后的连通域按面积从大到小排序
            - 最多返回component_max_count个连通域
            - 背景标签0会被自动跳过
        """
        # ========== 步骤1: 连通域分析 ==========
        # cv2.connectedComponentsWithStats()函数详细说明见前面注释
        #   返回值:
        #     - num_labels: 连通域数量（包括背景标签0）
        #     - labels: 标签图像，每个像素标记为所属连通域的ID（0为背景）
        #     - stats: 统计信息，形状为(num_labels, 5)
        #     - centroids: 质心坐标，形状为(num_labels, 2)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)
        components = []  # 存储筛选后的连通域
        
        # ========== 步骤2: 保存筛选前的所有连通域（用于调试）==========
        if debug_images is not None:
            all_components_vis = np.zeros_like(binary_mask)  # 创建全零图像
            # 遍历所有连通域（跳过背景标签0）
            for i in range(1, num_labels):  # i=1,2,3,...,num_labels-1
                # 布尔索引: labels == i 返回属于连通域i的所有像素位置
                #   将这些像素设为255（白色），可视化该连通域
                all_components_vis[labels == i] = 255
            debug_images['all_components_before_filter'] = all_components_vis.copy()
        
        # ========== 步骤3: 多条件筛选连通域 ==========
        # 遍历所有连通域（跳过背景标签0）
        for i in range(1, num_labels):  # i=1,2,3,...,num_labels-1
            # 获取连通域i的统计信息
            area = stats[i, cv2.CC_STAT_AREA]        # 面积（像素数）
            width = stats[i, cv2.CC_STAT_WIDTH]      # 宽度（像素）
            height = stats[i, cv2.CC_STAT_HEIGHT]    # 高度（像素）
            
            # 条件1: 面积筛选
            #   说明: 面积太小可能是噪声，太大可能是多个物体粘连
            if area < self.component_min_area or area > self.component_max_area:
                continue  # 跳过不符合条件的连通域
            
            # 条件2: 宽高比筛选
            #   宽高比 = 短边 / 长边
            #   说明: 宽高比接近1表示接近正方形，过小或过大表示过于细长
            
            # 实际参数值（基于default.yaml）:
            #   - component_min_aspect_ratio = 0.3（默认值，最小宽高比0.3）
            #   - component_max_aspect_ratio = 4.0（默认值，最大宽高比4.0）
            #   实际判断示例:
            #     宽度100，高度50 -> aspect_ratio = 50/100 = 0.5，在[0.3, 4.0]范围内，保留
            #     宽度100，高度20 -> aspect_ratio = 20/100 = 0.2，小于0.3，跳过（过于细长）
            #     宽度100，高度400 -> aspect_ratio = 100/400 = 0.25，小于0.3，跳过（过于细长）
            #     宽度80，高度80 -> aspect_ratio = 80/80 = 1.0，在[0.3, 4.0]范围内，保留（接近正方形）
            aspect_ratio = min(width, height) / max(width, height) if max(width, height) > 0 else 0
            if aspect_ratio < self.component_min_aspect_ratio or aspect_ratio > self.component_max_aspect_ratio:
                continue  # 跳过不符合条件的连通域
            
            # 条件3: 尺寸筛选
            #   说明: 宽度和高度必须都大于最小值，避免过小的连通域
            
            # 实际参数值（基于default.yaml）:
            #   - component_min_width = 60（默认值，最小宽度60像素）
            #   - component_min_height = 60（默认值，最小高度60像素）
            #   实际判断示例:
            #     宽度80，高度100 -> 都大于60，保留
            #     宽度50，高度100 -> 宽度小于60，跳过（太小）
            #     宽度100，高度40 -> 高度小于60，跳过（太小）
            if width < self.component_min_width or height < self.component_min_height:
                continue  # 跳过不符合条件的连通域
            
            # ========== 提取单个连通域 ==========
            # 创建新的二值图像，只包含当前连通域
            component = np.zeros_like(binary_mask)  # 创建全零图像
            component[labels == i] = 255  # 将连通域i的像素设为255（白色）
            components.append(component)  # 添加到列表
        
        # ========== 步骤4: 限制连通域数量 ==========
        # 如果连通域数量超过限制，只保留面积最大的N个
        if len(components) > self.component_max_count:
            # cv2.countNonZero()函数: 统计非零像素数量（连通域面积）
            #   函数语法:
            #     cv2.countNonZero(src) -> count
            #   参数:
            #     - src: 输入二值图像
            #   返回: 非零像素数量（面积）
            
            # 计算每个连通域的面积
            areas = [cv2.countNonZero(c) for c in components]
            
            # sorted()函数: 排序
            #   函数语法:
            #     sorted(iterable, key=None, reverse=False) -> list
            #   参数:
            #     - iterable: 可迭代对象（索引列表）
            #     - key: 排序键函数，lambda i: areas[i] 表示按面积排序
            #     - reverse=True: 降序排列（从大到小）
            #   返回: 排序后的索引列表
            sorted_indices = sorted(range(len(areas)), key=lambda i: areas[i], reverse=True)
            
            # 只保留面积最大的N个连通域
            #   列表切片: [:self.component_max_count] 取前N个
            components = [components[i] for i in sorted_indices[:self.component_max_count]]
        
        # 保存筛选后的连通域（用于调试）
        if debug_images is not None:
            filtered_components_vis = np.zeros_like(binary_mask)
            for component in components:
                filtered_components_vis = cv2.bitwise_or(filtered_components_vis, component)
            debug_images['components_after_filter'] = filtered_components_vis.copy()
        
        return components
    
    def preprocess(self, image: np.ndarray, debug_images: dict = None) -> List[np.ndarray]:
        """
        预处理图像的主函数
        
        实现与C++ Preprocessor::preprocess相同的处理流程：
        1. 保存原始图像
        2. 去除绿色背景
        3. 提取连通域
        4. 筛选连通域
        5. 恢复到原始尺寸
        
        函数语法:
            preprocess(image, debug_images=None)
        
        参数说明:
            image (np.ndarray): 
                - 类型: numpy数组，形状为(H, W, 3)
                - 格式: BGR颜色空间
                - 说明: 输入的彩色图像，需要预处理
                - 要求: 图像不能为空，必须是3通道BGR图像
            
            debug_images (dict, optional): 
                - 类型: 字典，默认值为None
                - 说明: 用于保存所有中间处理结果的字典
                - 如果提供，会保存以下键值对：
                  * 'original_image' (np.ndarray): 原始输入图像
                  * 'scaled_image' (np.ndarray): 缩放后的图像
                  * 'border_mask' (np.ndarray): 边缘掩码
                  * 'hsv_mask' (np.ndarray): HSV背景掩码
                  * 'lab_mask' (np.ndarray): Lab背景掩码
                  * 'bg_mask_after_morph' (np.ndarray): 形态学处理后的背景掩码
                  * 'non_green_before_noise' (np.ndarray): 去噪前的前景掩码
                  * 'non_green_after_noise' (np.ndarray): 去噪后的前景掩码
                  * 'foreground_mask' (np.ndarray): 最终前景掩码
                  * 'all_components_before_filter' (np.ndarray): 筛选前的所有连通域
                  * 'components_after_filter' (np.ndarray): 筛选后的连通域
                  * 'components' (List[np.ndarray]): 连通域列表
        
        返回:
            List[np.ndarray]: 
                - 类型: 列表，元素为numpy数组
                - 格式: 每个元素是一个二值掩码图像（uint8），255为连通域，0为背景
                - 说明: 预处理后的连通域列表（已恢复到原始尺寸）
                - 尺寸: 每个连通域掩码的尺寸与原始图像相同
                - 数量: 最多返回component_max_count个连通域
                - 异常: 如果处理失败或没有前景，返回空列表[]
        
        处理流程:
            1. 保存原始图像到debug_images（用于调试）
            2. 调用remove_green_background()去除绿色背景
               - 图像缩放（如果scale_factor < 1.0）
               - HSV和Lab颜色空间背景检测
               - 形态学处理和噪声去除
               - 恢复到原始尺寸
            3. 调用extract_connected_components()提取连通域
               - 连通域分析（8邻域）
               - 多条件筛选（面积、宽高比、尺寸）
               - 数量限制（保留最大的N个）
            4. 保存所有中间结果到debug_images（用于可视化）
        
        算法复杂度:
            - 时间复杂度: O(H * W)，其中H和W是图像高度和宽度
            - 空间复杂度: O(H * W)，需要存储多个中间掩码
        
        示例:
            >>> preprocessor = PreprocessorDebugger(config)
            >>> image = cv2.imread('test.jpg')
            >>> debug_images = {}
            >>> components = preprocessor.preprocess(image, debug_images)
            >>> print(f"找到 {len(components)} 个连通域")
            >>> print(f"调试图像数量: {len(debug_images)}")
            >>> # 现在debug_images包含了所有中间处理结果
        
        注意事项:
            - 如果scale_factor < 1.0，处理时会先缩小图像，最后恢复到原始尺寸
            - 所有中间掩码也会恢复到原始尺寸（在debug_images中）
            - 如果背景去除失败（返回空掩码），返回空列表
            - 连通域数量限制为component_max_count个（按面积从大到小）
            - 如果debug_images为None，会自动创建空字典（不保存调试图像）
        
        与C++代码的对应关系:
            - 对应C++ Preprocessor::preprocess()函数
            - 处理流程完全一致（确保Python和C++结果相同）
            - 参数名称和默认值与C++代码保持一致
        """
        if debug_images is None:
            debug_images = {}
        
        # 保存原始图像
        debug_images['original_image'] = image.copy()
        
        foreground_mask = self.remove_green_background(image, debug_images)
        if foreground_mask is None or foreground_mask.size == 0:
            return []
        
        debug_images['foreground_mask'] = foreground_mask.copy()
        
        components = self.extract_connected_components(foreground_mask, debug_images)
        debug_images['components'] = components
        
        return components


class FeatureExtractorDebugger:
    """
    特征提取调试器类
    
    实现与C++ FeatureExtractor类相同的特征提取逻辑，包括：
    - 工件外接圆提取（大圆）
    - 阀体外接圆提取（小圆）
    - 标准化角度计算
    
    该类用于调试和可视化特征提取过程，允许实时调整参数。
    
    属性:
        config (Dict): 配置参数字典
        feature_config (Dict): 特征提取配置
        min_component_area (int): 最小连通域面积
        max_component_area (int): 最大连通域面积
        big_circle_combine_contours (bool): 是否合并轮廓
        big_circle_min_area (int): 大圆最小轮廓面积
        small_circle_erode_kernel (int): 小圆腐蚀核大小
        ... (其他参数)
    """
    
    def __init__(self, config: Dict):
        """
        初始化特征提取调试器
        
        从配置字典中加载所有特征提取参数，如果参数不存在则使用默认值。
        
        参数:
            config (Dict): 配置参数字典，应包含'preprocess.feature_extraction'键
        """
        self.config = config
        self.feature_config = config.get('preprocess', {}).get('feature_extraction', {})
        
        # 特征提取参数
        self.min_component_area = self.feature_config.get('min_component_area', 2000)
        self.max_component_area = self.feature_config.get('max_component_area', 10000000)
        self.big_circle_combine_contours = self.feature_config.get('big_circle', {}).get('combine_contours', True)
        self.big_circle_min_area = self.feature_config.get('big_circle', {}).get('min_area', 100)
        self.small_circle_erode_kernel = self.feature_config.get('small_circle', {}).get('erode_kernel', 11)
        self.small_circle_erode_iterations = self.feature_config.get('small_circle', {}).get('erode_iterations', 5)
        self.small_circle_largest_cc = self.feature_config.get('small_circle', {}).get('largest_cc', True)
        self.small_circle_dilate_kernel = self.feature_config.get('small_circle', {}).get('dilate_kernel', 9)
        self.small_circle_dilate_iterations = self.feature_config.get('small_circle', {}).get('dilate_iterations', 1)
    
    def extract_workpiece_circle(self, mask: np.ndarray, debug_images: dict = None, component_idx: int = 0) -> Tuple[Tuple[float, float], float]:
        """
        提取工件外接圆（大圆）
        
        实现与C++ FeatureExtractor::extractWorkpieceCircle相同的算法：
        1. 查找轮廓
        2. 可选地合并多个轮廓
        3. 计算最小外接圆
        
        函数语法:
            extract_workpiece_circle(mask, debug_images=None, component_idx=0)
        
        参数说明:
            mask (np.ndarray): 
                - 类型: numpy数组，形状为(H, W)
                - 格式: 二值掩码图像，数据类型为uint8
                - 说明: 连通域掩码，255为连通域，0为背景
                - 要求: 图像不能为空，必须是单通道二值图像
            
            debug_images (dict, optional): 
                - 类型: 字典，默认值为None
                - 说明: 用于保存中间处理结果的字典
                - 如果提供，会保存以下键值对：
                  * f'wp_contours_{component_idx}' (np.ndarray): 轮廓可视化图像
                    - 形状: (H, W, 3)，BGR格式
                    - 内容: 在原掩码上绘制轮廓（黄色线条）
                  * f'wp_circle_{component_idx}' (np.ndarray): 外接圆可视化图像
                    - 形状: (H, W, 3)，BGR格式
                    - 内容: 在原掩码上绘制外接圆（绿色圆圈）
            
            component_idx (int): 
                - 类型: 整数，默认值为0
                - 说明: 连通域索引，用于调试图像键名（区分不同的连通域）
                - 范围: [0, component_max_count-1]
        
        返回:
            Tuple[Tuple[float, float], float]: 
                - 类型: 元组，格式为((center_x, center_y), radius)
                - 说明:
                  * center_x (float): 外接圆中心x坐标（像素）
                  * center_y (float): 外接圆中心y坐标（像素）
                  * radius (float): 外接圆半径（像素）
                - 异常: 如果未找到轮廓或计算失败，返回((0, 0), 0.0)
        
        算法原理:
            1. 轮廓检测: 使用cv2.findContours查找连通域的外边界
            2. 轮廓过滤: 过滤面积小于阈值的轮廓
            3. 轮廓合并: 可选地将多个轮廓合并为一个（适用于工件由多个部分组成）
            4. 外接圆计算: 使用cv2.minEnclosingCircle计算最小外接圆
               - 算法: Welzl算法（线性时间算法）
               - 原理: 找到能包含所有轮廓点的最小圆
        
        使用的关键参数（来自self）:
            - self.big_circle_combine_contours (bool): 是否合并轮廓，默认True
            - self.big_circle_min_area (int): 最小轮廓面积，默认100，范围[0, 10000]
        
        示例:
            >>> extractor = FeatureExtractorDebugger(config)
            >>> component = components[0]  # 第一个连通域
            >>> debug_images = {}
            >>> (center, radius) = extractor.extract_workpiece_circle(component, debug_images, 0)
            >>> print(f"工件中心: {center}, 半径: {radius}")
            >>> # 现在debug_images包含了轮廓和外接圆的可视化
        
        注意事项:
            - 外接圆可能比连通域实际面积大（因为圆是包含所有轮廓点的最小圆）
            - 如果启用轮廓合并，多个轮廓会被合并为一个轮廓后再计算外接圆
            - 只考虑面积大于big_circle_min_area的轮廓
            - 如果有多个轮廓，选择面积最大的轮廓计算外接圆
        """
        if mask is None or mask.size == 0:
            return ((0, 0), 0.0)
        
        # ========== 步骤1: 查找轮廓 ==========
        # cv2.findContours()函数: 查找轮廓
        #   函数语法:
        #     cv2.findContours(image, mode, method) -> (contours, hierarchy)
        #   参数:
        #     - image: 输入二值图像（uint8）
        #     - mode: 轮廓检索模式
        #       * cv2.RETR_EXTERNAL: 只检索最外层轮廓（不检测内部孔洞）
        #       * cv2.RETR_LIST: 检索所有轮廓，不建立层次关系
        #       * cv2.RETR_TREE: 检索所有轮廓，建立完整的层次结构
        #     - method: 轮廓近似方法
        #       * cv2.CHAIN_APPROX_NONE: 存储所有轮廓点
        #       * cv2.CHAIN_APPROX_SIMPLE: 压缩轮廓，只保留端点（节省内存）
        #   返回:
        #     - contours: 轮廓列表，每个轮廓是一个点集（numpy数组）
        #     - hierarchy: 层次信息（这里不使用，用_忽略）
        #   说明: RETR_EXTERNAL只找外轮廓，适合连通域分析
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 如果没有找到轮廓，返回默认值
        if not contours:
            return ((0, 0), 0.0)
        
        # 筛选前不绘制轮廓（原步骤2: 保存轮廓可视化 已移除）
        # 若需恢复轮廓可视化，可在此处根据筛选后的 contours 再绘制
        
        # ========== 步骤3: 轮廓过滤和合并 ==========
        # cv2.contourArea()函数: 计算轮廓面积
        #   函数语法:
        #     cv2.contourArea(contour) -> area
        #   参数:
        #     - contour: 轮廓点集（numpy数组）
        #   返回: 轮廓面积（像素数）
        #   说明: 使用格林公式计算轮廓包围的面积
        
        # 如果启用轮廓合并且轮廓数量大于1
        if self.big_circle_combine_contours and len(contours) > 1:
            # 过滤小轮廓（面积小于阈值的轮廓）
            #   列表推导式: [表达式 for 项 in 列表 if 条件]
            #   说明: 创建一个新列表，只包含面积大于等于阈值的轮廓
            filtered_contours = [c for c in contours if cv2.contourArea(c) >= self.big_circle_min_area]
            
            if filtered_contours:
                # np.vstack()函数: 垂直堆叠数组
                #   函数语法:
                #     np.vstack(tup) -> stacked_array
                #   参数:
                #     - tup: 数组序列（列表）
                #   返回: 堆叠后的数组
                #   说明: 将所有轮廓的点合并为一个点集
                #   例如: 如果有3个轮廓，每个轮廓有N个点，合并后总共有3N个点
                all_points = np.vstack(filtered_contours)
                contours = [all_points]  # 将合并后的点集作为唯一的轮廓
        else:
            # 不合并轮廓，只过滤小轮廓
            #   说明: 创建一个新列表，只包含面积大于等于阈值的轮廓
            contours = [c for c in contours if cv2.contourArea(c) >= self.big_circle_min_area]
        
        # 如果没有符合条件的轮廓，返回默认值
        if not contours:
            return ((0, 0), 0.0)
        
        # ========== 步骤4: 找到最大的轮廓 ==========
        # max()函数: 返回最大值
        #   函数语法:
        #     max(iterable, key=None)
        #   参数:
        #     - iterable: 可迭代对象（轮廓列表）
        #     - key: 键函数，cv2.contourArea表示按面积排序
        #   返回: 面积最大的轮廓
        #   说明: 如果有多个轮廓，选择面积最大的（通常是主要部分）
        largest_contour = max(contours, key=cv2.contourArea)
        
        # ========== 步骤5: 计算最小外接圆 ==========
        # cv2.minEnclosingCircle()函数: 计算最小外接圆
        #   函数语法:
        #     cv2.minEnclosingCircle(points) -> ((center_x, center_y), radius)
        #   参数:
        #     - points: 点集（轮廓点），numpy数组，形状为(N, 1, 2)
        #   返回:
        #     - (center_x, center_y): 圆心坐标（浮点数）
        #     - radius: 半径（浮点数）
        #   算法: Welzl算法（线性时间算法）
        #   原理: 找到能包含所有点的最小圆（最小覆盖圆问题）
        #   说明: 返回的圆是所有包含轮廓点的圆中最小的
        
        # 实际调用示例: cv2.minEnclosingCircle(largest_contour)
        #   输入: 轮廓点集（形状为(N, 1, 2)，N个点）
        #   输出: ((center_x, center_y), radius)
        #   示例: 如果轮廓是圆形工件（直径100像素）
        #     返回: ((320.5, 240.3), 50.2)
        #     说明: 圆心在(320.5, 240.3)，半径50.2像素
        #   算法复杂度: O(N)，其中N是轮廓点数量
        
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        
        # 保存工件外接圆可视化（用于调试）
        if debug_images is not None:
            circle_vis = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            circle_vis = mask_bgr.copy()
            if radius > 0:
                cv2.circle(circle_vis, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(circle_vis, (int(x), int(y)), 5, (0, 255, 0), -1)
            debug_images[f'wp_circle_{component_idx}'] = circle_vis
        
        return ((float(x), float(y)), float(radius))
    
    def extract_valve_circle(self, mask: np.ndarray, debug_images: dict = None, component_idx: int = 0) -> Tuple[Tuple[float, float], float, np.ndarray]:
        """
        提取阀体外接圆（小圆）
        
        实现与C++ FeatureExtractor::extractValveCircle相同的算法：
        1. 腐蚀操作（去除阀体周围的小细节）
        2. 可选地保留最大连通域
        3. 膨胀操作（恢复阀体大小）
        4. 计算最小外接圆
        
        函数语法:
            extract_valve_circle(mask, debug_images=None, component_idx=0)
        
        参数说明:
            mask (np.ndarray): 
                - 类型: numpy数组，形状为(H, W)
                - 格式: 二值掩码图像，数据类型为uint8
                - 说明: 连通域掩码，255为连通域，0为背景
                - 要求: 图像不能为空，必须是单通道二值图像
            
            debug_images (dict, optional): 
                - 类型: 字典，默认值为None
                - 说明: 用于保存中间处理结果的字典
                - 如果提供，会保存以下键值对：
                  * f'valve_original_{component_idx}' (np.ndarray): 原始掩码
                    - 形状: (H, W)，二值图像
                  * f'valve_eroded_{component_idx}' (np.ndarray): 腐蚀后的掩码
                    - 形状: (H, W)，二值图像
                  * f'valve_largest_cc_{component_idx}' (np.ndarray): 最大连通域掩码
                    - 形状: (H, W)，二值图像
                  * f'valve_dilated_{component_idx}' (np.ndarray): 膨胀后的掩码
                    - 形状: (H, W)，二值图像
                  * f'valve_circle_{component_idx}' (np.ndarray): 外接圆可视化图像
                    - 形状: (H, W, 3)，BGR格式
                    - 内容: 在原掩码上绘制外接圆（蓝色圆圈）
            
            component_idx (int): 
                - 类型: 整数，默认值为0
                - 说明: 连通域索引，用于调试图像键名
                - 范围: [0, component_max_count-1]
        
        返回:
            Tuple[Tuple[float, float], float, np.ndarray]: 
                - 类型: 元组，格式为((center_x, center_y), radius, processed_mask)
                - 说明:
                  * center_x (float): 外接圆中心x坐标（像素）
                  * center_y (float): 外接圆中心y坐标（像素）
                  * radius (float): 外接圆半径（像素）
                  * processed_mask (np.ndarray): 处理后的掩码，形状为(H, W)
                - 异常: 如果未找到轮廓或计算失败，返回((0, 0), 0.0, empty_array)
        
        算法原理:
            1. 腐蚀操作: 缩小阀体区域，去除周围的小细节和噪声
               - 原理: 用结构元素的最小值替换中心像素，缩小前景区域
               - 效果: 阀体缩小，小突起被去除
            2. 最大连通域选择: 可选地只保留面积最大的连通域
               - 原理: 阀体通常是一个完整的连通域，去除其他小区域
               - 效果: 去除阀体周围可能残留的小噪声区域
            3. 膨胀操作: 恢复阀体大小，补偿腐蚀造成的缩小
               - 原理: 用结构元素的最大值替换中心像素，扩大前景区域
               - 效果: 阀体恢复到接近原始大小
            4. 外接圆计算: 使用cv2.minEnclosingCircle计算最小外接圆
               - 原理: 找到能包含所有轮廓点的最小圆
               - 效果: 得到阀体的外接圆（用于定位和方向计算）
        
        使用的关键参数（来自self）:
            - self.small_circle_erode_kernel (int): 腐蚀核大小，默认11，必须为奇数，范围[3, 31]
            - self.small_circle_erode_iterations (int): 腐蚀次数，默认5，范围[1, 20]
            - self.small_circle_largest_cc (bool): 是否只保留最大连通域，默认True
            - self.small_circle_dilate_kernel (int): 膨胀核大小，默认9，必须为奇数，范围[3, 31]
            - self.small_circle_dilate_iterations (int): 膨胀次数，默认1，范围[1, 20]
        
        为什么需要腐蚀和膨胀:
            - 腐蚀去除小细节: 阀体周围可能有小的突起或噪声，腐蚀可以去除
            - 膨胀恢复大小: 腐蚀会缩小阀体，膨胀可以恢复大致大小
            - 这种组合称为"开运算"（Open）的反向操作，用于清理小细节
        
        示例:
            >>> extractor = FeatureExtractorDebugger(config)
            >>> component = components[0]  # 第一个连通域
            >>> debug_images = {}
            >>> (center, radius, processed_mask) = extractor.extract_valve_circle(component, debug_images, 0)
            >>> print(f"阀体中心: {center}, 半径: {radius}")
            >>> # 现在debug_images包含了所有中间处理步骤的可视化
        
        注意事项:
            - 腐蚀和膨胀的次数需要平衡：腐蚀太多会丢失阀体信息，太少无法去除噪声
            - 膨胀次数通常小于腐蚀次数，因为主要目的是恢复大小而不是扩大
            - 核大小必须是奇数（OpenCV要求）
            - 如果启用最大连通域选择，会去除阀体周围可能的小噪声区域
        """
        if mask is None or mask.size == 0:
            return ((0, 0), 0.0, np.array([]))
        
        # 保存原始掩码（用于调试）
        if debug_images is not None:
            debug_images[f'valve_original_{component_idx}'] = mask.copy()
        
        # ========== 步骤1: 腐蚀操作（去除小细节）==========
        # 腐蚀操作原理:
        #   1. 使用结构元素（核）扫描图像
        #   2. 用核的最小值替换中心像素
        #   3. 结果: 前景区域缩小，小突起被去除
        #   应用: 去除阀体周围的小细节和噪声
        
        # 确保核大小为奇数
        erode_kernel_size = int(round(self.small_circle_erode_kernel))
        if erode_kernel_size % 2 == 0:  # 如果是偶数
            erode_kernel_size += 1  # 加1变成奇数
        
        # 创建椭圆结构元素（适合圆形或类圆形物体）
        #   cv2.getStructuringElement()函数说明见前面注释
        erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_kernel_size, erode_kernel_size))
        
        # cv2.erode()函数: 腐蚀操作
        #   函数语法:
        #     cv2.erode(src, kernel, iterations) -> dst
        #   参数:
        #     - src: 输入二值图像
        #     - kernel: 结构元素
        #     - iterations: 迭代次数（执行腐蚀的次数）
        #   说明:
        #     - 每次迭代都会缩小前景区域
        #     - 多次迭代会累积效果（连续缩小）
        #     - 例如: iterations=5表示连续执行5次腐蚀操作
        eroded = cv2.erode(mask, erode_kernel, iterations=int(self.small_circle_erode_iterations))
        
        # 保存腐蚀后的结果（用于调试）
        if debug_images is not None:
            debug_images[f'valve_eroded_{component_idx}'] = eroded.copy()
        
        # ========== 步骤2: 最大连通域选择（可选）==========
        # 为什么需要最大连通域选择:
        #   1. 腐蚀后可能有多个分离的连通域
        #   2. 阀体通常是一个完整的连通域（面积最大）
        #   3. 其他小连通域可能是噪声或次要部分
        #   4. 只保留最大连通域可以提高检测稳定性
        
        if self.small_circle_largest_cc:
            # 连通域分析（说明见前面注释）
            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(eroded, connectivity=8)
            
            if num_labels > 1:  # 如果有多个连通域（包括背景0）
                # 计算所有连通域的面积（跳过背景标签0）
                #   列表推导式: 创建面积列表
                areas = [stats[i, cv2.CC_STAT_AREA] for i in range(1, num_labels)]
                
                # np.argmax()函数: 返回最大值的索引
                #   说明: 找到面积最大的连通域的索引（相对于areas列表）
                #   注意: areas的索引0对应标签1，所以需要+1
                max_idx = np.argmax(areas) + 1  # +1是因为跳过了背景标签0
                
                # 创建只包含最大连通域的掩码
                #   labels == max_idx: 布尔数组，True表示属于最大连通域
                #   .astype(np.uint8): 转换为整数（True->1, False->0）
                #   * 255: 转换为二值图像（1->255白色，0->0黑色）
                largest_cc = (labels == max_idx).astype(np.uint8) * 255
                eroded = largest_cc  # 更新腐蚀后的掩码
                
                # 保存最大连通域结果（用于调试）
                if debug_images is not None:
                    debug_images[f'valve_largest_cc_{component_idx}'] = largest_cc.copy()
        
        # ========== 步骤3: 膨胀操作（恢复大小）==========
        # 膨胀操作原理:
        #   1. 使用结构元素（核）扫描图像
        #   2. 用核的最大值替换中心像素
        #   3. 结果: 前景区域扩大
        #   应用: 恢复阀体大小，补偿腐蚀造成的缩小
        
        # 实际参数值（基于default.yaml）:
        #   - small_circle_dilate_kernel = 9（默认值，9x9椭圆核）
        #   - small_circle_dilate_iterations = 1（默认值，膨胀1次）
        #   说明: 9x9的椭圆核，膨胀1次，部分恢复阀体大小
        #   效果: 腐蚀5次（11x11核）缩小很多，膨胀1次（9x9核）恢复较少
        #   结果: 最终阀体约是原始大小的70-80%，小细节已被去除
        
        # 确保核大小为奇数
        dilate_kernel_size = int(round(self.small_circle_dilate_kernel))
        # 实际值: dilate_kernel_size = 9（已经是奇数，不需要调整）
        if dilate_kernel_size % 2 == 0:  # 如果是偶数
            dilate_kernel_size += 1  # 加1变成奇数
        
        # 创建椭圆结构元素
        #   实际调用示例: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        #      - 创建9x9的椭圆核（比腐蚀核稍小）
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_kernel_size, dilate_kernel_size))
        
        # cv2.dilate()函数: 膨胀操作
        #   函数语法:
        #     cv2.dilate(src, kernel, iterations) -> dst
        #   参数:
        #     - src: 输入二值图像（腐蚀后的掩码）
        #     - kernel: 结构元素（9x9椭圆核）
        #     - iterations: 迭代次数（执行膨胀的次数）= 1
        #   说明:
        #     - 每次迭代都会扩大前景区域
        #     - 多次迭代会累积效果（连续扩大）
        #     - 通常膨胀次数小于腐蚀次数（主要是恢复大小，不是扩大）
        #   实际调用示例: cv2.dilate(eroded, 9x9椭圆核, iterations=1)
        #      - 输入: 腐蚀5次后的掩码（阀体缩小）
        #      - 输出: 膨胀1次后的掩码（阀体部分恢复）
        #      - 效果: 如果腐蚀后阀体直径70像素，膨胀1次后约恢复到85-90像素
        #   例如: 腐蚀5次（11x11核），膨胀1次（9x9核），结果大约是原始大小的80-85%
        processed_mask = cv2.dilate(eroded, dilate_kernel, iterations=int(self.small_circle_dilate_iterations))
        
        # 保存膨胀后的结果（用于调试）
        if debug_images is not None:
            debug_images[f'valve_dilated_{component_idx}'] = processed_mask.copy()
        
        # 查找轮廓并计算最小外接圆
        contours, _ = cv2.findContours(processed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return ((0, 0), 0.0, processed_mask)
        
        # 找到最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        
        # 保存阀体外接圆可视化（用于调试）
        if debug_images is not None:
            circle_vis = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
            processed_mask_bgr = cv2.cvtColor(processed_mask, cv2.COLOR_GRAY2BGR)
            circle_vis = processed_mask_bgr.copy()
            if radius > 0:
                cv2.circle(circle_vis, (int(x), int(y)), int(radius), (255, 0, 0), 2)
                cv2.circle(circle_vis, (int(x), int(y)), 5, (255, 0, 0), -1)
            debug_images[f'valve_circle_{component_idx}'] = circle_vis
        
        return ((float(x), float(y)), float(radius), processed_mask)
    
    def extract_features(self, components: List[np.ndarray], debug_images: dict = None) -> List[Dict]:
        """
        提取特征的主函数
        
        实现与C++ FeatureExtractor::extractFeatures相同的处理流程：
        1. 筛选连通域（按面积）
        2. 提取工件外接圆
        3. 提取阀体外接圆
        4. 计算标准化角度
        5. 生成特征可视化
        
        函数语法:
            extract_features(components, debug_images=None)
        
        参数说明:
            components (List[np.ndarray]): 
                - 类型: 列表，元素为numpy数组
                - 格式: 每个元素是一个二值掩码图像（uint8），255为连通域，0为背景
                - 说明: 预处理后的连通域列表（已筛选）
                - 数量: 最多component_max_count个连通域
            
            debug_images (dict, optional): 
                - 类型: 字典，默认值为None
                - 说明: 用于保存所有特征提取中间结果的字典
                - 如果提供，会保存每个连通域（索引idx）的以下键值对：
                  * f'wp_contours_{idx}' (np.ndarray): 工件轮廓可视化图像
                  * f'wp_circle_{idx}' (np.ndarray): 工件外接圆可视化图像
                  * f'valve_original_{idx}' (np.ndarray): 阀体原始掩码
                  * f'valve_eroded_{idx}' (np.ndarray): 阀体腐蚀后掩码
                  * f'valve_largest_cc_{idx}' (np.ndarray): 阀体最大连通域掩码
                  * f'valve_dilated_{idx}' (np.ndarray): 阀体膨胀后掩码
                  * f'valve_circle_{idx}' (np.ndarray): 阀体外接圆可视化图像
                  * f'feature_final_{idx}' (np.ndarray): 最终特征可视化图像
        
        返回:
            List[Dict]: 
                - 类型: 列表，元素为字典
                - 说明: 每个连通域的特征字典，包含以下键值对：
                  * 'workpiece_center' (Tuple[float, float]): 工件外接圆中心坐标 (x, y)
                  * 'workpiece_radius' (float): 工件外接圆半径（像素）
                  * 'workpiece_area' (int): 工件面积（像素数）
                  * 'valve_center' (Tuple[float, float]): 阀体外接圆中心坐标 (x, y)
                  * 'valve_radius' (float): 阀体外接圆半径（像素）
                  * 'valve_area' (int): 阀体面积（像素数）
                  * 'standardized_angle' (float): 标准化角度（弧度）
                    - 说明: 从工件中心指向阀体中心的角度
                    - 范围: [-π, π]
                    - 计算: atan2(valve_y - wp_y, valve_x - wp_x)
                  * 'component_mask' (np.ndarray): 连通域掩码，形状为(H, W)
                  * 'valve_mask' (np.ndarray): 处理后的阀体掩码，形状为(H, W)
                - 数量: 返回所有符合面积条件的连通域特征
        
        算法原理:
            1. 面积筛选: 过滤面积不在范围内的连通域
            2. 工件外接圆: 使用extract_workpiece_circle提取工件外接圆（大圆）
            3. 阀体外接圆: 使用extract_valve_circle提取阀体外接圆（小圆）
            4. 角度计算: 使用atan2计算从工件中心到阀体中心的角度
               - atan2(y, x): 返回点(x, y)相对于原点的角度
               - 范围: [-π, π]，从正x轴逆时针测量
               - 例如: atan2(1, 0) = π/2 (90度，向上)
                       atan2(0, 1) = 0 (0度，向右)
                       atan2(-1, 0) = -π/2 (-90度，向下)
            5. 特征可视化: 在原掩码上绘制工件外接圆、阀体外接圆和连线
        
        使用的关键参数（来自self）:
            - self.min_component_area (int): 最小连通域面积，默认2000，范围[0, 100000]
            - self.max_component_area (int): 最大连通域面积，默认10000000，范围[0, 100000000]
            - 其他参数参见extract_workpiece_circle和extract_valve_circle
        
        示例:
            >>> extractor = FeatureExtractorDebugger(config)
            >>> components = [component1, component2]  # 连通域列表
            >>> debug_images = {}
            >>> features = extractor.extract_features(components, debug_images)
            >>> for i, feature in enumerate(features):
            ...     print(f"特征{i}: 工件中心={feature['workpiece_center']}, "
            ...           f"阀体中心={feature['valve_center']}, "
            ...           f"角度={math.degrees(feature['standardized_angle'])}度")
            >>> # 现在debug_images包含了所有特征提取的中间结果
        
        注意事项:
            - 只处理面积在[min_area, max_area]范围内的连通域
            - 标准化角度用于确定工件的方向（阀体相对于工件中心的位置）
            - 如果阀体半径<=0，角度设为0.0
            - 所有坐标和半径都是浮点数（精确到亚像素）
        """
        if debug_images is None:
            debug_images = {}
        
        features = []
        
        for idx, component in enumerate(components):
            # 筛选连通域
            area = cv2.countNonZero(component)
            if area < self.min_component_area or area > self.max_component_area:
                continue
            
            # 提取工件外接圆
            (wp_x, wp_y), wp_radius = self.extract_workpiece_circle(component, debug_images, idx)
            
            # 提取阀体外接圆
            (valve_x, valve_y), valve_radius, valve_mask = self.extract_valve_circle(component, debug_images, idx)
            
            # 计算标准化角度
            standardized_angle = 0.0
            if valve_radius > 0:
                dx = valve_x - wp_x
                dy = valve_y - wp_y
                standardized_angle = math.atan2(dy, dx)
            
            # ========== 创建最终特征可视化（工件+阀体）==========
            if debug_images is not None:
                # 创建彩色图像用于可视化
                final_vis = np.zeros((component.shape[0], component.shape[1], 3), dtype=np.uint8)
                
                # cv2.cvtColor()函数: 颜色空间转换（灰度转BGR）
                #   函数语法:
                #     cv2.cvtColor(src, code, dst) -> dst
                #   参数:
                #     - src: 输入图像（灰度图）
                #     - code: 转换代码，cv2.COLOR_GRAY2BGR表示灰度转BGR
                #     - dst: 输出图像（可选，这里直接使用）
                #   说明: 将单通道灰度图转换为三通道BGR图（便于绘制彩色图形）
                cv2.cvtColor(component, cv2.COLOR_GRAY2BGR, final_vis)
                
                # ========== 绘制工件外接圆（绿色）==========
                # cv2.circle()函数: 绘制圆形
                #   函数语法:
                #     cv2.circle(img, center, radius, color, thickness)
                #   参数:
                #     - img: 输出图像（会被修改）
                #     - center: 圆心坐标 (x, y)，必须是整数
                #     - radius: 半径（像素），必须是整数
                #     - color: 颜色，格式为(B, G, R)，(0, 255, 0)表示绿色
                #     - thickness: 线条粗细，2表示2像素，-1表示填充
                #   说明: 第一个circle绘制外接圆（空心），第二个circle绘制中心点（实心）
                if wp_radius > 0:
                    cv2.circle(final_vis, (int(wp_x), int(wp_y)), int(wp_radius), (0, 255, 0), 2)  # 外接圆（绿色，2像素）
                    cv2.circle(final_vis, (int(wp_x), int(wp_y)), 5, (0, 255, 0), -1)  # 中心点（绿色，填充）
                
                # ========== 绘制阀体外接圆（蓝色）==========
                # 说明: (255, 0, 0)表示蓝色（B=255, G=0, R=0）
                if valve_radius > 0:
                    cv2.circle(final_vis, (int(valve_x), int(valve_y)), int(valve_radius), (255, 0, 0), 2)  # 外接圆（蓝色，2像素）
                    cv2.circle(final_vis, (int(valve_x), int(valve_y)), 5, (255, 0, 0), -1)  # 中心点（蓝色，填充）
                    
                    # ========== 绘制连线（黄色）==========
                    # cv2.line()函数: 绘制直线
                    #   函数语法:
                    #     cv2.line(img, pt1, pt2, color, thickness)
                    #   参数:
                    #     - img: 输出图像（会被修改）
                    #     - pt1: 起点坐标 (x1, y1)
                    #     - pt2: 终点坐标 (x2, y2)
                    #     - color: 颜色，格式为(B, G, R)，(0, 255, 255)表示黄色
                    #     - thickness: 线条粗细，2表示2像素
                    #   说明: 绘制从工件中心到阀体中心的连线（表示方向）
                    cv2.line(final_vis, (int(wp_x), int(wp_y)), (int(valve_x), int(valve_y)), (0, 255, 255), 2)
                
                debug_images[f'feature_final_{idx}'] = final_vis
            
            feature = {
                'workpiece_center': (wp_x, wp_y),
                'workpiece_radius': wp_radius,
                'workpiece_area': area,
                'valve_center': (valve_x, valve_y),
                'valve_radius': valve_radius,
                'valve_area': cv2.countNonZero(valve_mask) if valve_mask.size > 0 else 0,
                'standardized_angle': standardized_angle,
                'component_mask': component,
                'valve_mask': valve_mask
            }
            
            features.append(feature)
        
        return features


def create_trackbars(window_name: str, preprocessor: PreprocessorDebugger, feature_extractor: FeatureExtractorDebugger) -> str:
    """
    创建参数调整滑动条窗口
    
    在OpenCV窗口中创建多个滑动条，用于实时调整预处理和特征提取参数。
    滑动条的值会实时更新到preprocessor和feature_extractor对象中。
    
    函数语法:
        create_trackbars(window_name, preprocessor, feature_extractor) -> control_window
    
    参数说明:
        window_name (str): 
            - 类型: 字符串
            - 说明: 主显示窗口名称（用于窗口管理）
            - 示例: 'Preprocess & Feature Debug'
        
        preprocessor (PreprocessorDebugger): 
            - 类型: PreprocessorDebugger实例
            - 说明: 预处理器对象，包含所有预处理参数
            - 说明: 滑动条的值会更新到该对象的属性中
        
        feature_extractor (FeatureExtractorDebugger): 
            - 类型: FeatureExtractorDebugger实例
            - 说明: 特征提取器对象，包含所有特征提取参数
            - 说明: 滑动条的值会更新到该对象的属性中
    
    返回:
        str: 
            - 类型: 字符串
            - 说明: 控制面板窗口名称（用于后续的参数更新）
            - 值: 'Control Panel'
    
    创建的滑动条说明:
        预处理参数滑动条:
            - 'Scale Factor (x10)': 图像缩放因子，范围[0, 10]，实际值=滑动条值/10
            - 'Border Ratio (x100)': 边缘采样比例，范围[0, 30]，实际值=滑动条值/100
            - 'Hue Margin': HSV色调容差，范围[0, 100]
            - 'Hue Std Mul (x10)': 色调标准差倍数，范围[0, 50]，实际值=滑动条值/10
            - 'Sat Margin': 饱和度容差，范围[0, 200]
            - 'Sat Std Mul (x10)': 饱和度标准差倍数，范围[0, 50]，实际值=滑动条值/10
            - 'Val Margin': 亮度容差，范围[0, 200]
            - 'Val Std Mul (x10)': 亮度标准差倍数，范围[0, 50]，实际值=滑动条值/10
            - 'Lab Threshold (x10)': Lab距离阈值，范围[0, 150]，实际值=滑动条值/10
            - 'Cleanup Kernel': 形态学核大小，范围[0, 21]，必须为奇数
            - 'FG Close Kernel': 前景闭运算核大小，范围[0, 21]，必须为奇数
            - 'Median KSize': 中值滤波核大小，范围[0, 15]，必须为奇数
            - 'Min Noise Area': 最小噪声面积，范围[0, 1000]
            - 'Component Min Area': 最小连通域面积，范围[0, 100000]，实际值=滑动条值*100
            - 'Component Max Area (x10k)': 最大连通域面积，范围[0, 1000]，实际值=滑动条值*10000
        
        特征提取参数滑动条:
            - 'Big Circle Min Area': 大圆最小轮廓面积，范围[0, 1000]
            - 'Small Erode Kernel': 小圆腐蚀核大小，范围[0, 31]，必须为奇数
            - 'Small Erode Iter': 小圆腐蚀次数，范围[0, 20]
            - 'Small Dilate Kernel': 小圆膨胀核大小，范围[0, 31]，必须为奇数
            - 'Small Dilate Iter': 小圆膨胀次数，范围[0, 20]
    
    cv2.createTrackbar()函数说明:
        函数语法:
            cv2.createTrackbar(trackbarName, windowName, value, count, onChange)
        参数:
            - trackbarName (str): 滑动条名称（唯一标识）
            - windowName (str): 窗口名称（滑动条所在的窗口）
            - value (int): 初始值（整数）
            - count (int): 最大值（整数）
            - onChange (callable): 回调函数（值改变时调用），这里使用lambda x: None（不执行操作）
        说明:
            - 滑动条的值只能取整数
            - 需要通过计算转换为实际参数值（如浮点数）
            - 值改变时不会自动更新参数，需要手动调用update_parameters_from_trackbars()
    
    窗口管理:
        - cv2.namedWindow(): 创建窗口
        - cv2.resizeWindow(): 设置窗口大小
        - cv2.moveWindow(): 设置窗口位置
        - 控制面板窗口: 400x1000像素，位于屏幕左上角
    
    示例:
        >>> preprocessor = PreprocessorDebugger(config)
        >>> extractor = FeatureExtractorDebugger(config)
        >>> window_name = 'Main Window'
        >>> control_window = create_trackbars(window_name, preprocessor, extractor)
        >>> print(f"控制面板窗口: {control_window}")
    
    注意事项:
        - 滑动条的初始值来自preprocessor和feature_extractor对象的当前参数值
        - 滑动条的值需要手动转换（如除以10、乘以100等）
        - 滑动条的值改变后不会自动更新，需要在主循环中调用update_parameters_from_trackbars()
        - 控制面板窗口需要单独显示，不会自动显示在主窗口中
    """
    # 创建主显示窗口
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    # 设置窗口初始大小
    cv2.resizeWindow(window_name, 1200, 800)
    
    # 创建控制面板窗口（用于滑动条）
    control_window = 'Control Panel'
    cv2.namedWindow(control_window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(control_window, 400, 1000)
    cv2.moveWindow(control_window, 50, 50)
    
    # 创建一个黑色背景用于显示滑动条
    control_panel = np.zeros((1000, 400, 3), dtype=np.uint8)
    cv2.putText(control_panel, 'Preprocess & Feature Control', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow(control_window, control_panel)
    
    # 预处理参数
    cv2.createTrackbar('Scale Factor (x10)', control_window, int(preprocessor.scale_factor * 10), 10, lambda x: None)
    cv2.createTrackbar('Border Ratio (x100)', control_window, int(preprocessor.border_ratio * 100), 30, lambda x: None)
    cv2.createTrackbar('Hue Margin', control_window, int(preprocessor.hue_margin), 100, lambda x: None)
    cv2.createTrackbar('Hue Std Mul (x10)', control_window, int(preprocessor.hue_std_mul * 10), 50, lambda x: None)
    cv2.createTrackbar('Sat Margin', control_window, int(preprocessor.sat_margin), 200, lambda x: None)
    cv2.createTrackbar('Sat Std Mul (x10)', control_window, int(preprocessor.sat_std_mul * 10), 50, lambda x: None)
    cv2.createTrackbar('Val Margin', control_window, int(preprocessor.val_margin), 200, lambda x: None)
    cv2.createTrackbar('Val Std Mul (x10)', control_window, int(preprocessor.val_std_mul * 10), 50, lambda x: None)
    cv2.createTrackbar('Lab Threshold (x10)', control_window, int(preprocessor.lab_threshold * 10), 150, lambda x: None)
    cv2.createTrackbar('Cleanup Kernel', control_window, preprocessor.cleanup_kernel, 21, lambda x: None)
    cv2.createTrackbar('FG Close Kernel', control_window, preprocessor.foreground_close_kernel, 21, lambda x: None)
    cv2.createTrackbar('Median KSize', control_window, preprocessor.median_ksize, 15, lambda x: None)
    cv2.createTrackbar('Min Noise Area', control_window, preprocessor.min_noise_area, 1000, lambda x: None)
    cv2.createTrackbar('Component Min Area', control_window, preprocessor.component_min_area // 100, 1000, lambda x: None)
    cv2.createTrackbar('Component Max Area (x10k)', control_window, preprocessor.component_max_area // 10000, 1000, lambda x: None)
    
    # 特征提取参数
    cv2.createTrackbar('Big Circle Min Area', control_window, feature_extractor.big_circle_min_area, 1000, lambda x: None)
    cv2.createTrackbar('Small Erode Kernel', control_window, feature_extractor.small_circle_erode_kernel, 31, lambda x: None)
    cv2.createTrackbar('Small Erode Iter', control_window, feature_extractor.small_circle_erode_iterations, 20, lambda x: None)
    cv2.createTrackbar('Small Dilate Kernel', control_window, feature_extractor.small_circle_dilate_kernel, 31, lambda x: None)
    cv2.createTrackbar('Small Dilate Iter', control_window, feature_extractor.small_circle_dilate_iterations, 20, lambda x: None)
    
    return control_window


def update_parameters_from_trackbars(control_window: str, preprocessor: PreprocessorDebugger, feature_extractor: FeatureExtractorDebugger) -> None:
    """
    从滑动条读取并更新参数
    
    从OpenCV滑动条中读取当前值，并将其转换为相应的参数值，
    然后更新到preprocessor和feature_extractor对象中。
    
    参数:
        control_window (str): 控制面板窗口名称
        preprocessor (PreprocessorDebugger): 预处理器实例
        feature_extractor (FeatureExtractorDebugger): 特征提取器实例
    """
    preprocessor.scale_factor = cv2.getTrackbarPos('Scale Factor (x10)', control_window) / 10.0
    preprocessor.border_ratio = cv2.getTrackbarPos('Border Ratio (x100)', control_window) / 100.0
    preprocessor.hue_margin = cv2.getTrackbarPos('Hue Margin', control_window)
    preprocessor.hue_std_mul = cv2.getTrackbarPos('Hue Std Mul (x10)', control_window) / 10.0
    preprocessor.sat_margin = cv2.getTrackbarPos('Sat Margin', control_window)
    preprocessor.sat_std_mul = cv2.getTrackbarPos('Sat Std Mul (x10)', control_window) / 10.0
    preprocessor.val_margin = cv2.getTrackbarPos('Val Margin', control_window)
    preprocessor.val_std_mul = cv2.getTrackbarPos('Val Std Mul (x10)', control_window) / 10.0
    preprocessor.lab_threshold = cv2.getTrackbarPos('Lab Threshold (x10)', control_window) / 10.0
    preprocessor.cleanup_kernel = cv2.getTrackbarPos('Cleanup Kernel', control_window)
    if preprocessor.cleanup_kernel % 2 == 0:
        preprocessor.cleanup_kernel += 1
    preprocessor.foreground_close_kernel = cv2.getTrackbarPos('FG Close Kernel', control_window)
    if preprocessor.foreground_close_kernel % 2 == 0:
        preprocessor.foreground_close_kernel += 1
    preprocessor.median_ksize = cv2.getTrackbarPos('Median KSize', control_window)
    if preprocessor.median_ksize % 2 == 0:
        preprocessor.median_ksize += 1
    preprocessor.min_noise_area = cv2.getTrackbarPos('Min Noise Area', control_window)
    preprocessor.component_min_area = cv2.getTrackbarPos('Component Min Area', control_window) * 100
    preprocessor.component_max_area = cv2.getTrackbarPos('Component Max Area (x10k)', control_window) * 10000
    
    feature_extractor.big_circle_min_area = cv2.getTrackbarPos('Big Circle Min Area', control_window)
    feature_extractor.small_circle_erode_kernel = cv2.getTrackbarPos('Small Erode Kernel', control_window)
    if feature_extractor.small_circle_erode_kernel % 2 == 0:
        feature_extractor.small_circle_erode_kernel += 1
    feature_extractor.small_circle_erode_iterations = cv2.getTrackbarPos('Small Erode Iter', control_window)
    feature_extractor.small_circle_dilate_kernel = cv2.getTrackbarPos('Small Dilate Kernel', control_window)
    if feature_extractor.small_circle_dilate_kernel % 2 == 0:
        feature_extractor.small_circle_dilate_kernel += 1
    feature_extractor.small_circle_dilate_iterations = cv2.getTrackbarPos('Small Dilate Iter', control_window)


def create_debug_montage(image: np.ndarray, debug_images: dict) -> np.ndarray:
    """
    创建调试图像蒙太奇（使用OpenCV显示预处理和特征检测的多个处理步骤）
    
    该方法创建一个大型拼接图像，将所有处理步骤的图像按照C++代码的处理顺序排列。
    使用2列布局，图像尺寸接近原始大小，标题清晰可见。
    
    处理顺序（与C++代码一致）：
    
    ========== 预处理阶段（Preprocessor::preprocess）==========
    0. Original Image - 原始输入图像（BGR格式）
    1. Scaled Image - 缩放后的图像（如果scale_factor < 1.0，用于加速处理）
    
    ========== 背景去除阶段（removeGreenBackground）==========
    2. Border Mask - 边缘掩码，用于采样背景颜色统计信息（白色区域为边缘采样区）
    3. HSV Background Mask - HSV颜色空间的背景掩码（白色=背景，黑色=前景）
    4. Lab Background Mask - Lab颜色空间的背景掩码（白色=背景，黑色=前景）
    5. Merged Bg Mask (After Morph) - 合并HSV和Lab掩码，并进行形态学处理后的背景掩码
    
    ========== 前景提取阶段 ==========
    6. Foreground Before Noise Removal - 去噪前的前景掩码（背景掩码取反）
    7. Foreground After Noise Removal - 去噪后的前景掩码（去除小噪声连通域）
    8. Final Foreground Mask - 最终前景掩码（经过形态学处理和中值滤波）
    
    ========== 连通域提取阶段（extractConnectedComponents + filterComponents）==========
    9. All Connected Components - 筛选前的所有连通域（显示所有检测到的连通域）
    10. Filtered Components - 筛选后的连通域（根据面积、宽高比、尺寸等条件筛选）
    
    ========== 特征提取阶段（FeatureExtractor）==========
    11. Workpiece Contours - 工件轮廓（黄色线条显示检测到的轮廓）
    12. Workpiece Circle - 工件外接圆（绿色圆圈，大圆）
    
    13. Valve Original Mask - 阀体原始掩码（连通域掩码）
    14. Valve After Erosion - 阀体腐蚀后的掩码（去除小细节）
    15. Valve Largest Connected Component - 阀体最大连通域（只保留最大的连通域）
    16. Valve After Dilation - 阀体膨胀后的掩码（恢复阀体大小）
    17. Valve Circle - 阀体外接圆（蓝色圆圈，小圆）
    
    18. Final Feature Visualization - 最终特征可视化（显示工件外接圆、阀体外接圆和连线）
    
    参数:
        image (np.ndarray): 原始图像，用于确定显示尺寸
        debug_images (dict): 调试图像字典，包含所有中间处理结果
    
    返回:
        np.ndarray: 拼接后的调试图像蒙太奇（BGR格式）
                   如果debug_images为空，返回提示图像
    """
    if not debug_images:
        # 如果没有调试图像，显示提示
        h, w = image.shape[:2]
        montage = np.ones((h, w, 3), dtype=np.uint8) * 50
        cv2.putText(montage, 'No debug images available', (w//4, h//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return montage
    
    # ========== 按照C++处理顺序定义步骤 ==========
    # 预处理阶段（与C++ Preprocessor::preprocess一致）
    preprocess_steps = []
    
    # 步骤1: 原始图像和缩放
    if 'original_image' in debug_images:
        preprocess_steps.append(('0. Original Image', 'original_image', False))
    
    if 'scaled_image' in debug_images:
        preprocess_steps.append(('1. Scaled Image', 'scaled_image', False))
    
    # 步骤2: 背景去除相关（按C++ removeGreenBackground顺序）
    if 'border_mask' in debug_images:
        preprocess_steps.append(('2. Border Mask', 'border_mask', True))
    
    if 'hsv_mask' in debug_images:
        preprocess_steps.append(('3. HSV Background Mask', 'hsv_mask', True))
    
    if 'lab_mask' in debug_images:
        preprocess_steps.append(('4. Lab Background Mask', 'lab_mask', True))
    
    if 'bg_mask_after_morph' in debug_images:
        preprocess_steps.append(('5. Merged Bg Mask (After Morph)', 'bg_mask_after_morph', True))
    
    # 步骤3: 前景提取（按C++处理顺序）
    if 'non_green_before_noise' in debug_images:
        preprocess_steps.append(('6. Foreground Before Noise Removal', 'non_green_before_noise', True))
    
    if 'non_green_after_noise' in debug_images:
        preprocess_steps.append(('7. Foreground After Noise Removal', 'non_green_after_noise', True))
    
    if 'foreground_mask' in debug_images:
        preprocess_steps.append(('8. Final Foreground Mask', 'foreground_mask', True))
    
    # 步骤4: 连通域提取（按C++ extractConnectedComponents和filterComponents顺序）
    if 'all_components_before_filter' in debug_images:
        preprocess_steps.append(('9. All Connected Components', 'all_components_before_filter', True))
    
    if 'components_after_filter' in debug_images:
        preprocess_steps.append(('10. Filtered Components', 'components_after_filter', True))
    
    # 特征提取阶段（与C++ FeatureExtractor一致）
    feature_steps = []
    component_idx = 0
    
    # 步骤5: 工件外接圆提取（按C++ extractWorkpieceCircle顺序）
    if f'wp_contours_{component_idx}' in debug_images:
        feature_steps.append((f'11. Workpiece Contours', f'wp_contours_{component_idx}', False))
    
    if f'wp_circle_{component_idx}' in debug_images:
        feature_steps.append((f'12. Workpiece Circle', f'wp_circle_{component_idx}', False))
    
    # 步骤6: 阀体外接圆提取（按C++ extractValveCircle顺序）
    if f'valve_original_{component_idx}' in debug_images:
        feature_steps.append((f'13. Valve Original Mask', f'valve_original_{component_idx}', True))
    
    if f'valve_eroded_{component_idx}' in debug_images:
        feature_steps.append((f'14. Valve After Erosion', f'valve_eroded_{component_idx}', True))
    
    if f'valve_largest_cc_{component_idx}' in debug_images:
        feature_steps.append((f'15. Valve Largest Connected Component', f'valve_largest_cc_{component_idx}', True))
    
    if f'valve_dilated_{component_idx}' in debug_images:
        feature_steps.append((f'16. Valve After Dilation', f'valve_dilated_{component_idx}', True))
    
    if f'valve_circle_{component_idx}' in debug_images:
        feature_steps.append((f'17. Valve Circle', f'valve_circle_{component_idx}', False))
    
    # 步骤7: 最终特征可视化
    if f'feature_final_{component_idx}' in debug_images:
        feature_steps.append((f'18. Final Feature Visualization', f'feature_final_{component_idx}', False))
    
    # 合并所有步骤（按照C++处理顺序）
    steps = preprocess_steps + feature_steps
    
    # ========== 新的布局逻辑：根据窗口宽度计算图像尺寸 ==========
    # 使用多列布局（3列），让图像填满窗口宽度
    cols = 3  # 3列布局
    rows = (len(steps) + cols - 1) // cols  # 计算需要的行数
    
    # 布局参数
    title_height = 80  # 标题区域高度（增大，让标题更清晰）
    image_spacing_h = 30  # 图像之间的垂直间距
    image_spacing_w = 30  # 图像之间的水平间距
    
    # 目标窗口宽度（3列布局应该填满窗口宽度）
    target_window_width = 3600  # 与窗口初始宽度一致
    
    # 根据目标窗口宽度计算每个图像单元的宽度
    # 总宽度 = cols * cell_w + (cols + 1) * image_spacing_w
    # 因此：cell_w = (target_window_width - (cols + 1) * image_spacing_w) / cols
    cell_w = int((target_window_width - (cols + 1) * image_spacing_w) / cols)
    
    # 获取原始图像尺寸，用于计算高度（保持宽高比）
    img_h, img_w = image.shape[:2]
    
    # 根据宽度和原始图像的宽高比计算高度（保持原始图像的宽高比）
    aspect_ratio = img_h / img_w if img_w > 0 else 1.0
    cell_h = int(cell_w * aspect_ratio)
    
    # 设置高度限制，避免图像太高
    max_cell_h = 900  # 最大高度限制
    if cell_h > max_cell_h:
        cell_h = max_cell_h
        # 如果高度受限，重新调整宽度以保持原始图像宽高比
        cell_w = int(cell_h / aspect_ratio) if aspect_ratio > 0 else cell_w
    
    # 确保最小尺寸
    cell_h = max(cell_h, 300)  # 最小高度
    cell_w = max(cell_w, 400)  # 最小宽度
    
    # 重新计算画布尺寸（基于实际计算的cell_w和cell_h）
    montage_h = rows * cell_h + (rows + 1) * title_height + (rows - 1) * image_spacing_h
    montage_w = cols * cell_w + (cols + 1) * image_spacing_w
    
    # 确保是整数
    montage_h = int(montage_h)
    montage_w = int(montage_w)
    
    # 创建画布（深灰色背景）
    montage = np.ones((montage_h, montage_w, 3), dtype=np.uint8) * 40
    
    # 绘制每个步骤的图像
    for idx, (title, key, is_mask) in enumerate(steps):
        row = idx // cols
        col = idx % cols
        
        # 计算图像位置
        y_start = row * (cell_h + title_height + image_spacing_h) + title_height
        x_start = col * (cell_w + image_spacing_w) + image_spacing_w
        
        # 确保是整数
        y_start = int(y_start)
        x_start = int(x_start)
        
        # 标题位置（在图像上方）
        title_y = int(y_start - 15)
        
        if key in debug_images:
            img = debug_images[key]
            
            # 转换掩码为彩色图像
            if len(img.shape) == 2:  # 灰度图
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            
            # 调整图像大小以适应单元尺寸（保持宽高比）
            img_h_orig, img_w_orig = img.shape[:2]
            scale_h = cell_h / img_h_orig
            scale_w = cell_w / img_w_orig
            scale = min(scale_h, scale_w)  # 使用较小的缩放比例，保持宽高比
            
            new_w = int(img_w_orig * scale)
            new_h = int(img_h_orig * scale)
            
            # 调整大小
            if new_w != img_w_orig or new_h != img_h_orig:
                img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
            
            # 计算居中位置（如果图像小于单元尺寸）
            y_offset = (cell_h - new_h) // 2
            x_offset = (cell_w - new_w) // 2
            
            # 放置图像（居中显示）
            y_end = min(y_start + new_h + y_offset, montage_h)
            x_end = min(x_start + new_w + x_offset, montage_w)
            y_img_start = y_start + y_offset
            x_img_start = x_start + x_offset
            
            if y_img_start >= 0 and x_img_start >= 0 and y_end > y_img_start and x_end > x_img_start:
                montage[y_img_start:y_end, x_img_start:x_end] = img[:y_end-y_img_start, :x_end-x_img_start]
            
            # 绘制标题（大字体，清晰可见）
            font_scale = 1.5  # 更大的字体
            thickness = 4     # 更粗的线条
            outline_thickness = 6  # 描边厚度
            
            # 获取文本尺寸以居中
            (text_w, text_h), baseline = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            text_x = x_start + (cell_w - text_w) // 2  # 居中显示
            
            # 先绘制黑色描边（让文字更清晰）
            for dx in range(-outline_thickness, outline_thickness + 1):
                for dy in range(-outline_thickness, outline_thickness + 1):
                    if dx != 0 or dy != 0:
                        cv2.putText(montage, title, (text_x + dx, title_y + dy),
                                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness)
            
            # 再绘制白色文字
            cv2.putText(montage, title, (text_x, title_y),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
        else:
            # 显示占位符
            font_scale = 1.2
            thickness = 3
            placeholder_text = f'{title} (N/A)'
            (text_w, text_h), baseline = cv2.getTextSize(placeholder_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            text_x = x_start + (cell_w - text_w) // 2
            text_y = y_start + cell_h // 2
            
            # 绘制占位符（带描边）
            cv2.putText(montage, placeholder_text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 2)
            cv2.putText(montage, placeholder_text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, (128, 128, 128), thickness)
    
    return montage


def display_debug_images_matplotlib(image: np.ndarray, debug_images: dict, max_cols: int = 1) -> None:
    """
    使用matplotlib显示调试图像（新的显示方法）
    
    相比OpenCV的单一拼接图像，matplotlib提供更好的交互功能：
    - 可以缩放和拖动
    - 可以保存图像
    - 更好的图像质量
    - 支持分页显示（如果图像太多）
    
    按照C++代码的处理顺序显示所有调试图像。
    
    参数:
        image (np.ndarray): 原始图像，用于确定尺寸
        debug_images (dict): 调试图像字典
        max_cols (int): 每行最多显示的图像数量，默认1列（单列布局）
    """
    if not MATPLOTLIB_AVAILABLE:
        print("警告: matplotlib不可用，将使用OpenCV显示")
        return
    
    if not debug_images:
        print("警告: 没有调试图像可显示")
        return
    
    # 按照C++处理顺序定义步骤（与create_debug_montage相同）
    steps = []
    
    # 预处理阶段
    if 'original_image' in debug_images:
        steps.append(('0. Original Image', 'original_image', False))
    if 'scaled_image' in debug_images:
        steps.append(('1. Scaled Image', 'scaled_image', False))
    if 'border_mask' in debug_images:
        steps.append(('2. Border Mask', 'border_mask', True))
    if 'hsv_mask' in debug_images:
        steps.append(('3. HSV Background Mask', 'hsv_mask', True))
    if 'lab_mask' in debug_images:
        steps.append(('4. Lab Background Mask', 'lab_mask', True))
    if 'bg_mask_after_morph' in debug_images:
        steps.append(('5. Merged Bg Mask (After Morph)', 'bg_mask_after_morph', True))
    if 'non_green_before_noise' in debug_images:
        steps.append(('6. Foreground Before Noise Removal', 'non_green_before_noise', True))
    if 'non_green_after_noise' in debug_images:
        steps.append(('7. Foreground After Noise Removal', 'non_green_after_noise', True))
    if 'foreground_mask' in debug_images:
        steps.append(('8. Final Foreground Mask', 'foreground_mask', True))
    if 'all_components_before_filter' in debug_images:
        steps.append(('9. All Connected Components', 'all_components_before_filter', True))
    if 'components_after_filter' in debug_images:
        steps.append(('10. Filtered Components', 'components_after_filter', True))
    
    # 特征提取阶段
    component_idx = 0
    if f'wp_contours_{component_idx}' in debug_images:
        steps.append((f'11. Workpiece Contours', f'wp_contours_{component_idx}', False))
    if f'wp_circle_{component_idx}' in debug_images:
        steps.append((f'12. Workpiece Circle', f'wp_circle_{component_idx}', False))
    if f'valve_original_{component_idx}' in debug_images:
        steps.append((f'13. Valve Original Mask', f'valve_original_{component_idx}', True))
    if f'valve_eroded_{component_idx}' in debug_images:
        steps.append((f'14. Valve After Erosion', f'valve_eroded_{component_idx}', True))
    if f'valve_largest_cc_{component_idx}' in debug_images:
        steps.append((f'15. Valve Largest Connected Component', f'valve_largest_cc_{component_idx}', True))
    if f'valve_dilated_{component_idx}' in debug_images:
        steps.append((f'16. Valve After Dilation', f'valve_dilated_{component_idx}', True))
    if f'valve_circle_{component_idx}' in debug_images:
        steps.append((f'17. Valve Circle', f'valve_circle_{component_idx}', False))
    if f'feature_final_{component_idx}' in debug_images:
        steps.append((f'18. Final Feature Visualization', f'feature_final_{component_idx}', False))
    
    if not steps:
        print("警告: 没有有效的调试图像步骤")
        return
    
    # 计算网格布局（单列布局，图像更大更清晰）
    num_images = len(steps)
    cols = 1  # 改为单列布局
    rows = num_images  # 所有图像垂直排列
    
    # 创建图形和子图
    # 使用更大的图形尺寸，让每个图像都清晰可见（单列布局时图像更宽）
    fig_width = 20 if cols == 1 else 16  # 单列布局使用更宽的窗口
    fig_height = max(10, rows * 2)  # 根据图像数量调整高度
    fig = plt.figure(figsize=(fig_width, fig_height))
    fig.suptitle('Debug Images - Processing Pipeline', fontsize=16, fontweight='bold')
    
    # 创建子图并显示图像
    for idx, (title, key, is_mask) in enumerate(steps):
        ax = fig.add_subplot(rows, cols, idx + 1)
        
        if key in debug_images:
            img = debug_images[key]
            
            # 转换掩码为彩色（如果是灰度图）
            if len(img.shape) == 2:
                img_display = img
                cmap = 'gray' if is_mask else 'gray'
            else:
                # BGR转RGB（OpenCV使用BGR，matplotlib使用RGB）
                img_display = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                cmap = None
            
            # 显示图像
            ax.imshow(img_display, cmap=cmap, interpolation='bilinear')
            ax.set_title(title, fontsize=12, fontweight='bold', pad=10)
            ax.axis('off')  # 不显示坐标轴
        else:
            # 显示占位符
            ax.text(0.5, 0.5, f'{title}\n(N/A)', 
                   ha='center', va='center', fontsize=12, color='gray',
                   transform=ax.transAxes)
            ax.axis('off')
    
    # 调整布局，确保所有子图都可见
    plt.tight_layout(rect=[0, 0, 1, 0.98])
    
    # 显示图形（非阻塞方式）
    plt.show(block=False)
    plt.pause(0.1)  # 短暂暂停以允许窗口更新


def visualize_results(image: np.ndarray, components: List[np.ndarray], features: List[Dict]) -> np.ndarray:
    """
    可视化处理结果
    
    在原始图像上绘制连通域和提取的特征（工件外接圆、阀体外接圆等）。
    用于在主窗口中显示最终的处理结果。
    
    函数语法:
        visualize_results(image, components, features) -> vis_image
    
    参数说明:
        image (np.ndarray): 
            - 类型: numpy数组，形状为(H, W, 3)
            - 格式: BGR颜色空间
            - 说明: 原始输入图像，作为背景
            - 要求: 图像不能为空，必须是3通道BGR图像
        
        components (List[np.ndarray]): 
            - 类型: 列表，元素为numpy数组
            - 格式: 每个元素是一个二值掩码图像（uint8），255为连通域，0为背景
            - 说明: 预处理后的连通域列表
            - 数量: 最多component_max_count个连通域
        
        features (List[Dict]): 
            - 类型: 列表，元素为字典
            - 说明: 每个连通域的特征字典，包含以下键值对：
              * 'workpiece_center' (Tuple[float, float]): 工件外接圆中心坐标 (x, y)
              * 'workpiece_radius' (float): 工件外接圆半径（像素）
              * 'valve_center' (Tuple[float, float]): 阀体外接圆中心坐标 (x, y)
              * 'valve_radius' (float): 阀体外接圆半径（像素）
            - 数量: 与components数量相同（每个连通域对应一个特征字典）
    
    返回:
        np.ndarray: 
            - 类型: numpy数组，形状为(H, W, 3)
            - 格式: BGR颜色空间
            - 说明: 可视化后的图像，包含以下内容：
              * 原始图像（背景）
              * 连通域的半透明叠加（前景区域以30%透明度叠加）
              * 工件外接圆（绿色圆圈和中心点）
              * 阀体外接圆（蓝色圆圈和中心点）
              * 两圆心连线（黄色直线，表示方向）
            - 异常: 如果输入图像为空，返回480x640的黑色图像
    
    绘制说明:
        连通域叠加:
            - cv2.addWeighted()函数: 图像加权混合
            - 语法: cv2.addWeighted(src1, alpha, src2, beta, gamma) -> dst
            - 参数:
              * src1: 原始图像，权重alpha=0.7
              * src2: 连通域掩码（转为BGR），权重beta=0.3
              * gamma: 常数，0表示不偏移
            - 效果: 原始图像70% + 连通域30% = 半透明叠加
        
        工件外接圆（绿色）:
            - cv2.circle()函数: 绘制圆形
            - 外接圆: 绿色圆圈，线宽2像素
            - 中心点: 绿色实心圆，半径5像素
        
        阀体外接圆（蓝色）:
            - 外接圆: 蓝色圆圈，线宽2像素
            - 中心点: 蓝色实心圆，半径5像素
        
        连线（黄色）:
            - cv2.line()函数: 绘制直线
            - 从工件中心到阀体中心，黄色，线宽2像素
            - 表示工件的方向（标准化角度的可视化）
    
    示例:
        >>> image = cv2.imread('test.jpg')
        >>> components = [component1, component2]  # 连通域列表
        >>> features = [feature1, feature2]  # 特征列表
        >>> vis_image = visualize_results(image, components, features)
        >>> cv2.imshow('Result', vis_image)
    
    注意事项:
        - 如果图像为空，返回480x640的黑色图像
        - 连通域列表为空时，只显示原始图像
        - 特征列表为空时，只显示连通域叠加
        - 所有坐标和半径必须是有效的浮点数（>0）
        - 坐标会自动转换为整数（cv2.circle和cv2.line要求整数坐标）
    """
    if image is None or image.size == 0:
        print("错误: 输入图像为空")
        return np.zeros((480, 640, 3), dtype=np.uint8)
    
    vis_image = image.copy()
    
    # 绘制所有连通域
    if components:
        all_components = np.zeros_like(image[:, :, 0])
        for component in components:
            if component is not None and component.size > 0:
                all_components = cv2.bitwise_or(all_components, component)
        
        # 转换为彩色显示
        if cv2.countNonZero(all_components) > 0:
            vis_mask = cv2.cvtColor(all_components, cv2.COLOR_GRAY2BGR)
            vis_image = cv2.addWeighted(vis_image, 0.7, vis_mask, 0.3, 0)
    
    # 绘制特征
    for feature in features:
        wp_center = feature['workpiece_center']
        wp_radius = feature['workpiece_radius']
        valve_center = feature['valve_center']
        valve_radius = feature['valve_radius']
        
        # 绘制工件外接圆（绿色）
        if wp_radius > 0:
            cv2.circle(vis_image, (int(wp_center[0]), int(wp_center[1])), 
                      int(wp_radius), (0, 255, 0), 2)
            cv2.circle(vis_image, (int(wp_center[0]), int(wp_center[1])), 
                      5, (0, 255, 0), -1)
        
        # 绘制阀体外接圆（蓝色）
        if valve_radius > 0:
            cv2.circle(vis_image, (int(valve_center[0]), int(valve_center[1])), 
                      int(valve_radius), (255, 0, 0), 2)
            cv2.circle(vis_image, (int(valve_center[0]), int(valve_center[1])), 
                      5, (255, 0, 0), -1)
            
            # 绘制连线（黄色）
            cv2.line(vis_image, 
                    (int(wp_center[0]), int(wp_center[1])),
                    (int(valve_center[0]), int(valve_center[1])),
                    (0, 255, 255), 2)
    
    return vis_image


def save_config(preprocessor: PreprocessorDebugger, feature_extractor: FeatureExtractorDebugger, output_path: str) -> None:
    """
    保存当前参数配置到YAML文件
    
    将当前调试器的所有参数保存到YAML格式的配置文件中，
    可以用于后续的批量处理或参数复用。
    
    函数语法:
        save_config(preprocessor, feature_extractor, output_path)
    
    参数说明:
        preprocessor (PreprocessorDebugger): 
            - 类型: PreprocessorDebugger实例
            - 说明: 预处理器对象，包含所有预处理参数
            - 说明: 从该对象读取所有参数并保存到文件
        
        feature_extractor (FeatureExtractorDebugger): 
            - 类型: FeatureExtractorDebugger实例
            - 说明: 特征提取器对象，包含所有特征提取参数
            - 说明: 从该对象读取所有参数并保存到文件
        
        output_path (str): 
            - 类型: 字符串
            - 说明: 输出配置文件路径（YAML格式）
            - 示例: 'debug_config.yaml', './configs/my_config.yaml'
            - 要求: 路径必须有效，父目录必须存在（如果不存在会创建失败）
    
    返回:
        None: 函数不返回值，直接保存文件
    
    YAML文件格式说明:
        配置文件结构:
            preprocess:
              scale_factor: <float>  # 图像缩放因子
              min_area: <int>  # 最小面积
              background:
                border_ratio: <float>  # 边缘采样比例
                hue_margin: <float>  # HSV色调容差
                hue_std_mul: <float>  # 色调标准差倍数
                sat_margin: <float>  # 饱和度容差
                sat_std_mul: <float>  # 饱和度标准差倍数
                val_margin: <float>  # 亮度容差
                val_std_mul: <float>  # 亮度标准差倍数
                lab_threshold: <float>  # Lab距离阈值
                cleanup_kernel: <int>  # 形态学核大小
                foreground_close_kernel: <int>  # 前景闭运算核大小
                median_ksize: <int>  # 中值滤波核大小
                enable_classic_hsv: <bool>  # 是否启用经典HSV检测
                use_histogram: <bool>  # 是否使用直方图
                min_noise_area: <int>  # 最小噪声面积
                erode_before_dilate: <bool>  # 是否先腐蚀后膨胀
                component_min_area: <int>  # 最小连通域面积
                component_max_area: <int>  # 最大连通域面积
                component_min_aspect_ratio: <float>  # 最小宽高比
                component_max_aspect_ratio: <float>  # 最大宽高比
                component_min_width: <int>  # 最小宽度
                component_min_height: <int>  # 最小高度
                component_max_count: <int>  # 最大连通域数量
              feature_extraction:
                min_component_area: <int>  # 最小连通域面积
                max_component_area: <int>  # 最大连通域面积
                big_circle:
                  combine_contours: <bool>  # 是否合并轮廓
                  min_area: <int>  # 最小轮廓面积
                small_circle:
                  erode_kernel: <int>  # 腐蚀核大小
                  erode_iterations: <int>  # 腐蚀次数
                  largest_cc: <bool>  # 是否只保留最大连通域
                  dilate_kernel: <int>  # 膨胀核大小
                  dilate_iterations: <int>  # 膨胀次数
    
    yaml.dump()函数说明:
        函数语法:
            yaml.dump(data, stream, **kwargs)
        参数:
            - data: 要保存的数据（字典）
            - stream: 文件流（open()返回的对象）
            - default_flow_style=False: 使用块样式（更易读）
            - allow_unicode=True: 允许Unicode字符（支持中文）
            - sort_keys=False: 不排序键（保持原始顺序）
        说明: 将Python字典转换为YAML格式并写入文件
    
    示例:
        >>> preprocessor = PreprocessorDebugger(config)
        >>> extractor = FeatureExtractorDebugger(config)
        >>> # 调整参数...
        >>> save_config(preprocessor, extractor, 'my_config.yaml')
        >>> print("配置已保存")
        >>> # 后续可以使用: config = yaml.safe_load(open('my_config.yaml'))
    
    注意事项:
        - 文件路径的父目录必须存在，否则会创建失败
        - 如果文件已存在，会被覆盖
        - 配置文件使用UTF-8编码（支持中文注释）
        - 保存的参数是当前对象的值（滑动条调整后的值）
        - 可以使用保存的配置文件重新初始化调试器
    """
    config = {
        'preprocess': {
            'scale_factor': preprocessor.scale_factor,
            'min_area': preprocessor.min_area,
            'background': {
                'border_ratio': preprocessor.border_ratio,
                'hue_margin': preprocessor.hue_margin,
                'hue_std_mul': preprocessor.hue_std_mul,
                'sat_margin': preprocessor.sat_margin,
                'sat_std_mul': preprocessor.sat_std_mul,
                'val_margin': preprocessor.val_margin,
                'val_std_mul': preprocessor.val_std_mul,
                'lab_threshold': preprocessor.lab_threshold,
                'cleanup_kernel': preprocessor.cleanup_kernel,
                'foreground_close_kernel': preprocessor.foreground_close_kernel,
                'median_ksize': preprocessor.median_ksize,
                'enable_classic_hsv': preprocessor.enable_classic_hsv,
                'use_histogram': preprocessor.use_histogram,
                'min_noise_area': preprocessor.min_noise_area,
                'erode_before_dilate': preprocessor.erode_before_dilate,
                'component_min_area': preprocessor.component_min_area,
                'component_max_area': preprocessor.component_max_area,
                'component_min_aspect_ratio': preprocessor.component_min_aspect_ratio,
                'component_max_aspect_ratio': preprocessor.component_max_aspect_ratio,
                'component_min_width': preprocessor.component_min_width,
                'component_min_height': preprocessor.component_min_height,
                'component_max_count': preprocessor.component_max_count
            },
            'feature_extraction': {
                'min_component_area': feature_extractor.min_component_area,
                'max_component_area': feature_extractor.max_component_area,
                'big_circle': {
                    'combine_contours': feature_extractor.big_circle_combine_contours,
                    'min_area': feature_extractor.big_circle_min_area
                },
                'small_circle': {
                    'erode_kernel': feature_extractor.small_circle_erode_kernel,
                    'erode_iterations': feature_extractor.small_circle_erode_iterations,
                    'largest_cc': feature_extractor.small_circle_largest_cc,
                    'dilate_kernel': feature_extractor.small_circle_dilate_kernel,
                    'dilate_iterations': feature_extractor.small_circle_dilate_iterations
                }
            }
        }
    }
    
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    
    print(f"配置已保存到: {output_path}")


def main() -> None:
    """
    主函数
    
    程序的入口点，负责：
    1. 解析命令行参数
    2. 加载图像和配置
    3. 创建调试器实例
    4. 创建GUI窗口
    5. 运行主循环（实时参数调整和可视化）
    """
    # 检查命令行参数
    if len(sys.argv) < 2:
        print("使用方法: python3 debug_preprocess_feature.py <image_path> [config_path]")
        print("  参数说明:")
        print("    <image_path>: 待处理的图像文件路径（必需）")
        print("    [config_path]: 配置文件路径（可选，YAML格式）")
        sys.exit(1)
    
    image_path = sys.argv[1]
    config_path = sys.argv[2] if len(sys.argv) > 2 else None
    
    # 加载图像
    print(f"正在加载图像: {image_path}")
    image = cv2.imread(image_path)
    if image is None:
        print(f"错误: 无法加载图像 {image_path}")
        print("请检查图像路径是否正确，以及图像文件是否存在")
        sys.exit(1)
    
    print(f"图像加载成功: {image.shape[1]}x{image.shape[0]} 像素")
    
    # 加载配置
    if config_path and os.path.exists(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
    else:
        # 使用默认配置
        default_config_path = Path(__file__).parent.parent / 'configs' / 'default.yaml'
        if default_config_path.exists():
            with open(default_config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
        else:
            print("警告: 未找到配置文件，使用默认参数")
            config = {}
    
    # 创建调试器
    preprocessor = PreprocessorDebugger(config)
    feature_extractor = FeatureExtractorDebugger(config)
    
    # 创建窗口和滑动条
    window_name = 'Preprocess & Feature Debug'
    debug_window = 'Debug Steps'
    print(f"创建窗口: {window_name}")
    control_window = create_trackbars(window_name, preprocessor, feature_extractor)
    
    # 创建调试步骤窗口（3列布局，窗口适应多列排列）
    cv2.namedWindow(debug_window, cv2.WINDOW_NORMAL)
    # 使用合适的初始窗口大小，适应3列布局
    cv2.resizeWindow(debug_window, 3600, 2500)  # 宽度3600（3列），高度2500
    
    # 移动窗口位置
    cv2.moveWindow(window_name, 500, 50)  # 主窗口在右侧
    cv2.moveWindow(debug_window, 500, 900)  # 调试窗口在主窗口下方
    cv2.moveWindow(control_window, 50, 50)  # 控制面板在左侧
    
    print("\n=== 调试说明 ===")
    print("1. 使用滑动条调整参数")
    print("2. 按 'r' 键重新处理图像")
    print("3. 按 's' 键保存当前配置")
    print("4. 按 'q' 或 ESC 键退出")
    print("================\n")
    print("提示: 如果看不到窗口，请尝试:")
    print("  - 使用 Alt+Tab 切换窗口")
    print("  - 检查窗口是否被最小化")
    print("  - 检查是否有多个显示器")
    
    # ========== 主循环：实时参数调整和图像处理 ==========
    print("正在启动GUI窗口...")
    print("提示: 如果窗口没有显示，请检查是否被其他窗口遮挡，或尝试Alt+Tab切换窗口")
    print("提示: 调整滑动条时，图像会实时更新（可能需要几毫秒处理时间）")
    print("提示: 过程图像使用OpenCV显示，3列布局，可以通过滚动查看所有图像")
    
    # 用于检测参数是否变化的哈希值（性能优化：只在参数变化时重新处理）
    last_params_hash = None
    
    # 初始显示原始图像，确保窗口有内容
    print("显示初始图像...")
    cv2.imshow(window_name, image)
    
    # 刷新控制面板（显示滑动条说明）
    control_panel = np.zeros((1000, 400, 3), dtype=np.uint8)
    cv2.putText(control_panel, 'Preprocess & Feature Control', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(control_panel, 'Adjust sliders to see changes', (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.imshow(control_window, control_panel)
    
    cv2.waitKey(100)  # 给窗口时间显示
    print("窗口已显示，开始主循环...")
    
    # 主循环计数器（用于控制matplotlib更新频率）
    frame_count = 0
    last_process_time = 0
    
    # ========== 主循环开始 ==========
    # 循环处理：检测参数变化 -> 重新处理图像 -> 更新显示
    while True:
        try:
            # 检查窗口是否还存在
            try:
                cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
                cv2.getWindowProperty(control_window, cv2.WND_PROP_VISIBLE)
            except:
                print("窗口已关闭")
                break
            
            # 更新参数
            update_parameters_from_trackbars(control_window, preprocessor, feature_extractor)
            
            # 计算当前参数的哈希值（用于检测变化）
            current_params = (
                preprocessor.scale_factor, preprocessor.border_ratio,
                preprocessor.hue_margin, preprocessor.sat_margin, preprocessor.val_margin,
                preprocessor.lab_threshold, preprocessor.component_min_area,
                feature_extractor.big_circle_min_area
            )
            current_params_hash = hash(current_params)
            
            # 性能优化：如果参数没有变化，只等待键盘输入，不重新处理
            if current_params_hash == last_params_hash and frame_count > 0:
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q') or key == 27:
                    break
                elif key == ord('r'):  # 强制重新处理
                    last_params_hash = None
                continue
            
            # 参数已变化，需要重新处理
            last_params_hash = current_params_hash
            frame_count += 1
            
            # ========== 步骤1: 预处理（与C++ Preprocessor::preprocess一致） ==========
            # 包括：图像缩放、去除绿色背景、提取和筛选连通域
            debug_images = {}
            try:
                components = preprocessor.preprocess(image, debug_images)
            except Exception as e:
                print(f"预处理错误: {e}")
                import traceback
                traceback.print_exc()
                components = []
                debug_images = {}
            
            # ========== 步骤2: 特征提取（与C++ FeatureExtractor::extractFeatures一致） ==========
            # 包括：提取工件外接圆、阀体外接圆、计算标准化角度
            try:
                features = feature_extractor.extract_features(components, debug_images)
            except Exception as e:
                print(f"特征提取错误: {e}")
                import traceback
                traceback.print_exc()
                features = []
            
            # ========== 步骤3: 可视化处理结果 ==========
            # 在原始图像上绘制连通域和特征
            try:
                vis_image = visualize_results(image, components, features)
            except Exception as e:
                print(f"可视化错误: {e}")
                vis_image = image.copy()
            
            # 如果图像为空，使用原始图像
            if vis_image is None or vis_image.size == 0:
                print("警告: 可视化图像为空，使用原始图像")
                vis_image = image.copy()
            elif vis_image.shape[0] == 0 or vis_image.shape[1] == 0:
                print("警告: 可视化图像尺寸为0，使用原始图像")
                vis_image = image.copy()
            
            # 调整图像大小以适应显示（如果太大）
            max_display_size = 1200
            h, w = vis_image.shape[:2]
            
            # 确保图像尺寸有效
            if h <= 0 or w <= 0:
                print(f"错误: 图像尺寸无效 {w}x{h}，使用原始图像")
                vis_image_display = image.copy()
            elif max(h, w) > max_display_size:
                scale = max_display_size / max(h, w)
                new_w = int(w * scale)
                new_h = int(h * scale)
                if new_w > 0 and new_h > 0:
                    vis_image_display = cv2.resize(vis_image, (new_w, new_h), interpolation=cv2.INTER_AREA)
                else:
                    vis_image_display = vis_image.copy()
            else:
                vis_image_display = vis_image.copy()
            
            # 确保显示图像有效
            if vis_image_display is None or vis_image_display.size == 0:
                print("错误: 显示图像无效，使用原始图像")
                vis_image_display = image.copy()
                if vis_image_display.shape[0] > max_display_size or vis_image_display.shape[1] > max_display_size:
                    scale = max_display_size / max(vis_image_display.shape[0], vis_image_display.shape[1])
                    new_w = int(vis_image_display.shape[1] * scale)
                    new_h = int(vis_image_display.shape[0] * scale)
                    vis_image_display = cv2.resize(vis_image_display, (new_w, new_h), interpolation=cv2.INTER_AREA)
            
            # 显示信息
            info_text = [
                f"Components: {len(components)}",
                f"Features: {len(features)}"
            ]
            if features:
                info_text.append(f"WP Radius: {features[0]['workpiece_radius']:.1f}")
                info_text.append(f"Valve Radius: {features[0]['valve_radius']:.1f}")
            
            # 添加参数信息（显示关键参数）
            info_text.append("---")
            info_text.append(f"Scale: {preprocessor.scale_factor:.1f}")
            info_text.append(f"Border: {preprocessor.border_ratio:.2f}")
            info_text.append(f"Hue M: {preprocessor.hue_margin:.0f}")
            
            # 在显示图像上绘制信息（使用较小的图像）
            y_offset = 20
            for text in info_text:
                cv2.putText(vis_image_display, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)  # 黑色描边
                cv2.putText(vis_image_display, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)  # 白色文字
                y_offset += 20
            
            # 显示主结果图像
            if vis_image_display is not None and vis_image_display.size > 0:
                try:
                    cv2.imshow(window_name, vis_image_display)
                except Exception as e:
                    print(f"显示图像错误: {e}")
                    print(f"图像形状: {vis_image_display.shape if vis_image_display is not None else 'None'}")
                    # 尝试显示原始图像
                    cv2.imshow(window_name, image)
            else:
                print("错误: 显示图像为空，显示原始图像")
                cv2.imshow(window_name, image)
            
            # 显示调试步骤图像（仅使用OpenCV显示，3列布局）
            try:
                # 使用OpenCV显示调试图像蒙太奇（3列布局）
                debug_montage = create_debug_montage(image, debug_images)
                if debug_montage is not None and debug_montage.size > 0:
                    # 显示逻辑：尽量保持原始大小，只在必要时缩小
                    h, w = debug_montage.shape[:2]
                    
                    # 设置合理的限制，适应3列布局
                    max_width = 4000  # 最大宽度限制（3列布局）
                    max_height = 3000  # 最大高度限制
                    
                    # 只在图像超过限制时才缩放
                    if w > max_width or h > max_height:
                        scale_w = max_width / w if w > max_width else 1.0
                        scale_h = max_height / h if h > max_height else 1.0
                        scale = min(scale_w, scale_h)  # 使用较小的缩放比例，保持宽高比
                        new_w = int(w * scale)
                        new_h = int(h * scale)
                        debug_montage = cv2.resize(debug_montage, (new_w, new_h), interpolation=cv2.INTER_AREA)
                    
                    # 直接显示（保持原始大小，让用户看到真实尺寸）
                    cv2.imshow(debug_window, debug_montage)
            except Exception as e:
                print(f"显示调试图像错误: {e}")
                import traceback
                traceback.print_exc()
            
            # 确保窗口在前台（只在第一次）
            if last_params_hash is None:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)
                cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 0)
                # 强制刷新窗口
                cv2.waitKey(1)
            
            # 键盘输入（使用较短的等待时间以确保实时响应）
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' 或 ESC
                print("退出程序...")
                break
            elif key == ord('r'):  # 重新处理
                print("重新处理图像...")
                last_params_hash = None  # 强制重新处理
            elif key == ord('s'):  # 保存配置
                output_path = 'debug_config.yaml'
                save_config(preprocessor, feature_extractor, output_path)
                print(f"配置已保存到: {output_path}")
            
        except KeyboardInterrupt:
            print("\n收到中断信号，退出...")
            break
        except Exception as e:
            print(f"处理错误: {e}")
            import traceback
            traceback.print_exc()
            # 显示原始图像作为后备
            try:
                cv2.imshow(window_name, image)
                key = cv2.waitKey(100) & 0xFF
                if key == ord('q') or key == 27:
                    break
            except:
                pass
            # 继续运行，不退出
            continue
    
    print("清理资源...")
    try:
        cv2.destroyAllWindows()
    except:
        pass
    print("程序退出")


if __name__ == '__main__':
    main()
