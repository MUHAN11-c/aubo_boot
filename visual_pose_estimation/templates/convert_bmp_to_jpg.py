#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将目录下所有BMP图片转换为高质量JPG文件，保持分辨率不变

python3 /home/nvidia/RVG_ws/src/visual_pose_estimation_cpp/templates/211242785/convert_bmp_to_jpg.py

"""

import os
from pathlib import Path
from PIL import Image

def convert_bmp_to_jpg(input_path, output_path=None, quality=95):
    """
    将BMP图片转换为高质量JPG
    
    Args:
        input_path: 输入BMP文件路径
        output_path: 输出JPG文件路径（如果为None，则自动生成）
        quality: JPG质量（1-100，95为高质量）
    """
    try:
        # 打开BMP图片
        with Image.open(input_path) as img:
            # 如果是RGBA模式，转换为RGB（JPG不支持透明度）
            if img.mode in ('RGBA', 'LA', 'P'):
                # 创建白色背景
                rgb_img = Image.new('RGB', img.size, (255, 255, 255))
                if img.mode == 'P':
                    img = img.convert('RGBA')
                rgb_img.paste(img, mask=img.split()[3] if img.mode == 'RGBA' else None)
                img = rgb_img
            elif img.mode != 'RGB':
                img = img.convert('RGB')
            
            # 如果没有指定输出路径，自动生成
            if output_path is None:
                output_path = str(Path(input_path).with_suffix('.jpg'))
            
            # 保存为高质量JPG
            img.save(output_path, 'JPEG', quality=quality, optimize=True)
            print(f"✓ 转换成功: {input_path} -> {output_path}")
            return True
            
    except Exception as e:
        print(f"✗ 转换失败: {input_path} - {str(e)}")
        return False

def convert_all_bmp_in_directory(directory_path, quality=95):
    """
    递归查找目录下所有BMP文件并转换为JPG
    
    Args:
        directory_path: 目标目录路径
        quality: JPG质量（1-100，95为高质量）
    """
    directory = Path(directory_path)
    if not directory.exists():
        print(f"错误: 目录不存在: {directory_path}")
        return
    
    # 查找所有BMP文件（包括.BMP和.bmp）
    bmp_files = list(directory.rglob('*.bmp')) + list(directory.rglob('*.BMP'))
    
    if not bmp_files:
        print(f"未找到BMP文件: {directory_path}")
        return
    
    print(f"找到 {len(bmp_files)} 个BMP文件，开始转换...\n")
    
    success_count = 0
    for bmp_file in bmp_files:
        if convert_bmp_to_jpg(bmp_file, quality=quality):
            success_count += 1
    
    print(f"\n转换完成: {success_count}/{len(bmp_files)} 个文件成功转换")

if __name__ == "__main__":
    # 获取脚本所在目录
    script_dir = Path(__file__).parent
    
    # 转换当前目录及子目录下所有BMP文件
    print(f"开始转换目录: {script_dir}\n")
    convert_all_bmp_in_directory(script_dir, quality=95)

