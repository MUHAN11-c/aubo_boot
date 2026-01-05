#!/usr/bin/env python3
"""
移除Python文件中的所有debug instrumentation
"""
import re

def remove_debug_logs(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 移除所有 # #region agent log ... # #endregion 块
    # Python使用 # #region (两个#号)
    pattern = r'\s*# #region agent log\s*\n.*?\n\s*# #endregion\s*\n'
    cleaned_content = re.sub(pattern, '', content, flags=re.DOTALL)
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(cleaned_content)
    
    # 统计移除的数量
    removed_count = content.count('# #region agent log')
    print(f"已从 {file_path} 移除 {removed_count} 个debug log块")
    
    return removed_count

if __name__ == '__main__':
    file_path = '/home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/hand_eye_calibration/hand_eye_calibration_node.py'
    count = remove_debug_logs(file_path)
    print(f"完成！共移除 {count} 个instrumentation块")

