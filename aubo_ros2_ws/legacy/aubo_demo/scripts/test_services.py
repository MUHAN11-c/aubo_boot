#!/usr/bin/env python3
"""
便捷脚本：运行 demo_driver 服务测试
用法：python3 test_services.py [参数]
"""

import sys
import os

# 获取脚本所在目录
script_dir = os.path.dirname(os.path.abspath(__file__))
test_script = os.path.join(script_dir, 'src', 'aubo_ros2_driver', 'demo_driver', 'test_services.py')

# 检查文件是否存在
if not os.path.exists(test_script):
    print(f"错误: 找不到测试脚本: {test_script}")
    sys.exit(1)

# 执行测试脚本
if __name__ == '__main__':
    # 将参数传递给实际脚本
    sys.argv[0] = test_script
    exec(open(test_script).read())

