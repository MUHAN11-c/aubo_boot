#!/usr/bin/env python3
"""
测试 Debug API 的连接性
"""

import requests
import json

API_BASE_URL = "http://localhost:8000"

def test_api(endpoint, data=None):
    """测试API端点"""
    url = f"{API_BASE_URL}{endpoint}"
    try:
        if data is None:
            data = {}
        response = requests.post(url, json=data, timeout=5)
        print(f"✓ {endpoint}: {response.status_code}")
        try:
            result = response.json()
            print(f"  Response: {json.dumps(result, indent=2, ensure_ascii=False)[:200]}")
        except:
            print(f"  Response: {response.text[:200]}")
        return True
    except requests.exceptions.ConnectionError as e:
        print(f"✗ {endpoint}: Connection Error - 服务器可能未运行")
        return False
    except requests.exceptions.Timeout:
        print(f"✗ {endpoint}: Timeout - 请求超时")
        return False
    except Exception as e:
        print(f"✗ {endpoint}: {type(e).__name__}: {str(e)}")
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("测试 Debug API 连接性")
    print("=" * 60)
    
    # 测试基本连接
    print("\n1. 测试服务器连接...")
    try:
        response = requests.get(API_BASE_URL, timeout=2)
        print(f"✓ 服务器在线 ({response.status_code})")
    except Exception as e:
        print(f"✗ 无法连接到服务器: {e}")
        print("\n请确保服务器正在运行:")
        print("  cd /home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui")
        print("  python3 scripts/http_bridge_server.py")
        exit(1)
    
    # 测试Debug API
    print("\n2. 测试 Debug API 端点...")
    test_api("/api/debug/get_params")
    test_api("/api/debug/get_images")
    test_api("/api/debug/capture", {"camera_id": "207000152740"})
    test_api("/api/debug/update_params", {"param_name": "min_depth", "param_value": 100})
    test_api("/api/debug/save_thresholds")
    
    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
