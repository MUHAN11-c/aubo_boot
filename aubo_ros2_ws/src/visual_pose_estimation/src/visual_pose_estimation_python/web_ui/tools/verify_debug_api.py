#!/usr/bin/env python3
"""
手工验证 FastAPI Debug API 的连通性。

用法：
    python3 web_ui/tools/verify_debug_api.py
    API_BASE_URL=http://127.0.0.1:9000 python3 web_ui/tools/verify_debug_api.py
"""

from __future__ import annotations

import json
import os

import requests


API_BASE_URL = os.environ.get("API_BASE_URL", "http://127.0.0.1:8088").rstrip("/")


def test_api(endpoint: str, data: dict | None = None) -> bool:
    url = f"{API_BASE_URL}{endpoint}"
    try:
        response = requests.post(url, json=data or {}, timeout=5)
        print(f"✓ {endpoint}: {response.status_code}")
        try:
            result = response.json()
            print(f"  Response: {json.dumps(result, indent=2, ensure_ascii=False)[:200]}")
        except Exception:
            print(f"  Response: {response.text[:200]}")
        return True
    except requests.exceptions.ConnectionError:
        print(f"✗ {endpoint}: Connection Error - 服务器可能未运行")
        return False
    except requests.exceptions.Timeout:
        print(f"✗ {endpoint}: Timeout - 请求超时")
        return False
    except Exception as exc:
        print(f"✗ {endpoint}: {type(exc).__name__}: {exc}")
        return False


if __name__ == "__main__":
    print("=" * 60)
    print("测试 Debug API 连接性")
    print("=" * 60)

    print("\n1. 测试服务器连接...")
    try:
        response = requests.get(f"{API_BASE_URL}/health", timeout=2)
        print(f"✓ 服务器在线 ({response.status_code})")
    except Exception as exc:
        print(f"✗ 无法连接到服务器: {exc}")
        print("\n请确保服务正在运行，例如：")
        print("  source /opt/ros/humble/setup.bash")
        print("  source <workspace>/install/setup.bash")
        print("  ros2 run visual_pose_estimation_python visual_pose_estimation_web --host 127.0.0.1 --port 8088")
        raise SystemExit(1)

    print("\n2. 测试 Debug API 端点...")
    test_api("/api/debug/get_params")
    test_api("/api/debug/get_images")
    test_api("/api/debug/capture", {"camera_id": "207000152740"})
    test_api("/api/debug/update_params", {"param_name": "min_depth", "param_value": 100})
    test_api("/api/debug/save_thresholds")

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
