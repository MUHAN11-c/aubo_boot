"""
可选急停：单独再起一条 roslibpy → rosbridge 连接，向 geometry_msgs/msg/Twist 话题发零速度。
在异步上下文中应通过 asyncio.to_thread 调用本模块，避免阻塞事件循环。
"""

from __future__ import annotations

import json
import logging
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ivg_gateway.config import Settings

logger = logging.getLogger(__name__)


def emergency_stop_sync(settings: "Settings") -> tuple[bool, str]:
    """
    同步阻塞实现。返回 (是否成功发布, 人类可读说明)。
    未配置话题名时返回失败说明，供审计记录「仅逻辑熔断」。
    """
    topic = (settings.emergency_cmd_vel_topic or "").strip()
    if not topic:
        return False, "未配置 IVG_GATEWAY_EMERGENCY_CMD_VEL_TOPIC，仅记录事件"
    try:
        import roslibpy
    except ImportError:
        return False, "roslibpy 未安装"

    host = settings.rosbridge_host
    port = settings.rosbridge_port
    try:
        payload = json.loads(settings.emergency_twist_message_json)
    except json.JSONDecodeError as e:
        return False, f"紧急停止 Twist JSON 无效: {e}"

    ros = roslibpy.Ros(host=host, port=port)
    try:
        ros.run()  # 在后台线程拉起与 rosbridge 的 WebSocket
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not ros.is_connected:
            time.sleep(0.05)
        if not ros.is_connected:
            return False, "roslibpy 连接 rosbridge 超时"
        t = roslibpy.Topic(ros, topic, "geometry_msgs/msg/Twist")
        t.publish(roslibpy.Message(payload))
        time.sleep(0.15)
        return True, f"已发布紧急停止到 {topic}"
    except Exception as e:
        logger.exception("emergency_stop")
        return False, str(e)
    finally:
        try:
            ros.terminate()
        except Exception:
            pass
