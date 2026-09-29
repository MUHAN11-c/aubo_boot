"""
stdlib logging → rclpy 节点日志桥（观测性 P0）.

纯核模块按约束走 stdlib logging（不 import ROS）；缺本桥时其输出只经
lastResort 到 stderr（且仅 ≥WARNING），/rosout 与会话 bag 看不见——
BoundedWorker 连续异常、落盘线程失败等关键信号因此流失。本模块把指定
模块 logger 的记录转发到节点 logger，使其进入 /rosout。

用法（节点构造或 on_configure 调一次）::

    from peach_common.ros_log_bridge import bridge_module_loggers

    bridge_module_loggers(self, [
        'peach_harvester.vision.common.runtime',
        'peach_harvester.vision.target_reconstruction.session_recorder',
    ])

幂等：同一 logger 重复桥接不叠 handler。node 只需提供 get_logger()
（rclpy 节点均满足），便于 pytest 用假 logger 验证级别映射与格式。
"""
from __future__ import annotations

import logging

_LEVEL_INFO = logging.INFO


class RosLoggerHandler(logging.Handler):
    """把 logging.LogRecord 按级别映射转发到 rclpy 节点 logger."""

    def __init__(self, node_logger):
        """node_logger 为 rclpy logger（只需 error/warning/info/debug）."""
        super().__init__()
        self._node_logger = node_logger

    def emit(self, record: logging.LogRecord) -> None:
        """格式化并转发；格式化失败退回原始消息，不反噬调用线程."""
        try:
            message = self.format(record)
        except Exception:  # noqa: BLE001 记录路径不得抛出
            message = record.getMessage()
        logger = self._node_logger
        if record.levelno >= logging.ERROR:
            logger.error(message)
        elif record.levelno >= logging.WARNING:
            logger.warning(message)
        elif record.levelno >= logging.INFO:
            logger.info(message)
        else:
            logger.debug(message)


def bridge_module_loggers(
        node, module_names, *, level: int = _LEVEL_INFO) -> list:
    """
    把纯核模块 logger 桥接到节点的 /rosout 流.

    Args:
        node: rclpy 节点（或任何带 get_logger() 的对象）.
        module_names: 模块 logger 名列表（即模块的 __name__）.
        level: 转发下限；仅当下调（模块 logger 原级别更高时压到该级）.

    Returns:
        已桥接的 logger 列表（幂等：重复调用不重复挂 handler）.

    """
    handler = RosLoggerHandler(node.get_logger())
    handler.setFormatter(logging.Formatter('%(name)s: %(message)s'))
    bridged = []
    for name in module_names:
        logger = logging.getLogger(name)
        if not any(isinstance(h, RosLoggerHandler) for h in logger.handlers):
            logger.addHandler(handler)
        if logger.level == logging.NOTSET or logger.level > level:
            logger.setLevel(level)
        bridged.append(logger)
    return bridged
