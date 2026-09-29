"""ros_log_bridge 单测：级别映射、格式、幂等（零 ROS，假 logger）."""
import logging

from peach_common.ros_log_bridge import (
    bridge_module_loggers,
    RosLoggerHandler,
)


class _FakeLogger:
    """记录调用供断言；接口对齐 rclpy logger 的四级方法."""

    def __init__(self):
        self.calls = []

    def debug(self, msg):
        self.calls.append(('debug', msg))

    def info(self, msg):
        self.calls.append(('info', msg))

    def warning(self, msg):
        self.calls.append(('warning', msg))

    def error(self, msg):
        self.calls.append(('error', msg))


class _FakeNode:

    def __init__(self):
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger


def _fresh_logger(name):
    logger = logging.getLogger(name)
    logger.handlers = [
        h for h in logger.handlers if not isinstance(h, RosLoggerHandler)]
    return logger


def test_level_mapping_and_format():
    name = 'test_bridge.module_a'
    _fresh_logger(name)
    node = _FakeNode()
    bridge_module_loggers(node, [name])
    logger = logging.getLogger(name)
    logger.error('boom')
    logger.warning('careful')
    logger.info('note')
    logger.debug('trace')  # 默认桥级 INFO：DEBUG 在 logger 级被截，属预期
    levels = [level for level, _ in node.logger.calls]
    assert levels == ['error', 'warning', 'info']
    assert node.logger.calls[0][1].startswith(f'{name}: boom')


def test_debug_forwarded_when_bridge_level_debug():
    name = 'test_bridge.module_debug'
    _fresh_logger(name)
    node = _FakeNode()
    bridge_module_loggers(node, [name], level=logging.DEBUG)
    logging.getLogger(name).debug('trace')
    assert ('debug', f'{name}: trace') in node.logger.calls


def test_idempotent_bridge():
    name = 'test_bridge.module_b'
    _fresh_logger(name)
    node1, node2 = _FakeNode(), _FakeNode()
    bridge_module_loggers(node1, [name])
    bridge_module_loggers(node2, [name])
    logger = logging.getLogger(name)
    handlers = [h for h in logger.handlers
                if isinstance(h, RosLoggerHandler)]
    assert len(handlers) == 1
    logger.warning('once')
    assert node1.logger.calls and not node2.logger.calls


def test_info_level_forwarded_when_default_notset():
    """模块 logger 默认 NOTSET 时 INFO 会被 logger 级别拦，须压级放行."""
    name = 'test_bridge.module_c'
    _fresh_logger(name)
    node = _FakeNode()
    bridge_module_loggers(node, [name])
    logging.getLogger(name).info('visible')
    assert ('info', f'{name}: visible') in node.logger.calls


def test_formatter_fallback_on_bad_record():
    handler = RosLoggerHandler(_FakeLogger())
    record = logging.LogRecord(
        'x', logging.ERROR, 'path', 1, 'plain %s', ('arg',), None)
    assert handler.format(record) == 'plain arg'
