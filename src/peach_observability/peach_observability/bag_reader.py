"""
MCAP 会话读取与反序列化：bag → {topic: [(t_ns, dict|None)]}.

rosbag2_py / rclpy.serialization / rosidl_runtime_py 全部延迟导入，本模块
可在无 ROS 上下文下被导入（调用读取才需要 source Jazzy + install）。
metadata.yaml 缺失（kill -9 等非正常退出）时用 rosbag2_py.Reindexer 进程内
兜底重建索引后重读，不执行外部命令。

转换约定：图像/点云（HEAVY_TOPICS）只留时间戳（dict 恒 None）防内存爆；
其余消息按类型转换——CanonicalEvent/PeachTargetObservationArray 复用
state 的既有转换（与旧 jsonl 字段一致），其余走通用 dict 转换（bytes
字段折叠为 ``{'byte_count': n}``，不展开图像/点云原始数据）。
"""

from __future__ import annotations

from pathlib import Path

# 报告只需时间戳的话题（全量 dict 转换会拖慢且无分析价值）
HEAVY_TOPICS = (
    '/peach/perception/debug_image',
    '/peach/perception/debug_image_raw',
    '/peach/reconstruction/tsdf_cloud',
)


def ensure_index(bag_dir) -> Path:
    """metadata.yaml 缺失时用 rosbag2_py.Reindexer 兜底重建；返回 bag 目录."""
    from rosbag2_py import Reindexer, StorageOptions

    bag = Path(bag_dir)
    if (bag / 'metadata.yaml').exists():
        return bag
    try:
        Reindexer().reindex(
            StorageOptions(uri=str(bag), storage_id='mcap'))
    except Exception as error:  # noqa: BLE001 rosbag2 存储层异常类型不定
        raise RuntimeError(f'reindex 失败（{bag}）: {error}') from error
    if not (bag / 'metadata.yaml').exists():
        raise RuntimeError(f'reindex 后仍缺 metadata.yaml（{bag}）')
    return bag


def type_name_of(message) -> str:
    """消息实例 → rosbag2 类型串（与 recorder.message_type_name 一致）."""
    cls = type(message)
    return f'{cls.__module__.split(".")[0]}/msg/{cls.__name__}'


def _to_dict(value):
    """
    消息/字段递归转 dict；bytes 折叠为 byte_count（不展开图像原始数据）.

    rclpy 消息槽名带下划线前缀（``_data``/``_header``），此处统一剥掉，
    使键名与消息字段名（data/header/…）一致。
    """
    import array
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (bytes, bytearray, array.array)):
        return {'byte_count': len(value)}
    if isinstance(value, (list, tuple)):
        return [_to_dict(item) for item in value]
    slots = getattr(value, '__slots__', None)
    if slots:
        return {
            slot.lstrip('_'): _to_dict(getattr(value, slot))
            for slot in slots
        }
    return str(value)


def read_bag(bag_dir, *, stamps_only=HEAVY_TOPICS) -> dict:
    """
    读一个 bag：返回 {topic: [(t_ns, dict|None), ...]}（bag 原序，即写入序）.

    stamps_only 中（默认图像/点云）的话题只记时间戳。
    """
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    from .state import (
        to_grasp_decision,
        to_grasp_hypothesis,
        to_harvest_event,
        to_reconstruction_status,
        to_target_observations,
    )

    bag = ensure_index(bag_dir)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id='mcap'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()}
    type_cache: dict[str, type] = {}
    streams: dict[str, list] = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        records = streams.setdefault(topic, [])
        if topic in stamps_only:
            records.append((int(t_ns), None))
            continue
        msg_type = type_cache.get(topic)
        if msg_type is None:
            type_name = topic_types.get(topic)
            if type_name is None:
                continue
            msg_type = get_message(type_name)
            type_cache[topic] = msg_type
        message = deserialize_message(data, msg_type)
        kind = type_name_of(message)
        converters = {
            'peach_interfaces/msg/CanonicalEvent': to_harvest_event,
            'peach_interfaces/msg/PeachTargetObservationArray':
                to_target_observations,
            'peach_interfaces/msg/ReconstructionStatus':
                to_reconstruction_status,
            'peach_interfaces/msg/GraspDecision': to_grasp_decision,
            'peach_interfaces/msg/GraspHypothesis': to_grasp_hypothesis,
        }
        converter = converters.get(kind)
        records.append(
            (int(t_ns), converter(message) if converter else _to_dict(message)))
    return streams


def topic_streams(streams: dict, *names: str) -> list:
    """取指定话题的记录流（缺失给空表；多话题按时间归并升序）."""
    merged: list = []
    for name in names:
        merged.extend(streams.get(name) or [])
    merged.sort(key=lambda item: item[0])
    return merged
