"""foxglove_ws_client — Foxglove WebSocket v1 协议 Python 测试客户端.

与 JS 侧 `web/public/js/transport/foxglove/protocol.js` 同源的帧格式（已与
vendored ivg_foxglove_bridge 服务器源码逐字节核对）：

  服务器→客户端（首字节 opcode，小端）：
    [0x01][subscriptionId u32][timestamp u64][payload]       消息数据
    [0x02][timestamp u64]                                     时间
    [0x03][serviceId u32][callId u32][encLen u32][enc][data]  服务响应
  客户端→服务器：
    [0x01][channelId u32][payload]                            客户端发布
    [0x02][serviceId u32][callId u32][encLen u32][enc][data]  服务调用请求

  JSON op（vendored 桥为批量形式）：serverInfo/advertise/unadvertise/
  advertiseServices/unadvertiseServices/serviceCallFailure/status/
  connectionGraphUpdate；subscribe/unsubscribe/advertise/unadvertise/
  subscribeConnectionGraph。

CDR 编解码复用 rclpy 官方序列化（serialize_message/deserialize_message），
与浏览器侧自研 CDR codec 走同一份 .idl 语义，互为交叉验证。

本模块仅测试/诊断用途（对齐 rosbridge 生态的 roslibpy 定位），不进运行链路。
"""
from __future__ import annotations

import asyncio
import json
import logging
import struct
import time as _time
from typing import Any, Callable, Optional

import websockets
from websockets.exceptions import ConnectionClosed

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.serialization import deserialize_message, serialize_message

_logger = logging.getLogger('foxglove_ws_client')

__all__ = ['FoxgloveClientError', 'Subscription', 'FoxgloveWsClient', 'SUBPROTOCOL']

SUBPROTOCOL = 'foxglove.websocket.v1'

SERVER_OP_MESSAGE_DATA = 0x01
SERVER_OP_TIME = 0x02
SERVER_OP_SERVICE_RESPONSE = 0x03

CLIENT_OP_MESSAGE_DATA = 0x01
CLIENT_OP_SERVICE_CALL = 0x02


class FoxgloveClientError(RuntimeError):
    """协议/连接层错误."""


class Subscription:
    """订阅句柄：asyncio.Queue 缓冲 + next(timeout) 拉取."""

    def __init__(self, topic: str, msg_type: type, client: 'FoxgloveWsClient'):
        self.topic = topic
        self.msg_type = msg_type
        self._client = client
        self.id: Optional[int] = None
        self.queue: asyncio.Queue = asyncio.Queue(maxsize=64)
        self.stats = {'messages': 0, 'bytes': 0}

    async def next(self, timeout: float = 5.0):
        """等下一条消息并 CDR 解码；超时抛 asyncio.TimeoutError."""
        item = await asyncio.wait_for(self.queue.get(), timeout)
        self.stats['messages'] += 1
        self.stats['bytes'] += len(item)
        return deserialize_message(bytes(item), self.msg_type)

    async def cancel(self) -> None:
        if self.id is not None and self._client.is_open:
            await self._client._send_json({
                'op': 'unsubscribe',
                'subscriptionIds': [self.id],
            })
        self._client._subscriptions.pop(self.id, None)


class FoxgloveWsClient:
    """单连接 foxglove v1 客户端（经网关代理或直连桥均可）."""

    def __init__(self) -> None:
        self._ws = None
        self._reader_task: Optional[asyncio.Task] = None
        self.server_info: dict[str, Any] = {}
        self.channels: dict[str, dict] = {}       # topic → channel
        self.services: dict[str, dict] = {}       # name → service
        self.graph: dict[str, Any] = {
            'publishedTopics': {}, 'subscribedTopics': {}, 'advertisedServices': {},
        }
        self._subscriptions: dict[int, Subscription] = {}
        self._next_sub_id = 1
        self._next_client_channel_id = 1
        self._client_channels: dict[str, int] = {}
        self._pending_calls: dict[int, asyncio.Future] = {}
        self._next_call_id = 1
        self._service_futures: dict[str, list[asyncio.Future]] = {}
        self._channel_futures: dict[str, list[asyncio.Future]] = {}
        self._closed = asyncio.Event()
        # 桥端 status op（错误通告）：记录 + 日志，不再静默忽略
        self.last_status: dict[str, Any] = {}
        self.status_history: list[dict[str, Any]] = []

    # ── 连接 ──────────────────────────────────────────────────────────────

    @property
    def is_open(self) -> bool:
        return self._ws is not None and not self._ws.closed

    async def connect(self, url: str, timeout: float = 10.0) -> dict:
        """连接并等待 serverInfo（协议首帧）."""
        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(
                    url,
                    subprotocols=[SUBPROTOCOL],
                    max_size=128 * 1024 * 1024,
                    ping_interval=20,
                    ping_timeout=60,
                ),
                timeout)
        except Exception as exc:
            raise FoxgloveClientError(f'connect failed: {exc}') from exc

        negotiated = getattr(self._ws, 'subprotocol', None)
        if negotiated != SUBPROTOCOL:
            await self._ws.close()
            self._ws = None
            raise FoxgloveClientError(
                f'subprotocol mismatch: server picked {negotiated!r}')

        self._reader_task = asyncio.create_task(self._read_loop())
        try:
            await asyncio.wait_for(self._wait_server_info(), timeout)
        except Exception:
            await self.close()
            raise
        return self.server_info

    async def close(self) -> None:
        self._closed.set()
        if self._reader_task:
            self._reader_task.cancel()
            try:
                await self._reader_task
            except asyncio.CancelledError:
                pass  # 主动 cancel 的正常完成路径
            except Exception:
                pass
            self._reader_task = None
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None
        for fut in self._pending_calls.values():
            if not fut.done():
                fut.set_exception(FoxgloveClientError('client closed'))
        self._pending_calls.clear()

    async def _wait_server_info(self) -> None:
        while not self.server_info:
            await asyncio.sleep(0.02)

    # ── 读循环 ────────────────────────────────────────────────────────────

    async def _read_loop(self) -> None:
        try:
            async for raw in self._ws:
                if isinstance(raw, str):
                    self._handle_json(json.loads(raw))
                elif isinstance(raw, (bytes, bytearray)):
                    self._handle_binary(raw)
        except ConnectionClosed:
            pass
        except asyncio.CancelledError:
            raise
        finally:
            for fut in self._pending_calls.values():
                if not fut.done():
                    fut.set_exception(FoxgloveClientError('connection closed'))
            self._pending_calls.clear()

    def _handle_json(self, msg: dict) -> None:
        op = msg.get('op')
        if op == 'serverInfo':
            self.server_info = msg
        elif op == 'advertise':
            for ch in msg.get('channels', []):
                self.channels[ch['topic']] = ch
                self._wake(self._channel_futures, ch['topic'])
        elif op == 'unadvertise':
            for cid in msg.get('channelIds', []):
                for topic, ch in list(self.channels.items()):
                    if ch.get('id') == cid:
                        self.channels.pop(topic, None)
        elif op == 'advertiseServices':
            for svc in msg.get('services', []):
                self.services[svc['name']] = svc
                self._wake(self._service_futures, svc['name'])
        elif op == 'unadvertiseServices':
            for sid in msg.get('serviceIds', []):
                for name, svc in list(self.services.items()):
                    if svc.get('id') == sid:
                        self.services.pop(name, None)
        elif op == 'serviceCallFailure':
            fut = self._pending_calls.pop(msg.get('callId'), None)
            if fut and not fut.done():
                fut.set_exception(
                    FoxgloveClientError(msg.get('message', 'service call failed')))
        elif op == 'connectionGraphUpdate':
            self._apply_graph_update(msg)
        elif op == 'status':
            # 桥端错误/警告通告：记录并按级别日志（旧实现静默忽略，桥端报错对测试不可见）
            self.last_status = msg
            self.status_history.append(msg)
            if len(self.status_history) > 32:
                del self.status_history[:-32]
            if msg.get('level', 1) == 0:
                _logger.debug('server status: %s', msg)
            else:
                _logger.warning('server status: %s', msg)
        # time 等其余 op：忽略

    @staticmethod
    def _wake(store: dict, key: str) -> None:
        for fut in store.pop(key, []):
            if not fut.done():
                fut.set_result(True)

    def _apply_graph_update(self, msg: dict) -> None:
        def to_map(arr, key):
            out = {}
            for e in arr or []:
                if isinstance(e, dict) and 'name' in e:
                    out[e['name']] = e.get(key, [])
            return out

        self.graph['publishedTopics'].update(
            to_map(msg.get('publishedTopics'), 'publisherIds'))
        self.graph['subscribedTopics'].update(
            to_map(msg.get('subscribedTopics'), 'subscriberIds'))
        self.graph['advertisedServices'].update(
            to_map(msg.get('advertisedServices'), 'providerIds'))
        for t in msg.get('removedTopics', []) or []:
            self.graph['publishedTopics'].pop(t, None)
            self.graph['subscribedTopics'].pop(t, None)
        for s in msg.get('removedServices', []) or []:
            self.graph['advertisedServices'].pop(s, None)

    def _handle_binary(self, raw: bytes) -> None:
        if len(raw) < 1:
            return
        op = raw[0]
        if op == SERVER_OP_MESSAGE_DATA and len(raw) >= 13:
            sub_id = struct.unpack_from('<I', raw, 1)[0]
            payload = raw[13:]
            sub = self._subscriptions.get(sub_id)
            if sub is not None:
                try:
                    sub.queue.put_nowait(payload)
                except asyncio.QueueFull:
                    # 队列满 → 丢最旧再入队（与文档声明一致；旧实现丢新帧）
                    try:
                        sub.queue.get_nowait()
                        sub.queue.put_nowait(payload)
                    except (asyncio.QueueEmpty, asyncio.QueueFull):
                        pass
        elif op == SERVER_OP_SERVICE_RESPONSE and len(raw) >= 13:
            service_id, call_id, enc_len = struct.unpack_from('<III', raw, 1)
            enc_end = 13 + enc_len
            if len(raw) < enc_end:
                return
            payload = raw[enc_end:]
            fut = self._pending_calls.pop(call_id, None)
            if fut and not fut.done():
                fut.set_result((service_id, payload))

    # ── 发送 ──────────────────────────────────────────────────────────────

    async def _send_json(self, obj: dict) -> None:
        if not self.is_open:
            raise FoxgloveClientError('not connected')
        # await 发送：与 _send_binary 保持同一发送路径，保证 JSON 控制帧
        # 与二进制数据帧的先后顺序（旧实现 ensure_future fire-and-forget，
        # 发送异常被吞且两路径无顺序保证）
        await self._ws.send(json.dumps(obj))

    async def _send_binary(self, frame: bytes) -> None:
        if not self.is_open:
            raise FoxgloveClientError('not connected')
        await self._ws.send(frame)

    # ── 话题订阅 ──────────────────────────────────────────────────────────

    async def wait_channel(self, topic: str, timeout: float = 10.0) -> dict:
        """等话题出现在桥的 advertise 列表（图上须已有发布者）."""
        deadline = _time.monotonic() + timeout
        while topic not in self.channels:
            remain = deadline - _time.monotonic()
            if remain <= 0:
                raise asyncio.TimeoutError(f'channel not advertised: {topic}')
            fut = asyncio.get_event_loop().create_future()
            self._channel_futures.setdefault(topic, []).append(fut)
            try:
                await asyncio.wait_for(fut, min(remain, 1.0))
            except asyncio.TimeoutError:
                pass
            finally:
                waiters = self._channel_futures.get(topic, [])
                if fut in waiters:
                    waiters.remove(fut)
        return self.channels[topic]

    async def subscribe(self, topic: str, msg_type: type,
                        timeout: float = 10.0) -> Subscription:
        ch = await self.wait_channel(topic, timeout)
        sub = Subscription(topic, msg_type, self)
        sub.id = self._next_sub_id
        self._next_sub_id += 1
        self._subscriptions[sub.id] = sub
        await self._send_json({
            'op': 'subscribe',
            'subscriptions': [{'id': sub.id, 'channelId': ch['id']}],
        })
        return sub

    # ── 客户端发布 ────────────────────────────────────────────────────────

    async def advertise(self, topic: str, schema_name: str) -> int:
        """客户端发布通道（vendored 桥只需 schemaName + encoding=cdr）."""
        if topic in self._client_channels:
            return self._client_channels[topic]
        channel_id = self._next_client_channel_id
        self._next_client_channel_id += 1
        await self._send_json({
            'op': 'advertise',
            'channels': [{
                'id': channel_id,
                'topic': topic,
                'encoding': 'cdr',
                'schemaName': schema_name,
            }],
        })
        self._client_channels[topic] = channel_id
        return channel_id

    async def publish(self, topic: str, schema_name: str, message) -> None:
        """发布一条 rclpy 消息实例（serialize_message → 客户端发布帧）."""
        channel_id = await self.advertise(topic, schema_name)
        payload = bytes(serialize_message(message))
        frame = struct.pack('<BI', CLIENT_OP_MESSAGE_DATA, channel_id) + payload
        await self._send_binary(frame)

    # ── 服务调用 ──────────────────────────────────────────────────────────

    async def wait_service(self, name: str, timeout: float = 10.0) -> dict:
        deadline = _time.monotonic() + timeout
        while name not in self.services:
            remain = deadline - _time.monotonic()
            if remain <= 0:
                raise asyncio.TimeoutError(f'service not advertised: {name}')
            fut = asyncio.get_event_loop().create_future()
            self._service_futures.setdefault(name, []).append(fut)
            try:
                await asyncio.wait_for(fut, min(remain, 1.0))
            except asyncio.TimeoutError:
                pass
            finally:
                waiters = self._service_futures.get(name, [])
                if fut in waiters:
                    waiters.remove(fut)
        return self.services[name]

    async def call_service(self, name: str, request, response_type: type,
                           timeout: float = 15.0):
        """调用 ROS 服务：request 为 rclpy 请求实例，返回解码后的响应."""
        svc = await self.wait_service(name, timeout)
        payload = bytes(serialize_message(request))
        call_id = self._next_call_id
        self._next_call_id += 1
        fut = asyncio.get_event_loop().create_future()
        self._pending_calls[call_id] = fut

        enc = b'cdr'
        frame = (struct.pack('<BII', CLIENT_OP_SERVICE_CALL, svc['id'], call_id)
                 + struct.pack('<I', len(enc)) + enc + payload)
        await self._send_binary(frame)
        try:
            _, resp_payload = await asyncio.wait_for(fut, timeout)
        finally:
            self._pending_calls.pop(call_id, None)
        return deserialize_message(bytes(resp_payload), response_type)

    # ── 参数（标准 ~/get_parameters / ~/set_parameters 服务）─────────────

    @staticmethod
    def _value_from_parameter_value(pv: ParameterValue):
        """rcl_interfaces ParameterValue → Python 值（未知类型返回 None）."""
        t = int(pv.type)
        if t == ParameterType.PARAMETER_BOOL:
            return bool(pv.bool_value)
        if t == ParameterType.PARAMETER_INTEGER:
            return int(pv.integer_value)
        if t == ParameterType.PARAMETER_DOUBLE:
            return float(pv.double_value)
        if t == ParameterType.PARAMETER_STRING:
            return pv.string_value
        if t == ParameterType.PARAMETER_BYTE_ARRAY:
            return bytes(pv.byte_array_value)
        if t == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(pv.bool_array_value)
        if t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return [int(v) for v in pv.integer_array_value]
        if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return [float(v) for v in pv.double_array_value]
        if t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(pv.string_array_value)
        return None

    @staticmethod
    def _parameter_value_from(value) -> ParameterValue:
        """Python 值 → rcl_interfaces ParameterValue（类型推断，对齐 JS 门面）."""
        if isinstance(value, bool):
            return ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                  bool_value=value)
        if isinstance(value, int):
            return ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                  integer_value=value)
        if isinstance(value, float):
            return ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                  double_value=value)
        if isinstance(value, str):
            return ParameterValue(type=ParameterType.PARAMETER_STRING,
                                  string_value=value)
        if isinstance(value, (bytes, bytearray)):
            return ParameterValue(type=ParameterType.PARAMETER_BYTE_ARRAY,
                                  byte_array_value=list(value))
        if isinstance(value, (list, tuple)):
            vals = list(value)
            if not vals:
                return ParameterValue(type=ParameterType.PARAMETER_NOT_SET)
            if all(isinstance(v, bool) for v in vals):
                return ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY,
                                      bool_array_value=vals)
            if all(isinstance(v, int) and not isinstance(v, bool) for v in vals):
                return ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY,
                                      integer_array_value=vals)
            if all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   for v in vals):
                return ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                      double_array_value=[float(v) for v in vals])
            if all(isinstance(v, str) for v in vals):
                return ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY,
                                      string_array_value=vals)
        return ParameterValue(type=ParameterType.PARAMETER_NOT_SET)

    async def get_params(self, node: str, names: list[str] | tuple[str, ...] = (),
                         timeout: float = 10.0) -> dict:
        """读取节点参数（names 为空 = 全部）；返回 {name: value}."""
        node = node.rstrip('/') or '/'
        req = GetParameters.Request(names=list(names))
        resp = await self.call_service(
            f'{node}/get_parameters', req, GetParameters.Response, timeout=timeout)
        out = {}
        for i, pv in enumerate(resp.values):
            name = names[i] if i < len(names) else f'#{i}'
            out[name] = self._value_from_parameter_value(pv)
        return out

    async def set_param(self, node: str, name: str, value,
                        timeout: float = 10.0):
        """写回节点参数；返回 SetParameters 的单个 result（失败抛异常）."""
        node = node.rstrip('/') or '/'
        req = SetParameters.Request(parameters=[
            Parameter(name=name, value=self._parameter_value_from(value))])
        resp = await self.call_service(
            f'{node}/set_parameters', req, SetParameters.Response, timeout=timeout)
        if not resp.results:
            raise FoxgloveClientError(f'set_param 无结果: {node}/{name}')
        result = resp.results[0]
        if not result.successful:
            raise FoxgloveClientError(
                f'set_param 失败: {node}/{name}: {result.reason}')
        return result

    # ── 连接图 ────────────────────────────────────────────────────────────

    async def subscribe_connection_graph(self) -> None:
        await self._send_json({'op': 'subscribeConnectionGraph'})

    def graph_snapshot(self) -> dict:
        return {
            'topics': sorted(set(self.graph['publishedTopics'])
                             | set(self.graph['subscribedTopics'])),
            'services': sorted(self.graph['advertisedServices']),
            'nodes': sorted({
                n
                for ids in (list(self.graph['publishedTopics'].values())
                            + list(self.graph['subscribedTopics'].values())
                            + list(self.graph['advertisedServices'].values()))
                for n in ids
            }),
        }
