// transport/foxglove/__tests__/protocol.test.js — L0 协议客户端纯核单测
// 协议帧的编解码与服务器源码逐字节核对（vendor websocket_server.hpp / serialization.cpp）：
//   服务器→客户端: [0x01][subId u32][ts u64][payload] / [0x02][ts u64] /
//                  [0x03][serviceId u32][callId u32][encLen u32][enc][data]
//   客户端→服务器: [0x01][channelId u32][payload] /
//                  [0x02][serviceId u32][callId u32][encLen u32][enc][data]
// 测试用伪 WebSocket 注入（零网络）。
import { describe, it, expect, beforeEach } from 'vitest';

// 协议模块只依赖全局 WebSocket 符号——注入伪类
class FakeWebSocket {
  constructor(url, subprotocol) {
    this.url = url;
    this.subprotocolRequests = subprotocol;
    this.sent = [];
    this.readyState = 0;
    FakeWebSocket.instances.push(this);
    queueMicrotask(() => {
      this.readyState = 1;
      this.onopen && this.onopen();
      // serverInfo 首帧
      this._serverSend(JSON.stringify({
        op: 'serverInfo', name: 'ivg_foxglove_bridge', sessionId: 's1',
        capabilities: ['clientPublish', 'services', 'connectionGraph'],
        supportedEncodings: ['cdr'],
      }));
    });
  }
  _serverSend(data) {
    const ev = { data };
    this.onmessage && this.onmessage(ev);
  }
  send(x) { this.sent.push(x); }
  close() { this.readyState = 3; this.onclose && this.onclose({ code: 1000 }); }
}
FakeWebSocket.instances = [];

async function loadProtocol() {
  globalThis.WebSocket = FakeWebSocket;
  return import('../protocol.js');
}

function bytesEqual(a, b) {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}

describe('FoxgloveProtocolClient', () => {
  let proto;
  beforeEach(async () => {
    FakeWebSocket.instances = [];
    const { FoxgloveProtocolClient } = await loadProtocol();
    proto = new FoxgloveProtocolClient();
  });

  it('connect 协商 foxglove.websocket.v1 并 resolve serverInfo', async () => {
    const info = await proto.connect('ws://x/ws/foxglove');
    expect(info.op).toBe('serverInfo');
    expect(proto.isOpen).toBe(true);
    expect(FakeWebSocket.instances[0].subprotocolRequests).toEqual('foxglove.websocket.v1');
  });

  it('subscribe/unsubscribe/advertise/unadvertise 用批量格式（0.8.5 服务器线）', async () => {
    await proto.connect('ws://x');
    proto.subscribe(7, 42);
    proto.unsubscribe(9);
    proto.advertise(3, { topic: '/t', schemaName: 'std_msgs/msg/String', schema: 'string data' });
    proto.unadvertise(4);
    const sent = FakeWebSocket.instances[0].sent.map(JSON.parse);
    expect(sent[0]).toEqual({ op: 'subscribe', subscriptions: [{ id: 7, channelId: 42 }] });
    expect(sent[1]).toEqual({ op: 'unsubscribe', subscriptionIds: [9] });
    expect(sent[2].op).toBe('advertise');
    expect(sent[2].channels[0].id).toBe(3);
    expect(sent[2].channels[0].topic).toBe('/t');
    expect(sent[3]).toEqual({ op: 'unadvertise', channelIds: [4] });
  });

  it('客户端发布帧 [0x01][channelId u32 LE][payload]', async () => {
    await proto.connect('ws://x');
    proto.sendMessageData(0x0a0b0c0d, new Uint8Array([0xaa, 0xbb]));
    const frame = FakeWebSocket.instances[0].sent.find((x) => x instanceof Uint8Array);
    expect(bytesEqual(Array.from(frame), [0x01, 0x0d, 0x0c, 0x0b, 0x0a, 0xaa, 0xbb])).toBe(true);
  });

  it('服务调用请求帧 [0x02][serviceId][callId][encLen]["cdr"][payload]', async () => {
    await proto.connect('ws://x');
    proto.sendServiceCallRequest(0x11223344, 5, new Uint8Array([0x99]));
    const frame = FakeWebSocket.instances[0].sent.find((x) => x instanceof Uint8Array);
    const expectHead = [0x02, 0x44, 0x33, 0x22, 0x11, 5, 0, 0, 0, 3, 0, 0, 0, ...[...('cdr')].map(c => c.charCodeAt(0)), 0x99];
    expect(bytesEqual(Array.from(frame), expectHead)).toBe(true);
  });

  it('解析服务器消息帧（含 u64 时间戳）', async () => {
    await proto.connect('ws://x');
    const got = new Promise((r) => proto.on('message', r));
    const buf = new ArrayBuffer(13 + 2);
    const u8 = new Uint8Array(buf);
    const dv = new DataView(buf);
    u8[0] = 0x01;
    dv.setUint32(1, 3, true);
    dv.setBigUint64(5, 1790000000123456789n, true);
    u8[13] = 1; u8[14] = 2;
    FakeWebSocket.instances[0]._serverSend(buf);
    const m = await got;
    expect(m.subscriptionId).toBe(3);
    expect(m.timestamp).toBe(1790000000123456789);
    expect(m.payload.byteLength).toBe(2);
  });

  it('解析服务响应帧 [0x03][serviceId][callId][encLen][enc][data]', async () => {
    await proto.connect('ws://x');
    const got = new Promise((r) => proto.on('serviceCallResponse', r));
    const enc = new TextEncoder().encode('cdr');
    const buf = new ArrayBuffer(13 + enc.length + 1);
    const u8 = new Uint8Array(buf);
    const dv = new DataView(buf);
    u8[0] = 0x03;
    dv.setUint32(1, 77, true);
    dv.setUint32(5, 88, true);
    dv.setUint32(9, enc.length, true);
    u8.set(enc, 13);
    u8[13 + enc.length] = 0x5a;
    FakeWebSocket.instances[0]._serverSend(buf);
    const r = await got;
    expect(r.serviceId).toBe(77);
    expect(r.callId).toBe(88);
    expect(r.encoding).toBe('cdr');
    expect(r.payload[0]).toBe(0x5a);
  });

  it('主动 close 不再触发 close 事件（防自研重连调度误启）', async () => {
    await proto.connect('ws://x');
    let closeEvents = 0;
    proto.on('close', () => closeEvents++);
    proto.close();
    await new Promise((r) => setTimeout(r, 5));
    expect(closeEvents).toBe(0);
    expect(proto.isOpen).toBe(false);
  });

  it('迟到的旧 socket close 不误杀新连接', async () => {
    await proto.connect('ws://x');
    const oldWs = FakeWebSocket.instances[0];
    // 模拟重连：断开后旧 socket 才回调 onclose
    oldWs.readyState = 3;
    proto.close();
    const info = await proto.connect('ws://x');
    expect(info.op).toBe('serverInfo');
    const stale = FakeWebSocket.instances[0];
    stale.onclose({ code: 1006 }); // 迟到事件
    expect(proto.isOpen).toBe(true); // 新连接仍在
  });
});
