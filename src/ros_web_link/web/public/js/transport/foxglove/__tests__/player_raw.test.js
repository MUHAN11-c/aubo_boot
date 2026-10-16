// player_raw.test.js — L1 player raw 派发模式纯核单测（零网络，伪 WebSocket 注入）
//
// raw 模式语义（2026-10-16 优化轮）：latest 同款微任务合并，但回调拿到原始
// CDR 载荷（data=Uint8Array, raw=true），不做任何解码；codec 也不构建。
// 场景：话题卡等统计-only 订阅——大消息全量 CDR 解码是主线程纯浪费。
import { describe, it, expect, beforeEach } from 'vitest';

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
			this._serverSend(JSON.stringify({
				op: 'serverInfo', name: 'ivg_foxglove_bridge', sessionId: 's1',
				capabilities: ['clientPublish', 'services'],
				supportedEncodings: ['cdr'],
			}));
		});
	}
	_serverSend(data) {
		this.onmessage && this.onmessage({ data });
	}
	send(x) { this.sent.push(x); }
	close() { this.readyState = 3; this.onclose && this.onclose({ code: 1000 }); }
}
FakeWebSocket.instances = [];

const flushMicro = () => new Promise((r) => setTimeout(r, 0));

function msgDataFrame(subscriptionId, timestampNs, payload) {
	const buf = new ArrayBuffer(13 + payload.length);
	const u8 = new Uint8Array(buf);
	const dv = new DataView(buf);
	u8[0] = 0x01;
	dv.setUint32(1, subscriptionId, true);
	dv.setBigUint64(5, timestampNs, true);
	u8.set(payload, 13);
	return buf;
}

describe('FoxglovePlayer raw 派发模式', () => {
	let player;
	let ws;
	beforeEach(async () => {
		FakeWebSocket.instances = [];
		globalThis.WebSocket = FakeWebSocket;
		const { FoxglovePlayer } = await import('../player.js');
		player = new FoxglovePlayer();
		await player.connect('ws://x');
		ws = FakeWebSocket.instances[0];
		ws._serverSend(JSON.stringify({
			op: 'advertise',
			channels: [{
				id: 7, topic: '/big_cloud', encoding: 'cdr',
				schemaName: 'std_msgs/msg/String',
				schema: 'string data',
				schemaEncoding: 'ros2msg',
			}],
		}));
	});

	it('raw 模式：连续两帧合并为一帧、零解码、载荷原样透传', async () => {
		const got = [];
		player.subscribe('/big_cloud', (m) => got.push(m), { dispatchMode: 'raw' });
		// 订阅请求已发出（subscriptionId 由客户端分配，首个为 1）
		expect(ws.sent.some((x) => String(x).includes('"subscribe"'))).toBe(true);

		// 同一微任务轮内连续到达两帧：只派发最新一帧
		ws._serverSend(msgDataFrame(1, 1790000000123456789n, new TextEncoder().encode('first')));
		ws._serverSend(msgDataFrame(1, 1790000000999999999n, new TextEncoder().encode('second')));
		await flushMicro();

		expect(got.length).toBe(1);
		const m = got[0];
		expect(m.raw).toBe(true);
		expect(m.topic).toBe('/big_cloud');
		expect(m.type).toBe('std_msgs/msg/String');
		expect(m.data instanceof Uint8Array).toBe(true);
		expect(new TextDecoder().decode(m.data)).toBe('second');
		// 两帧都进了字节/条数统计（raw 只省解码，不省记账）
		expect(player.stats.messagesReceived).toBe(2);
		// raw 订阅不构建 codec（player 内部 sub.codec === null）
		const sub = player._topicSubs.get('/big_cloud');
		expect(sub.codec).toBeNull();
		expect(player.stats.decodeErrors).toBe(0);
	});

	it('latest 模式仍走解码路径（回归对照）', async () => {
		const got = [];
		player.subscribe('/big_cloud', (m) => got.push(m), { dispatchMode: 'latest' });
		// 畸形 CDR 载荷：latest 会尝试解码并计 decodeErrors；raw 不会
		ws._serverSend(msgDataFrame(1, 1n, new Uint8Array([0xff, 0xff, 0xff])));
		await flushMicro();
		expect(got.length).toBe(0);
		expect(player.stats.decodeErrors).toBeGreaterThan(0);
	});
});
