// transport/foxglove/codec.js — L2 运行时编解码缓存
//
// 职责：channel/service 的 ros2msg schema 文本 → @foxglove MessageReader/MessageWriter，
// 按 schema 内容缓存（channel id 仅在 schema 完全不变时才被服务器复用，但以
// schemaName+schema 为键最稳）。参考 Lichtblick parseChannel 模式（见 VENDOR_PROVENANCE）。
//
// 服务器 advertiseServices 下发的 service 定义（serialization.cpp to_json）：
//   { id, name, type, request?: {encoding, schemaName, schemaEncoding, schema},
//     response?: {...}, requestSchema?, responseSchema? }
// glue 端服务链路是 CDR 透传（ros2_foxglove_bridge.cpp serviceRequest），故请求编码固定 cdr。

import { parse, MessageReader, MessageWriter } from './vendor/foxglove_cdr.js';

function buildCodec(schemaText, schemaName) {
	// parse(schema, { ros2: true }) → MessageDefinition[]；MessageReader/Writer 直接吃该结构
	const definitions = parse(schemaText, { ros2: true });
	return {
		schemaName,
		reader: new MessageReader(definitions),
		writer: new MessageWriter(definitions),
		definitions,
	};
}

class CodecCache {
	constructor() {
		this._channels = new Map(); // cacheKey → codec
		this._services = new Map(); // service name → {req, resp, service}
		this._stats = { channels: 0, services: 0, decodeErrors: 0, encodeErrors: 0 };
	}

	get stats() {
		return { ...this._stats, channels: this._channels.size, services: this._services.size };
	}

	// ── 话题 channel ─────────────────────────────────────────────────────

	/**
	 * @param {{topic:string, encoding:string, schemaName:string, schema:string, schemaEncoding?:string}} channel
	 * @returns {{readMessage(Uint8Array):object, writeMessage(object):Uint8Array, schemaName:string}|null}
	 *   json 编码 channel 返回 null（调用方按 JSON 处理）。
	 */
	channelCodec(channel) {
		if (!channel || !channel.schema) return null;
		if (channel.encoding === 'json') return null;
		const key = `${channel.schemaName}\n${channel.schema}`;
		let codec = this._channels.get(key);
		if (!codec) {
			try {
				codec = buildCodec(channel.schema, channel.schemaName);
			} catch (e) {
				console.warn(`[fgcodec] schema parse failed for ${channel.topic} (${channel.schemaName}):`, e);
				return null;
			}
			this._channels.set(key, codec);
		}
		return codec;
	}

	// ── 服务 ─────────────────────────────────────────────────────────────

	/** 记录 advertiseServices 下发的服务定义，构建请求 writer/响应 reader。 */
	registerService(service) {
		if (!service || !service.name) return;
		// 注意：schema 为空字符串是合法形态（空请求/空响应段，如 RunLatteWorkflow、
		// ListHardwareInterfaces 的请求段）——零字段 writer 只写 CDR 封装头。
		// 仅 undefined/null 视为“未提供”。
		const reqDef = service.request
			|| (service.requestSchema !== undefined && service.requestSchema !== null
				? { schema: String(service.requestSchema) } : null);
		const respDef = service.response
			|| (service.responseSchema !== undefined && service.responseSchema !== null
				? { schema: String(service.responseSchema) } : null);
		let req = null;
		let resp = null;
		try {
			if (reqDef) req = buildCodec(reqDef.schema || '', reqDef.schemaName || `${service.type}_Request`);
		} catch (e) {
			console.warn(`[fgcodec] service request schema parse failed (${service.name}):`, e);
		}
		try {
			if (respDef) resp = buildCodec(respDef.schema || '', respDef.schemaName || `${service.type}_Response`);
		} catch (e) {
			console.warn(`[fgcodec] service response schema parse failed (${service.name}):`, e);
		}
		this._services.set(service.name, { req, resp, service });
	}

	unregisterServices(names) {
		for (const n of names) this._services.delete(n);
	}

	serviceCodec(name) {
		return this._services.get(name) || null;
	}

	/** 服务名 → serviceId（上层调用前查询；未广播返回 undefined） */
	serviceId(name) {
		const entry = this._services.get(name);
		return entry ? entry.service.id : undefined;
	}

	/** 编码服务请求；无 schema 时抛错（无定义的服务无法构造 CDR）。 */
	encodeServiceRequest(name, requestObj) {
		const entry = this._services.get(name);
		if (!entry) throw new Error(`service not advertised: ${name}`);
		if (!entry.req) throw new Error(`service ${name} has no request schema`);
		try {
			return entry.req.writer.writeMessage(requestObj);
		} catch (e) {
			this._stats.encodeErrors++;
			throw new Error(`service ${name} request encode failed: ${e.message}`);
		}
	}

	/** 解码服务响应；无 schema 时原样返回字节（供调试）。 */
	decodeServiceResponse(name, payload) {
		const entry = this._services.get(name);
		if (!entry || !entry.resp) return payload;
		try {
			return entry.resp.reader.readMessage(payload);
		} catch (e) {
			this._stats.decodeErrors++;
			throw new Error(`service ${name} response decode failed: ${e.message}`);
		}
	}

	/** 会话重置（sessionId 变化时全部失效）。 */
	reset() {
		this._channels.clear();
		this._services.clear();
	}
}

export { CodecCache, buildCodec };
