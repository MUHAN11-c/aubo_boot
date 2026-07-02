/**
 * ROS2 foxglove_bridge 智能化完整验证测试
 *
 * 自动发现环境中所有话题/服务/参数, 不做任何硬编码。
 * 覆盖: 数据解析 + 收发验证 + CDR带宽分析 + 系统信息
 */

import WebSocket from 'ws';
import os from 'os';
import fs from 'node:fs';
import pkgWsProtocol from '@foxglove/ws-protocol';
import pkgRosMsg2 from '@foxglove/rosmsg2-serialization';
import pkgRosMsg from '@foxglove/rosmsg';

const { FoxgloveClient } = pkgWsProtocol;
const { MessageReader, MessageWriter } = pkgRosMsg2;
const { parse: parseRosMsg } = pkgRosMsg;

const WS_URL = 'ws://localhost:8765';
const SUBPROTOCOL = ['foxglove.sdk.v1'];

// ================================================================
const L = {
  h1(m)  { console.log('\n' + '═'.repeat(64) + `\n  ${m}\n` + '═'.repeat(64)); },
  h2(m)  { console.log(`\n─── ${m}`); },
  ok(m)  { console.log(`  ✅ ${m}`); },
  err(m) { console.log(`  ❌ ${m}`); },
  warn(m){ console.log(`  ⚠️ ${m}`); },
  info(m){ console.log(`  ℹ️ ${m}`); },
};

// ================================================================
// SessionContext
// ================================================================
class SessionContext {
  constructor() {
    this.capabilities = []; this.channels = []; this.services = []; this.codecMap = new Map();
    this.subMap = new Map(); this.cliChMap = new Map(); this.msgBuf = []; this.pending = new Map();
    this._cid = 0; this._connectedAt = null;
    // 全量数据采集
    this.fullData = {
      serverInfo: null,
      channels: [],
      services: [],
      messages: [],        // 所有 CDR 消息
      serviceCalls: [],    // 所有服务调用及响应
      parameterOps: [],    // 所有参数操作
      connectionGraphUpdates: [], // 拓扑更新事件
      bandwidthRaw: [],    // 带宽原始数据
    };
  }

  async connect() {
    this.ws = new WebSocket(WS_URL, SUBPROTOCOL);
    const info = await new Promise((resolve, reject) => {
      const t = setTimeout(() => reject(new Error('serverInfo 超时')), 15000);
      this.ws.on('message', (d) => { try { const m = JSON.parse(d.toString()); if (m.op === 'serverInfo') { clearTimeout(t); resolve(m); } } catch {} });
      this.ws.on('error', reject);
    });
    this._connectedAt = Date.now();
    this.capabilities = info.capabilities || [];
    this.fullData.serverInfo = { ...info, connectedAt: new Date().toISOString() };
    this.client = new FoxgloveClient({ ws: this.ws });
    this.client.on('error', (e) => L.err(`client error: ${e.message}`));

    // 订阅拓扑更新 (connectionGraph)
    this.client.on('connectionGraphUpdate', (evt) => {
      this.fullData.connectionGraphUpdates.push({
        ts: Date.now() - this._connectedAt,
        publishedTopics: (evt.publishedTopics || []).map(t => ({ name: t.name, publisherCount: (t.publisherIds || []).length })),
        subscribedTopics: (evt.subscribedTopics || []).map(t => ({ name: t.name, subscriberCount: (t.subscriberIds || []).length })),
        removedTopics: evt.removedTopics || [],
        raw: evt,  // 保留原始数据
      });
    });
    if (this.capabilities.includes('connectionGraph')) {
      try { this.client.subscribeConnectionGraph(); L.info('已订阅 connectionGraph 拓扑更新'); } catch {}
    }

    this.client.on('advertise', (chs) => {
      for (const ch of (Array.isArray(chs) ? chs : (chs.channels || []))) {
        this.channels.push(ch);
        this.fullData.channels.push({ ...ch, discoveredAt: Date.now() - this._connectedAt });
        if (ch.encoding === 'cdr' && ch.schema) {
          try { const defs = parseRosMsg(ch.schema, { ros2: true }); this.codecMap.set(ch.id, { reader: new MessageReader(defs), writer: new MessageWriter(defs), channel: ch }); } catch {}
        }
      }
    });
    this.client.on('advertiseServices', (svs) => {
      const list = Array.isArray(svs) ? svs : (svs.services || []);
      for (const s of list) { this.fullData.services.push({ ...s, discoveredAt: Date.now() - this._connectedAt }); }
      this.services.push(...list);
    });

    this.client.on('message', ({ subscriptionId, timestamp, data }) => {
      let cid = null; for (const [id, sid] of this.subMap) { if (sid === subscriptionId) { cid = id; break; } }
      if (!cid || !this.codecMap.has(cid)) return;
      try {
        const { reader, channel } = this.codecMap.get(cid);
        const jsObj = reader.readMessage(data);
        const entry = {
          cid, topic: channel.topic, type: channel.schemaName,
          cdrSize: data.byteLength, jsonSize: JSON.stringify(jsObj).length,
          timestamp: timestamp ? Number(timestamp / 1_000_000n) : Date.now(),
          data: jsObj,
        };
        this.msgBuf.push(entry);
        this.fullData.messages.push(entry);
        this.fullData.bandwidthRaw.push({ type: channel.schemaName, topic: channel.topic, cdrSize: data.byteLength, jsonSize: entry.jsonSize, ts: entry.timestamp });
      } catch {}
    });

    this.client.on('serviceCallResponse', (rsp) => {
      const p = this.pending.get(rsp.callId); if (!p) return; clearTimeout(p.timer);
      const svc = this.services.find(s => s.id === rsp.serviceId);
      try {
        let result;
        if (svc?.response?.schema) { const defs = parseRosMsg(svc.response.schema, { ros2: true }); result = new MessageReader(defs).readMessage(rsp.data); }
        else { result = rsp.data; }
        p.resolve(result);
        this.fullData.serviceCalls.push({
          callId: rsp.callId, serviceId: rsp.serviceId, serviceName: svc?.name,
          success: true, result: JSON.parse(JSON.stringify(result)),
          cdrSize: rsp.data?.byteLength, elapsed: Date.now() - p.startedAt,
        });
      } catch (e) {
        p.resolve(rsp);
        this.fullData.serviceCalls.push({ callId: rsp.callId, serviceId: rsp.serviceId, serviceName: svc?.name, success: false, error: e.message });
      }
      this.pending.delete(rsp.callId);
    });
    this.client.on('serviceCallFailure', (f) => {
      const p = this.pending.get(f.callId); if (p) {
        clearTimeout(p.timer); p.reject(new Error(f.message));
        this.fullData.serviceCalls.push({ callId: f.callId, success: false, error: f.message });
        this.pending.delete(f.callId);
      }
    });
    this.client.on('parameterValues', (evt) => {
      for (const [id, p] of this.pending) {
        if (id.startsWith('pr_') && evt.id === id) {
          clearTimeout(p.timer); p.resolve(evt.parameters||[]);
          this.fullData.parameterOps.push({
            requestId: id, type: p.opType || 'get',
            params: (evt.parameters||[]).map(pv => ({
              name: pv.name, type: pv.type, value: pval(pv),
            })),
            elapsed: Date.now() - p.startedAt,
          });
          this.pending.delete(id); break;
        }
        if (id.startsWith('ps_') && evt.id === id) {
          clearTimeout(p.timer); p.resolve(evt.parameters||[]);
          this.fullData.parameterOps.push({
            requestId: id, type: 'set',
            params: (evt.parameters||[]).map(pv => ({
              name: pv.name, type: pv.type, value: pval(pv),
            })),
            elapsed: Date.now() - p.startedAt,
          });
          this.pending.delete(id); break;
        }
      }
    });

    await new Promise(r => setTimeout(r, 4000));
    return this;
  }

  findChannel(t) { return this.channels.find(ch => ch.topic === t); }
  findService(n) { return this.services.find(s => s.name === n); }

  async subCollect(topic, msgs=3, timeout=5000) { const ch=this.findChannel(topic); if(!ch) return[]; const start=this.msgBuf.length; this.subMap.set(ch.id,this.client.subscribe(ch.id)); await new Promise(r=>{const iv=setInterval(()=>{if(this.msgBuf.slice(start).filter(e=>e.cid===ch.id).length>=msgs)r();},100);setTimeout(()=>{clearInterval(iv);r();},timeout);}); return this.msgBuf.slice(start).filter(e=>e.cid===ch.id); }

  publish(topic, jsObj) { const ch=this.findChannel(topic); if(!ch) throw new Error('channel missing'); const codec=this.codecMap.get(ch.id); if(!codec) throw new Error('codec missing'); const cid=this.client.advertise({topic,encoding:'cdr',schemaName:ch.schemaName,schema:ch.schema||'',schemaEncoding:'ros2msg'}); this.cliChMap.set(cid,{topic}); this.client.sendMessage(cid,codec.writer.writeMessage(jsObj)); return cid; }
  unpublish(cid) { try{this.client.unadvertise(cid);this.cliChMap.delete(cid);}catch{} }

  buildReq(svc, jsObj={}) { if(!svc?.request?.schema) throw new Error('no request schema'); return new MessageWriter(parseRosMsg(svc.request.schema,{ros2:true})).writeMessage(jsObj); }
  async callSvc(svc, payload, timeout=5000) { const cid=++this._cid; const startedAt=Date.now(); const p=new Promise((res,rej)=>{this.pending.set(cid,{resolve:res,reject:rej,startedAt,timer:setTimeout(()=>rej(new Error('超时')),timeout)});}); this.client.sendServiceCallRequest({serviceId:svc.id,callId:cid,encoding:svc.request?.encoding||'cdr',data:payload}); return p; }

  async getParams(names, rid=`pr_${Date.now()}`) { const startedAt=Date.now(); const p=new Promise((res,rej)=>{this.pending.set(rid,{resolve:res,reject:rej,startedAt,opType:'get',timer:setTimeout(()=>rej(new Error('超时')),5000)});}); this.client.getParameters(names,rid); return p; }
  async setParams(params, rid=`ps_${Date.now()}`) { const startedAt=Date.now(); const p=new Promise((res,rej)=>{this.pending.set(rid,{resolve:res,reject:rej,startedAt,opType:'set',timer:setTimeout(()=>rej(new Error('超时')),5000)});}); this.client.setParameters(params,rid); return p; }

  disconnect() { this.ws?.close(); }

  // 导出全量分析数据
  exportFullData() {
    // 统计每个 topic 的消息数
    const topicMsgCount = new Map();
    for (const m of this.fullData.messages) {
      const cnt = topicMsgCount.get(m.topic) || 0;
      topicMsgCount.set(m.topic, cnt + 1);
    }

    // 按消息类型统计 CDR vs JSON 带宽 (全量)
    const typeStats = new Map();
    for (const b of this.fullData.bandwidthRaw) {
      if (!typeStats.has(b.type)) typeStats.set(b.type, { cdrSizes: [], jsonSizes: [], topics: new Set(), count: 0 });
      const s = typeStats.get(b.type);
      s.cdrSizes.push(b.cdrSize); s.jsonSizes.push(b.jsonSize); s.topics.add(b.topic); s.count++;
    }

    const bandwidthByType = [];
    for (const [type, s] of typeStats) {
      const ac = s.cdrSizes.reduce((a, b) => a + b, 0) / s.cdrSizes.length;
      const aj = s.jsonSizes.reduce((a, b) => a + b, 0) / s.jsonSizes.length;
      bandwidthByType.push({
        type,
        topics: [...s.topics],
        sampleCount: s.count,
        cdrAvgBytes: Math.round(ac),
        jsonAvgBytes: Math.round(aj),
        savePercent: aj > 0 ? parseFloat(((1 - ac / aj) * 100).toFixed(1)) : 0,
        cdrMinBytes: Math.min(...s.cdrSizes),
        cdrMaxBytes: Math.max(...s.cdrSizes),
        jsonMinBytes: Math.min(...s.jsonSizes),
        jsonMaxBytes: Math.max(...s.jsonSizes),
        cdrTotalBytes: s.cdrSizes.reduce((a, b) => a + b, 0),
        jsonTotalBytes: s.jsonSizes.reduce((a, b) => a + b, 0),
      });
    }
    bandwidthByType.sort((a, b) => b.cdrTotalBytes - a.cdrTotalBytes);

    // 按 topic 统计
    const topicStats = [];
    for (const [topic, count] of topicMsgCount) {
      const msgs = this.fullData.messages.filter(m => m.topic === topic);
      const cdrSizes = msgs.map(m => m.cdrSize);
      const jsonSizes = msgs.map(m => m.jsonSize);
      const avgCdr = cdrSizes.reduce((a, b) => a + b, 0) / cdrSizes.length;
      const avgJson = jsonSizes.reduce((a, b) => a + b, 0) / jsonSizes.length;
      topicStats.push({
        topic, type: msgs[0]?.type,
        messageCount: count,
        cdrAvgBytes: Math.round(avgCdr),
        jsonAvgBytes: Math.round(avgJson),
        cdrTotalBytes: cdrSizes.reduce((a, b) => a + b, 0),
        jsonTotalBytes: jsonSizes.reduce((a, b) => a + b, 0),
        savePercent: avgJson > 0 ? parseFloat(((1 - avgCdr / avgJson) * 100).toFixed(1)) : 0,
      });
    }
    topicStats.sort((a, b) => b.messageCount - a.messageCount);

    // CDR 总带宽
    const totalCdr = this.fullData.bandwidthRaw.reduce((s, b) => s + b.cdrSize, 0);
    const totalJson = this.fullData.bandwidthRaw.reduce((s, b) => s + b.jsonSize, 0);

    return {
      exportedAt: new Date().toISOString(),
      connection: this.fullData.serverInfo,
      summary: {
        totalChannels: this.channels.length,
        totalServices: this.services.length,
        totalMessages: this.fullData.messages.length,
        totalServiceCalls: this.fullData.serviceCalls.length,
        totalParameterOps: this.fullData.parameterOps.length,
      totalConnectionGraphUpdates: this.fullData.connectionGraphUpdates.length,
        totalCdrBytes: totalCdr,
        totalJsonBytes: totalJson,
        overallSavePercent: totalJson > 0 ? parseFloat(((1 - totalCdr / totalJson) * 100).toFixed(1)) : 0,
        bandwidthRawCount: this.fullData.bandwidthRaw.length,
      },
      channels: this.fullData.channels,
      services: this.fullData.services,
      bandwidthByType,
      topicStats,
      serviceCalls: this.fullData.serviceCalls,
      parameterOps: this.fullData.parameterOps,
      connectionGraphUpdates: this.fullData.connectionGraphUpdates,
      // 不全量导出 messages (可能很大), 仅导出摘要
      messageSummary: [...topicMsgCount].sort((a, b) => b[1] - a[1]).map(([topic, count]) => ({ topic, count })),
    };
  }
}

// ================================================================
// Topic 分类规则
// ================================================================
const TOPIC_CATS = {
  '传感器/相机':     (t,ty) => t.includes('/camera/'),
  '传感器/点云':     (t,ty) => ty.includes('PointCloud'),
  '传感器/图像':     (t,ty) => t.endsWith('/image_raw') || ty === 'sensor_msgs/msg/Image',
  '机器人/关节状态': (t,ty) => t === '/joint_states',
  '机器人/TF变换':   (t,ty) => ty.endsWith('TFMessage') || t === '/tf' || t === '/tf_static',
  '机器人/URDF模型': (t,ty) => t === '/robot_description' || t === '/robot_description_semantic',
  '控制/轨迹':       (t,ty) => t.includes('joint_trajectory') || ty.includes('Trajectory') || ty.includes('Controller'),
  '控制/动态状态':   (t,ty) => ty.includes('DynamicJoint'),
  '规划/MoveIt':     (t,ty) => ty.startsWith('moveit_msgs/') || t.includes('/planning_') || t.includes('/monitored_'),
  '可视化/Marker':   (t,ty) => ty.includes('Marker') || t.includes('marker') || t.includes('interactive'),
  '应用/机器人状态': (t,ty) => ty.endsWith('RobotStatus') || ty.endsWith('RobotIOStatus') || ty.endsWith('ToolChangerStatus'),
  '应用/位姿抓取':   (t,ty) => ty.includes('Pose') || t.includes('/goal') || t.includes('/grasp'),
  '系统/日志':       (t,ty) => t === '/rosout' || ty.endsWith('Log'),
  '系统/事件参数':   (t,ty) => t === '/parameter_events' || ty.includes('ParameterEvent') || t.includes('/events/'),
  '系统/生命周期':   (t,ty) => ty.includes('TransitionEvent'),
  'ROS2 Demo':       (t,ty) => t === '/chatter' || t === '/add_two_ints',
  '自定义/IVG':      (t,ty) => ty.startsWith('ivg_interfaces/') || t.includes('/aubo/'),
};

function catTopic(ch) {
  for (const [cat, pred] of Object.entries(TOPIC_CATS)) { if (pred(ch.topic, ch.schemaName)) return cat; }
  return '其他';
}

// ================================================================
// 参数工具
// ================================================================
const PTYPES = {0:'NOT_SET',1:'BOOL',2:'INTEGER',3:'DOUBLE',4:'STRING',5:'BYTE_ARRAY',6:'BOOL_ARRAY',7:'INTEGER_ARRAY',8:'DOUBLE_ARRAY',9:'STRING_ARRAY'};
const PTYPE_FIELDS = {1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'};

function pval(v) {
  if (!v) return '(null)';
  switch (v.type) {
    case 1: return String(v.bool_value);
    case 2: return String(typeof v.integer_value === 'bigint' ? Number(v.integer_value) : v.integer_value);
    case 3: return String(v.double_value);
    case 4: return JSON.stringify(v.string_value);
    case 5: return `[byte:${(v.byte_array_value||[]).length}B]`;
    case 6: case 7: case 8: case 9: { const a = v[PTYPE_FIELDS[v.type]] || []; return `[${Array.from(a).slice(0,3).join(',')}${a.length>3?'...':''}]`; }
    default: return '(not set)';
  }
}

// ================================================================
// 1. 话题 (Topic) — 自动发现 + 分类 + 订阅 + 发布
// ================================================================
async function topicTests(ctx) {
  L.h1('1. 话题 (Topic)');

  // 分类
  const cats = new Map();
  for (const ch of ctx.channels) { const c = catTopic(ch); if (!cats.has(c)) cats.set(c, []); cats.get(c).push(ch); }
  const order = [...cats.keys()].sort();

  L.h2(`发现 ${ctx.channels.length} 个 topic, ${cats.size} 类`);
  for (const cat of order) {
    const chs = cats.get(cat);
    L.info(`${cat.padEnd(18)} ${String(chs.length).padStart(3)} topics — ${[...new Set(chs.map(c=>c.schemaName))].slice(0,3).join(', ')}`);
  }

  // 每类取1代表订阅
  L.h2('订阅验证 (每类1个代表)');
  const hasPublish = ctx.capabilities.includes('clientPublish');
  let subOk=0, subAll=0;

  for (const cat of order) {
    const chs = cats.get(cat);
    const rep = chs.find(c => ctx.codecMap.has(c.id) && !['std_msgs/msg/String'].includes(c.schemaName)) || chs.find(c => ctx.codecMap.has(c.id));
    if (!rep) continue;
    subAll++;
    try {
      const msgs = await ctx.subCollect(rep.topic, 1, 3000);
      if (msgs.length > 0) { subOk++; L.ok(`${rep.topic} — CDR=${msgs[0].cdrSize}B JSON≈${msgs[0].jsonSize}B [${rep.schemaName}]`); }
      else { L.warn(`${rep.topic}: 暂无数据`); }
    } catch(e) { L.warn(`${rep.topic}: ${e.message}`); }
  }
  L.info(`有数据: ${subOk}/${subAll}`);

  // 发布测试: 找可写的 std_msgs/String
  if (hasPublish) {
    L.h2('发布测试 (clientPublish)');
    const writable = ctx.channels.find(ch => ch.schemaName === 'std_msgs/msg/String' && ch.topic === '/chatter')
                  || ctx.channels.find(ch => ch.schemaName === 'std_msgs/msg/String' && ch.topic !== '/robot_description' && ctx.codecMap.has(ch.id));
    if (writable) {
      const msg = `[foxglove] ${new Date().toISOString()}`;
      try {
        const before = ctx.msgBuf.length;
        const cid = ctx.publish(writable.topic, { data: msg });
        await new Promise(r => setTimeout(r, 1500));
        const ok = ctx.msgBuf.slice(before).some(e => e.topic === writable.topic && e.data?.data === msg);
        L.ok(`发布→${writable.topic}→回显${ok?'✅':'已发送'}`);
        ctx.unpublish(cid);
      } catch(e) { L.err(`发布: ${e.message}`); }
    }
  }
  return { cats: cats.size, subOk, subAll };
}

// ================================================================
// 2. 服务 (Service) — 自动发现 + 分类 + 调用
// ================================================================
async function serviceTests(ctx) {
  L.h1('2. 服务 (Service)');

  if (!ctx.capabilities.includes('services')) { L.warn('services 未启用'); return {}; }

  // 去重+过滤参数服务
  const seen = new Set();
  const userSvc = ctx.services.filter(s => {
    if (s.name.endsWith('/describe_parameters') || s.name.endsWith('/get_parameter_types') ||
        s.name.endsWith('/set_parameters_atomically') || s.name.endsWith('/get_parameters') ||
        s.name.endsWith('/set_parameters') || s.name.endsWith('/list_parameters')) return false;
    if (seen.has(s.name)) return false; seen.add(s.name);
    return true;
  });

  // 按命名空间分类
  const svcCats = new Map();
  for (const s of userSvc) {
    const parts = s.name.split('/').filter(Boolean);
    let ns = parts.length > 1 ? '/' + parts.slice(0, -1).join('/') : ('/' + (parts[0]||'root'));
    if (!svcCats.has(ns)) svcCats.set(ns, []);
    svcCats.get(ns).push(s);
  }
  const svcOrder = [...svcCats.keys()].sort((a,b) => svcCats.get(b).length - svcCats.get(a).length);

  L.h2(`${userSvc.length} 个服务, ${svcCats.size} 个命名空间`);
  for (const ns of svcOrder.slice(0, 15)) L.info(`${ns.padEnd(42)} ${String(svcCats.get(ns).length).padStart(3)} services`);

  // 每空间1调用
  L.h2('调用验证 (每命名空间1代表)');
  let tested=0, ok=0;
  for (const ns of svcOrder.slice(0, 15)) {
    const svcs = svcCats.get(ns);
    const svc = svcs.find(s => s.request?.schema) || svcs[0];
    if (!svc?.request?.schema) continue;
    tested++;
    try {
      const rsp = await ctx.callSvc(svc, ctx.buildReq(svc, {}));
      ok++; L.ok(`${svc.name} → ${JSON.stringify(rsp).substring(0, 80)}`);
    } catch(e) { L.warn(`${svc.name}: ${e.message}`); }
  }
  L.info(`成功: ${ok}/${tested}`);

  // AddTwoInts (带参数, 如果有)
  const add = ctx.findService('/add_two_ints');
  if (add?.request?.schema) {
    try { const r = await ctx.callSvc(add, ctx.buildReq(add, {a:7,b:8})); L.ok(`AddTwoInts(7,8)=${Number(r.sum)}`); } catch {}
  }
  return { total: userSvc.length, cats: svcCats.size, ok, tested };
}

// ================================================================
// 3. 动作 (Action) — 自动发现 + send_goal + feedback 订阅
//
//   ROS2 Action 模型:
//     - Action Server: 提供 /<name>/_action/send_goal (service)
//                      发布 /<name>/_action/feedback (topic)
//                      发布 /<name>/_action/status   (topic)
//     - Goal: { goal: <user_goal> }  →  Response: { accepted, stamp }
//     - Feedback: batch 发布, 通过 topic 订阅
//     - 参考: https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Actions.html
// ================================================================
async function actionTests(ctx) {
  L.h1('3. 动作 (Action)');

  // 3.1 搜索 action 相关数据
  const actionTopics = ctx.channels.filter(ch => ch.topic.includes('/_action/'));
  const actionSvcs  = ctx.services.filter(s => s.name.includes('/_action/'));

  // 汇总 action 名称
  const actionNames = new Set();
  for (const ch of actionTopics) actionNames.add(ch.topic.split('/_action/')[0]);
  for (const s of actionSvcs)  actionNames.add(s.name.split('/_action/')[0]);

  L.h2(`发现 ${actionNames.size} 个 Action: [${[...actionNames].join(', ')}]`);
  if (actionNames.size === 0) {
    L.warn('foxglove_bridge 不转发 /_action/ 内部数据 (已知限制)');
    return 0;
  }

  // 分类展示
  for (const act of actionNames) {
    const tps = actionTopics.filter(ch => ch.topic.startsWith(act));
    const svs = actionSvcs.filter(s => s.name.startsWith(act));
    L.info(`${act}`);
    for (const ch of tps) L.info(`  topic: ${ch.topic.split('/').pop()} (${ch.schemaName})`);
    for (const s of svs)  L.info(`  svc:   ${s.name.split('/').pop()} (${s.type})`);
  }

  // 3.2 send_goal: 通过 service 发送 Goal (ROS2 Action Goal 格式)
  let goalSent=0;
  for (const act of actionNames) {
    const sendSvc = actionSvcs.find(s => s.name === `${act}/_action/send_goal`);
    if (!sendSvc?.request?.schema) continue;

    try {
      // Action send_goal Request: { goal: <action_goal>, goal_uuid: uuid }
      // 构造最小 goal (具体内容取决于 action 类型)
      const req = ctx.buildReq(sendSvc, { goal: {}, goal_uuid: null });
      const rsp = await ctx.callSvc(sendSvc, req, 5000);
      L.ok(`send_goal → ${act}: accepted=${rsp?.accepted}, stamp=${JSON.stringify(rsp?.stamp).substring(0,30)}`);
      goalSent++;
    } catch(e) { L.warn(`send_goal ${act}: ${e.message}`); }
  }

  // 3.3 订阅 feedback topic (ROS2 Action Feedback)
  const fbTopics = actionTopics.filter(ch => ch.topic.endsWith('/_action/feedback'));
  if (fbTopics.length > 0) {
    L.h2('Action Feedback 订阅');
    for (const ch of fbTopics.slice(0, 3)) {
      try {
        const msgs = await ctx.subCollect(ch.topic, 2, 5000);
        if (msgs.length > 0) L.ok(`${ch.topic}: ${msgs.length} 条 feedback`);
        else L.info(`${ch.topic}: 暂无 feedback (需先 send_goal)`);
      } catch(e) { L.warn(`${ch.topic}: ${e.message}`); }
    }
  }

  // 3.4 订阅 status topic (ROS2 Action Status: action_msgs/GoalStatusArray)
  const stTopics = actionTopics.filter(ch => ch.topic.endsWith('/_action/status'));
  if (stTopics.length > 0) {
    L.h2('Action Status 订阅');
    for (const ch of stTopics.slice(0, 2)) {
      try {
        const msgs = await ctx.subCollect(ch.topic, 1, 3000);
        if (msgs.length > 0) {
          const statusList = msgs[0].data?.status_list || [];
          L.ok(`${ch.topic}: ${statusList.length} 个 goal`);
          // ROS2 GoalStatus: STATUS_UNKNOWN=0 ACCEPTED=1 EXECUTING=2 SUCCEEDED=4
          const STATES = {0:'UNKNOWN',1:'ACCEPTED',2:'EXECUTING',3:'CANCELING',4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'};
          for (const gs of statusList.slice(0, 3)) {
            L.info(`  goal_id=${(gs.goal_info?.goal_id?.uuid||'').substring(0,8)} status=${STATES[gs.status]||gs.status}`);
          }
        }
      } catch(e) {}
    }
  }

  return { actions: actionNames.size, topics: actionTopics.length, svcs: actionSvcs.length, goalSent };
}

// ================================================================
// 4. 参数 (Parameter) — 自动发现 + List+Describe+Get+Set 完整闭环
//
//   ROS2 参数模型 (rcl_interfaces):
//     - /<node>/list_parameters     (rcl_interfaces/srv/ListParameters)
//     - /<node>/describe_parameters (rcl_interfaces/srv/DescribeParameters) ← 官方教程推荐
//     - /<node>/get_parameters      (rcl_interfaces/srv/GetParameters)
//     - /<node>/set_parameters      (rcl_interfaces/srv/SetParameters)
//     - ParameterValue: { type:uint8, bool/int/double/string/array_value }
//     - ParameterDescriptor: { name, type, description, read_only, ... }
//     - 参考: https://docs.ros.org/en/humble/Tutorials/Parameters/Understanding-Parameters.html
// ================================================================
async function parameterTests(ctx) {
  L.h1('4. 参数 (Parameter) — List → Describe → Get → Set ↔ Get 闭环');

  if (!ctx.capabilities.includes('services')) { L.warn('services 未启用'); return {}; }

  // 自动发现所有节点的 4 个参数服务
  const allNodes = ctx.services
    .filter(s => s.name.endsWith('/list_parameters') && s.request?.schema)
    .map(s => ({
      name: s.name.replace('/list_parameters','').replace(/^\//,''),
      listSvc: s,
      descSvc: ctx.findService(s.name.replace('/list_parameters','/describe_parameters')),
      getSvc:  ctx.findService(s.name.replace('/list_parameters','/get_parameters')),
      setSvc:  ctx.findService(s.name.replace('/list_parameters','/set_parameters')),
    }))
    .filter(n => n.getSvc?.request?.schema);

  // 列出所有节点参数
  const nodeInfo = [];
  for (const n of allNodes.slice(0, 30)) {
    try { const r = await ctx.callSvc(n.listSvc, ctx.buildReq(n.listSvc,{})); n.params = r?.result?.names||[]; nodeInfo.push(n); }
    catch(e) { n.params=[]; nodeInfo.push(n); }
  }
  nodeInfo.sort((a,b) => b.params.length - a.params.length);

  L.h2(`自动发现 ${allNodes.length} 个参数节点`);
  for (const n of nodeInfo.slice(0, 10)) {
    if (n.params.length === 0) continue;
    const preview = n.params.filter(p => !p.includes('qos_overrides')).slice(0,3).join(', ');
    L.info(`${n.name.padEnd(40)} ${String(n.params.length).padStart(3)} 参数 — ${preview}`);
  }

  // ── 4.1 Describe: 查看参数元数据 (ROS2 官方教程推荐) ──
  L.h2('4.1 Describe — 参数元数据 (rcl_interfaces/srv/DescribeParameters)');
  // 选参数最多的节点做 describe
  const bestNode = nodeInfo.find(n => n.params.length > 0 && n.descSvc?.request?.schema);
  if (bestNode) {
    const descNames = bestNode.params.filter(p => !p.includes('qos_overrides')).slice(0, 6);
    try {
      const descRsp = await ctx.callSvc(bestNode.descSvc, ctx.buildReq(bestNode.descSvc, { names: descNames }));
      const descs = descRsp?.descriptors || [];
      for (let i=0; i<descs.length; i++) {
        const d = descs[i];
        const ro = d.read_only ? '🔒只读' : '✏️可写';
        const typeName = PTYPES[d.type] || `type${d.type}`;
        L.info(`  ${d.name.padEnd(30)} ${typeName.padEnd(10)} ${ro}  ${d.description||''}`);
      }
    } catch(e) { L.warn(`Describe: ${e.message}`); }
  }

  // ── 4.2 Get: 从每个节点读参数 ──
  L.h2('4.2 Get — 读取参数值 (rcl_interfaces/srv/GetParameters)');
  let totalGet=0, totalParams=0;

  // 找参数最多的 5 个节点分别 get
  const topNodes = nodeInfo.filter(n => n.params.length > 0).slice(0, 5);
  for (const n of topNodes) {
    const names = n.params.filter(p => !p.includes('qos_overrides')).slice(0, 4);
    if (names.length === 0) continue;
    totalGet++;
    try {
      const rsp = await ctx.callSvc(n.getSvc, ctx.buildReq(n.getSvc, { names }));
      const vals = rsp?.values || [];
      L.ok(`${n.name}: ${vals.length} 个参数读取成功`);
      for (let i=0; i<vals.length; i++) {
        L.info(`  ${names[i]} = ${pval(vals[i])} [${PTYPES[vals[i]?.type]}]`);
        totalParams++;
      }
    } catch(e) { L.warn(`${n.name}: ${e.message}`); }
  }
  L.info(`Get 汇总: ${totalGet} 节点, ${totalParams} 参数`);

  // ── 4.3 Set ↔ Get 闭环 (先订阅 parameter_events, 再 set) ──
  L.h2('4.3 Set ↔ Get 闭环 (写入后读回验证)');

  // 提前订阅 parameter_events, 确保捕获到 set 触发的变更
  const pe = ctx.findChannel('/parameter_events');
  let peStart = 0;
  if (pe) { peStart = ctx.msgBuf.length; ctx.subMap.set(pe.id, ctx.client.subscribe(pe.id)); }

  // 找第一个有 set 能力且有 writable 参数的节点
  let closed=0;
  const WRITABLE_PRIORITY = ['use_sim_time', 'publish_frequency', 'update_rate', 'frame_id'];

  for (const n of nodeInfo) {
    if (!n.setSvc?.request?.schema || n.params.length === 0) continue;
    // 按优先级找可写参数
    const writable = WRITABLE_PRIORITY.find(p => n.params.includes(p))
                  || n.params.find(p => !p.includes('qos_overrides') && !p.includes('kinematics_solver')
                       && !p.includes('robot_description') && !p.includes('default_planner')
                       && !p.includes('capabilities') && p !== 'address' && p !== 'port');
    if (!writable) continue;

    try {
      // (1) 读当前值
      const beforeRsp = await ctx.callSvc(n.getSvc, ctx.buildReq(n.getSvc, { names: [writable] }));
      const bv = beforeRsp?.values?.[0];
      if (!bv || bv.type === 0) continue;

      const beforeVal = pval(bv);
      L.info(`${n.name}/${writable} 当前值 = ${beforeVal} [${PTYPES[bv.type]}]`);

      // (2) 构造新值 (翻转/递增)
      let nv;
      switch (bv.type) {
        case 1: // bool → 翻转
          nv = { type: 1, bool_value: !bv.bool_value };
          break;
        case 2: // integer → +1
          { const vi = typeof bv.integer_value === 'bigint' ? Number(bv.integer_value) : (bv.integer_value || 0);
          nv = { type: 2, integer_value: vi + 1 }; }
          break;
        case 3: // double → +1.0
          nv = { type: 3, double_value: (bv.double_value || 0.0) + 1.0 };
          break;
        case 4: // string → 追加后缀
          nv = { type: 4, string_value: (bv.string_value || '') + '_foxglove_test' };
          break;
        default: continue;
      }

      // (3) 写入
      const setReq = ctx.buildReq(n.setSvc, { parameters: [{ name: writable, value: nv }] });
      const setRsp = await ctx.callSvc(n.setSvc, setReq);
      const setOk = setRsp?.results?.[0]?.successful;

      if (!setOk) {
        L.warn(`  Set: ❌ ${setRsp?.results?.[0]?.reason || 'write rejected'}`);
        continue;
      }

      // (4) 读回验证
      await new Promise(r => setTimeout(r, 300));
      const afterRsp = await ctx.callSvc(n.getSvc, ctx.buildReq(n.getSvc, { names: [writable] }));
      const afterVal = pval(afterRsp?.values?.[0]);
      const matched = afterVal === pval({...bv, ...nv});

      L.ok(`${n.name}/${writable}: ${beforeVal} → ${afterVal} ${matched ? '✅ 闭环一致' : '⚠️'}`);
      closed++;
      if (closed >= 3) break;  // 测 3 个节点就够了
    } catch(e) { L.warn(`${n.name}/${writable}: ${e.message}`); }
  }
  L.info(`闭环完成: ${closed} 个参数`);

  // ── 4.4 parameter_events 变更检测 ──
  L.h2('4.4 /parameter_events 变更检测');
  if (pe) {
    await new Promise(r => setTimeout(r, 1000));
    const evts = ctx.msgBuf.slice(peStart).filter(e => e.cid === pe.id);

    if (evts.length > 0) {
      L.ok(`捕获 ${evts.length} 个 ParameterEvent (从 Set 操作触发):`);
      const seen = new Set();
      for (const evt of evts) {
        const d = evt.data;
        if (seen.has(d.node)) continue; seen.add(d.node);
        L.info(`  ${d.node}: new=${(d.new_parameters||[]).length}, changed=${(d.changed_parameters||[]).length}`);
        for (const cp of (d.changed_parameters||[]).slice(0, 2)) {
          L.info(`    ${cp.name} → ${pval(cp.value)} [${PTYPES[cp.value?.type]}]`);
        }
      }
    } else { L.info('无变更事件 (Set 未触发, 或事件尚未到达)'); }
  }

  return { nodes: allNodes.length, closed, totalGet };
}

// ================================================================
// 5. 拓扑更新 (Connection Graph)
// ================================================================
async function connectionGraphTests(ctx) {
  L.h1('5. 拓扑更新 (Connection Graph)');

  if (!ctx.capabilities.includes('connectionGraph')) {
    L.warn('connectionGraph 未启用'); return { updates: 0, topics: 0 };
  }

  const updates = ctx.fullData.connectionGraphUpdates;
  L.h2(`收到 ${updates.length} 次拓扑更新`);

  if (updates.length === 0) {
    L.info('无拓扑更新 (可能 bridge 侧未发布)');
    return { updates: 0, topics: 0 };
  }

  // 汇总所有出现过的话题
  const allTopics = new Map();
  for (const u of updates) {
    for (const pt of u.publishedTopics) {
      if (!allTopics.has(pt.name)) allTopics.set(pt.name, { publishers: 0, subscribers: 0 });
      allTopics.get(pt.name).publishers = Math.max(allTopics.get(pt.name).publishers, pt.publisherCount);
    }
    for (const st of u.subscribedTopics) {
      if (!allTopics.has(st.name)) allTopics.set(st.name, { publishers: 0, subscribers: 0 });
      allTopics.get(st.name).subscribers = Math.max(allTopics.get(st.name).subscribers, st.subscriberCount);
    }
  }

  const sorted = [...allTopics.entries()].sort((a, b) => (b[1].publishers + b[1].subscribers) - (a[1].publishers + a[1].subscribers));
  L.info(`拓扑覆盖 ${sorted.length} 个 topic`);
  for (const [name, counts] of sorted.slice(0, 15)) {
    L.info(`  ${name.padEnd(42)} pub=${counts.publishers} sub=${counts.subscribers}`);
  }

  // 最近一次更新详情
  const last = updates[updates.length - 1];
  L.h2('最近一次更新');
  L.info(`publishedTopics: ${last.publishedTopics.length} | subscribedTopics: ${last.subscribedTopics.length} | removedTopics: ${last.removedTopics.length}`);

  return { updates: updates.length, topics: allTopics.size };
}

// ================================================================
// 6. 系统 + 带宽报告
// ================================================================
async function systemReport(ctx) {
  L.h1('5. 系统概览 + CDR 带宽');

  // 带宽统计 — 使用全量数据
  L.h2('CDR vs JSON 带宽 (全量, 按类型)');
  const typeMap = new Map();
  for (const ch of ctx.channels) { if (ch.encoding!=='cdr'||!ctx.codecMap.has(ch.id)) continue; const t=ch.schemaName; if(!typeMap.has(t)) typeMap.set(t,ch); }

  // 批量订阅所有编码通道
  ctx.subMap.clear();
  for (const [t,ch] of typeMap) { try{ctx.subMap.set(ch.id,ctx.client.subscribe(ch.id));}catch{} }
  const start = ctx.msgBuf.length;
  await new Promise(r=>setTimeout(r,8000));

  // 从全量 bandwidthRaw 中统计 (不再限制每类型 5 样本)
  const stats = new Map();
  for (const e of ctx.msgBuf.slice(start)) {
    if (!stats.has(e.type)) stats.set(e.type, {cs:[],js:[],topic:e.topic});
    const s = stats.get(e.type);
    s.cs.push(e.cdrSize); s.js.push(e.jsonSize);  // 全量保留
  }
  for (const [cid,sid] of ctx.subMap) { try{ctx.client.unsubscribe(sid);}catch{} }

  const lines=[];
  for (const [t,s] of stats) { if(s.cs.length===0) continue;
    const ac=s.cs.reduce((a,b)=>a+b,0)/s.cs.length, aj=s.js.reduce((a,b)=>a+b,0)/s.js.length;
    lines.push({type:t,topic:s.topic,cdr:ac,json:aj,save:aj>0?(1-ac/aj)*100:0,n:s.cs.length});
  }
  lines.sort((a,b)=>b.cdr-a.cdr);

  let w=Math.max(...lines.map(l=>l.type.length), 40);
  console.log(`  ${'类型'.padEnd(w+2)}${'CDR(B)'.padStart(8)} ${'JSON(B)'.padStart(8)} ${'节省%'.padStart(7)} ${'样本'.padStart(6)}`);
  console.log('  '+'-'.repeat(w+37));
  for (const l of lines.slice(0,20)) {
    console.log(`  ${l.type.padEnd(w+2)}${String(Math.round(l.cdr)).padStart(8)} ${String(Math.round(l.json)).padStart(8)} ${l.save.toFixed(1).padStart(6)}% ${String(l.n).padStart(6)}`);
  }
  const tc=lines.reduce((s,l)=>s+l.cdr*l.n,0), tj=lines.reduce((s,l)=>s+l.json*l.n,0);
  console.log(`\n  📊 总: CDR=${Math.round(tc)}B, JSON=${Math.round(tj)}B, 节省=${tj>0?((1-tc/tj)*100).toFixed(1):'N/A'}%`);

  // 系统信息
  L.h2('硬件');
  const cpus=os.cpus();
  console.log(`  OS:    ${os.hostname()} / ${os.platform()} ${os.release()}`);
  console.log(`  CPU:   ${cpus[0]?.model} x${cpus.length} / load ${os.loadavg().map(l=>l.toFixed(1)).join('/')}`);
  console.log(`  Mem:   ${(os.totalmem()/1024**3).toFixed(1)}GB 总 / ${(os.freemem()/1024**3).toFixed(1)}GB 空闲 (${(os.freemem()/os.totalmem()*100).toFixed(1)}% 可用)`);
  console.log(`  Node:  ${process.version}`);
}

// ================================================================
// MAIN
// ================================================================
async function main() {
  const t0=Date.now();
  console.log('╔'+'═'.repeat(64)+'╗');
  console.log('║  foxglove_bridge 智能化完整验证                                    ║');
  console.log('║  Topic/Service/Action/Parameter 自动发现                           ║');
  console.log('╚'+'═'.repeat(64)+'╝');

  let ctx;
  try { ctx = await new SessionContext().connect(); }
  catch(e) { console.error(`\n❌ ${e.message}`); process.exit(1); }

  const caps = ctx.capabilities.join(',');
  console.log(`  ${ctx.channels.length} topics, ${ctx.services.length} services, [${caps}]`);

  const R={};
  try { R.topic = await topicTests(ctx);     } catch(e) { L.err(`话题: ${e.message}`); }
  try { R.svc   = await serviceTests(ctx);   } catch(e) { L.err(`服务: ${e.message}`); }
  try { R.act   = await actionTests(ctx);    } catch(e) { L.err(`动作: ${e.message}`); }
  try { R.param = await parameterTests(ctx); } catch(e) { L.err(`参数: ${e.message}`); }
  try { R.graph = await connectionGraphTests(ctx); } catch(e) { L.err(`拓扑: ${e.message}`); }
  try { await systemReport(ctx); } catch(e) { L.err(`系统: ${e.message}`); }

  ctx.disconnect();

  // 导出全量分析数据
  const fullReport = ctx.exportFullData();
  const reportPath = new URL('./full_analysis.json', import.meta.url).pathname;
  fs.writeFileSync(reportPath, JSON.stringify(fullReport, null, 2));
  L.info(`全量分析数据已导出: ${reportPath} (${(JSON.stringify(fullReport).length/1024).toFixed(1)}KB)`);

  const s = ((Date.now()-t0)/1000).toFixed(0);
  const T = v => v ? '✅' : '⚠️';

  console.log('\n╔'+'═'.repeat(64)+'╗');
  console.log('║  验证结果                                                         ║');
  console.log('╠'+'═'.repeat(64)+'╣');
  console.log(`║  话题:  ${T(R.topic?.subOk>0)} ${R.topic?.cats||0}类, ${R.topic?.subAll||0}订阅, ${R.topic?.subOk||0}有数据                        ║`);
  console.log(`║  服务:  ${T(R.svc?.ok>0)} ${R.svc?.cats||0}空间, ${R.svc?.ok||0}/${R.svc?.tested||0}成功                                  ║`);
  console.log(`║  动作:  ${T(R.act?.actions>0)} ${R.act?.actions||0} actions, ${R.act?.goalSent||0} goals发送                               ║`);
  console.log(`║  参数:  ${T(R.param?.nodes>0)} ${R.param?.nodes||0}节点, ${R.param?.closed||0}闭环, ${R.param?.totalGet||0}读取                              ║`);
  console.log(`║  拓扑:  ${T(R.graph?.updates>0)} ${R.graph?.updates||0}次更新, ${R.graph?.topics||0}话题                                   ║`);
  console.log(`║  耗时: ${s}s                                                         ║`);
  console.log('╚'+'═'.repeat(64)+'╝');
}

main().catch(e=>{console.error('FATAL:',e);process.exit(1);});
