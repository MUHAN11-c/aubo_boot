/** Project-selected ROS evidence checks. Existing SDK clients, no robot imports. */
import {encodeParameter} from '../sdk/parameters.js';
import type {ParameterType, ParameterValue, RosClient, TopicOptions} from '../sdk/index.js';
import {decodeImage} from '../media/image.js';
import type {RawImage} from '../media/image.js';
import {decodePointCloud} from '../scene/pointcloud.js';
import {TfBuffer} from '../scene/tf.js';

export type CheckKind = 'graph'|'topic'|'image'|'cloud'|'parameters'|'lifecycle'|'tf'|'interface'|'service'|'parameterRoundtrip'|'action';
export type CheckStatus = 'PASS'|'FAIL'|'CANCELLED'|'NOT_RUN';
export type JsonValue = null|boolean|number|string|JsonValue[]|{[key:string]:JsonValue};
export interface ServiceSpec {name:string;type:string;request?:JsonValue;expected?:Record<string,JsonValue>}
export interface ParameterExpectation {node:string;names:string[];expected?:Record<string,JsonValue>}
export interface TopicSpec {
  name:string;type:string;qos?:TopicOptions['qos'];throttleMs?:number;minSamples?:number;maxAgeMs?:number;
  requiredFields?:string[];encoding?:string;depthScale?:number;minPoints?:number;width?:number;height?:number;minHz?:number;
  expected?:Record<string,JsonValue>;jsonField?:string;
  /** Recorded headers keep their original epoch; they cannot prove live freshness. */
  timestampMode?:'live'|'recorded';
  compression?:TopicOptions['compression'];
}
export interface MockGuard {topic:string;type:string;qos?:TopicOptions['qos'];field:string;includes:string}
export interface CaseBase {id:string;label:string;group?:string;description?:string;interactive?:boolean;defaultSelected?:boolean;timeoutMs?:number;mockOnly?:boolean}
export type CheckSpec = CaseBase & (
  {kind:'graph';nodes?:string[];topics?:string[];actions?:string[]}|
  {kind:'topic'|'image'|'cloud';topic:TopicSpec}|
  {kind:'parameters';node:string;names:string[];expected?:Record<string,JsonValue>}|
  {kind:'lifecycle';node:string;state:string|number}|
  {kind:'tf';target:string;sources:string[]}|
  {kind:'interface';endpoint:string;interfaceKind:'topic'|'service'|'action';interfaceType:string}|
  {kind:'service';service:ServiceSpec}|
  {kind:'parameterRoundtrip';parameter:{node:string;name:string;type:ParameterType;value:ParameterValue}}|
  {kind:'action';action:{name:string;type:string;goal:JsonValue;requireFeedback?:boolean};setup?:ServiceSpec[];cleanup?:ServiceSpec[];preconditions?:ParameterExpectation[];afterSetup?:TopicSpec[];afterCleanup?:TopicSpec[]}
);
export interface ProjectSpec {id:string;title:string;description?:string;cases:CheckSpec[];view?:Record<string,unknown>;mockGuard?:MockGuard;[key:string]:unknown}
export interface CheckResult {id:string;label:string;kind:CheckKind;interactive:boolean;status:CheckStatus;startedAt:string|null;elapsedMs:number;evidence:Record<string,unknown>;error?:string}
export interface AuditProgress {runId:string;completed:number;total:number;caseId:string;label:string;phase:'running'|'finished'}
export interface ProjectReport {projectId:string;title:string;url:string;connectionMode?:'shared'|'separate';runId:string;startedAt:string;finishedAt:string;elapsedMs:number;selectedIds:string[];results:CheckResult[];totals:Record<CheckStatus,number>;notRun:string[]}
export interface RunOptions {
  /** Optional second client to the same URL for topic/TF evidence. The caller
   * owns both connections; commands, parameters and mockGuard stay on client.
   */
  observationClient?:RosClient;
  signal?:AbortSignal;onResult?:(result:CheckResult)=>void;onProgress?:(progress:AuditProgress)=>void;
}

type Wire=Record<string,unknown>;
const KINDS:readonly string[]=['graph','topic','image','cloud','parameters','lifecycle','tf','interface','service','parameterRoundtrip','action'];
const ROS_NAME=/^\/[A-Za-z_][A-Za-z0-9_]*(\/[A-Za-z_][A-Za-z0-9_]*)*$/;
const INTERFACE=/^[A-Za-z_][A-Za-z0-9_]*\/(msg|srv|action)\/[A-Za-z_][A-Za-z0-9_]*$/;
const MAX_REPORT_STRING=65536;
const CLEANUP_TIMEOUT=5000;

function object(value:unknown,label:string):Wire {
  if(!value||typeof value!=='object'||Array.isArray(value))throw new TypeError(`${label} must be an object`);
  return value as Wire;
}
function string(value:unknown,label:string):asserts value is string {
  if(typeof value!=='string'||!value.length||value.length>4096)throw new TypeError(`${label} must be a nonempty bounded string`);
}
function rosName(value:unknown,label:string):void {string(value,label);if(!ROS_NAME.test(value))throw new TypeError(`${label} must be an absolute ROS name`);}
function interfaceName(value:unknown,category:string,label:string):void {string(value,label);if(!INTERFACE.test(value)||value.split('/')[1]!==category)throw new TypeError(`${label} must be a ROS ${category} interface type`);}
function strings(value:unknown,label:string,required=false):void {
  if(!Array.isArray(value)||value.length>512||(required&&value.length===0))throw new TypeError(`${label} must be a bounded string array`);
  value.forEach(item=>string(item,label));
}
function json(value:unknown,label:string,depth=0):void {
  if(depth>30)throw new TypeError(`${label} exceeds JSON nesting budget`);
  if(value===null||typeof value==='boolean'||typeof value==='string')return;
  if(typeof value==='number'&&Number.isFinite(value))return;
  if(Array.isArray(value)){if(value.length>10000)throw new TypeError(`${label} exceeds JSON array budget`);value.forEach(v=>json(v,label,depth+1));return;}
  const item=object(value,label);for(const [key,v]of Object.entries(item)){if(['__proto__','constructor','prototype'].includes(key))throw new TypeError(`${label} contains an unsafe key`);json(v,label,depth+1);}
}
function pathTokens(path:string):(string|number)[] {
  if(!/^[A-Za-z_][A-Za-z0-9_]*(?:(?:\.[A-Za-z_][A-Za-z0-9_]*)|(?:\[\d+\]))*$/.test(path))throw new TypeError(`Invalid plain field path: ${path}`);
  const parts=path.match(/[A-Za-z_][A-Za-z0-9_]*|\d+/g)!;
  if(parts.some(p=>['__proto__','constructor','prototype'].includes(p)))throw new TypeError(`Unsafe field path: ${path}`);
  return parts.map(p=>/^\d+$/.test(p)?Number(p):p);
}
function field(value:unknown,path:string):{found:boolean;value:unknown} {
  let current=value;
  for(const part of pathTokens(path)) {
    if(current===null||typeof current!=='object'||!Object.prototype.hasOwnProperty.call(current,part))return {found:false,value:undefined};
    current=(current as Wire)[part];
  }
  return {found:true,value:current};
}
function expected(value:unknown,label:string,paths=true):void {
  const entries=object(value,label);for(const [key,v]of Object.entries(entries)){string(key,label);if(paths)pathTokens(key);json(v,label);}
}
function numeric(value:unknown,label:string,min:number,integer=false):void {
  if(typeof value!=='number'||!Number.isFinite(value)||value<min||(integer&&!Number.isSafeInteger(value)))throw new TypeError(`${label} must be ${integer?'an integer':'finite'} >= ${min}`);
}
function qos(value:unknown,label:string):void {
  if(value===undefined)return;
  if(typeof value==='string'){if(!['reliable','sensor','latched','staticTf'].includes(value))throw new TypeError(`${label}: unknown QoS profile`);return;}
  const q=object(value,label);if(q.history!=='keep_last'||!['reliable','best_effort'].includes(String(q.reliability))||!['volatile','transient_local'].includes(String(q.durability)))throw new TypeError(`${label}: unsupported QoS`);numeric(q.depth,label,1,true);
}
function service(value:unknown,label:string):void {
  const s=object(value,label);rosName(s.name,`${label}.name`);interfaceName(s.type,'srv',`${label}.type`);
  if(s.request!==undefined)json(s.request,`${label}.request`);if(s.expected!==undefined)expected(s.expected,`${label}.expected`);
}
function parameters(value:Wire,label:string):void {
  rosName(value.node,`${label}.node`);strings(value.names,`${label}.names`,true);
  if(value.expected!==undefined){expected(value.expected,`${label}.expected`,false);for(const name of Object.keys(value.expected as Wire))if(!(value.names as string[]).includes(name))throw new TypeError(`${label}.expected parameter is absent from names: ${name}`);}
}
function topic(value:unknown,label:string):void {
  const t=object(value,label);rosName(t.name,`${label}.name`);interfaceName(t.type,'msg',`${label}.type`);qos(t.qos,`${label}.qos`);
  for(const key of ['minSamples','minPoints','width','height'])if(t[key]!==undefined)numeric(t[key],`${label}.${key}`,1,true);
  if(Number(t.minSamples??2)>1000)throw new TypeError(`${label}.minSamples exceeds sample budget`);
  for(const key of ['maxAgeMs','depthScale'])if(t[key]!==undefined)numeric(t[key],`${label}.${key}`,Number.MIN_VALUE);
  for(const key of ['throttleMs','minHz'])if(t[key]!==undefined)numeric(t[key],`${label}.${key}`,0,key==='throttleMs');
  if(t.encoding!==undefined)string(t.encoding,`${label}.encoding`);
  if(t.requiredFields!==undefined){strings(t.requiredFields,`${label}.requiredFields`);(t.requiredFields as string[]).forEach(pathTokens);}
  if(t.expected!==undefined)expected(t.expected,`${label}.expected`);
  if(t.jsonField!==undefined){string(t.jsonField,`${label}.jsonField`);pathTokens(t.jsonField);}
  if(t.timestampMode!==undefined&&!['live','recorded'].includes(String(t.timestampMode)))throw new TypeError(`${label}.timestampMode must be live or recorded`);
  if(t.compression!==undefined&&!['none','cbor'].includes(String(t.compression)))throw new TypeError(`${label}.compression must be none or cbor`);
}
/** Validate data only; no YAML expression evaluation, ROS call or implicit selection. */
export function validateProject(value:unknown):ProjectSpec {
  const p=object(value,'project');string(p.id,'project.id');string(p.title,'project.title');if(p.description!==undefined)string(p.description,'project.description');
  if(p.view!==undefined){const view=object(p.view,'project.view');if(view.scene!==undefined){const scene=object(view.scene,'project.view.scene');for(const key of ['tf','tfStatic'])if(scene[key]!==undefined)rosName(scene[key],`project.view.scene.${key}`);}}
  if(!Array.isArray(p.cases)||p.cases.length>256)throw new TypeError('project.cases must be an array of at most 256 cases');
  if(p.mockGuard!==undefined){const g=object(p.mockGuard,'mockGuard');rosName(g.topic,'mockGuard.topic');interfaceName(g.type,'msg','mockGuard.type');if(g.type!=='std_msgs/msg/String'||!String(g.topic).endsWith('/robot_description')||g.field!=='data')throw new TypeError('mockGuard requires the actual robot_description std_msgs/msg/String data');string(g.field,'mockGuard.field');pathTokens(g.field);string(g.includes,'mockGuard.includes');if(!/mock_components\/(GenericSystem|GenericSensor)/.test(g.includes))throw new TypeError('mockGuard.includes must identify an actual mock_components plugin');qos(g.qos,'mockGuard.qos');}
  const ids=new Set<string>();
  for(const entry of p.cases) {
    const c=object(entry,'case');string(c.id,'case.id');string(c.label,'case.label');if(ids.has(c.id))throw new TypeError(`Duplicate case id: ${c.id}`);ids.add(c.id);
    if(!KINDS.includes(String(c.kind)))throw new TypeError(`Unknown case kind: ${String(c.kind)}`);
    for(const key of ['interactive','defaultSelected','mockOnly'])if(c[key]!==undefined&&typeof c[key]!=='boolean')throw new TypeError(`case.${key} must be boolean`);
    if(c.timeoutMs!==undefined){numeric(c.timeoutMs,'case.timeoutMs',1,true);if(Number(c.timeoutMs)>2147483647)throw new TypeError('case.timeoutMs exceeds timer limit');}
    if(c.mockOnly&&!p.mockGuard)throw new TypeError('mockOnly case requires a ROS mockGuard');
    // A read service such as AddTwoInts can be read-only. Configuration authors
    // explicitly mark mutating services; parameter writes/actions always opt in.
    if(['parameterRoundtrip','action'].includes(String(c.kind))&&c.interactive!==true)throw new TypeError(`${c.kind} must explicitly declare interactive: true`);
    if(c.interactive&&c.defaultSelected===true)throw new TypeError('Interactive cases cannot be selected by default');
    switch(c.kind) {
      case 'graph':for(const key of ['nodes','topics','actions'])if(c[key]!==undefined){strings(c[key],`graph.${key}`);(c[key] as string[]).forEach(v=>rosName(v,`graph.${key}`));}break;
      case 'topic':case 'image':case 'cloud':{
        topic(c.topic,'topic');const t=c.topic as Wire;
        if(c.kind==='image'&&t.type!=='sensor_msgs/msg/Image')throw new TypeError('image checks require raw sensor_msgs/msg/Image for decodeImage');
        if(c.kind==='cloud'&&t.type!=='sensor_msgs/msg/PointCloud2')throw new TypeError('cloud checks require sensor_msgs/msg/PointCloud2');break;
      }
      case 'parameters':parameters(c,'parameters');break;
      case 'lifecycle':rosName(c.node,'lifecycle.node');if(typeof c.state==='string')string(c.state,'lifecycle.state');else numeric(c.state,'lifecycle.state',0,true);break;
      case 'tf':string(c.target,'tf.target');strings(c.sources,'tf.sources',true);break;
      case 'interface':rosName(c.endpoint,'interface.endpoint');if(!['topic','service','action'].includes(String(c.interfaceKind)))throw new TypeError('Unknown interfaceKind');interfaceName(c.interfaceType,({topic:'msg',service:'srv',action:'action'} as Wire)[String(c.interfaceKind)] as string,'interface.interfaceType');break;
      case 'service':service(c.service,'service');break;
      case 'parameterRoundtrip':{const param=object(c.parameter,'parameter');rosName(param.node,'parameter.node');string(param.name,'parameter.name');encodeParameter(param.type as ParameterType,param.value as ParameterValue);break;}
      case 'action':{
        const a=object(c.action,'action');rosName(a.name,'action.name');interfaceName(a.type,'action','action.type');json(a.goal,'action.goal');if(a.requireFeedback!==undefined&&typeof a.requireFeedback!=='boolean')throw new TypeError('action.requireFeedback must be boolean');
        for(const key of ['setup','cleanup'])if(c[key]!==undefined){if(!Array.isArray(c[key])||(c[key] as unknown[]).length>32)throw new TypeError(`${key} must be a bounded service array`);(c[key] as unknown[]).forEach(s=>service(s,key));}
        for(const key of ['afterSetup','afterCleanup'])if(c[key]!==undefined){if(!Array.isArray(c[key])||(c[key] as unknown[]).length>32)throw new TypeError(`${key} must be a bounded topic array`);(c[key] as unknown[]).forEach(s=>topic(s,key));}
        if(c.preconditions!==undefined){if(!Array.isArray(c.preconditions)||c.preconditions.length>32)throw new TypeError('preconditions must be a bounded parameter expectation array');c.preconditions.forEach(v=>parameters(object(v,'precondition'),'precondition'));}break;
      }
    }
  }
  return p as unknown as ProjectSpec;
}

class AuditError extends Error {constructor(readonly reason:'cancelled'|'timeout'|'disconnected'|'failed',message:string){super(message);}}
function errorText(error:unknown):string {return String(error instanceof Error?error.message:error).slice(0,8192);}
function same(a:unknown,b:unknown):boolean {
  if(Object.is(a,b))return true;if(!a||!b||typeof a!=='object'||typeof b!=='object'||Array.isArray(a)!==Array.isArray(b))return false;
  const keys=Object.keys(a);return keys.length===Object.keys(b).length&&keys.every(k=>Object.hasOwn(b,k)&&same((a as Wire)[k],(b as Wire)[k]));
}
/** Evidence summaries never retain camera data, cloud buffers or complete messages. */
function compact(value:unknown,depth=0,budget={remaining:262144}):unknown {
  if(budget.remaining<=0)return '[evidence budget exhausted]';budget.remaining-=16;
  if(typeof value==='string'){const limit=Math.max(0,Math.min(MAX_REPORT_STRING,budget.remaining));budget.remaining-=Math.min(value.length,limit);return value.length>limit?`${value.slice(0,limit)}… [truncated]`:value;}
  if(value===null||value===undefined||typeof value==='number'||typeof value==='boolean')return value;
  if(ArrayBuffer.isView(value))return {omitted:true,bytes:value.byteLength};
  if(depth>15)return '[nesting truncated]';
  if(Array.isArray(value))return value.length>512?{length:value.length,first:value.slice(0,32).map(v=>compact(v,depth+1,budget)),truncated:true}:value.map(v=>compact(v,depth+1,budget));
  if(typeof value==='object')return Object.fromEntries(Object.entries(value).slice(0,512).map(([k,v])=>[k,k==='data'||k==='pixels'||k==='positions'?{omitted:true}:compact(v,depth+1,budget)]));
  return String(value);
}
function replaceRunId(value:unknown,runId:string):unknown {
  if(value==='$runId')return runId;if(Array.isArray(value))return value.map(v=>replaceRunId(v,runId));
  if(value&&typeof value==='object')return Object.fromEntries(Object.entries(value).map(([k,v])=>[k,replaceRunId(v,runId)]));return value;
}
class Context {
  readonly controller=new AbortController();readonly cleanups:(()=>void)[]=[];fault?:AuditError;
  constructor(readonly client:RosClient,readonly timeoutMs:number,signal?:AbortSignal,readonly observationClient:RosClient=client) {
    const abort=()=>this.stop(new AuditError('cancelled','用户停止；远端已发送的服务结果可能未知'));
    signal?.addEventListener('abort',abort,{once:true});this.cleanups.push(()=>signal?.removeEventListener('abort',abort));
    const timer=setTimeout(()=>this.stop(new AuditError('timeout',`等待 ${timeoutMs}ms 超时；远端结果未知`)),timeoutMs);this.cleanups.push(()=>clearTimeout(timer));
    for(const link of new Set([client,observationClient]))this.cleanups.push(link.onState(state=>{if(state!=='connected')this.stop(new AuditError('disconnected',`${link===client?'控制':'观测'}ROS连接中断；远端结果未知，未重放请求`));}));
    if(signal?.aborted)abort();
  }
  get options(){return {signal:this.controller.signal,timeoutMs:this.timeoutMs};}
  stop(error:AuditError){if(this.fault)return;this.fault=error;this.controller.abort(error);}
  check(){if(this.fault)throw this.fault;if(!this.client.connected||!this.observationClient.connected)throw new AuditError('disconnected','控制或观测ROS未连接；未发送请求');}
  async wait<T>(work:Promise<T>):Promise<T> {
    // A dependency may synchronously signal cancellation while returning an
    // already rejected promise. Always observe it, even when check() now fails.
    try{this.check();}catch(error){void work.catch(()=>{});throw error;}let abort:()=>void=()=>{};
    const stopped=new Promise<never>((_,reject)=>{abort=()=>reject(this.fault??new AuditError('cancelled','已停止'));this.controller.signal.addEventListener('abort',abort,{once:true});});
    try{return await Promise.race([work,stopped]);}catch(error){throw this.fault??error;}finally{this.controller.signal.removeEventListener('abort',abort);}
  }
  dispose(){for(const cleanup of this.cleanups.splice(0).reverse())cleanup();}
}
async function guard(ctx:Context,spec:MockGuard,evidence:Wire):Promise<void> {
  // Guard the command source itself, even when bulk observations are separate.
  let off=()=>{};
  try {
    await ctx.wait(new Promise<void>((resolve,reject)=>{
      off=ctx.client.subscribe(spec.topic,spec.type,message=>{
        const read=field(message,spec.field);evidence.mockGuard={topic:spec.topic,field:spec.field,requiredPlugin:spec.includes,matched:read.found&&typeof read.value==='string'&&read.value.includes(spec.includes)};
        if(!read.found||typeof read.value!=='string'||!read.value.includes(spec.includes))reject(new AuditError('failed','ROS robot_description 未确认所需 mock_components 插件；拒绝交互操作'));else resolve();
      },{qos:spec.qos??'latched'});
    }));
  }finally{off();}
}
async function sample(ctx:Context,spec:Extract<CheckSpec,{kind:'topic'|'image'|'cloud'}>,evidence:Wire):Promise<void> {
  const t=spec.topic,min=t.minSamples??2,maxAge=t.maxAgeMs??2000;
  const stamps=new Set<string>();const received:number[]=[];const sourceTimes:number[]=[];let off=()=>{};
  evidence.topic=t.name;evidence.type=t.type;evidence.samples=0;
  evidence.timestampMode=t.timestampMode??'live';evidence.liveFreshnessChecked=t.timestampMode!=='recorded';
  try {
    await ctx.wait(new Promise<void>((resolve,reject)=>{
      off=ctx.observationClient.subscribe<Wire>(t.name,t.type,(message,info)=>{
        try {
          const stamp=field(message,'header.stamp');let time=info.receivedAt,key=`receive:${info.receivedAt}`;
          if(stamp.found){const s=object(stamp.value,'header.stamp');if(typeof s.sec!=='number'||!Number.isInteger(s.sec)||typeof s.nanosec!=='number'||!Number.isInteger(s.nanosec)||s.nanosec<0||s.nanosec>=1e9)throw new Error('Invalid header.stamp');time=s.sec*1000+s.nanosec/1e6;key=`stamp:${s.sec}:${s.nanosec}`;}
          // A rosbag loop starts a new recorded epoch. Collect a complete
          // sample window within that epoch, never compute Hz across rewind.
          if(t.timestampMode==='recorded'&&sourceTimes.length&&time<sourceTimes.at(-1)!){stamps.clear();received.length=0;sourceTimes.length=0;evidence.epochResets=Number(evidence.epochResets??0)+1;}
          if(stamps.has(key))return;
          const age=info.receivedAt-time;if(t.timestampMode!=='recorded'&&(age>maxAge||age < -maxAge))throw new Error(`源时间不新鲜或时钟不一致：age=${age.toFixed(1)}ms，要求±${maxAge}ms`);
          let content:unknown=message;
          if(t.jsonField){const encoded=field(message,t.jsonField);if(!encoded.found||typeof encoded.value!=='string')throw new Error(`JSON字段 ${t.jsonField} 不存在或非字符串`);if(encoded.value.length>1048576)throw new Error('JSON话题内容超过1MiB预算');content=JSON.parse(encoded.value);}
          for(const path of t.requiredFields??[])if(!field(content,path).found)throw new Error(`缺少字段 ${path}`);
          const assertions=Object.fromEntries(Object.entries(t.expected??{}).map(([path,expectedValue])=>{const actual=field(content,path);return [path,{expected:expectedValue,actual:compact(actual.value),matched:actual.found&&same(actual.value,expectedValue)}];}));
          if(Object.keys(assertions).length){evidence.assertions=assertions;if(Object.values(assertions).some(assertion=>!assertion.matched))return;}
          const summary:Wire={header:compact(message.header),sourceAgeMs:age};
          if(spec.kind==='image') {
            if(t.encoding&&String(message.encoding).toLowerCase()!==t.encoding.toLowerCase())throw new Error(`图像编码 ${String(message.encoding)} 不等于 ${t.encoding}`);
            const decoded=decodeImage(message as unknown as RawImage,{depthScale:t.depthScale});
            if(t.width!==undefined&&decoded.width!==t.width||t.height!==undefined&&decoded.height!==t.height)throw new Error(`图像尺寸 ${decoded.width}×${decoded.height} 不匹配要求`);
            Object.assign(summary,{width:decoded.width,height:decoded.height,encoding:message.encoding,decodedPixels:decoded.pixels.length/4,depthScale:t.depthScale});
          }else if(spec.kind==='cloud'){
            const cloud=decodePointCloud(message,{maxPoints:100000,maxBytes:16000000});if(cloud.count<(t.minPoints??1))throw new Error(`有限点 ${cloud.count} 少于 ${t.minPoints??1}`);
            Object.assign(summary,{width:message.width,height:message.height,finiteDecodedPoints:cloud.count,fields:compact(message.fields)});
          }else{
            summary.requiredFields=Object.fromEntries((t.requiredFields??[]).map(path=>[path,compact(field(content,path).value)]));
          }
          stamps.add(key);received.push(info.receivedAt);sourceTimes.push(time);evidence.samples=stamps.size;evidence.latest=summary;
          if(stamps.size>=min){const seconds=(sourceTimes.at(-1)!-sourceTimes[0]!)/1000;const receiveSeconds=(received.at(-1)!-received[0]!)/1000;const hz=seconds>0?(stamps.size-1)/seconds:null;evidence.sourceHz=hz;evidence.receiveHz=receiveSeconds>0?(stamps.size-1)/receiveSeconds:null;evidence.distinctSourceStamps=stamp.found;
            if(t.minHz!==undefined&&(hz===null||hz<t.minHz))throw new Error(`采样频率 ${String(hz)}Hz 少于 ${t.minHz}Hz`);resolve();}
        }catch(error){reject(error);}
      },{qos:t.qos??'sensor',throttleMs:t.throttleMs??0,compression:t.compression??'none'});
    }));
  }finally{off();}
}
async function readParameters(ctx:Context,spec:ParameterExpectation,evidence:Wire):Promise<void> {
  ctx.check();const api=ctx.client.parameters(spec.node);
  const [read,described]=await ctx.wait(Promise.all([api.get(spec.names,ctx.options),api.describe(spec.names,ctx.options)]));
  if(read.values.length!==spec.names.length||described.descriptors.length!==spec.names.length)throw new Error('参数响应数量不匹配');
  evidence.node=spec.node;evidence.parameters=Object.fromEntries(spec.names.map((name,i)=>[name,{...read.values[i],descriptor:compact(described.descriptors[i])}]));
  for(const [name,value]of Object.entries(spec.expected??{})){const actual=read.values[spec.names.indexOf(name)]!;if(actual.type===0||!same(actual.value,value))throw new Error(`参数 ${name} 实际 ${JSON.stringify(actual.value)} 不等于 ${JSON.stringify(value)}`);}
}
async function invokeService(ctx:Context,spec:ServiceSpec,runId:string,cleanup=false):Promise<Wire> {
  if(!cleanup)ctx.check();else if(!ctx.client.connected)throw new Error('断线无法发送清理服务；远端状态未知');
  const request=replaceRunId(spec.request??{},runId);
  const options=cleanup?{timeoutMs:CLEANUP_TIMEOUT}:ctx.options;
  const pending=ctx.client.callService<Wire>(spec.name,spec.type,request,options);const response=cleanup?await pending:await ctx.wait(pending);
  for(const [path,value]of Object.entries(spec.expected??{})){const actual=field(response,path);const substituted=replaceRunId(value,runId);if(!actual.found||!same(actual.value,substituted))throw new Error(`${spec.name} 响应 ${path} 不等于 ${JSON.stringify(substituted)}，实际 ${JSON.stringify(actual.value)}`);}
  return {name:spec.name,type:spec.type,request:compact(request),response:compact(response)};
}
function wrote(response:{results:{successful:boolean;reason:string}[]}):void {if(response.results.length!==1||response.results.some(r=>!r.successful))throw new Error(`参数写入拒绝：${response.results.map(r=>r.reason).join('; ')}`);}
async function roundtrip(ctx:Context,spec:Extract<CheckSpec,{kind:'parameterRoundtrip'}>,runId:string,evidence:Wire):Promise<void> {
  const p=spec.parameter,api=ctx.client.parameters(p.node);ctx.check();
  const value=replaceRunId(p.value,runId) as ParameterValue;
  const original=await ctx.wait(api.get([p.name],ctx.options));if(original.values.length!==1)throw new Error('未读取到唯一原参数值');const previous=original.values[0]!;
  evidence.node=p.node;evidence.parameter=p.name;evidence.original=compact(previous);let attempted=false,primary:unknown;
  try {
    ctx.check();attempted=true;wrote(await ctx.wait(api.set([{name:p.name,type:p.type,value}],ctx.options)));
    const read=await ctx.wait(api.get([p.name],ctx.options));evidence.readback=compact(read.values[0]);if(read.values.length!==1||read.values[0]!.type!==p.type||!same(read.values[0]!.value,value))throw new Error('写入后的参数读回不匹配');ctx.check();
  }catch(error){primary=error;}
  finally{
    if(attempted)try{
      if(!ctx.client.connected)throw new Error('断线，参数恢复未发送');
      wrote(await api.set([{name:p.name,...previous}],{timeoutMs:CLEANUP_TIMEOUT}));
      const restored=await api.get([p.name],{timeoutMs:CLEANUP_TIMEOUT});if(restored.values.length!==1||!same(restored.values[0],previous))throw new Error('恢复读回不等于原值');evidence.restored=compact(restored.values[0]);evidence.restoreConfirmed=true;
    }catch(error){evidence.restoreConfirmed=false;evidence.restoreError=errorText(error);throw new AuditError('failed',`${primary?`${errorText(primary)}；`:''}参数恢复失败：${errorText(error)}`);}
  }
  if(primary)throw primary;
}
async function boundedTerminal<T>(promise:Promise<T>,ms:number):Promise<T> {
  let timer:ReturnType<typeof setTimeout>|undefined;try{return await Promise.race([promise,new Promise<never>((_,reject)=>{timer=setTimeout(()=>reject(new Error('取消后等待终态超时；远端结果未知')),ms);})]);}finally{clearTimeout(timer);}
}
async function action(ctx:Context,spec:Extract<CheckSpec,{kind:'action'}>,runId:string,evidence:Wire):Promise<void> {
  const setup:unknown[]=[],cleanup:unknown[]=[];evidence.setup=setup;evidence.cleanup=cleanup;let primary:unknown;let feedback=0,setupAttempted=false;
  try{
    for(const precondition of spec.preconditions??[]){const proof:Wire={};await readParameters(ctx,precondition,proof);((evidence.preconditions??=[]) as unknown[]).push(proof);}
    setupAttempted=true;for(const step of spec.setup??[])setup.push(await invokeService(ctx,step,runId));
    for(const check of spec.afterSetup??[]){const proof:Wire={};((evidence.afterSetup??=[]) as unknown[]).push(proof);await sample(ctx,{id:'afterSetup',label:'afterSetup',kind:'topic',topic:{...check,minSamples:check.minSamples??1}},proof);}
    ctx.check();
    // The audit signal must not erase the result listener before cancellation
    // is confirmed. This separate lifetime ends only after the terminal window.
    const resultWait=new AbortController();
    const handle=ctx.client.sendGoal(spec.action.name,spec.action.type,replaceRunId(spec.action.goal,runId),{signal:resultWait.signal,timeoutMs:Math.min(ctx.timeoutMs+CLEANUP_TIMEOUT,2147483647),onFeedback:message=>{feedback++;evidence.feedbackCount=feedback;evidence.lastFeedback=compact(message);}});
    evidence.action=spec.action.name;evidence.type=spec.action.type;evidence.goalId=handle.id;
    try{
      const result=await ctx.wait(handle.result);evidence.terminalStatus=result.status;evidence.result=compact(result.value);
      const nativeMessage=field(result.value,'message').value;
      const detail=result.error??(typeof nativeMessage==='string'?nativeMessage:'');
      if(result.status!==4||!result.successful)throw new Error(`Action未成功：status=${result.status}${detail?` ${detail}`:''}`);
      if(spec.action.requireFeedback&&!feedback)throw new Error('Action成功但未收到要求的反馈');
    }catch(error){
      if(ctx.fault){
        try{
          evidence.cancelAttempted=true;handle.cancel();evidence.cancelRequested=true;
          const terminal=await boundedTerminal(handle.result,CLEANUP_TIMEOUT);evidence.terminalStatus=terminal.status;evidence.result=compact(terminal.value);evidence.cancelConfirmed=terminal.status===5;
          if(terminal.status!==5){evidence.cancelError=`取消后终态为 ${terminal.status}，未确认CANCELED`;throw new Error(String(evidence.cancelError));}
        }catch(cancelError){
          evidence.cancelConfirmed=false;evidence.cancelError=errorText(cancelError);
          throw new AuditError('failed',`${errorText(ctx.fault)}；取消未确认：${errorText(cancelError)}`);
        }
      }
      throw error;
    }finally{resultWait.abort();}
  }catch(error){primary=error;}
  finally{
    // Run every cleanup, including partial setup failure and local cancellation.
    if(setupAttempted){
      for(const step of spec.cleanup??[])try{cleanup.push(await invokeService(ctx,step,runId,true));}catch(error){cleanup.push({name:step.name,error:errorText(error)});evidence.cleanupError=errorText(error);primary=new AuditError('failed',`${primary?`${errorText(primary)}；`:''}清理失败：${errorText(error)}`);}
      for(const check of spec.afterCleanup??[]){const proof:Wire={};((evidence.afterCleanup??=[]) as unknown[]).push(proof);const independent=new Context(ctx.client,CLEANUP_TIMEOUT,undefined,ctx.observationClient);
        try{await sample(independent,{id:'afterCleanup',label:'afterCleanup',kind:'topic',topic:{...check,minSamples:check.minSamples??1}},proof);}catch(error){evidence.cleanupError=errorText(error);primary=new AuditError('failed',`${primary?`${errorText(primary)}；`:''}清理状态未确认：${errorText(error)}`);}finally{independent.dispose();}}
    }
  }
  if(primary)throw primary;
}
async function inspectInterface(ctx:Context,spec:Extract<CheckSpec,{kind:'interface'}>,evidence:Wire):Promise<void> {
  const api={topic:['topic_type','TopicType'],service:['service_type','ServiceType'],action:['action_type','ActionType']}[spec.interfaceKind];
  const actual=await ctx.wait(ctx.client.callService<{type:string}>(`/rosapi/${api[0]}`,`rosapi_msgs/srv/${api[1]}`,{[spec.interfaceKind]:spec.endpoint},ctx.options));
  evidence.endpoint=spec.endpoint;evidence.actualType=actual.type;if(actual.type!==spec.interfaceType)throw new Error(`实际类型 ${actual.type} 不等于 ${spec.interfaceType}`);
  const definitions=spec.interfaceKind==='topic'?[['message','message_details','MessageDetails']]:spec.interfaceKind==='service'?[['request','service_request_details','ServiceRequestDetails'],['response','service_response_details','ServiceResponseDetails']]:[['goal','action_goal_details','ActionGoalDetails'],['feedback','action_feedback_details','ActionFeedbackDetails'],['result','action_result_details','ActionResultDetails']];
  const definition=Object.fromEntries(await ctx.wait(Promise.all(definitions.map(async([key,endpoint,type])=>[key,await ctx.client.callService(`/rosapi/${endpoint}`,`rosapi_msgs/srv/${type}`,{type:actual.type},ctx.options)]))));
  for(const [part,body]of Object.entries(definition)){if(!Array.isArray((body as Wire).typedefs)||(body as {typedefs:unknown[]}).typedefs.length===0)throw new Error(`接口定义 ${part} 为空`);}
  // Complete definitions are useful review evidence; fail rather than claim a
  // complete definition when it exceeds this report's explicit byte budget.
  if(JSON.stringify(definition).length>262144)throw new Error('完整接口定义超过报告256KiB预算');evidence.definition=definition;
}
async function execute(ctx:Context,spec:CheckSpec,project:ProjectSpec,runId:string,evidence:Wire):Promise<void> {
  ctx.check();if(spec.mockOnly)await guard(ctx,project.mockGuard!,evidence);
  switch(spec.kind) {
    case 'topic':case 'image':case 'cloud':return sample(ctx,spec,evidence);
    case 'parameters':return readParameters(ctx,spec,evidence);
    case 'service':evidence.service=await invokeService(ctx,spec.service,runId);return;
    case 'parameterRoundtrip':return roundtrip(ctx,spec,runId,evidence);
    case 'action':return action(ctx,spec,runId,evidence);
    case 'interface':return inspectInterface(ctx,spec,evidence);
    case 'lifecycle':{const result=await ctx.wait(ctx.client.lifecycle(spec.node).getState(ctx.options));evidence.node=spec.node;evidence.state=result.current_state;if(typeof spec.state==='number'?result.current_state.id!==spec.state:result.current_state.label.toLowerCase()!==spec.state.toLowerCase())throw new Error(`Lifecycle状态 ${result.current_state.label}(${result.current_state.id}) 不匹配 ${spec.state}`);return;}
    case 'graph':{
      const [nodes,topics,actions]=await ctx.wait(Promise.all([
        ctx.client.callService<{nodes:string[]}>('/rosapi/nodes','rosapi_msgs/srv/Nodes',{},ctx.options),ctx.client.callService<{topics:string[];types:string[]}>('/rosapi/topics','rosapi_msgs/srv/Topics',{},ctx.options),ctx.client.callService<{action_servers:string[]}>('/rosapi/action_servers','rosapi_msgs/srv/GetActionServers',{},ctx.options)]));
      evidence.nodes=compact(nodes.nodes);evidence.topics=compact(topics);evidence.actions=compact(actions.action_servers);
      const missing=[...(spec.nodes??[]).filter(n=>!nodes.nodes.includes(n)).map(n=>`node ${n}`),...(spec.topics??[]).filter(n=>!topics.topics.includes(n)).map(n=>`topic ${n}`),...(spec.actions??[]).filter(n=>!actions.action_servers.includes(n)).map(n=>`action ${n}`)];if(missing.length)throw new Error(`ROS图缺少：${missing.join(', ')}`);return;
    }
    case 'tf':{
      const scene=project.view?.scene as {tf?:string;tfStatic?:string}|undefined;
      const tfTopic=scene?.tf??'/tf',tfStaticTopic=scene?.tfStatic??'/tf_static';
      evidence.tfTopic=tfTopic;evidence.tfStaticTopic=tfStaticTopic;
      const buffer=new TfBuffer(ctx.observationClient,{tfTopic,tfStaticTopic,clockTopic:false,maxFrames:256,maxSamplesPerFrame:100});let timer:ReturnType<typeof setInterval>|undefined;
      try{buffer.start();await ctx.wait(new Promise<void>(resolve=>{const check=()=>{evidence.target=spec.target;evidence.frames=buffer.frames();evidence.transforms=Object.fromEntries(spec.sources.map(source=>[source,buffer.lookup(spec.target,source)??null]));const missing=spec.sources.filter(source=>!buffer.lookup(spec.target,source));evidence.missing=missing;evidence.policy='latest common timestamp; display/graph connectivity only';if(!missing.length)resolve();};check();timer=setInterval(check,50);}));}finally{clearInterval(timer);buffer.dispose();}return;
    }
  }
}

/** Run exactly the explicit IDs, sequentially. FAIL does not hide later cases. */
export async function runProjectChecks(client:RosClient,project:ProjectSpec,selectedIds:readonly string[],options:RunOptions={}):Promise<ProjectReport> {
  const observationClient=options.observationClient??client;
  if(observationClient.options.url!==client.options.url)throw new TypeError('控制与观测客户端必须使用同一 rosbridge URL');
  validateProject(project);if(!Array.isArray(selectedIds)||selectedIds.some(id=>typeof id!=='string'))throw new TypeError('selectedIds must be an explicit string array');
  const chosen=new Set(selectedIds);if(chosen.size!==selectedIds.length)throw new TypeError('selectedIds contains duplicate IDs');for(const id of chosen)if(!project.cases.some(c=>c.id===id))throw new TypeError(`Unknown case id: ${id}`);
  const start=Date.now(),runId=`audit-${start.toString(36)}-${Math.random().toString(36).slice(2,10)}`;
  const results:CheckResult[]=project.cases.map(c=>({id:c.id,label:c.label,kind:c.kind,interactive:!!c.interactive,status:'NOT_RUN',startedAt:null,elapsedMs:0,evidence:{},error:chosen.has(c.id)?'尚未执行':'未选择'}));
  let completed=0;
  for(const spec of project.cases){if(!chosen.has(spec.id)||options.signal?.aborted)continue;
    const at=Date.now(),result=results.find(r=>r.id===spec.id)!;result.startedAt=new Date(at).toISOString();delete result.error;
    // Callback failures are presentation errors and must never bypass cleanup.
    try{options.onProgress?.({runId,completed,total:chosen.size,caseId:spec.id,label:spec.label,phase:'running'});}catch{}
    const context=new Context(client,spec.timeoutMs??10000,options.signal,observationClient);
    try{await execute(context,spec,project,runId,result.evidence);context.check();result.status='PASS';}
    catch(error){const actual=error instanceof AuditError?error:context.fault??error;result.status=actual instanceof AuditError&&actual.reason==='cancelled'?'CANCELLED':'FAIL';result.error=errorText(actual);}
    finally{context.dispose();result.elapsedMs=Date.now()-at;}
    if(JSON.stringify(result.evidence).length>524288){result.evidence=compact(result.evidence,0,{remaining:262144}) as Wire;result.status='FAIL';result.error=`${result.error?`${result.error}；`:''}证据超过单项512KiB报告预算，已截断，不能宣称完整验收`;}
    completed++;try{options.onResult?.(result);}catch{}try{options.onProgress?.({runId,completed,total:chosen.size,caseId:spec.id,label:spec.label,phase:'finished'});}catch{}
  }
  for(const result of results)if(result.status==='NOT_RUN'&&chosen.has(result.id)&&options.signal?.aborted)result.error='运行停止后未执行；未重放';
  const end=Date.now(),totals:Record<CheckStatus,number>={PASS:0,FAIL:0,CANCELLED:0,NOT_RUN:0};for(const result of results)totals[result.status]++;
  return {projectId:project.id,title:project.title,url:client.options.url,connectionMode:observationClient===client?'shared':'separate',runId,startedAt:new Date(start).toISOString(),finishedAt:new Date(end).toISOString(),elapsedMs:end-start,selectedIds:[...selectedIds],results,totals,notRun:results.filter(r=>r.status==='NOT_RUN').map(r=>r.id)};
}
