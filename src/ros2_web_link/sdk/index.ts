/**
 * Framework-independent ROS 2 browser client, backed by upstream roslibjs.
 * Adapts wire budgets, QoS, request lifetime and ROS 2 parameter/status gaps.
 * No robot business logic, REST proxy, command queue or hardware authority lives here.
 */
import {Ros, Topic} from 'roslib';
import {assertSafeJson, decodeParameter, encodeParameter} from './parameters.js';
import type {ParameterWrite} from './parameters.js';
import {LinkError,errorText} from './errors.js';
import {BoundedTransport,TransportError,transportLimits} from './transport.js';
import type {TransportLimits} from './limits.js';
export {LinkError} from './errors.js';
export {DEFAULT_TRANSPORT_LIMITS} from './limits.js';
export type {TransportLimits} from './limits.js';
export {assertSafeJson, decodeParameter, encodeParameter} from './parameters.js';
export type {ParameterType, ParameterWrite, ParameterValue, DecodedParameter} from './parameters.js';

export type ConnectionState = 'idle' | 'connecting' | 'connected' | 'reconnecting' | 'disconnected' | 'disposed';
export interface ClientOptions {
  url:string;reconnect?:boolean;retryMs?:number;connectTimeoutMs?:number;transportLimits?:Partial<TransportLimits>;
  /** Shared Service/Action local waits, 1..4096; default 256. Excess calls are not sent or queued. */
  maxPendingRequests?:number;
}
export interface RequestOptions { timeoutMs?: number; signal?: AbortSignal }
export interface Qos { history: 'keep_last'; depth: number; reliability: 'reliable' | 'best_effort'; durability: 'volatile' | 'transient_local' }
export const QOS: Readonly<Record<'reliable' | 'sensor' | 'latched' | 'staticTf', Readonly<Qos>>> = Object.freeze({
  reliable: Object.freeze({history:'keep_last',depth:10,reliability:'reliable',durability:'volatile'}),
  sensor: Object.freeze({history:'keep_last',depth:5,reliability:'best_effort',durability:'volatile'}),
  latched: Object.freeze({history:'keep_last',depth:1,reliability:'reliable',durability:'transient_local'}),
  // Unlike one retained configuration, /tf_static has multiple publishers.
  // Depth 1 lost entire robot/static batches on this Jazzy/FastDDS host.
  staticTf: Object.freeze({history:'keep_last',depth:100,reliability:'reliable',durability:'transient_local'}),
});
export interface TopicOptions { qos?: keyof typeof QOS | Qos; throttleMs?: number; compression?: 'none' | 'cbor' }
export interface SampleInfo { receivedAt: number }
export interface ActionResult<T> { status: number; successful: boolean; value: T; error?: string }
export interface GoalHandle<T> { id: string; result: Promise<ActionResult<T>>; cancel(): void }
type Wire = Record<string, unknown>;
type Pending = { fail(error: Error): void };
type Subscription = { name: string; type: string; options: TopicOptions; callbacks: Set<(m:unknown,i:SampleInfo)=>void>; topic?: Topic<unknown> };

// Correlation IDs are opaque protocol strings, not ROS GoalUUIDs or secrets.
// crypto.randomUUID is restricted to secure contexts; plain LAN HTTP must work.
const clientPrefix=`${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
let nextId=0;
const requestId=()=>`web-link:${clientPrefix}:${++nextId}`;

function name(value: string): string {
  if (!/^\/[A-Za-z_][A-Za-z0-9_]*(\/[A-Za-z_][A-Za-z0-9_]*)*$/.test(value)) {
    throw new TypeError(`Use an absolute ROS graph name: ${value}`);
  }
  return value;
}
const MAX_TIMER_DELAY=2147483647;
function timeout(value: number): number {
  if (!Number.isFinite(value) || value <= 0 || value > MAX_TIMER_DELAY) throw new TypeError('timeoutMs must be a positive finite timer duration');
  return value;
}
function delayUntil(deadline:number):number {
  // Rounding a valid maximum deadline can exceed the timer's int32 range by
  // one. Clamp after ceil; otherwise browsers/Node may turn days into 1ms.
  return Math.max(1,Math.min(MAX_TIMER_DELAY,Math.ceil(deadline-performance.now())));
}
function qos(options: TopicOptions): Qos {
  const profile=typeof options.qos === 'string' ? QOS[options.qos] : options.qos ?? QOS.reliable;
  if (!profile || profile.history !== 'keep_last' || !Number.isSafeInteger(profile.depth) || profile.depth < 1 ||
    !['reliable','best_effort'].includes(profile.reliability) || !['volatile','transient_local'].includes(profile.durability)) {
    throw new TypeError('Unsupported QoS: supply keep_last, positive depth, reliability and durability');
  }
  if (options.throttleMs !== undefined && (!Number.isSafeInteger(options.throttleMs) || options.throttleMs < 0)) throw new TypeError('throttleMs must be a nonnegative integer');
  return {history:'keep_last',depth:profile.depth,reliability:profile.reliability,durability:profile.durability};
}
function topicOptions(options:TopicOptions):TopicOptions {
  const compression=options.compression ?? 'none';
  if(!['none','cbor'].includes(compression)) throw new TypeError('Unsupported compression; use none or cbor');
  return {qos:qos(options),throttleMs:options.throttleMs ?? 0,compression};
}

/** One client owns one WebSocket session. A reconnect creates a fresh ROSLIB.Ros. */
export class RosClient {
  readonly options: Required<Omit<ClientOptions,'transportLimits'|'maxPendingRequests'>>&{readonly maxPendingRequests:number;transportLimits:Readonly<TransportLimits>};
  state: ConnectionState = 'idle';
  private ros?: Ros;
  private transport?:BoundedTransport;
  private wanted = false;
  private connecting?: Promise<void>;
  private retry?: ReturnType<typeof setTimeout>;
  private connectionTimer?: ReturnType<typeof setTimeout>;
  private rejectConnect?: (error: Error) => void;
  private stateListeners = new Set<(state:ConnectionState)=>void>();
  private statusListeners = new Set<(message:Wire)=>void>();
  private subscriptions = new Map<string,Subscription>();
  private publishers = new Set<{reset():void;dispose():void}>();
  private pending = new Map<string,Pending>();

  constructor(options: ClientOptions) {
    const parsed=new URL(options.url);
    if (!['ws:','wss:'].includes(parsed.protocol)) throw new TypeError('rosbridge URL must use ws:// or wss://');
    this.options={reconnect:true,retryMs:1000,connectTimeoutMs:5000,...options,
      maxPendingRequests:options.maxPendingRequests===undefined?256:options.maxPendingRequests,
      transportLimits:transportLimits(options.transportLimits)};
    timeout(this.options.retryMs); timeout(this.options.connectTimeoutMs);
    if(!Number.isInteger(this.options.maxPendingRequests)||this.options.maxPendingRequests<1||this.options.maxPendingRequests>4096){
      throw new TypeError('maxPendingRequests must be an integer from 1 to 4096');
    }
    // Keep the validated bound immutable even for untyped JS consumers, while
    // preserving the existing mutability of other connection options.
    Object.defineProperty(this.options,'maxPendingRequests',{writable:false,configurable:false});
  }
  get connected(): boolean { return this.state === 'connected' && !!this.ros?.isConnected; }
  /** Listen for connection state, separately from message freshness. Returns cleanup. */
  onState(listener: (state:ConnectionState)=>void): ()=>void {
    this.stateListeners.add(listener);this.stateCallback(listener,this.state);return ()=>{this.stateListeners.delete(listener);};
  }
  /** Uncorrelated bridge status (e.g. denied topic subscription), for diagnostics. */
  onStatus(listener:(message:Wire)=>void):()=>void {
    this.statusListeners.add(listener); return ()=>{this.statusListeners.delete(listener);};
  }
  private notifyStatus(message:Wire):void {
    for(const listener of this.statusListeners){try{listener(message);}catch{console.error('ROS Web status listener threw; other listeners remain active');}}
  }
  private stateCallback(listener:(state:ConnectionState)=>void,state:ConnectionState):void {
    try{listener(state);}catch(error){this.notifyStatus({op:'status',level:'error',code:'CALLBACK_ERROR',source:'state',msg:errorText(error)});}
  }
  private setState(state:ConnectionState):void {this.state=state;for(const listener of this.stateListeners)this.stateCallback(listener,state);}
  /** Local cleanup must finish even if its remote unregister cannot be queued.
   * Closing the session lets rosbridge release its connection-owned resources;
   * an unsent unregister is reported, never deferred as an offline command.
   */
  private cleanupSend(operation:()=>void):void {
    try{operation();}catch(error){
      this.notifyStatus({op:'status',level:'error',code:'CLEANUP_SEND_FAILED',msg:`Unregister could not be sent: ${errorText(error)}; closing this session`});
      this.disconnect();
    }
  }
  private live():Ros {
    if (!this.connected) throw new LinkError('DISCONNECTED','rosbridge is disconnected; operation was not sent');
    return this.ros!;
  }
  private send(ros:Ros, message:Wire):void {
    ros.callOnConnection(message as unknown as Parameters<Ros['callOnConnection']>[0]);
  }

  /** Resolves only after WebSocket opens. Explicit disconnect disables retries. */
  connect():Promise<void> {
    if (this.state === 'disposed') return Promise.reject(new LinkError('DISPOSED','Client is disposed'));
    if (this.connected) return Promise.resolve();
    if (this.connecting) return this.connecting;
    this.wanted=true; clearTimeout(this.retry);
    const deadline=performance.now()+this.options.connectTimeoutMs;
    const ros=new Ros({transportFactory:async url=>{
      const transport=new BoundedTransport(url,this.options.transportLimits,id=>this.pending.has(id));
      if(this.ros===ros)this.transport=transport;else transport.close();return transport;
    }}); this.ros=ros;
    // ROSLIB normally queues callOnConnection. Disallow that for every operation,
    // including Topic internals, to prevent stale commands being replayed.
    const original=ros.callOnConnection.bind(ros);
    ros.callOnConnection=(message)=>{
      if (this.ros !== ros || !ros.isConnected) throw new LinkError('DISCONNECTED','No offline command queue');
      original(message);
    };
    const attempt=new Promise<void>((resolve,reject)=>{
      this.rejectConnect=reject;
      ros.on('connection',()=>{
        if (this.ros !== ros) return;
        // The native open event may be queued behind a long UI task. Retain
        // ordinary timeout/reconnect semantics even if its timer runs later.
        if(performance.now()>=deadline){lost();return;}
        clearTimeout(this.connectionTimer);
        this.setState('connected');
        if(this.ros!==ros)return; // A state consumer may explicitly disconnect.
        try{for(const sub of this.subscriptions.values())this.attach(sub);}catch(error){lost(error);return;}
        this.rejectConnect=undefined;this.connecting=undefined;
        resolve();
      });
      const lost=(reason?:unknown)=>{
        if (this.ros !== ros) return;
        clearTimeout(this.connectionTimer); this.ros=undefined;
        this.connecting=undefined; this.rejectConnect=undefined;
        const error=reason instanceof TransportError?new LinkError(reason.code,`${reason.message}; in-flight operation outcome is unknown`):new LinkError('DISCONNECTED','Connection lost; in-flight operation outcome is unknown');
        if(reason instanceof TransportError)this.notifyStatus({op:'status',level:'error',code:reason.code,msg:reason.message});
        reject(error); for(const item of [...this.pending.values()]) item.fail(error);
        for(const sub of this.subscriptions.values()) sub.topic=undefined;
        for(const pub of this.publishers) pub.reset();
        ros.removeAllListeners(); ros.close();
        const retryable=!(reason instanceof TransportError)||reason.code==='FRAGMENT_TIMEOUT';
        if (this.wanted && this.options.reconnect && retryable) {
          // Register before notifying: disconnect() inside onState must be able
          // to cancel this retry, and the timer rechecks the user's intent.
          this.retry=setTimeout(()=>{this.retry=undefined;if(this.wanted&&this.state==='reconnecting')void this.connect().catch(()=>{});}, this.options.retryMs);
          this.setState('reconnecting');
        } else this.setState('disconnected');
      };
      ros.on('close',lost); ros.on('error',lost);
      ros.on('status',(raw)=>this.notifyStatus(raw as unknown as Wire));
      this.connectionTimer=setTimeout(lost,delayUntil(deadline));
      queueMicrotask(()=>{
        if(this.ros!==ros||!this.wanted)return;
        if(performance.now()>=deadline){lost();return;}
        void ros.connect(this.options.url).then(()=>{
          // A disconnect may precede the asynchronous transport factory result.
          if(this.ros !== ros) ros.close();
        }).catch(lost);
      });
    });
    // Install termination capability before invoking user state listeners.
    this.connecting=attempt;this.setState('connecting');return attempt;
  }
  /** Close locally; does NOT cancel remote services or actions. */
  disconnect():void {
    this.wanted=false; clearTimeout(this.retry); clearTimeout(this.connectionTimer);
    const error=new LinkError('DISCONNECTED','Disconnected locally; remote outcome is unknown');
    this.rejectConnect?.(error); this.rejectConnect=undefined; this.connecting=undefined;
    for(const item of [...this.pending.values()]) item.fail(error);
    const ros=this.ros; this.ros=undefined;
    ros?.removeAllListeners(); ros?.close();
    for(const sub of this.subscriptions.values()) sub.topic=undefined;
    for(const pub of this.publishers) pub.reset();
    if(this.state !== 'disposed') this.setState('disconnected');
  }
  /** Release all SDK resources. Reuse requires a new client. */
  dispose():void {
    if(this.state === 'disposed') return;
    for(const pub of [...this.publishers]) pub.dispose();
    this.disconnect(); this.subscriptions.clear(); this.setState('disposed');
    this.stateListeners.clear(); this.statusListeners.clear();
  }
  stats():{subscriptions:number;publishers:number;pending:number;pendingLimit:number;listeners:number} {
    return {subscriptions:this.subscriptions.size,publishers:this.publishers.size,pending:this.pending.size,pendingLimit:this.options.maxPendingRequests,
      listeners:this.ros?.eventNames().reduce((sum,event)=>sum+this.ros!.listenerCount(event),0) ?? 0};
  }
  /** Current/last session's wire counters and bounded fragment/send queues. */
  transportStats(){return this.transport?.stats()??{receivedFrames:0,receivedBytes:0,sentFrames:0,sentBytes:0,ignoredResponses:0,fragmentMessages:0,fragmentBytes:0,sendBufferedBytes:0,limits:this.options.transportLimits,lastError:undefined};}
  private makeTopic(ros:Ros, topicName:string, type:string, options:TopicOptions):Topic<unknown> {
    const profile=qos(options);
    // Retained TF arrives as a burst from several writers. A one-message
    // rosbridge queue drops every static batch except the last even when the
    // DDS history is deep enough. Keep this queue bounded by the TF QoS depth.
    const queueLength=type==='tf2_msgs/msg/TFMessage'&&profile.durability==='transient_local'?profile.depth:1;
    const topic=new Topic({ros,name:topicName,messageType:type,reconnect_on_close:false,
      throttle_rate:options.throttleMs ?? 0,queue_length:queueLength,compression:options.compression ?? 'none'});
    // Upstream public hook; 2.1.0 Topic has no qos constructor option.
    topic.callForSubscribeAndAdvertise=(message)=>{
      this.send(ros,{...message,qos:profile});
    };
    // With reconnect_on_close=false, upstream advertise() adds an anonymous
    // close listener that unadvertise() never removes. This client owns session
    // reset already, so discard only the listeners introduced by this call.
    const advertise=topic.advertise.bind(topic);
    topic.advertise=()=>{
      const before=new Set(ros.listeners('close')); advertise();
      for(const listener of ros.listeners('close')) if(!before.has(listener)) ros.off('close',listener);
    };
    return topic;
  }
  private attach(sub:Subscription):void {
    if(sub.topic || !this.connected) return;
    const ros=this.live(),topic=this.makeTopic(ros,sub.name,sub.type,sub.options);
    const before=new Set(ros.listeners(sub.name));
    try{
      topic.subscribe(message=>{
        const info={receivedAt:Date.now()};
        for(const callback of sub.callbacks){try{callback(message,info);}catch(error){
          this.notifyStatus({op:'status',level:'error',code:'CALLBACK_ERROR',topic:sub.name,msg:errorText(error)});
        }}
      });sub.topic=topic;
    }catch(error){
      // Upstream registers its ROS listener before sending subscribe. No remote
      // unregister is needed for a locally refused send; remove only new hooks.
      for(const listener of ros.listeners(sub.name))if(!before.has(listener))ros.off(sub.name,listener);
      topic.removeAllListeners();throw error;
    }
  }
  /** Register subscriptions before/after connect. Only subscriptions are restored. */
  subscribe<T=unknown>(topicName:string,type:string,callback:(message:T,info:SampleInfo)=>void,options:TopicOptions={}):()=>void {
    name(topicName); const normalized=topicOptions(options);
    if(this.state === 'disposed') throw new LinkError('DISPOSED','Client is disposed');
    let sub=this.subscriptions.get(topicName);
    if(sub && (sub.type !== type || JSON.stringify(sub.options) !== JSON.stringify(normalized))) {
      throw new TypeError('One topic must use the same type/options within a client');
    }
    if(!sub) {sub={name:topicName,type,options:normalized,callbacks:new Set()}; this.subscriptions.set(topicName,sub);}
    const wrapped=(message:unknown,info:SampleInfo)=>callback(message as T,info);
    sub.callbacks.add(wrapped);
    try{this.attach(sub);}catch(error){sub.callbacks.delete(wrapped);if(!sub.callbacks.size)this.subscriptions.delete(topicName);throw error;}
    let stopped=false;
    return ()=>{
      if(stopped) return; stopped=true; sub!.callbacks.delete(wrapped);
      if(sub!.callbacks.size===0) {
        this.subscriptions.delete(topicName);
        if(this.connected)this.cleanupSend(()=>sub!.topic?.unsubscribe());sub!.topic=undefined;
      }
    };
  }
  /** Advertise online; DDS discovery must complete before the first publish.
   * publish is never retried automatically and is not an application-level ACK.
   */
  publisher<T=unknown>(topicName:string,type:string,options:TopicOptions={}):{publish(message:T):void;dispose():void} {
    name(topicName); const normalized=topicOptions(options);
    if(this.state === 'disposed') throw new LinkError('DISPOSED','Client is disposed');
    let topic:Topic<unknown>|undefined; let disposed=false;
    const resource={reset:()=>{topic=undefined;},dispose:()=>{
      if(disposed) return; disposed=true;
      try{if(this.connected)this.cleanupSend(()=>topic?.unadvertise());}
      finally{this.publishers.delete(resource);topic=undefined;}
    }};
    this.publishers.add(resource);
    if(this.connected) {try{topic=this.makeTopic(this.live(),topicName,type,normalized);topic.advertise();}catch(error){resource.dispose();throw error;}}
    return {publish:(message:T)=>{
      if(disposed) throw new LinkError('DISPOSED','Publisher is disposed');
      const ros=this.live(); assertSafeJson(message);
      topic ??= this.makeTopic(ros,topicName,type,normalized); topic.publish(message);
    },dispose:resource.dispose};
  }

  /** Standard call_service envelope over ROSLIB, with bounded, removable listeners.
   * ROSLIB.Service hides its correlation ID, so using its callback API would leak
   * listeners on timeout. These few protocol fields are deliberately kept here.
   */
  callService<T=Record<string,unknown>>(service:string,serviceType:string,args:unknown={},options:RequestOptions={}):Promise<T> {
    try {
      name(service); this.live(); assertSafeJson(args);
      return this.request<T>({op:'call_service',service,type:serviceType,args,timeout:timeout(options.timeoutMs ?? 5000)/1000},options);
    } catch(error) {return Promise.reject(error);}
  }
  private request<T>(message:Wire,options:RequestOptions={},feedback?:(m:unknown)=>void):Promise<T> {
    const ros=this.live();
    // Resolve caller getters before registration. Structural validation accepts
    // real AbortSignals from other browser realms without instanceof checks.
    const signal=options.signal,addAbort=signal?.addEventListener,removeAbort=signal?.removeEventListener;
    if(signal!==undefined&&(typeof signal?.aborted!=='boolean'||typeof addAbort!=='function'||typeof removeAbort!=='function')){
      throw new TypeError('signal must be an AbortSignal');
    }
    const id=message.id as string || requestId();
    const duration=timeout(options.timeoutMs ?? 5000),deadline=performance.now()+duration;
    // Read caller options first: a getter can synchronously start another call.
    // Bound timers/listeners immediately before registration, with no queue.
    if(this.pending.size>=this.options.maxPendingRequests)throw new LinkError('REQUEST_LIMIT','Local Service/Action wait budget is full; operation was not sent');
    return new Promise<T>((resolve,reject)=>{
      let timer:ReturnType<typeof setTimeout>,settled=false;
      const expired=()=>performance.now()>=deadline;
      const timeoutError=()=>new LinkError('TIMEOUT','Timed out waiting; remote outcome is unknown');
      const finish=(error?:Error,value?:T)=>{
        if(settled)return;
        // UI/decoder/GL long tasks delay timers. A queued response must not
        // turn an expired wait green, including time spent validating values.
        if(!error&&expired())error=timeoutError();
        settled=true;
        clearTimeout(timer); ros.off(id,receive); ros.off(`status:${id}`,status);
        this.pending.delete(id);
        // Even a supplied/overridden cleanup method must not prevent promise
        // settlement or leave the client permanently at its admission limit.
        try{removeAbort?.call(signal,'abort',abort);}catch(cleanup){error??=cleanup instanceof Error?cleanup:new Error(errorText(cleanup));}
        if(error) reject(error); else resolve(value!);
      };
      const item:Pending={fail:error=>finish(error)};
      const abort=()=>finish(new LinkError('WAIT_ABORTED','Stopped waiting locally; remote request was not canceled'));
      const status=(raw:unknown)=>{
        if(expired()){finish(timeoutError());return;}
        const m=raw as Wire;
        if(m.level === 'error') finish(new LinkError('BRIDGE_ERROR',String(m.msg ?? 'rosbridge error')));
      };
      const receive=(raw:unknown)=>{
        if(expired()){finish(timeoutError());return;}
        const m=raw as Wire;
        if(m.op==='action_feedback') {try{feedback?.(m.values);}catch(error){this.notifyStatus({op:'status',level:'error',code:'CALLBACK_ERROR',source:'feedback',msg:errorText(error)});}return;}
        if(m.op==='action_result') {
          finish(undefined,{status:m.status,successful:m.result === true && m.status === 4,
            value:m.values,...(m.result === false ? {error:String(m.values)}:{})} as T);
        } else if(m.op==='service_response') {
          if(m.result) {
            try {assertSafeJson(m.values);finish(undefined,m.values as T);} catch(error) {finish(error as Error);}
          } else finish(new LinkError('SERVICE_FAILED',String(m.values)));
        }
      };
      this.pending.set(id,item);
      try{
        ros.on(id,receive); ros.on(`status:${id}`,status);
        timer=setTimeout(()=>finish(timeoutError()),delayUntil(deadline));
        addAbort?.call(signal,'abort',abort,{once:true});
        if(settled)return; // A supplied listener hook may abort synchronously.
        if(signal?.aborted) {abort();return;}
        if(expired()){finish(timeoutError());return;}
        this.send(ros,{...message,id});
      }catch(error){finish(error instanceof Error?error:new Error(errorText(error)));}
    });
  }
  /** ROS 2 Action preserves numeric terminal status (4,5,6) and bridge failures (0).
   * cancel() requests cancellation; only the eventual result confirms CANCELED.
   */
  sendGoal<G=unknown,F=unknown,R=unknown>(action:string,actionType:string,goal:G,options:RequestOptions & {onFeedback?:(feedback:F)=>void}={}):GoalHandle<R> {
    name(action); const ros=this.live(); assertSafeJson(goal);
    const id=requestId();
    const result=this.request<ActionResult<R>>({op:'send_action_goal',id,action,action_type:actionType,args:goal,feedback:true},
      {...options,timeoutMs:options.timeoutMs ?? 60000},m=>options.onFeedback?.(m as F));
    return {id,result,cancel:()=>{
      if(this.live() !== ros) throw new LinkError('SESSION_CHANGED','Cannot cancel a goal from a different WebSocket session');
      this.send(ros,{op:'cancel_action_goal',id,action});
    }};
  }
  /** Basic ROS graph discovery. Introspection is on demand, without polling. */
  async graph():Promise<{topics:{topics:string[];types:string[]};services:{services:string[]};nodes:{nodes:string[]};actions:{action_servers:string[]}}> {
    const [topics,services,nodes,actions]=await Promise.all([
      this.callService<{topics:string[];types:string[]}>('/rosapi/topics','rosapi_msgs/srv/Topics'),
      this.callService<{services:string[]}>('/rosapi/services','rosapi_msgs/srv/Services'),
      this.callService<{nodes:string[]}>('/rosapi/nodes','rosapi_msgs/srv/Nodes'),
      this.callService<{action_servers:string[]}>('/rosapi/action_servers','rosapi_msgs/srv/GetActionServers'),
    ]); return {topics,services,nodes,actions};
  }
  /** Resolve live interface types using rosapi, instead of guessing from names. */
  async interfaceType(kind:'topic'|'service'|'action', endpoint:string):Promise<string> {
    name(endpoint);
    const api={topic:['topic_type','TopicType'],service:['service_type','ServiceType'],action:['action_type','ActionType']}[kind];
    if(!api) throw new TypeError('Unknown interface kind');
    return (await this.callService<{type:string}>(`/rosapi/${api[0]}`,`rosapi_msgs/srv/${api[1]}`,{[kind]:endpoint})).type;
  }
  /** Read installed ROS definitions, including nested request/result types. */
  async describeInterface(type:string):Promise<Record<string,unknown>> {
    const category=type.split('/')[1];
    const definitions=category==='msg' ? [['message','message_details','MessageDetails']] :
      category==='srv' ? [['request','service_request_details','ServiceRequestDetails'],['response','service_response_details','ServiceResponseDetails']] :
      category==='action' ? [['goal','action_goal_details','ActionGoalDetails'],['feedback','action_feedback_details','ActionFeedbackDetails'],['result','action_result_details','ActionResultDetails']] : undefined;
    if(!definitions) throw new TypeError('Use pkg/msg/Type, pkg/srv/Type or pkg/action/Type');
    return Object.fromEntries(await Promise.all(definitions.map(async([key,endpoint,serviceType])=>
      [key,await this.callService(`/rosapi/${endpoint}`,`rosapi_msgs/srv/${serviceType}`,{type})])));
  }
  /** Native lifecycle services; the application owns sequencing and authority. */
  lifecycle(node:string) {
    name(node);
    return {
      getState:(options?:RequestOptions)=>this.callService<{current_state:{id:number;label:string}}>(`${node}/get_state`,'lifecycle_msgs/srv/GetState',{},options),
      availableTransitions:(options?:RequestOptions)=>this.callService<{available_transitions:unknown[]}>(`${node}/get_available_transitions`,'lifecycle_msgs/srv/GetAvailableTransitions',{},options),
      change:(id:number,options?:RequestOptions)=>{
        if(!Number.isInteger(id)||id<1||id>255) return Promise.reject(new TypeError('Transition ID must be uint8, excluding zero'));
        return this.callService<{success:boolean}>(`${node}/change_state`,'lifecycle_msgs/srv/ChangeState',{transition:{id}},options);
      },
    };
  }
  /** Observe external parameter edits without polling or automatically writing back. */
  watchParameterEvents(node:string,callback:(message:Record<string,unknown>)=>void):()=>void {
    name(node);return this.subscribe<Record<string,unknown>>('/parameter_events','rcl_interfaces/msg/ParameterEvent',message=>{
      if(message.node===node) callback(message);
    });
  }
  /** ROS 2 node-local parameter services, not the legacy ROS 1 Param abstraction. */
  parameters(node:string) {
    name(node); const call=<T=Record<string,unknown>>(suffix:string,type:string,args:unknown,options?:RequestOptions)=>
      this.callService<T>(`${node}/${suffix}`,`rcl_interfaces/srv/${type}`,args,options);
    const writes=(items:ParameterWrite[])=>items.map(p=>({name:p.name,value:encodeParameter(p.type,p.value)}));
    return {
      list:(prefixes:string[]=[],depth=0,options?:RequestOptions)=>call<{result:{names:string[];prefixes:string[]}}>('list_parameters','ListParameters',{prefixes,depth},options).then(r=>r.result),
      describe:(names:string[],options?:RequestOptions)=>call<{descriptors:Record<string,unknown>[]}>('describe_parameters','DescribeParameters',{names},options),
      get:(names:string[],options?:RequestOptions)=>call<{values:Record<string,unknown>[]}>('get_parameters','GetParameters',{names},options).then(r=>({values:r.values.map(decodeParameter)})),
      set:(items:ParameterWrite[],options?:RequestOptions)=>call<{results:{successful:boolean;reason:string}[]}>('set_parameters','SetParameters',{parameters:writes(items)},options),
      setAtomically:(items:ParameterWrite[],options?:RequestOptions)=>call<{result:{successful:boolean;reason:string}}>('set_parameters_atomically','SetParametersAtomically',{parameters:writes(items)},options),
    };
  }
}
