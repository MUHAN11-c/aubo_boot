/** Versioned, data-only workspaces. Import never connects or issues ROS commands. */
import {QOS} from '../sdk/index.js';
import type {TopicOptions} from '../sdk/index.js';
import type {DisplayRegistry} from './registry.js';

export interface TopicBinding extends TopicOptions {topic:string;type:string}
export interface DisplaySpec {
  id:string;name:string;plugin:string;enabled:boolean;
  inputs:Record<string,TopicBinding>;options:Record<string,unknown>;
}
export interface WorkspaceSpec {
  schemaVersion:1;fixedFrame:string;tf:string;tfStatic:string;useSimTime:boolean;
  layout:'split'|'stacked';displays:DisplaySpec[];
}
export const WORKSPACE_LIMITS=Object.freeze({jsonBytes:1048576,displays:16,spatial:8,cloudPoints:400000});
const rosName=/^\/[A-Za-z_][A-Za-z0-9_]*(\/[A-Za-z_][A-Za-z0-9_]*)*$/;
const messageType=/^[A-Za-z_][A-Za-z0-9_]*\/msg\/[A-Za-z_][A-Za-z0-9_]*$/;
const unsafe=new Set(['__proto__','prototype','constructor']);
const record=(value:unknown):value is Record<string,unknown>=>!!value&&typeof value==='object'&&!Array.isArray(value);
function text(value:unknown,key:string,max=1024):string {
  if(typeof value!=='string'||!value.trim()||value.length>max)throw new TypeError(`Invalid ${key}`);
  return value;
}
function keys(value:Record<string,unknown>,allowed:string[],label:string):void {
  if(Object.keys(value).some(key=>!allowed.includes(key)))throw new TypeError(`Unknown ${label} key`);
}
/** Reject cycles, accessors, exotic prototypes and excessive nesting before JSON.stringify. */
function plainCopy(value:unknown):unknown {
  const seen=new WeakSet<object>();let visits=0,bytes=0;
  const account=(value:string)=>{bytes+=new TextEncoder().encode(value).length+2;if(bytes>WORKSPACE_LIMITS.jsonBytes)throw new RangeError('Workspace JSON budget is 1 MiB');};
  const copy=(item:unknown,depth:number):unknown=>{
    if(++visits>20000||depth>16)throw new RangeError('Workspace JSON structure budget exceeded');
    if(item===null||typeof item==='boolean')return item;
    if(typeof item==='string'){if(item.length>65536)throw new RangeError('Workspace string budget exceeded');account(item);return item;}
    if(typeof item==='number'&&Number.isFinite(item))return item;
    if(typeof item!=='object'||item===null)throw new TypeError('Workspace must contain plain JSON');
    if(seen.has(item))throw new TypeError('Workspace contains a cycle');seen.add(item);
    const array=Array.isArray(item),proto=Object.getPrototypeOf(item);
    if(array?proto!==Array.prototype:proto!==Object.prototype&&proto!==null)throw new TypeError('Workspace must contain plain objects');
    if(Object.getOwnPropertySymbols(item).length)throw new TypeError('Unsafe JSON symbols');
    const descriptors=Object.getOwnPropertyDescriptors(item);
    if(array&&(item.length>20000||Object.keys(item).length!==item.length))throw new TypeError('Unsafe or oversized sparse array');
    const result:Record<string,unknown>|unknown[]=array?[]:{};
    for(const [key,descriptor] of Object.entries(descriptors)){
      if(array&&key==='length')continue;
      account(key);
      if(unsafe.has(key)||descriptor.get||descriptor.set)throw new TypeError('Unsafe JSON key or accessor');
      if(array&&!/^(0|[1-9][0-9]*)$/.test(key))throw new TypeError('Unsafe array key');
      (result as Record<string,unknown>)[key]=copy(descriptor.value,depth+1);
    }
    seen.delete(item);return result;
  };
  const result=copy(value,0);
  if(new TextEncoder().encode(JSON.stringify(result)).length>WORKSPACE_LIMITS.jsonBytes)throw new RangeError('Workspace JSON budget is 1 MiB');
  return result;
}
function binding(value:unknown):TopicBinding {
  if(!record(value))throw new TypeError('Input must be a Topic binding');
  keys(value,['topic','type','qos','throttleMs','compression'],'Topic');
  const topic=text(value.topic,'Topic'),type=text(value.type,'message type',256);
  if(!rosName.test(topic))throw new TypeError('Choose an absolute ROS Topic');
  if(!messageType.test(type))throw new TypeError('Use a ROS 2 package/msg/Type');
  const qos=value.qos??'reliable';
  if(typeof qos==='string'){
    if(!Object.hasOwn(QOS,qos))throw new TypeError('Unknown QoS preset');
  }else{
    if(!record(qos))throw new TypeError('Invalid QoS');
    keys(qos,['history','depth','reliability','durability'],'QoS');
    if(qos.history!=='keep_last'||!Number.isSafeInteger(qos.depth)||Number(qos.depth)<1||Number(qos.depth)>10000||
      typeof qos.reliability!=='string'||!['reliable','best_effort'].includes(qos.reliability)||typeof qos.durability!=='string'||!['volatile','transient_local'].includes(qos.durability))throw new TypeError('Invalid QoS profile');
  }
  const throttleMs=value.throttleMs??0,compression=value.compression??'none';
  if(!Number.isSafeInteger(throttleMs)||Number(throttleMs)<0||Number(throttleMs)>60000)throw new TypeError('Invalid throttleMs (0..60000)');
  if(typeof compression!=='string'||!['none','cbor'].includes(compression))throw new TypeError('Unknown compression');
  return {topic,type,qos:qos as TopicOptions['qos'],throttleMs:Number(throttleMs),compression:compression as TopicOptions['compression']};
}
/** Canonical form matches RosClient sharing semantics without creating a DDS reader. */
export function bindingKey(input:TopicBinding):string {
  const qos=typeof input.qos==='string'?QOS[input.qos]:input.qos??QOS.reliable;
  return JSON.stringify([input.type,qos.history,qos.depth,qos.reliability,qos.durability,input.throttleMs??0,input.compression??'none']);
}
export function validateWorkspace(value:unknown,registry:DisplayRegistry):WorkspaceSpec {
  const item=plainCopy(value);
  if(!record(item))throw new TypeError('Workspace must be an object');
  keys(item,['schemaVersion','fixedFrame','tf','tfStatic','useSimTime','layout','displays'],'workspace');
  if(item.schemaVersion!==1)throw new TypeError('Unknown workspace schemaVersion');
  const fixedFrame=text(item.fixedFrame,'fixed frame',1024),tf=text(item.tf,'TF'),tfStatic=text(item.tfStatic,'static TF');
  if(!rosName.test(tf)||!rosName.test(tfStatic))throw new TypeError('Use absolute TF Topics');
  if(typeof item.useSimTime!=='boolean'||typeof item.layout!=='string'||!['split','stacked'].includes(item.layout))throw new TypeError('Invalid workspace clock/layout');
  if(!Array.isArray(item.displays)||item.displays.length>WORKSPACE_LIMITS.displays)throw new RangeError('Display count budget is 16');
  const ids=new Set<string>();let spatial=0,cloudPoints=0;
  const consumers=new Map<string,string>();
  const claim=(input:TopicBinding)=>{
    const key=bindingKey(input),previous=consumers.get(input.topic);
    if(previous&&previous!==key)throw new TypeError(`Topic conflict: use the same type/options for ${input.topic}`);
    consumers.set(input.topic,key);
  };
  const displays=item.displays.map(value=>{
    if(!record(value))throw new TypeError('Invalid display');
    keys(value,['id','name','plugin','enabled','inputs','options'],'display');
    const id=text(value.id,'display id',64),name=text(value.name,'display name',120),plugin=text(value.plugin,'plugin',64);
    if(!/^[a-zA-Z_][a-zA-Z0-9_-]*$/.test(id)||ids.has(id))throw new TypeError('Invalid or duplicate display id');ids.add(id);
    if(typeof value.enabled!=='boolean'||!record(value.inputs)||!record(value.options))throw new TypeError('Invalid display configuration');
    const definition=registry.get(plugin),inputs:Record<string,TopicBinding>={};
    keys(value.inputs,definition.ports.map(port=>port.id),'input port');
    for(const port of definition.ports){
      const input=binding(value.inputs[port.id]);
      if(!port.types.includes('*')&&!port.types.includes(input.type))throw new TypeError(`Wrong type for ${plugin}.${port.id}`);
      inputs[port.id]=input;if(value.enabled)claim(input);
    }
    const options=definition.validate(value.options);
    if(value.enabled&&definition.spatial)spatial++;
    if(value.enabled&&plugin==='cloud')cloudPoints+=Number(options.maxPoints);
    return {id,name,plugin,enabled:value.enabled,inputs,options};
  });
  if(spatial>WORKSPACE_LIMITS.spatial||cloudPoints>WORKSPACE_LIMITS.cloudPoints)throw new RangeError('Spatial/cloud budget exceeded (8 displays / 400000 points)');
  if(spatial){
    claim(binding({topic:tf,type:'tf2_msgs/msg/TFMessage',qos:'reliable'}));
    claim(binding({topic:tfStatic,type:'tf2_msgs/msg/TFMessage',qos:'staticTf'}));
    if(item.useSimTime)claim(binding({topic:'/clock',type:'rosgraph_msgs/msg/Clock',qos:'sensor'}));
  }
  return {schemaVersion:1,fixedFrame,tf,tfStatic,useSimTime:item.useSimTime,layout:item.layout as WorkspaceSpec['layout'],displays};
}
export function parseWorkspace(json:string,registry:DisplayRegistry):WorkspaceSpec {
  if(new TextEncoder().encode(json).length>WORKSPACE_LIMITS.jsonBytes)throw new RangeError('Workspace JSON budget is 1 MiB');
  return validateWorkspace(JSON.parse(json),registry);
}
/** Convert existing view profiles to editable templates. Every display starts disabled. */
export function workspaceFromProfile(profile:Record<string,unknown>,registry:DisplayRegistry):WorkspaceSpec {
  const scene=(profile.scene??{}) as Record<string,unknown>,image=profile.image as Record<string,unknown>|undefined;
  const depth=profile.depthImage as Record<string,unknown>|undefined,plot=profile.plot as Record<string,unknown>|undefined;
  const data=(topic:unknown,type:string,options:TopicOptions={}):TopicBinding=>({topic:String(topic),type,...options});
  const displays:DisplaySpec[]=[];
  const add=(id:string,plugin:string,inputs:DisplaySpec['inputs'],options:DisplaySpec['options']={})=>displays.push({id,name:id==='image'?'彩色图像':id==='depth'?'深度图像':registry.get(plugin).title,plugin,enabled:false,inputs,options});
  add('robot','robot',{description:data(scene.robotDescription??'/robot_description','std_msgs/msg/String',{qos:'latched',...scene.robotDescriptionOptions as TopicOptions}),joints:data(scene.jointStates??'/joint_states','sensor_msgs/msg/JointState',{qos:'sensor',...scene.jointStatesOptions as TopicOptions})});
  add('tf','tf',{});
  add('markers','markers',{data:data(scene.markers??'/visualization_marker_array','visualization_msgs/msg/MarkerArray',{qos:'reliable',...scene.markersOptions as TopicOptions})});
  add('cloud','cloud',{data:data(scene.pointCloud??'/points','sensor_msgs/msg/PointCloud2',{qos:'sensor',compression:'cbor',throttleMs:100,...scene.pointCloudOptions as TopicOptions})});
  add('path','path',{data:data(scene.path??'/plan','nav_msgs/msg/Path',{qos:'reliable',...scene.pathOptions as TopicOptions})});
  for(const [id,view] of [['image',image],['depth',depth]] as const)if(view){
    const options:Record<string,unknown>={};if(view.depthScale!==undefined)options.depthScale=view.depthScale;
    add(id,'image',{data:data(view.topic,String(view.type),{qos:'sensor',compression:'cbor',throttleMs:500,...view.subscription as TopicOptions})},options);
  }
  if(plot)add('plot','plot',{data:data(plot.topic,String(plot.type),{qos:'sensor',...plot.subscription as TopicOptions})},{fields:plot.fields??['data']});
  return validateWorkspace({schemaVersion:1,fixedFrame:scene.fixedFrame??'base_link',tf:scene.tf??'/tf',tfStatic:scene.tfStatic??'/tf_static',useSimTime:scene.useSimTime??false,layout:'split',displays},registry);
}
