/** Local, trusted display factories. JSON imports name a plugin; they never load code. */
import type {RosClient} from '../sdk/index.js';
import {ImageView,SeriesPlot} from '../media/index.js';
import type {ImageOptions} from '../media/index.js';
import type {SceneView,SceneOptions,SceneLayer} from '../scene/index.js';
import type {DisplaySpec,TopicBinding} from './config.js';

export interface DisplayInstance {start():void;dispose():void;stats?():unknown;assetStats?():ReturnType<SceneView['assetStats']>}
export interface DisplayContext {
  client:RosClient;root:HTMLElement;scene(options:Partial<SceneOptions>&{layers:SceneLayer[]}):SceneView;
  onStatus(message:string):void;
}
export interface DisplayDefinition {
  id:string;title:string;spatial:boolean;ports:{id:string;types:string[]}[];
  defaults?:{inputs?:Record<string,TopicBinding>;options?:Record<string,unknown>};
  validate(options:Record<string,unknown>):Record<string,unknown>;
  create(context:DisplayContext,spec:DisplaySpec):DisplayInstance;
}
export class DisplayRegistry {
  private definitions=new Map<string,DisplayDefinition>();
  register(definition:DisplayDefinition):this {
    if(!/^[a-z][a-z0-9_-]{0,63}$/.test(definition.id)||this.definitions.has(definition.id))throw new TypeError('Invalid or already registered plugin');
    if(typeof definition.create!=='function'||typeof definition.validate!=='function'||!Array.isArray(definition.ports)||new Set(definition.ports.map(p=>p.id)).size!==definition.ports.length)throw new TypeError('Invalid display definition');
    for(const port of definition.ports)if(!/^[a-z][a-z0-9_]{0,63}$/.test(port.id)||!port.types.length)throw new TypeError('Invalid input port');
    this.definitions.set(definition.id,definition);return this;
  }
  get(id:string):DisplayDefinition {const definition=this.definitions.get(id);if(!definition)throw new TypeError(`Unknown display plugin: ${id}`);return definition;}
  list():DisplayDefinition[] {return [...this.definitions.values()];}
}
function optionsOnly(options:Record<string,unknown>,keys:string[]):void {
  if(Object.keys(options).some(key=>!keys.includes(key)))throw new TypeError('Unknown display option');
}
function integer(value:unknown,fallback:number,max:number,min=1):number {
  const result=value??fallback;
  if(!Number.isSafeInteger(result)||Number(result)<min||Number(result)>max)throw new RangeError(`Option must be an integer ${min}..${max}`);
  return Number(result);
}
/** Bounded raw fallback for any message type, including CBOR typed arrays/cycles. */
export function previewMessage(message:unknown,maxDepth=5):string {
  if(!Number.isSafeInteger(maxDepth)||maxDepth<1||maxDepth>10)throw new RangeError('Preview depth must be 1..10');
  const seen=new WeakSet<object>();let entries=0;
  const summarize=(value:unknown,depth:number):unknown=>{
    if(++entries>160)return '…';
    if(typeof value==='string')return value.length>256?`${value.slice(0,256)}… (${value.length} chars)`:value;
    if(!value||typeof value!=='object')return typeof value==='bigint'?value.toString():value;
    if(ArrayBuffer.isView(value))return `${value.constructor.name} (${value.byteLength} bytes)`;
    if(value instanceof ArrayBuffer)return `ArrayBuffer (${value.byteLength} bytes)`;
    if(seen.has(value))return '[cycle]';seen.add(value);
    if(depth>=maxDepth)return '[depth limit]';
    if(Array.isArray(value))return [...value.slice(0,12).map(v=>summarize(v,depth+1)),...(value.length>12?[`… (${value.length} entries)`]:[])];
    const result:Record<string,unknown>=Object.create(null);
    for(const key of Object.keys(value).slice(0,24)){
      const property=Object.getOwnPropertyDescriptor(value,key);result[key.slice(0,80)]=property&&'value' in property?summarize(property.value,depth+1):'[accessor]';
    }
    return result;
  };
  const result=JSON.stringify(summarize(message,0),null,2)??String(message);return result.length>8192?`${result.slice(0,8180)}\n…`:result;
}
/** Built-ins reuse the package's existing decoders, TF buffer and upstream viewers. */
export function createDefaultRegistry():DisplayRegistry {
  const registry=new DisplayRegistry();
  const input=(topic:string,type:string,qos:TopicBinding['qos']='reliable'):TopicBinding=>({topic,type,qos});
  const spatial=(id:SceneLayer,title:string,type?:string)=>registry.register({id,title,spatial:true,
    ports:type?[{id:'data',types:[type]}]:[],defaults:{inputs:type?{data:input(`/${id}`,type,id==='cloud'?'sensor':'reliable')}:{}},
    validate(options){optionsOnly(options,id==='cloud'?['maxPoints']:id==='markers'?['maxMarkers']:[]);return id==='cloud'?{maxPoints:integer(options.maxPoints,100000,100000)}:id==='markers'?{maxMarkers:integer(options.maxMarkers,1000,10000)}:{};},
    create(context,spec){const binding=spec.inputs.data;return context.scene({layers:[id],
      ...(id==='cloud'?{pointCloud:binding.topic,pointCloudOptions:binding,maxPoints:Number(spec.options.maxPoints)}:{}),
      ...(id==='markers'?{markers:binding.topic,markersOptions:binding,maxMarkers:Number(spec.options.maxMarkers)}:{}),
      ...(id==='path'?{path:binding.topic,pathOptions:binding}:{}),onStatus:context.onStatus});}
  });
  registry.register({id:'robot',title:'机器人模型',spatial:true,
    ports:[{id:'description',types:['std_msgs/msg/String']},{id:'joints',types:['sensor_msgs/msg/JointState']}],
    defaults:{inputs:{description:input('/robot_description','std_msgs/msg/String','latched'),joints:input('/joint_states','sensor_msgs/msg/JointState','sensor')}},
    validate(options){optionsOnly(options,[]);return{};},
    create(context,spec){return context.scene({layers:['robot'],robotDescription:spec.inputs.description.topic,robotDescriptionOptions:spec.inputs.description,jointStates:spec.inputs.joints.topic,jointStatesOptions:spec.inputs.joints,onStatus:context.onStatus});}});
  spatial('tf','TF 坐标系');spatial('markers','MarkerArray','visualization_msgs/msg/MarkerArray');
  spatial('cloud','PointCloud2','sensor_msgs/msg/PointCloud2');spatial('path','Path','nav_msgs/msg/Path');
  registry.register({id:'image',title:'图像 / 深度',spatial:false,ports:[{id:'data',types:['sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage']}],
    defaults:{inputs:{data:{...input('/camera/image_raw','sensor_msgs/msg/Image','sensor'),compression:'cbor',throttleMs:500}}},
    validate(options){optionsOnly(options,['depthScale','maxDepth','maxPixels']);const result:Record<string,unknown>={maxPixels:integer(options.maxPixels,4000000,4000000)};
      for(const key of ['depthScale','maxDepth'])if(options[key]!==undefined){if(typeof options[key]!=='number'||!Number.isFinite(options[key])||Number(options[key])<=0)throw new TypeError(`Invalid ${key}`);result[key]=options[key];}return result;},
    create(context,spec){return new ImageView(context.root,context.client,{...spec.options,topic:spec.inputs.data.topic,type:spec.inputs.data.type as ImageOptions['type'],subscription:spec.inputs.data,onStatus:context.onStatus});}});
  registry.register({id:'plot',title:'数值曲线',spatial:false,ports:[{id:'data',types:['*']}],
    defaults:{inputs:{data:input('/sensor','std_msgs/msg/Float64','sensor')},options:{fields:['data'],maxPoints:600}},
    validate(options){optionsOnly(options,['fields','maxPoints']);const fields=options.fields??['data'];
      if(!Array.isArray(fields)||fields.length<1||fields.length>8||fields.some(field=>typeof field!=='string'||field.length>256||!/^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z0-9_]+)*$/.test(field)||field.split('.').some(key=>['__proto__','constructor','prototype'].includes(key))))throw new TypeError('Choose 1..8 plain numeric field paths');
      return {fields,maxPoints:integer(options.maxPoints,600,600,2)};},
    create(context,spec){return new SeriesPlot(context.root,context.client,{topic:spec.inputs.data.topic,type:spec.inputs.data.type,fields:spec.options.fields as string[],maxPoints:Number(spec.options.maxPoints),subscription:spec.inputs.data,onStatus:context.onStatus});}});
  registry.register({id:'raw',title:'原始消息',spatial:false,ports:[{id:'data',types:['*']}],defaults:{inputs:{data:input('/topic','std_msgs/msg/String','sensor')}},
    validate(options){optionsOnly(options,[]);return{};},create(context,spec){
      const output=document.createElement('pre');context.root.replaceChildren(output);let off:(()=>void)|undefined,count=0;
      return {start(){if(!off)off=context.client.subscribe(spec.inputs.data.topic,spec.inputs.data.type,message=>{count++;output.textContent=previewMessage(message);context.onStatus(`已接收 ${count} 条 · 有界摘要`);},spec.inputs.data);},
        dispose(){off?.();off=undefined;context.root.replaceChildren();},stats:()=>({received:count})};
    }});
  return registry;
}
