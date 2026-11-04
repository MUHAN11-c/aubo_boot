/** Read-only ROS discovery, independent of DOM, graph layout and applications. */
import type {RosClient} from '../sdk/index.js';
import {errorText} from '../sdk/errors.js';

export interface GraphOptions {
  signal?:AbortSignal;timeoutMs?:number;concurrency?:number;
  maxNodes?:number;maxTopics?:number;maxEdges?:number;
}
export interface GraphNode {id:string;name:string;duplicates:number;details:'known'|'unknown'}
export interface GraphTopic {id:string;name:string;types:string[]}
export interface GraphEdge {id:string;source:string;target:string;kind:'publish'|'subscribe';topic:string}
export interface GraphService {name:string;providers:string[]}
export interface GraphSnapshot {
  status:'complete'|'partial'|'failed';receivedAt:number;elapsedMs:number;cancelled:boolean;
  nodes:GraphNode[];topics:GraphTopic[];edges:GraphEdge[];services:GraphService[];actions:{name:string}[];
  errors:{source:string;message:string}[];omittedErrors:number;
  scope:string;
  capabilities:{topicRelations:true;serviceProviders:true;serviceClients:false;actionClients:false;endpointQos:false;endpointTypes:false;hiddenEntities:false};
}
export interface GraphElement {data:{id:string;label:string;kind:'node'|'topic'|'edge';source?:string;target?:string;name?:string;duplicates?:number}}
const rosName=/^\/[A-Za-z_][A-Za-z0-9_]*(?:\/[A-Za-z_][A-Za-z0-9_]*)*$/;
const id=(kind:string,...parts:string[])=>JSON.stringify([kind,...parts]);
function integer(value:number,fallback:number,key:string,max:number):number {
  const result=value??fallback;
  if(!Number.isSafeInteger(result)||result<1||result>max)throw new TypeError(`${key} must be an integer from 1 to ${max}`);
  return result;
}
const validName=(value:unknown):value is string=>typeof value==='string'&&value.length<=1024&&rosName.test(value);

/** Sample declared pub/sub relations through standard rosapi services.
 * A snapshot is discovery evidence, not proof of delivery, type matching or QoS.
 * Each round has one deadline and bounded parallelism; it never publishes data.
 */
export async function discoverGraph(client:Pick<RosClient,'callService'|'onState'|'connected'>,options:GraphOptions={}):Promise<GraphSnapshot> {
  const timeoutMs=integer(options.timeoutMs!,10000,'timeoutMs',60000);
  const concurrency=integer(options.concurrency!,4,'concurrency',8);
  const maxNodes=integer(options.maxNodes!,256,'maxNodes',1024);
  const maxTopics=integer(options.maxTopics!,512,'maxTopics',4096);
  const maxEdges=integer(options.maxEdges!,2048,'maxEdges',16384);
  const at=performance.now(),deadline=at+timeoutMs,controller=new AbortController();
  const nodes=new Map<string,GraphNode>(),topics=new Map<string,GraphTopic>(),edges=new Map<string,GraphEdge>();
  const services=new Map<string,Set<string>>(),actions=new Set<string>();
  const errors:GraphSnapshot['errors']=[],reported=new Set<string>();let omittedErrors=0,providerEdges=0;
  const issue=(source:string,message:string)=>{
    const key=`${source}:${message}`;if(reported.has(key))return;
    if(errors.length<64){reported.add(key);errors.push({source:source.slice(0,1024),message:message.slice(0,512)});}
    else omittedErrors++;
  };
  const stop=(reason:string)=>{if(!controller.signal.aborted){issue('snapshot',reason);controller.abort();}};
  const externalStop=()=>stop('Refresh cancelled');
  const expired=()=>{
    if(performance.now()>=deadline)stop('Graph snapshot deadline exceeded');
    return controller.signal.aborted;
  };
  const names=(value:unknown,source:string,limit:number):string[]=>{
    if(!Array.isArray(value)){issue(source,'Expected a ROS name array');return [];}
    if(value.length>limit)issue(source,`Name count exceeds ${limit}; snapshot is incomplete`);
    const result:string[]=[];
    for(let i=0;i<Math.min(value.length,limit);i++){
      if(expired())break;
      if(validName(value[i]))result.push(value[i]);else issue(source,'Malformed or oversized ROS name');
    }
    return result;
  };
  const topic=(name:string)=>{
    let item=topics.get(name);
    if(!item){
      if(topics.size>=maxTopics){issue('topics',`Topic count exceeds ${maxTopics}`);return;}
      item={id:id('topic',name),name,types:[]};topics.set(name,item);
    }
    return item;
  };
  const service=(name:string)=>{
    let providers=services.get(name);
    if(!providers){if(services.size>=maxTopics){issue('services',`Service count exceeds ${maxTopics}`);return;}providers=new Set();services.set(name,providers);}
    return providers;
  };
  const edge=(node:GraphNode,name:string,kind:GraphEdge['kind'])=>{
    const target=topic(name);if(!target)return;
    const key=id('edge',node.name,name,kind);if(edges.has(key))return;
    if(edges.size>=maxEdges){issue('edges',`Edge count exceeds ${maxEdges}`);return;}
    edges.set(key,{id:key,source:kind==='publish'?node.id:target.id,target:kind==='publish'?target.id:node.id,kind,topic:name});
  };
  const call=async(endpoint:string,type:string,request:Record<string,unknown>={})=>{
    if(expired())throw new Error('Graph refresh has ended');
    const result=await client.callService<Record<string,unknown>>(endpoint,type,request,
      {signal:controller.signal,timeoutMs:Math.max(1,Math.min(3000,Math.ceil(deadline-performance.now())))});
    if(expired())throw new Error('Late graph response discarded');
    if(!result||typeof result!=='object'||Array.isArray(result))throw new TypeError('Malformed rosapi response');
    return result;
  };
  let timer:ReturnType<typeof setTimeout>|undefined,off:(()=>void)|undefined;
  try {
    if(options.signal?.aborted)stop('Refresh cancelled');
    else options.signal?.addEventListener('abort',externalStop,{once:true});
    if(!client.connected)stop('Connection is not ready');
    if(!controller.signal.aborted){
      off=client.onState(state=>{if(state!=='connected')stop('Connection generation ended');});
      timer=setTimeout(()=>stop('Graph snapshot deadline exceeded'),timeoutMs);
      // Unlike RosClient.graph(), these calls accept the same absolute round
      // deadline/signal, and retain successful lists if another query fails.
      const requests=[['topics','Topics'],['services','Services'],['nodes','Nodes'],['action_servers','GetActionServers']];
      const results=await Promise.allSettled(requests.map(([endpoint,type])=>call(`/rosapi/${endpoint}`,`rosapi_msgs/srv/${type}`)));
      const data:Record<string,Record<string,unknown>>={};
      for(let i=0;i<results.length;i++){
        const result=results[i],key=requests[i][0];
        if(result.status==='fulfilled')data[key]=result.value;
        else issue(key,errorText(result.reason));
      }
      if(!expired()){
        // Keep the parallel topic/type index before discarding malformed names.
        const listed=data.topics?.topics,types=data.topics?.types;
        if(Array.isArray(listed)){
          if(listed.length>maxTopics)issue('topics',`Topic count exceeds ${maxTopics}`);
          for(let i=0;i<Math.min(listed.length,maxTopics);i++){
            if(!validName(listed[i])){issue('topics','Malformed or oversized ROS name');continue;}
            const item=topic(listed[i]);
            const type=Array.isArray(types)?types[i]:undefined;
            if(item&&typeof type==='string'&&type.length<=512)item.types=[type];
            else issue('topics','Topic type summary is unavailable');
          }
        }else if(data.topics)issue('topics','Expected a ROS name array');
        for(const name of names(data.nodes?.nodes,'nodes',maxNodes)){
          const old=nodes.get(name);
          if(old){old.duplicates++;issue(name,'Duplicate node name; endpoint ownership is ambiguous');}
          else nodes.set(name,{id:id('node',name),name,duplicates:1,details:'unknown'});
        }
        for(const name of names(data.services?.services,'services',maxTopics))service(name);
        for(const name of names(data.action_servers?.action_servers,'actions',maxTopics))actions.add(name);
        const queue=[...nodes.values()];let next=0;
        const worker=async()=>{
          while(!expired()&&next<queue.length){
            const node=queue[next++];
            try{
              const detail=await call('/rosapi/node_details','rosapi_msgs/srv/NodeDetails',{node:node.name});
              const publishing=names(detail.publishing,node.name,maxTopics),subscribing=names(detail.subscribing,node.name,maxTopics);
              for(const name of publishing)edge(node,name,'publish');
              for(const name of subscribing)edge(node,name,'subscribe');
              for(const name of names(detail.services,node.name,maxTopics)){
                const providers=service(name);
                if(providers&&!providers.has(node.name)){
                  if(providerEdges>=maxEdges){issue('services',`Provider association count exceeds ${maxEdges}`);break;}
                  providers.add(node.name);providerEdges++;
                }
              }
              node.details='known';
            }catch(error){issue(node.name,errorText(error));}
          }
        };
        await Promise.all(Array.from({length:Math.min(concurrency,queue.length)},worker));
      }
    }
  }catch(error){issue('snapshot',errorText(error));}
  finally{
    clearTimeout(timer);off?.();options.signal?.removeEventListener('abort',externalStop);
  }
  const cancelled=controller.signal.aborted;
  const hasData=nodes.size+topics.size+services.size+actions.size>0;
  return {status:errors.length||omittedErrors||cancelled?(hasData?'partial':'failed'):'complete',
    receivedAt:Date.now(),elapsedMs:performance.now()-at,cancelled,
    nodes:[...nodes.values()].sort((a,b)=>a.name.localeCompare(b.name)),
    topics:[...topics.values()].sort((a,b)=>a.name.localeCompare(b.name)),
    edges:[...edges.values()].sort((a,b)=>a.id.localeCompare(b.id)),
    services:[...services].map(([name,providers])=>({name,providers:[...providers].sort()})).sort((a,b)=>a.name.localeCompare(b.name)),
    actions:[...actions].sort().map(name=>({name})),errors,omittedErrors,
    scope:'rosapi-visible entities in the connected bridge domain; declared relations, not verified traffic',
    capabilities:{topicRelations:true,serviceProviders:true,serviceClients:false,actionClients:false,endpointQos:false,endpointTypes:false,hiddenEntities:false}};
}

/** Build bounded layout input; graph libraries own placement and rendering.
 * Node-only mode is the projection of declared publisher/topic/subscriber
 * associations. Its Cartesian product is capped independently of snapshot edges.
 */
export function graphElements(snapshot:GraphSnapshot,options:{mode?:'topics'|'nodes';maxEdges?:number}={}):{elements:GraphElement[];truncated:boolean} {
  const maxEdges=integer(options.maxEdges!,2048,'maxEdges',16384),mode=options.mode??'topics';
  if(!['topics','nodes'].includes(mode))throw new TypeError('Unknown graph mode');
  const elements:GraphElement[]=snapshot.nodes.map(n=>({data:{id:n.id,label:n.name,name:n.name,kind:'node',duplicates:n.duplicates}}));
  let truncated=false;
  if(mode==='topics'){
    elements.push(...snapshot.topics.map(t=>({data:{id:t.id,label:t.name,name:t.name,kind:'topic' as const}})));
    for(const e of snapshot.edges){
      if(elements.length-snapshot.nodes.length-snapshot.topics.length>=maxEdges){truncated=true;break;}
      elements.push({data:{id:e.id,source:e.source,target:e.target,label:'',kind:'edge'}});
    }
  }else{
    const pubs=new Map<string,GraphEdge[]>(),subs=new Map<string,GraphEdge[]>();
    for(const e of snapshot.edges){const map=e.kind==='publish'?pubs:subs;const list=map.get(e.topic)??[];list.push(e);map.set(e.topic,list);}
    let count=0;
    outer:for(const [topic,publishers]of pubs)for(const p of publishers)for(const s of subs.get(topic)??[]){
      if(count>=maxEdges){truncated=true;break outer;}
      elements.push({data:{id:id('projected',p.source,s.target,topic),source:p.source,target:s.target,label:topic,kind:'edge'}});count++;
    }
  }
  return {elements,truncated};
}
