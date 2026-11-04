/** Bounded native WebSocket adapter for roslib's public Transport API.
 * Uses the same cbor2 decoder as upstream roslib 2.1.0. The only protocol gap
 * implemented here is bounded JSON fragment assembly: upstream's private map
 * has no expiry/byte budget and cannot be cleaned through its public API.
 */
import {AbstractTransport,isRosbridgeMessage} from 'roslib';
import type {RosbridgeMessage} from 'roslib';
import {decode} from 'cbor2';
import {LinkError,errorText} from './errors.js';
import {DEFAULT_TRANSPORT_LIMITS} from './limits.js';
import type {TransportLimits} from './limits.js';
export function transportLimits(input:Partial<TransportLimits>={}):Readonly<TransportLimits>{
  if(!input||typeof input!=='object'||Array.isArray(input))throw new TypeError('transportLimits must be an object');
  const result={...DEFAULT_TRANSPORT_LIMITS,...input};
  const caps:TransportLimits={maxMessageBytes:268435456,maxFragmentBytes:536870912,maxFragmentMessages:1024,maxFragmentsPerMessage:65536,fragmentTimeoutMs:300000,maxSendBufferedBytes:268435456,maxCborDepth:512};
  for(const key of Object.keys(result) as (keyof TransportLimits)[]){
    if(!Object.hasOwn(caps,key)||!Number.isSafeInteger(result[key])||result[key]<1||result[key]>caps[key])throw new TypeError(`Invalid transport limit: ${key}`);
  }
  return Object.freeze(result);
}
export class TransportError extends LinkError {}
type Parts={chunks:Map<number,string>;total:number;bytes:number;deadline:number};
/** Count encoded bytes; common Base64/JSON ASCII frames need no extra buffer. */
function textBytes(value:string):number {
  return /[^\x00-\x7f]/.test(value)?new TextEncoder().encode(value).byteLength:value.length;
}

export class BoundedTransport extends AbstractTransport {
  private socket:WebSocket;private closing=false;private fragments=new Map<string,Parts>();
  private fragmentBytes=0;private timer?:ReturnType<typeof setTimeout>;
  private receivedFrames=0;private receivedBytes=0;private sentFrames=0;private sentBytes=0;
  private lastError?:{code:string;message:string};
  private ignoredResponses=0;
  constructor(url:string,private limits:Readonly<TransportLimits>,private acceptResponse:(id:string)=>boolean){
    super();this.socket=new WebSocket(url);this.socket.binaryType='arraybuffer';
    this.socket.onopen=event=>{if(!this.closing)this.emit('open',event);};
    this.socket.onerror=event=>{if(!this.closing)this.emit('error',event);};
    this.socket.onclose=event=>{
      this.closing=true;this.clear();this.emit('close',event);
      this.socket.onopen=this.socket.onmessage=this.socket.onerror=this.socket.onclose=null;
      this.removeAllListeners();
    };
    this.socket.onmessage=event=>{
      if(this.closing)return;
      try{
        const raw:unknown=event.data;
        const bytes=typeof raw==='string'?textBytes(raw):raw instanceof ArrayBuffer?raw.byteLength:-1;
        this.receivedFrames++;this.receivedBytes+=Math.max(0,bytes);
        if(bytes<0)throw new TransportError('UNSUPPORTED_ENCODING','Expected JSON text or binary CBOR');
        if(bytes>this.limits.maxMessageBytes)throw new TransportError('MESSAGE_TOO_LARGE','Incoming frame exceeds maxMessageBytes');
        const message=typeof raw==='string'?JSON.parse(raw):decode(new Uint8Array(raw as ArrayBuffer),{maxDepth:this.limits.maxCborDepth});
        this.accept(message);
      }catch(error){this.fail(error instanceof TransportError?error:new TransportError('INVALID_FRAME',`Invalid rosbridge frame: ${errorText(error)}`));}
    };
  }
  private accept(message:unknown,assembled=false):void {
    // Timers can be delayed by a busy UI thread. Check absolute deadlines on
    // receipt as well; a late tail/new ID must never rescue an expired cache.
    if([...this.fragments.values()].some(parts=>parts.deadline<=performance.now()))throw new TransportError('FRAGMENT_TIMEOUT','Incomplete rosbridge message expired; remote operation outcome is unknown');
    if(!isRosbridgeMessage(message)||Array.isArray(message))throw new TransportError('INVALID_FRAME','Expected a rosbridge object with op');
    if(message.op==='png')throw new TransportError('UNSUPPORTED_ENCODING','Protocol PNG compression is unsupported; use JSON/CBOR (CompressedImage remains supported)');
    const route=message as unknown as Record<string,unknown>;
    const field=message.op==='publish'?'topic':message.op==='call_service'?'service':message.op==='send_action_goal'?'action':undefined;
    if(field&&(typeof route[field]!=='string'||!(route[field] as string).startsWith('/')))throw new TransportError('INVALID_FRAME',`Expected an absolute ${field} route`);
    if(['service_response','action_feedback','action_result','cancel_action_goal'].includes(message.op)){
      if(typeof route.id!=='string'||!route.id)throw new TransportError('INVALID_FRAME','Response ID must be a nonempty string');
      // Late responses are normal after timeout/cancel. They must not be emitted
      // as arbitrary ROSLIB event names (e.g. "close" or an existing topic).
      if(!this.acceptResponse(route.id)){this.ignoredResponses++;return;}
    }
    // Emit the already-decoded envelope through the public transport contract.
    // Calling upstream handleRawMessage would swallow dispatch exceptions as a
    // generic network Error, turning deterministic malformed routes into retries.
    if(message.op!=='fragment'){this.emit('message',message);return;}
    if(assembled)throw new TransportError('INVALID_FRAGMENT','Nested fragment envelopes are unsupported');
    const {id,num,total,data}=message as unknown as {id:unknown;num:unknown;total:unknown;data:unknown};
    if(typeof id!=='string'||!id||id.length>1024||typeof data!=='string'||!Number.isSafeInteger(num)||!Number.isSafeInteger(total)||
      (num as number)<0||(total as number)<1||(num as number)>=(total as number)||(total as number)>this.limits.maxFragmentsPerMessage){
      throw new TransportError('INVALID_FRAGMENT','Invalid fragment ID, index, total or content');
    }
    const index=num as number,count=total as number;
    let parts=this.fragments.get(id);
    if(!parts){
      if(this.fragments.size>=this.limits.maxFragmentMessages)throw new TransportError('FRAGMENT_LIMIT','Too many incomplete fragment messages');
      parts={chunks:new Map(),total:count,bytes:0,deadline:performance.now()+this.limits.fragmentTimeoutMs};this.fragments.set(id,parts);
    }
    if(parts.total!==count)throw new TransportError('INVALID_FRAGMENT','Fragment total changed within one message');
    if(parts.chunks.has(index)){
      if(parts.chunks.get(index)!==data)throw new TransportError('INVALID_FRAGMENT','Conflicting duplicate fragment');
      return;
    }
    const bytes=textBytes(data);
    if(parts.bytes+bytes>this.limits.maxMessageBytes)throw new TransportError('MESSAGE_TOO_LARGE','Assembled message exceeds maxMessageBytes');
    if(this.fragmentBytes+bytes>this.limits.maxFragmentBytes)throw new TransportError('FRAGMENT_LIMIT','Aggregate fragment bytes exceed maxFragmentBytes');
    parts.chunks.set(index,data);parts.bytes+=bytes;this.fragmentBytes+=bytes;
    if(parts.chunks.size===count){
      const text=Array.from({length:count},(_,i)=>parts!.chunks.get(i)!).join('');
      this.fragments.delete(id);this.fragmentBytes-=parts.bytes;this.armExpiry();
      this.accept(JSON.parse(text),true);
    }else this.armExpiry();
  }
  private armExpiry():void {
    clearTimeout(this.timer);this.timer=undefined;
    if(!this.fragments.size)return;
    const deadline=Math.min(...[...this.fragments.values()].map(p=>p.deadline));
    this.timer=setTimeout(()=>this.fail(new TransportError('FRAGMENT_TIMEOUT','Incomplete rosbridge message expired; remote operation outcome is unknown')),Math.max(1,Math.ceil(deadline-performance.now())));
  }
  private clear():void {clearTimeout(this.timer);this.timer=undefined;this.fragments.clear();this.fragmentBytes=0;}
  private fail(error:TransportError):void {
    if(this.closing)return;this.lastError={code:error.code,message:error.message};
    this.closing=true;this.clear();this.socket.close(4000,'ROS Web transport fault');this.emit('error',error);
  }
  send(message:RosbridgeMessage):void {
    if(!this.isOpen())throw new TransportError('DISCONNECTED','Socket is not open; nothing was queued');
    const text=JSON.stringify(message),bytes=textBytes(text);
    if(bytes>this.limits.maxMessageBytes)throw new TransportError('MESSAGE_TOO_LARGE','Outgoing message exceeds maxMessageBytes; nothing was sent');
    if(this.socket.bufferedAmount+bytes>this.limits.maxSendBufferedBytes)throw new TransportError('SEND_BUFFER_FULL','Socket send budget exceeded; nothing was queued or retried');
    this.socket.send(text);this.sentFrames++;this.sentBytes+=bytes;
  }
  close():void {this.closing=true;this.clear();if(this.socket.readyState<2)this.socket.close();}
  isConnecting():boolean{return !this.closing&&this.socket.readyState===0;}
  isOpen():boolean{return !this.closing&&this.socket.readyState===1;}
  isClosing():boolean{return this.closing&&this.socket.readyState!==3;}
  isClosed():boolean{return this.socket.readyState===3;}
  stats(){return {receivedFrames:this.receivedFrames,receivedBytes:this.receivedBytes,sentFrames:this.sentFrames,sentBytes:this.sentBytes,
    ignoredResponses:this.ignoredResponses,
    fragmentMessages:this.fragments.size,fragmentBytes:this.fragmentBytes,sendBufferedBytes:this.socket.bufferedAmount,
    limits:this.limits,lastError:this.lastError};}
}
