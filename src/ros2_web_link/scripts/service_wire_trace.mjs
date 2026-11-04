/** Opt-in CLI observation of the existing native WebSocket event boundary.
 * No second socket, protocol rewrite, response listener in the SDK, or retry.
 * Only small JSON service envelopes are inspected; values/args are never kept.
 * The hook buffers bounded JSONL metadata. Explicit flush happens after RPCs
 * and sockets finish; filesystem IO must not run inside the measured deadline.
 * "receive" means the native message event ran, not NIC/kernel packet arrival.
 */
import {AsyncLocalStorage} from 'node:async_hooks';
import {diagnosticWriteError} from './load_progress.mjs';

/** Shared bounded CLI metadata storage; explicit flush is outside RPC waits. */
export function createMetadataBuffer({write,onError=()=>{},maxRecords=24000,maxBytes=8388608}){
  if(typeof write!=='function'||typeof onError!=='function')throw new TypeError('Expected diagnostic callbacks');
  if(!Number.isSafeInteger(maxRecords)||maxRecords<1||maxRecords>100000||!Number.isSafeInteger(maxBytes)||maxBytes<1||maxBytes>16777216)throw new RangeError('Invalid wire diagnostic budget');
  const encoder=new TextEncoder(),pending=[];let records=0,bytes=0,bufferedBytes=0,sinkErrors=0,budgetExceeded=false;
  const notice=message=>{try{onError(message);}catch{/* Observability cannot replace a socket outcome. */}};
  const keep=record=>{
    if(budgetExceeded||sinkErrors>=8)return;
    const line=JSON.stringify(record)+'\n',size=encoder.encode(line).byteLength;
    if(records>=maxRecords||bytes+size>maxBytes){budgetExceeded=true;notice('Native service trace budget exceeded');return;}
    records++;bytes+=size;bufferedBytes+=size;pending.push(line);
  };
  return {
    push:keep,
    flush(){
      if(!pending.length)return;
      const text=pending.join('');pending.length=0;bufferedBytes=0;
      if(sinkErrors>=8)return;
      // No retry: a failed write may already have persisted some bytes.
      const error=diagnosticWriteError(()=>write(text));
      if(error){sinkErrors++;notice(error.slice(0,1024));}
    },
    stats(){return {records,bytes,bufferedRecords:pending.length,bufferedBytes,sinkErrors,budgetExceeded};},
  };
}

export function createServiceWireTrace(Base,{endpoint,...options}){
  if(typeof Base!=='function')throw new TypeError('Expected native WebSocket');
  if(typeof endpoint!=='string'||endpoint.length>256||!endpoint.startsWith('/'))throw new TypeError('Expected one absolute service endpoint');
  const sessions=new AsyncLocalStorage(),buffer=createMetadataBuffer(options);
  const inspect=(phase,data,tag)=>{
    // The native API already decoded text. Do not stringify blobs/ArrayBuffers,
    // copy a cloud, scan a huge string, or parse ordinary high-rate Topic frames.
    if(typeof data!=='string'||data.length>16384)return;
    const prefix=data.slice(0,256);
    if(!prefix.includes('call_service')&&!prefix.includes('service_response'))return;
    let value;try{value=JSON.parse(data);}catch{return;}
    if(!value||value.service!==endpoint||!['call_service','service_response'].includes(value.op)||typeof value.id!=='string'||!value.id)return;
    buffer.push({phase,wall:Date.now()/1000,monotonic:performance.now(),slot:tag?.slot,generation:tag?.generation,
      op:value.op,service:endpoint,cid:value.id.slice(0,128),result:typeof value.result==='boolean'?value.result:undefined});
  };
  class TracedWebSocket extends Base {
    constructor(...args){
      super(...args);const tag=sessions.getStore();this.serviceTraceTag=tag;
      // Register before the SDK assigns onmessage, preserving the same native
      // implementation and standard listener order for the timing comparison.
      this.addEventListener('message',event=>inspect('receive',event.data,tag));
    }
    send(data){
      inspect('send',data,this.serviceTraceTag);
      const result=super.send(data);
      inspect('sendReturn',data,this.serviceTraceTag);
      return result;
    }
  }
  return {
    WebSocket:TracedWebSocket,
    withSession(tag,fn){
      if(!tag||!Number.isSafeInteger(tag.slot)||!Number.isSafeInteger(tag.generation))throw new TypeError('Expected numeric CLI session identity');
      // ROSLIB may construct the transport in a later microtask. Use Node's
      // standard context propagation, including concurrent connection attempts.
      return sessions.run({slot:tag.slot,generation:tag.generation},fn);
    },
    flush:buffer.flush,
    stats:buffer.stats,
  };
}
