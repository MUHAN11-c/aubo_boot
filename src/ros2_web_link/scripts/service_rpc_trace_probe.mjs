// Exercise the actual CLI query/trace functions with only transport and IO replaced.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import {recordRpcResult,diagnosticWriteError} from './load_progress.mjs';
import * as traceTools from './service_wire_trace.mjs';
const source=readFileSync(process.env.WEB_LINK_MULTI_SOURCE??new URL('./multi_client_probe.mjs',import.meta.url),'utf8');
const trace=source.match(/^const rpcTrace=.*$/m)?.[0],fail=source.match(/^const fail=.*$/m)?.[0];
const start=source.indexOf('async function query('),end=source.indexOf("\nlet status=",start);
assert.ok(trace&&fail&&start>=0&&end>start,'locate actual trace and query implementation');
for(const phase of ['request','response','error']){
  if(process.env.WEB_LINK_RPC_PHASE&&process.env.WEB_LINK_RPC_PHASE!==phase)continue;
  const errors=[];let calls=0,writes=0;
  const session={slot:1,generation:2,startedAt:Date.now(),rpc:[],rpcCalls:0,rpcFailed:0,tfChecks:[],
    tf:{lookup:()=>({}),frames:()=>['base'],stats:()=>({})},client:{callService:async()=>{
      calls++;if(phase==='error')throw {code:'TIMEOUT'};return {nodes:['native-fixture']};
    }}};
  const writer=(_file,text)=>{
    writes++;
    if(text.trimEnd().split('\n').some(line=>JSON.parse(line).phase===phase))throw Error('controlled ENOSPC');
  };
  const rpcBuffer=traceTools.createMetadataBuffer?.({write:text=>writer('/unused',text),onError:message=>errors.push('RPC trace: '+message)});
  const context=vm.createContext({errors,assert,Date,performance,setTimeout,clearTimeout,path,out:'/unused',rpcBuffer,
    process:{env:{WEB_LINK_WAIT_SERVICE:'/rosapi/nodes'}},recordRpcResult,diagnosticWriteError,
    tfCase:{target:'root',sources:['base']},writeFileSync:writer});
  vm.runInContext(fail+'\n'+trace+'\n'+source.slice(start,end)+'\nthis.run=query;',context);
  await assert.doesNotReject(context.run(session),`${phase} trace IO must not replace the query result`);
  assert.equal(writes,0,'RPC diagnostic sink must not run inside a query deadline');
  rpcBuffer.flush();
  assert.equal(calls,1,'diagnostic IO must not suppress or retry the read-only query');
  assert.equal(session.rpcCalls,1);assert.equal(session.rpcFailed,phase==='error'?1:0);
  assert.ok(errors.some(value=>value.includes('RPC trace')&&value.includes('ENOSPC')),errors.join(';'));
  if(phase==='error')assert.ok(errors.some(value=>value.includes('query: TIMEOUT')),errors.join(';'));
  else assert.ok(errors.every(value=>!value.includes('query:')),errors.join(';'));
  console.log('PASS '+phase+' trace write failure; RPC result and counts preserved');
}
