/** Run actual CLI stack-request logic with a controlled delayed file boundary. */
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import {diagnosticWriteError,recordRpcResult} from './load_progress.mjs';
const source=readFileSync(new URL('./multi_client_probe.mjs',import.meta.url),'utf8');
const fail=source.match(/^const fail=.*$/m)[0],trace=source.match(/^const rpcTrace=.*$/m)[0];
const stack=source.match(/^const requestStack=.*$/m)?.[0]??'',drain=source.match(/^const drainStack=.*$/m)?.[0]??'';
const start=source.indexOf('async function query('),end=source.indexOf('\nlet status=',start);
let timer,syncWrites=0,asyncWrites=0,rejectWrite;const errors=[];
const ctx=vm.createContext({assert,path,Date,performance,Promise,errors,out:'/unused',rpcBuffer:undefined,
  process:{env:{WEB_LINK_TRACE_SERVICE:'/rosapi/nodes'}},diagnosticWriteError,recordRpcResult,
  setTimeout:callback=>{timer=callback;return 1;},clearTimeout:()=>{},
  writeFileSync:()=>{syncWrites++;},writeFile:()=>{asyncWrites++;return new Promise((_,reject)=>{rejectWrite=reject;});},
  tfCase:{target:'root',sources:['base']}});
vm.runInContext('let pendingStack;\n'+fail+'\n'+trace+'\n'+stack+'\n'+drain+'\n'+source.slice(start,end)+'\nthis.run=query;this.drain=typeof drainStack===\'function\'?drainStack:async()=>{};',ctx);
const session={slot:0,generation:1,startedAt:Date.now(),rpc:[],rpcCalls:0,rpcFailed:0,tfChecks:[],
  tf:{lookup:()=>({}),frames:()=>['base'],stats:()=>({})},client:{callService:async()=>({nodes:['fixture']})}};
await ctx.run(session);assert.equal(typeof timer,'function');timer();timer();
assert.equal(syncWrites,0,'Stack request must not synchronously write inside an RPC window');
await Promise.resolve();assert.equal(asyncWrites,1,'Concurrent stack flags must share one in-flight write');
let drained=false;const waiting=ctx.drain().then(()=>{drained=true;});await Promise.resolve();assert.equal(drained,false);
rejectWrite(Error('controlled delayed ENOSPC'));await waiting;
assert.ok(errors.some(x=>x.includes('ENOSPC')));assert.equal(session.rpcCalls,1);assert.equal(session.rpcFailed,0);
const cleanup=source.slice(source.indexOf('globalThis.WebSocket=NativeSocket;'));
assert.ok(cleanup.indexOf('await drainStack();')>=0&&cleanup.indexOf('await drainStack();')<cleanup.indexOf("if(errors.length){status='FAIL'"),'Final verdict must wait for stack IO');
console.log('PASS async stack flag coalescing, delayed error accounting, and final drain order');
