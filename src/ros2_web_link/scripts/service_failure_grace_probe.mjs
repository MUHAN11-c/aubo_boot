// Execute the actual load probe's failure handler, without ROS or sockets.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
const source=readFileSync(process.env.WEB_LINK_MULTI_SOURCE??new URL('./multi_client_probe.mjs',import.meta.url),'utf8');
const handler=source.match(/\}catch\(e\)\{failure=e\.message;[^\n]*throw e;\}/)?.[0];
assert.ok(handler,'locate the actual load failure handler');
for(const [name,env,expected] of [
  ['default',{},[]],
  ['metadata',{WEB_LINK_TRACE_SERVICE:'/rosapi/nodes'},[5000]],
  ['wait',{WEB_LINK_WAIT_SERVICE:'/rosapi/nodes'},[5000]],
]){
  const waits=[];
  const context=vm.createContext({process:{env},sleep:async ms=>{waits.push(ms);}});
  vm.runInContext('let failure; this.fail=async e=>{try{throw e;'+handler+'};this.reason=()=>failure;',context);
  const original=Error('original query TIMEOUT');
  await assert.rejects(context.fail(original),error=>error===original);
  assert.equal(context.reason(),original.message);
  assert.deepEqual(waits,expected,`${name} must preserve the expected post-failure evidence window`);
  console.log('PASS '+name+' failure window; original failure preserved');
}
