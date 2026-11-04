/** Shared bridge admission against the hardware-free demo; no robot commands. */
import assert from 'node:assert/strict';
import {RosClient} from '../dist/sdk/index.js';
const clients=[0,1].map(()=>new RosClient({url:process.argv[2],reconnect:false}));
try{
  await Promise.all(clients.map(c=>c.connect()));
  const result=await Promise.allSettled(clients.flatMap(client=>[0,1].map(()=>
    client.callService('/web_link_demo/slow','std_srvs/srv/Trigger',{}, {timeoutMs:5000}))));
  const completed=result.filter(r=>r.status==='fulfilled');
  const rejected=result.filter(r=>r.status==='rejected');
  console.log(JSON.stringify({completed:completed.length,rejected:rejected.map(r=>String(r.reason))}));
  assert.equal(completed.length,2,'Two simultaneous clients must share bridge limit=2');
  assert.equal(rejected.length,2,'Excess calls must fail without being queued');
  assert(rejected.every(r=>/budget|limit/i.test(String(r.reason))));
  assert.deepEqual(await clients[0].callService('/web_link_demo/add_two_ints','example_interfaces/srv/AddTwoInts',{a:20,b:22}),{sum:42});
  assert(clients.every(c=>c.stats().pending===0));
  console.log('PASS shared cache-independent Service admission, explicit rejection and recovery');
}finally{clients.forEach(c=>c.dispose());}
