/** Real rosapi -> native rosbridge fragmented RPC -> SDK correlation check.
 * No topic writes, robot command, browser or server is started by this CLI.
 */
import assert from 'node:assert/strict';
import {RosClient} from '../dist/sdk/index.js';
const NativeSocket=globalThis.WebSocket;let fragments=0;
globalThis.WebSocket=class extends NativeSocket {
  constructor(...args){super(...args);this.addEventListener('message',event=>{
    if(typeof event.data==='string'&&/"op"\s*:\s*"fragment"/.test(event.data.slice(0,160)))fragments++;
  });}
  send(text){const message=JSON.parse(text);if(message.op==='call_service')message.fragment_size=512;super.send(JSON.stringify(message));}
};
const client=new RosClient({url:process.argv[2]??'ws://127.0.0.1:9290',reconnect:false});
try{
  await client.connect();
  for(const type of ['sensor_msgs/msg/PointCloud2','sensor_msgs/msg/Image','geometry_msgs/msg/PoseStamped']){
    const definitions=await client.describeInterface(type);
    assert.ok(definitions.message.typedefs.some(d=>d.type===type||d.type===type.replace('/msg/','/')));
  }
  assert.ok(fragments>3,'native rosbridge did not fragment RPC responses');
  assert.equal(client.stats().pending,0);assert.equal(client.transportStats().ignoredResponses,0);assert.equal(client.transportStats().fragmentMessages,0);
  console.log(JSON.stringify({status:'PASS',fragments,transport:client.transportStats()}));
}finally{client.dispose();globalThis.WebSocket=NativeSocket;}
console.log('PASS native fragmented RPC inner IDs preserved; no pending/fragment resources; client closed');
