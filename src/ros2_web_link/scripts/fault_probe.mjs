// Starts/kills only its OWN temporary local bridge. Keep self_test demo running.
// Requires source /opt/ros/jazzy/setup.bash and source workspace install/setup.bash.
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import {RosClient} from '../dist/sdk/index.js';

const port=Number(process.env.WEB_LINK_FAULT_PORT || 19090);
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const waitFor=async fn=>{
  const end=Date.now()+12000;
  while(!fn() && Date.now()<end) await pause(30);
  assert(fn(),'fault probe condition timed out');
};
let child;let output='';let stopping;let shuttingDown=false;let spawnError;
function start() {
  if(shuttingDown) throw Error('Probe interrupted');
  child=spawn('ros2',['run','ros2_web_link','bridge_node.py','--ros-args','-p',`port:=${port}`,'-p','address:=127.0.0.1'],
    {detached:true,stdio:['ignore','pipe','pipe']});
  child.on('error',error=>{spawnError=error;});
  for(const stream of [child.stdout,child.stderr]) stream.on('data',data=>{output=(output+data).slice(-20000);});
}
function stop() {
  if(stopping) return stopping;
  const target=child;child=undefined;
  if(!target?.pid || target.exitCode!==null || target.signalCode!==null) return Promise.resolve();
  stopping=(async()=>{
    const exiting=once(target,'exit');
    try {process.kill(-target.pid,'SIGINT');} catch(error) {if(error.code!=='ESRCH') throw error;}
    const timer=setTimeout(()=>{try{process.kill(-target.pid,'SIGKILL');}catch{}},5000);
    try {await exiting;} finally {clearTimeout(timer);}
  })().finally(()=>{stopping=undefined;});
  return stopping;
}
const client=new RosClient({url:`ws://127.0.0.1:${port}`,retryMs:150});
const states=[];client.onState(state=>states.push(state));
// A detached bridge is not in the terminal's foreground process group. Node
// would otherwise exit on Ctrl+C without running finally, orphaning the bridge.
for(const [signal,code] of [['SIGINT',130],['SIGTERM',143]]) process.on(signal,()=>{
  if(shuttingDown) return;shuttingDown=true;client.dispose();
  void stop().finally(()=>process.exit(code));
});
let count=0;
try {
  start();void client.connect().catch(()=>{});await waitFor(()=>{if(spawnError) throw spawnError;return client.connected;});
  const off=client.subscribe('/web_link_demo/sensor','std_msgs/msg/Float64',()=>{count++;},{qos:'sensor'});
  await waitFor(()=>count>1);
  const pending=client.callService('/web_link_demo/slow','std_srvs/srv/Trigger',{}, {timeoutMs:6000})
    .then(()=>({code:'UNEXPECTED_SUCCESS'}),error=>error);
  const task=client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:20}).result
    .then(()=>({code:'UNEXPECTED_SUCCESS'}),error=>error);
  await pause(100);await stop();
  await waitFor(()=>states.includes('reconnecting'));
  assert.equal((await pending).code,'DISCONNECTED');assert.equal((await task).code,'DISCONNECTED');
  const pub=client.publisher('/web_link_demo/input','std_msgs/msg/String');
  assert.throws(()=>pub.publish({data:'must-not-replay'}),{code:'DISCONNECTED'});
  const before=count;start();await waitFor(()=>client.connected&&count>before+1);
  assert.equal(client.stats().pending,0);pub.dispose();off();
  console.log('PASS bridge process loss settles service/action; subscriptions recover');
  console.log('PASS offline publishing rejected; commands are never queued');
  client.dispose();
  for(let i=0;i<15;i++) {
    const rapid=new RosClient({url:`ws://127.0.0.1:${port}`,reconnect:false});
    const connected=rapid.connect().catch(error=>error);rapid.disconnect();await connected;rapid.dispose();
  }
  console.log('PASS rapid connect/disconnect/dispose releases SDK sessions');
} catch(error) {console.error(output);throw error;}
finally {client.dispose();await stop();}
