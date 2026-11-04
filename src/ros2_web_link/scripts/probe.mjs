// Hardware-free integration probe. Run explicitly against self_test.launch.py.
// This is a CLI smoke check, not an ament/custom test framework.
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';

assert(existsSync(new URL('../dist/sdk/index.js', import.meta.url)),
  'SDK build is missing: npm ci && npm run build');
const {RosClient, encodeParameter, decodeParameter} = await import('../dist/sdk/index.js');
const url = process.argv[2] || 'ws://127.0.0.1:9090';
const client = new RosClient({url, reconnect: false});
let count = 0;
const check = async (name, fn) => {
  await fn(); count++; console.log(`PASS ${name}`);
};
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
const waitFor = async (predicate, ms = 8000) => {
  const end = Date.now() + ms;
  while (!predicate() && Date.now() < end) await pause(30);
  assert(predicate(), 'timed out waiting for observable result');
};
try {
  await check('parameter encodings: all nine ROS 2 types', async () => {
    for (const [type, value] of [[1,true],[2,3],[3,0.5],[4,'hello'],
      [5,[0,255]],[6,[true,false]],[7,[1,-2]],[8,[1.5,2]],[9,['a','b']]]) {
      assert.deepEqual(decodeParameter(encodeParameter(type, value)).value, value);
    }
    assert.throws(() => encodeParameter(2, Number.MAX_SAFE_INTEGER + 1));
    assert.throws(() => encodeParameter(1, 'true'));
    assert.throws(() => encodeParameter(3, NaN));
    assert.equal(decodeParameter(encodeParameter(3,1e20)).value,1e20);
    assert.deepEqual(decodeParameter(encodeParameter(8,[1e20])).value,[1e20]);
  });
  await check('offline commands rejected without replay', async () => {
    assert.throws(() => client.publisher('/web_link_demo/input','std_msgs/msg/String').publish({data:'offline'}));
    await assert.rejects(client.callService('/web_link_demo/add_two_ints','example_interfaces/srv/AddTwoInts',{a:1,b:2}), {code:'DISCONNECTED'});
  });
  await client.connect();
  await check('rosapi discovery', async () => {
    const graph = await client.graph();
    assert(graph.topics.topics.includes('/web_link_demo/ticks'));
    assert(graph.services.services.includes('/web_link_demo/add_two_ints'));
  });
  await check('native endpoint types and nested definitions', async () => {
    for(const [kind,endpoint,type] of [['topic','ticks','std_msgs/msg/String'],['service','add_two_ints','example_interfaces/srv/AddTwoInts'],['action','fibonacci','example_interfaces/action/Fibonacci']])
      assert.equal(await client.interfaceType(kind,`/web_link_demo/${endpoint}`),type);
    for(const type of ['std_msgs/msg/String','example_interfaces/srv/AddTwoInts','example_interfaces/action/Fibonacci']) {
      const details=await client.describeInterface(type);for(const detail of Object.values(details))assert(detail.typedefs.length>0);
    }
  });
  await check('publisher disposal leaves no ROSLIB close listeners', async () => {
    const before=client.stats().listeners;
    for(let i=0;i<30;i++) client.publisher('/web_link_demo/input','std_msgs/msg/String').dispose();
    assert.equal(client.stats().listeners,before);
  });
  await check('equivalent subscription defaults share one ROS subscription', async () => {
    const baseline=client.stats().subscriptions;
    const off1=client.subscribe('/web_link_demo/echo','std_msgs/msg/String',()=>{}, {qos:'reliable'});
    const off2=client.subscribe('/web_link_demo/echo','std_msgs/msg/String',()=>{});
    const off3=client.subscribe('/web_link_demo/echo','std_msgs/msg/String',()=>{},
      {compression:'none',throttleMs:0,qos:{durability:'volatile',reliability:'reliable',depth:10,history:'keep_last'}});
    assert.equal(client.stats().subscriptions,baseline+1);off1();off2();off3();
    assert.equal(client.stats().subscriptions,baseline);
  });
  await check('topic publish / subscribe / unsubscribe', async () => {
    let received;
    const stop = client.subscribe('/web_link_demo/echo','std_msgs/msg/String', m => {received=m.data;});
    const pub = client.publisher('/web_link_demo/input','std_msgs/msg/String');
    await pause(400);
    pub.publish({data:'web-link-probe'});
    await waitFor(() => received === 'web-link-probe');
    stop(); pub.dispose();
  });
  await check('best effort sensor and transient local late subscription', async () => {
    let sensor = 0; let retained;
    const off1 = client.subscribe('/web_link_demo/sensor','std_msgs/msg/Float64', () => {sensor++;}, {qos:'sensor'});
    const off2 = client.subscribe('/web_link_demo/retained','std_msgs/msg/String', m => {retained=m.data;}, {qos:'latched'});
    await waitFor(() => sensor>1 && retained === 'retained-once');
    off1(); off2();
  });
  await check('service success / failure / timeout', async () => {
    assert.equal((await client.callService('/web_link_demo/add_two_ints','example_interfaces/srv/AddTwoInts',{a:3,b:4})).sum,7);
    await assert.rejects(client.callService('/web_link_demo/missing','example_interfaces/srv/AddTwoInts',{}, {timeoutMs:350}));
    await assert.rejects(client.callService('/web_link_demo/slow','std_srvs/srv/Trigger',{}, {timeoutMs:100}), {code:'TIMEOUT'});
  });
  await check('standard parameters: list / describe / get / write / rejected / atomic', async () => {
    const p = client.parameters('/web_link_demo/demo');
    assert((await p.list()).names.includes('gain'));
    assert.equal((await p.describe(['gain'])).descriptors[0].name,'gain');
    assert((await p.set([{name:'gain',type:3,value:0.75}])).results[0].successful);
    assert.equal((await p.get(['gain'])).values[0].value,0.75);
    assert.equal((await p.set([{name:'gain',type:3,value:-1}])).results[0].successful,false);
    assert.equal((await p.setAtomically([{name:'gain',type:3,value:0.5}])).result.successful,true);
    const nine=['enabled','count','gain','label','bytes','bools','ints','doubles','strings'];
    assert.deepEqual((await p.get(nine)).values.map(v=>v.type),[1,2,3,4,5,6,7,8,9]);
    assert.deepEqual((await p.get(['bytes'])).values[0].value,[0,255]);
    assert((await p.set([{name:'bytes',type:5,value:[1,2,255]}])).results[0].successful);
    assert.deepEqual((await p.get(['bytes'])).values[0].value,[1,2,255]);
    await p.set([{name:'bytes',type:5,value:[0,255]}]);
    assert((await p.set([{name:'doubles',type:8,value:[1e20]}])).results[0].successful);
    assert.deepEqual((await p.get(['doubles'])).values[0].value,[1e20]);
    await p.set([{name:'doubles',type:8,value:[0.1,0.2]}]);
    await p.set([{name:'gain',type:3,value:1.0}]);
    assert.equal((await p.set([{name:'read_only',type:4,value:'changed'}])).results[0].successful,false);
    assert.equal((await p.setAtomically([{name:'gain',type:3,value:0.25},
      {name:'read_only',type:4,value:'changed'}])).result.successful,false);
    assert.equal((await p.get(['gain'])).values[0].value,1);
  });
  await check('CBOR sensor transport and standard parameter events', async () => {
    let sample=false, event=false;
    const off1=client.subscribe('/web_link_demo/sensor','std_msgs/msg/Float64',()=>{sample=true;},{qos:'sensor',compression:'cbor'});
    const off2=client.watchParameterEvents('/web_link_demo/demo',m=>{if(m.changed_parameters.some(p=>p.name==='gain'))event=true;});
    try {await pause(250);await client.parameters('/web_link_demo/demo').set([{name:'gain',type:3,value:0.5}]);
      await waitFor(()=>sample&&event);
    } finally {off1();off2();await client.parameters('/web_link_demo/demo').set([{name:'gain',type:3,value:1}]);}
  });
  await check('bridge rejects direct non-allowlisted calls', async () => {
    // Upstream unconditionally appends /rosapi/* to services_glob. The rosapi
    // node's params_glob=[] must therefore reject its legacy parameter writes.
    const legacy=await client.callService('/rosapi/set_param','rosapi_msgs/srv/SetParam',
      {name:'/web_link_demo/demo:gain',value:'9'}, {timeoutMs:1000});
    assert.equal(legacy.successful,false);
    assert.equal((await client.parameters('/web_link_demo/demo').get(['gain'])).values[0].value,1);
    await assert.rejects(client.callService('/rosbridge_websocket/get_parameters','rcl_interfaces/srv/GetParameters',
      {names:['port']},{timeoutMs:250}));
  });
  await check('action feedback / succeeded', async () => {
    let feedback = 0;
    const goal = client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:5},{onFeedback:()=>{feedback++;}, timeoutMs:10000});
    const r=await goal.result;
    assert.equal(r.status,4); assert(feedback>0); assert(r.value.sequence.length>3);
  });
  await check('action rejected / canceled / aborted', async () => {
    assert.equal((await client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:-1}).result).status,0);
    const goal=client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:20});
    await pause(550); goal.cancel(); assert.equal((await goal.result).status,5);
    assert.equal((await client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:13}).result).status,6);
  });
  await check('lifecycle standard service transitions', async () => {
    const lifecycle=client.lifecycle('/web_link_demo/lifecycle');
    assert((await lifecycle.availableTransitions()).available_transitions.length>0);
    await assert.rejects(lifecycle.change(0),TypeError);
    const root='/web_link_demo/lifecycle';
    const current=await client.callService(`${root}/get_state`,'lifecycle_msgs/srv/GetState',{});
    if(current.current_state.id===1) assert((await client.callService(`${root}/change_state`,'lifecycle_msgs/srv/ChangeState',{transition:{id:1}})).success);
    assert.equal((await client.callService(`${root}/get_state`,'lifecycle_msgs/srv/GetState',{})).current_state.id,2);
    assert((await client.callService(`${root}/change_state`,'lifecycle_msgs/srv/ChangeState',{transition:{id:3}})).success);
    assert((await client.callService(`${root}/change_state`,'lifecycle_msgs/srv/ChangeState',{transition:{id:4}})).success);
    assert((await client.callService(`${root}/change_state`,'lifecycle_msgs/srv/ChangeState',{transition:{id:2}})).success);
  });
  await check('bounded native Service/Action waits reject overflow without sending', async () => {
    const limited=new RosClient({url,reconnect:false,maxPendingRequests:2});
    try{
      await limited.connect();
      const waiting=limited.callService('/web_link_demo/slow','std_srvs/srv/Trigger',{}, {timeoutMs:10000});
      void waiting.catch(()=>{}); // finally may dispose the client after a failed assertion.
      let feedback=0;
      const goal=limited.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:20},
        {timeoutMs:10000,onFeedback:()=>feedback++});
      void goal.result.catch(()=>{});
      assert.equal(limited.stats().pending,2);assert.equal(limited.stats().pendingLimit,2);
      const sent=limited.transportStats().sentFrames;
      await assert.rejects(limited.callService('/web_link_demo/add_two_ints','example_interfaces/srv/AddTwoInts',{a:1,b:2}),{code:'REQUEST_LIMIT'});
      assert.throws(()=>limited.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:2}),{code:'REQUEST_LIMIT'});
      assert.equal(limited.transportStats().sentFrames,sent,'neither excess request reaches rosbridge');
      await waitFor(()=>feedback>0);goal.cancel();assert.equal((await goal.result).status,5);
      assert.equal((await waiting).success,true);assert.equal(limited.stats().pending,0);
      assert.equal((await limited.callService('/web_link_demo/add_two_ints','example_interfaces/srv/AddTwoInts',{a:2,b:3})).sum,5);
    }finally{limited.dispose();assert.equal(limited.stats().pending,0);assert.equal(limited.stats().listeners,0);}
  });
  await check('disconnect settles in-flight calls; disposal releases resources', async () => {
    const pending=client.callService('/web_link_demo/slow','std_srvs/srv/Trigger',{});
    client.disconnect(); await assert.rejects(pending,{code:'DISCONNECTED'});
    await client.connect();
    assert.equal(client.stats().pending,0);
    client.dispose(); assert.equal(client.stats().subscriptions,0);
  });
  console.log(`PASS ${count}/${count} CLI scenarios`);
} finally { client.dispose(); }
