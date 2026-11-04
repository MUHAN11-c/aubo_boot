/** Execute the actual Cytoscape adapter headlessly; browser rendering is separate. */
import assert from 'node:assert/strict';
import {GraphView} from '../dist/graph/index.js';
const snapshot={status:'complete',receivedAt:Date.now(),elapsedMs:1,cancelled:false,scope:'fixture',errors:[],omittedErrors:0,services:[],actions:[],capabilities:{endpointQos:false},
 nodes:[{id:'n1',name:'/lab/a',duplicates:1},{id:'n2',name:'/lab/b',duplicates:1},{id:'n3',name:'/logger',duplicates:1}],
 topics:[{id:'t1',name:'/lab/data',types:['std_msgs/msg/String']},{id:'t2',name:'/rosout',types:['rcl_interfaces/msg/Log']}],
 edges:[{id:'e1',source:'n1',target:'t1',kind:'publish',topic:'/lab/data'},{id:'e2',source:'t1',target:'n2',kind:'subscribe',topic:'/lab/data'},{id:'e3',source:'n3',target:'t2',kind:'publish',topic:'/rosout'}]};
const selected=[],view=new GraphView(undefined,{onSelect:item=>selected.push(item)});
// A matched Topic must not expand through its publisher into unrelated Topics.
const sharedPublisher={...snapshot,
 topics:[...snapshot.topics,{id:'t3',name:'/unrelated/image',types:['sensor_msgs/msg/Image']}],
 edges:[...snapshot.edges,{id:'e4',source:'n1',target:'t3',kind:'publish',topic:'/unrelated/image'},{id:'e5',source:'t3',target:'n3',kind:'subscribe',topic:'/unrelated/image'}]};
view.setSnapshot(sharedPublisher);view.filter({text:'/lab/data'});
assert.deepEqual([view.stats().nodes,view.stats().topics,view.stats().edges],[2,1,2],'Topic search shows only matched Topic and its publisher/subscriber');
view.filter({text:'/lab/a'});assert.deepEqual([view.stats().nodes,view.stats().topics,view.stats().edges],[3,2,4],'node search retains its Topics and peer nodes, not peer Topics');
view.filter({text:'std_msgs/msg/String',mode:'nodes'});assert.deepEqual([view.stats().nodes,view.stats().topics,view.stats().edges],[2,0,1]);
view.filter({text:'',mode:'topics'});

view.setSnapshot(snapshot);assert.deepEqual([view.stats().nodes,view.stats().topics,view.stats().edges],[3,2,3]);
view.filter({hideInternal:true});assert.equal(view.stats().topics,1);assert.equal(view.stats().edges,2);
view.filter({text:'/lab/data'});assert(view.stats().nodes>=2);assert.equal(view.stats().edges,2);
view.filter({text:'std_msgs/msg/String'});assert.equal(view.stats().topics,1,'message type search must keep matching Topic and actual peers');assert.equal(view.stats().edges,2);
view.filter({text:'/nothing'});assert.equal(view.stats().nodes,0);assert.equal(view.stats().edges,0);
view.filter({text:'',mode:'nodes'});assert.equal(view.stats().topics,0);assert.equal(view.stats().edges,1);
view.focus('n1');assert.equal(selected.at(-1).name,'/lab/a');
view.filter({mode:'topics',hideInternal:false});const element=view.cy.getElementById('t1');element.emit('tap');assert.equal(selected.at(-1).kind,'topic');
const beforeLayout=view.stats();view.filter({layout:'grid'});assert.equal(view.stats().layout,'grid');
for(const count of ['nodes','topics','edges'])assert.equal(view.stats()[count],beforeLayout[count],'layout must preserve declared relations');
view.setZoom(1);assert.equal(view.stats().zoom,1);view.setZoom(99);assert.equal(view.stats().zoom,4);view.setZoom(.001);assert.equal(view.stats().zoom,.05);
assert.throws(()=>view.setZoom(NaN),/finite/);assert.throws(()=>view.setZoom(0),/positive/);assert.throws(()=>view.filter({layout:'unknown'}),/layout/);
const count=selected.length;view.dispose();view.dispose();assert(view.stats().disposed);assert.equal(view.stats().nodes,0);
element.emit('tap');assert.equal(selected.length,count,'disposed adapter must remove its listener');
assert.throws(()=>view.setSnapshot(snapshot),/disposed/);
console.log('PASS graph headless layout/filter/focus/disposal against Cytoscape; not a browser rendering test');
