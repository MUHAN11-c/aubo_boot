// Explicit in-memory acceptance of scene placement; no DOM, WebGL or ROS session.
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {SceneView, TfBuffer} from '../dist/scene/index.js';
const stamp=sec=>({sec,nanosec:0});
const tf=new TfBuffer({});
const add=(sec,x)=>tf.ingest({transforms:[{header:{frame_id:'map',stamp:stamp(sec)},child_frame_id:'tool',transform:{translation:{x,y:0,z:0},rotation:{x:0,y:0,z:0,w:1}}}]});
// Use the shipped placement implementation with Three's pure object model. This
// narrow regression probe deliberately avoids constructing a renderer in Node.
const view=Object.create(SceneView.prototype);
view.tfBuffer=tf;view.options={fixedFrame:'map'};
const resource=(sec,latest=false)=>({object:new THREE.Group(),header:{frame_id:'tool',stamp:stamp(sec)},latest});
const place=r=>{const issues=new Set();view.place(r,issues);return issues;};
add(10,1);
const persistent=resource(10);assert.equal(place(persistent).size,0);assert.equal(persistent.object.position.x,1);
add(21,2);assert.equal(tf.lookup('map','tool',stamp(10)),undefined);
assert.equal(place(persistent).size,0);assert(persistent.object.visible);assert.equal(persistent.object.position.x,1);
console.log('PASS persistent stamped objects retain resolved placement after TF expires');
const zero=resource(0);assert.equal(place(zero).size,0);assert.equal(zero.object.position.x,2);
add(22,3);place(zero);assert.equal(zero.object.position.x,2);
console.log('PASS zero display stamps resolve latest once then retain placement');
const locked=resource(10,true);place(locked);assert.equal(locked.object.position.x,3);add(23,4);place(locked);assert.equal(locked.object.position.x,4);
console.log('PASS frame_locked objects follow current TF');
view.options.fixedFrame='unknown';assert(place(persistent).size>0);assert(!persistent.object.visible);
view.options.fixedFrame='map';place(persistent);assert(persistent.object.visible);
console.log('PASS changing fixed frame invalidates cached placement and reports unavailable TF');
tf.dispose();
console.log('PASS 4 scene placement groups; no process or port started');
