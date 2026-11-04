/** Material precedence and ownership through actual SceneView/Three resources. */
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {URDFVisual} from 'urdf-loader/src/URDFClasses.js';
import {SceneView} from '../dist/scene/index.js';

const disposed=object=>{const count={value:0};object.addEventListener('dispose',()=>count.value++);return count;};
const view=Object.create(SceneView.prototype);
const root=new URDFVisual();root.urdfNode={children:[{tagName:'material'}]};
const object=new THREE.Group();root.add(object);
const oldTexture=new THREE.Texture(),keptTexture=new THREE.Texture();
const oldA=new THREE.MeshPhongMaterial({map:oldTexture,opacity:0,transparent:true});
const oldB=new THREE.MeshPhongMaterial({map:keptTexture,opacity:.2,transparent:true});
const replacement=new THREE.MeshPhongMaterial({map:keptTexture,opacity:1});
const geometries=[new THREE.BoxGeometry(),new THREE.BoxGeometry()];
const meshes=geometries.map(g=>new THREE.Mesh(g,[oldA,oldB]));object.add(...meshes);
const counters=[oldA,oldB,oldTexture,keptTexture,replacement,...geometries].map(disposed);
view.applyRobotMaterial(object,replacement);
assert(meshes.every(mesh=>mesh.material===replacement));
assert.deepEqual(counters.map(c=>c.value),[1,1,1,0,0,0,0],
  'Only replaced materials/unretained textures may be released; no geometry loss');
Object.assign(view,{generation:0,robot:root,markerMap:new Map(),axes:new Map(),pendingJoints:new Map(),children:new Set()});
view.resetData();
assert.deepEqual(counters.map(c=>c.value),[1,1,1,1,1,1,1],
  'Shared retained material/texture must release once when its model resets');

for(const explicit of [false,true]){
  const visual=explicit?new THREE.Group():new URDFVisual();
  visual.urdfNode={children:explicit?[{tagName:'material'}]:[]};
  const old=new THREE.MeshPhongMaterial({opacity:.35,transparent:true});
  const next=new THREE.MeshPhongMaterial(),mesh=new THREE.Mesh(new THREE.BoxGeometry(),old);
  const asset=new THREE.Group();asset.add(mesh);visual.add(asset);const count=disposed(old);
  view.applyRobotMaterial(asset,next);
  assert.equal(mesh.material,old,'No URDF material / non-robot mesh must retain embedded appearance');
  assert.equal(count.value,0);mesh.geometry.dispose();old.dispose();next.dispose();
}
console.log('PASS URDF material resource ownership, shared textures, geometry, reset, embedded fallback and marker isolation; no GL/ROS created');
