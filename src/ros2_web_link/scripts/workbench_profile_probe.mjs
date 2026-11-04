/** Verify public profile-to-workspace conversion without ROS, DOM or WebGL. */
import assert from 'node:assert/strict';
import {workspaceFromProfile,createDefaultRegistry} from '../dist/workbench/index.js';

const subscription={qos:'sensor',compression:'cbor',throttleMs:250};
const profile={scene:{robotDescription:'/custom/robot',robotDescriptionOptions:subscription,
  markers:'/custom/markers',markersOptions:subscription,path:'/custom/path',pathOptions:subscription},
  plot:{topic:'/custom/plot',type:'std_msgs/msg/Float64',fields:['data'],subscription}};
const workspace=workspaceFromProfile(profile,createDefaultRegistry());
for(const [id,port] of [['robot','description'],['markers','data'],['path','data'],['plot','data']]){
  const display=workspace.displays.find(display=>display.id===id);
  const {qos,compression,throttleMs}=display.inputs[port];
  assert.deepEqual({qos,compression,throttleMs},subscription,`${id} lost requested subscription options`);
  assert.equal(display.enabled,false,'Template conversion must not start subscriptions');
}
const defaults=workspaceFromProfile({},createDefaultRegistry());
assert.equal(defaults.displays.find(d=>d.id==='robot').inputs.description.qos,'latched');
assert.equal(defaults.displays.find(d=>d.id==='markers').inputs.data.qos,'reliable');
assert.equal(defaults.displays.find(d=>d.id==='path').inputs.data.qos,'reliable');
console.log('PASS URDF / Marker / Path / plot profile subscription options and paused defaults');
