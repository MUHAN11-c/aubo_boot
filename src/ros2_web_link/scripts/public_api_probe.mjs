/** Compile six public entries as an external strict TypeScript ESM consumer.
 * This explicit CLI probe starts no ROS node/browser and removes its fixture.
 */
import assert from 'node:assert/strict';
import {mkdtempSync,mkdirSync,writeFileSync,symlinkSync,rmSync} from 'node:fs';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';
import {build} from 'esbuild';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const fixture=mkdtempSync(path.join(tmpdir(),'ros2-web-public-api-'));
const consumer=`import {RosClient,QOS,DEFAULT_TRANSPORT_LIMITS,type TopicOptions,type TransportLimits} from '@local/ros2-web-link';
import {SceneView,TfBuffer} from '@local/ros2-web-link/scene';
import {ImageView,SeriesPlot,compressedImagePayload} from '@local/ros2-web-link/media';
import {runProjectChecks,validateProject} from '@local/ros2-web-link/audit';
import {discoverGraph,GraphView} from '@local/ros2-web-link/graph';
import {WorkbenchController,WorkbenchView,createDefaultRegistry} from '@local/ros2-web-link/workbench';
const client=new RosClient({url:'ws://127.0.0.1:1',maxPendingRequests:16});
const qos:TopicOptions={qos:QOS.sensor,compression:'cbor'};
const limits:Readonly<TransportLimits>=DEFAULT_TRANSPORT_LIMITS;
// @ts-expect-error budgets remain readonly for downstream TypeScript callers.
client.options.maxPendingRequests=32;
// @ts-expect-error unsupported QoS presets must not become an untyped string.
const bad:TopicOptions={qos:'automatic'};
export {client,qos,limits,SceneView,TfBuffer,ImageView,SeriesPlot,compressedImagePayload,runProjectChecks,validateProject,discoverGraph,GraphView,WorkbenchController,WorkbenchView,createDefaultRegistry};
`;
try{
  mkdirSync(path.join(fixture,'node_modules/@local'),{recursive:true});
  symlinkSync(root,path.join(fixture,'node_modules/@local/ros2-web-link'),'dir');
  writeFileSync(path.join(fixture,'package.json'),'\n{"type":"module","private":true}\n');
  writeFileSync(path.join(fixture,'consumer.ts'),consumer);
  for(const [module,moduleResolution] of [['NodeNext','NodeNext'],['ESNext','Bundler']]){
    writeFileSync(path.join(fixture,'tsconfig.json'),JSON.stringify({compilerOptions:{target:'ES2022',module,moduleResolution,strict:true,skipLibCheck:false,noEmit:true,types:[],lib:['ES2022','DOM','DOM.Iterable']},include:['consumer.ts']}));
    const result=spawnSync(process.execPath,[path.join(root,'node_modules/typescript/bin/tsc'),'--project',path.join(fixture,'tsconfig.json')],{encoding:'utf8',timeout:30000});
    assert.equal(result.status,0,`${moduleResolution} public declarations must compile with skipLibCheck=false\n${result.stdout}\n${result.stderr}`);
    console.log(`PASS six public API declarations in strict ${moduleResolution}, including negative type checks`);
  }
  const bundle=await build({absWorkingDir:fixture,entryPoints:['consumer.ts'],bundle:true,platform:'browser',format:'esm',target:'es2022',write:false,logLevel:'silent'});
  assert(bundle.outputFiles[0].contents.length>0);
  console.log('PASS external browser bundler resolves all six package exports');
}finally{rmSync(fixture,{recursive:true,force:true});}
