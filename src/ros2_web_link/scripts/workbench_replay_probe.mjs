/** Bounded desktop check of the new displays using finalized Peach recordings. */
import assert from 'node:assert/strict';
import {mkdirSync,writeFileSync,readFileSync} from 'node:fs';
import {chromium} from 'playwright';
import {showTool,showConnection} from './browser_controls.mjs';
const out=process.env.WEB_LINK_EVIDENCE??'/tmp/ros2-web-workbench-replay';mkdirSync(out,{recursive:true});
const timing=JSON.parse(readFileSync(process.env.WEB_LINK_REPLAY_TIMING,'utf8'));
const firstCloudOffsetMs=timing.topics['/camera/depth_registered/points'].firstOffsetSeconds*1000;
assert(Number.isFinite(firstCloudOffsetMs)&&firstCloudOffsetMs>=0&&firstCloudOffsetMs<9000,'finalized recording timing is required');
const rewindRefillMs=Math.ceil(firstCloudOffsetMs)+1000;
const browser=await chromium.launch({headless:false,executablePath:process.env.CHROME_PATH??'/usr/bin/google-chrome',args:['--no-proxy-server']});
const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[],samples=[],results=[],interactions=[];
page.setDefaultTimeout(15000);page.on('pageerror',error=>errors.push(error.message));
// Probe-only native browser evidence. No extra RAF loop, payload retention or
// application instrumentation. Timestamp-based phases avoid delayed observer
// delivery attributing an earlier task to a later phase. Buffers are bounded.
await page.addInitScript(()=>{
 const rows=[],observers=[],supported={},dropped={},phases=[{name:'navigation',at:0}],limit=6000;
 for(const type of ['longtask','long-animation-frame','event']){
  supported[type]=PerformanceObserver.supportedEntryTypes.includes(type);dropped[type]=0;if(!supported[type])continue;
  const accept=entries=>{for(const e of entries){if(rows.length>=limit){dropped[type]++;continue;}
   const row={type,start:e.startTime,duration:e.duration};
   if(type==='event')Object.assign(row,{name:e.name,id:e.interactionId,inputDelay:e.processingStart-e.startTime,processing:e.processingEnd-e.processingStart});
   if(type==='long-animation-frame')Object.assign(row,{blocking:e.blockingDuration,renderStart:e.renderStart,scripts:[...(e.scripts??[])].sort((a,b)=>b.duration-a.duration).slice(0,4).map(s=>({duration:s.duration,invoker:s.invoker?.slice(0,200),source:s.sourceURL?.slice(0,256),fn:s.sourceFunctionName?.slice(0,128),position:s.sourceCharPosition,forcedLayout:s.forcedStyleAndLayoutDuration}))});
   rows.push(row);
  }};
  const observer=new PerformanceObserver(list=>accept(list.getEntries()));
  observer.observe(type==='event'?{type,buffered:false,durationThreshold:16}:{type,buffered:false});observers.push({observer,accept});
 }
 window.__replayMetrics={phase(name){phases.push({name,at:performance.now()});},snapshot(){for(const {observer,accept} of observers)accept(observer.takeRecords());return {supported,dropped,limit,phases,rows,now:performance.now(),timeOrigin:performance.timeOrigin};},stop(){for(const {observer,accept} of observers){accept(observer.takeRecords());observer.disconnect();}}};
});
await page.addInitScript(()=>{window.__workbenchGl=[];const original=HTMLCanvasElement.prototype.getContext;HTMLCanvasElement.prototype.getContext=function(...args){const gl=original.apply(this,args);if(gl&&/webgl/i.test(args[0])&&!__workbenchGl.includes(gl))__workbenchGl.push(gl);return gl;};});
const wb=id=>page.locator(`[data-wb="${id}"]`),state=()=>page.evaluate(()=>workbenchView.stats());
const phase=name=>page.evaluate(name=>__replayMetrics.phase(name),name);
const act=async(name,callback)=>{const at=Date.now();await phase(name);await callback();interactions.push({name,wallMs:Date.now()-at});await phase('steady-streams');};
let failure,clockRewinds=0,previousClock,flowBaseline,rewindRecoveryDeadline=0,rewindEmptySamples=0;
try{
 await page.goto((process.argv[2]??'http://127.0.0.1:22588')+'/#workbench');await phase('workbench-load');await page.locator('#workbench-load').click();await wb('run').waitFor();
 assert.equal(await page.locator('#project-select').inputValue(),'peach_replay_large');await showConnection(page);await page.locator('#connect').click();await page.waitForFunction(()=>client?.connected);await page.locator('#connection-close').click();
 for(const id of ['robot','tf','markers','cloud','path','image','depth','plot'])await page.locator(`[data-display-id="${id}"] input[type=checkbox]`).check();
 await phase('initial-eight-displays');await wb('run').click();await page.waitForFunction(()=>{const stats=workbenchView.stats().controller,robot=stats.displays.find(d=>d.id==='robot');return stats.scene?.robotLoaded&&stats.scene.movableJoints===6&&stats.scene.points>0&&robot?.assets?.assetsTotal===12&&robot.assets.assetsLoaded===12&&robot.assets.assetReadyMs!==null;});
 for(const id of ['image','depth'])await page.locator(`[data-output-id="${id}"] canvas`).waitFor({state:'visible'});
 await page.waitForFunction(()=>workbenchView.stats().controller.displays.find(d=>d.id==='plot')?.stats.samples>2);results.push('Peach recorded URDF/6 joints/12 mesh assets, TF, cloud, RGB-D and plot');
 await phase('four-clouds');await wb('add-panel').locator(':scope > summary').click();await wb('plugin').selectOption('cloud');await wb('add-topic').fill('/camera/depth_registered/points');for(let i=0;i<3;i++)await wb('add').click();
 await page.waitForFunction(()=>workbenchView.stats().controller.scene?.points>250000);
 const before=(await state()).workspace;assert.equal((await state()).controller.spatialDisplays,8);assert.equal((await state()).controller.activeDisplays,11);await wb('add').click();assert.deepEqual((await state()).workspace,before);assert.match(await wb('status').textContent(),/budget|预算/i);assert.equal(await page.evaluate(()=>__workbenchGl.filter(gl=>!gl.isContextLost()).length),1);results.push('4 clouds / 400000 configured points / 8 spatial limit; overflow rejected without disrupting peers');
 // Test-only observers reuse the same SDK subscription entries/options. They
 // retain counts/stamps only, never payloads, and do not add network readers.
 await page.evaluate(()=>{
  window.__wbFlows={};window.__wbFlowOff=[];const inputs=new Map();
  for(const display of workbenchView.stats().workspace.displays)if(display.enabled)for(const binding of Object.values(display.inputs))if(!['/robot_description','/tf_static'].includes(binding.topic))inputs.set(binding.topic,binding);
  inputs.set('/tf',{topic:'/tf',type:'tf2_msgs/msg/TFMessage',qos:'reliable'});
  for(const [name,binding] of inputs){const flow=__wbFlows[name]={count:0,lastAt:0,lastStamp:''};__wbFlowOff.push(client.subscribe(name,binding.type,message=>{flow.count++;flow.lastAt=Date.now();if(message.header?.stamp)flow.lastStamp=`${message.header.stamp.sec}.${message.header.stamp.nanosec}`;},binding));}
 });
 await wb('add-panel').locator(':scope > summary').click();
 await page.locator('.wb-settings > summary').click();await wb('save').click();await page.locator('.wb-settings > summary').click();
 await phase('graph-discovery');await wb('tab-graph').click();await wb('graph-refresh').click();await page.waitForFunction(()=>!!workbenchView.stats().graphSnapshot&&!workbenchView.stats().graphRefreshing);assert.notEqual((await state()).graphSnapshot.status,'failed');results.push('read-only graph RPCs complete while shared connection carries full recorded streams');
 await wb('graph').screenshot({path:out+'/peach-graph.png'});
 await wb('graph-layout').selectOption('grid');await page.screenshot({path:out+'/peach-graph-grid.png',fullPage:true});
 await wb('graph-zoom-reset').click();assert.equal((await state()).graph.zoom,1);
 await wb('graph-zoom-in').click();assert.equal((await state()).graph.zoom,1.25);
 await wb('tab-space').click();await page.waitForTimeout(200);await wb('tab-graph').click();await page.waitForTimeout(200);assert.equal((await state()).graph.zoom,1.25);
 await wb('fit').click();await wb('tab-space').click();await page.screenshot({path:out+'/peach-displays.png',fullPage:true});
 const device=await page.evaluate(()=>{const gl=__workbenchGl.find(gl=>!gl.isContextLost()),ext=gl.getExtension('WEBGL_debug_renderer_info');return {renderer:gl.getParameter(ext?.UNMASKED_RENDERER_WEBGL??gl.RENDERER),userAgent:navigator.userAgent,width:innerWidth,height:innerHeight,dpr:devicePixelRatio};});writeFileSync(out+'/device.json',JSON.stringify(device,null,2));
 await phase('steady-streams');
 const start=Date.now();let lastInspection=0;
 const actions=[
  ['orbit-and-zoom',async()=>{await wb('scene').scrollIntoViewIfNeeded();const b=await wb('scene').boundingBox();await page.mouse.move(b.x+b.width*.5,b.y+b.height*.5);await page.mouse.down();await page.mouse.move(b.x+b.width*.65,b.y+b.height*.6,{steps:24});await page.mouse.up();await page.mouse.wheel(0,-240);await wb('camera').click();}],
  ['select-display-properties',async()=>{await page.locator('[data-display-id="cloud"] .wb-display-name').click();assert(await wb('editor').isVisible());for(const summary of await wb('editor').locator('details > summary').all()){await summary.click();await summary.click();}}],
  ['graph-filter-and-fit',async()=>{await wb('tab-graph').click();await wb('graph-search').fill('/camera/depth_registered/points');assert.equal((await state()).graph.topics,1);await wb('fit').click();await page.screenshot({path:out+'/graph-filtered.png',fullPage:true});await wb('graph-search').fill('');await wb('tab-space').click();}],
  ['read-ros-interface',async()=>{await wb('tab-interfaces').click();await wb('endpoint-kind').selectOption('topic');await wb('endpoint').selectOption('/camera/depth_registered/points');await wb('inspect').click();await page.waitForFunction(()=>document.querySelector('[data-wb="interface"]').textContent.includes('PointField'));await page.screenshot({path:out+'/interface.png',fullPage:true});await wb('tab-space').click();}],
  ['toggle-cloud-with-peers-live',async()=>{const checkbox=page.locator('[data-display-id="cloud"] input[type=checkbox]');await checkbox.uncheck();assert.equal((await state()).controller.activeDisplays,10);await checkbox.check();assert.equal((await state()).controller.activeDisplays,11);}],
  ['navigate-and-return',async()=>{await showTool(page,'overview');await showTool(page,'workbench');assert.equal((await state()).controller.activeDisplays,11);await showConnection(page);assert(await page.locator('#disconnect').isEnabled());await page.locator('#connection-close').click();}],
  ['scroll-images-and-plot',async()=>{await page.locator('[data-output-id="plot"]').scrollIntoViewIfNeeded();await page.screenshot({path:out+'/peach-images-plot.png',fullPage:true});await wb('scene').scrollIntoViewIfNeeded();}],
 ];
 let nextAction=0;
 while(Date.now()-start<120000){
  const sample=await page.evaluate(()=>({at:Date.now(),state:workbenchView.stats(),flows:structuredClone(__wbFlows),heapBytes:performance.memory?.usedJSHeapSize,activeGl:__workbenchGl.filter(gl=>!gl.isContextLost()).length}));
  const current=sample.state.controller.scene.clockSeconds;if(previousClock!==undefined&&current<previousClock){clockRewinds++;rewindRecoveryDeadline=Date.now()+rewindRefillMs;}previousClock=current;
  assert.equal(sample.state.controller.running,true);assert.equal(sample.activeGl,1);
  // A rewind deliberately clears old-epoch geometry. Only that boundary may be
  // empty; the recording itself has a 3.319s initial cloud gap. Use its frozen
  // receive-time offset plus 1s delivery slack, below the 10s stream-health gate.
  if(sample.state.controller.scene.points===0){assert(Date.now()<=rewindRecoveryDeadline,'point stream empty outside bounded rewind recovery');rewindEmptySamples++;}
  assert(sample.state.controller.scene.points<=400000);assert.equal(sample.state.controller.renderer.renderErrors,0);assert.deepEqual(sample.state.controller.renderer.shaderErrors,[]);assert.equal(sample.state.graphStale,false);
  if(Date.now()-start>15000)for(const [topic,flow] of Object.entries(sample.flows)){assert(flow.count>0,`no ${topic} samples`);assert(Date.now()-flow.lastAt<10000,`${topic} stalled`);}
  if(Date.now()-start-lastInspection>=10000){
   lastInspection=Date.now()-start;if(flowBaseline)for(const [topic,flow] of Object.entries(sample.flows))assert(flow.count>flowBaseline[topic],`${topic} did not advance`);
   flowBaseline=Object.fromEntries(Object.entries(sample.flows).map(([topic,flow])=>[topic,flow.count]));samples.push(sample);console.log(`LOAD ${Math.round((Date.now()-start)/1000)}s · ${sample.state.controller.scene.points} points · ${clockRewinds} rewinds`);
   if(nextAction<actions.length){const [name,callback]=actions[nextAction++];await act(name,callback);console.log('INTERACTION',name);}
  }
  await new Promise(resolve=>setTimeout(resolve,500));
 }
 assert(clockRewinds>=1,'recorded loop must exercise clock rewind');results.push('120s bounded new workbench load with progressing per-topic counters and recorded clock rewind');
 await phase('pause-release');await page.evaluate(()=>{__wbFlowOff.forEach(off=>off());__wbFlowOff=[];});await wb('pause').click();assert.equal((await state()).controller.activeDisplays,0);assert.equal(await page.evaluate(()=>client.stats().subscriptions),0);assert.equal(await page.evaluate(()=>__workbenchGl.filter(gl=>!gl.isContextLost()).length),0);
 await act('restart-after-pause',async()=>{await wb('run').click();await page.waitForFunction(()=>workbenchView.stats().controller.scene?.points>0);assert.equal((await state()).controller.activeDisplays,11);assert.equal(await page.evaluate(()=>__workbenchGl.filter(gl=>!gl.isContextLost()).length),1);});
 await phase('disconnect-release');await showConnection(page);await page.locator('#disconnect').click();await page.locator('#connection-close').click();assert.equal(await page.evaluate(()=>client.stats().pending),0);assert.equal(await page.evaluate(()=>client.stats().subscriptions),0);assert.equal(await page.evaluate(()=>__workbenchGl.filter(gl=>!gl.isContextLost()).length),0);assert.deepEqual(errors,[]);results.push('real pointer/keyboard/form/tab interactions, pause/restart and disconnect release');
}catch(error){failure=String(error);await page.screenshot({path:out+'/failure.png'}).catch(()=>{});}
finally{
 const metrics=await page.evaluate(()=>{const evidence=window.__replayMetrics?.snapshot();window.__replayMetrics?.stop();return evidence;}).catch(()=>null);writeFileSync(out+'/browser-performance.json',JSON.stringify({metrics,interactions,limitations:'Native reported events >=16ms, duration quantized to 8ms; not INP, GPU time or continuous pointer/wheel latency. Action wall time includes automation waits and screenshots. Long tasks/LoAF include startup and diagnostics; separate phase timestamps.'},null,2));
 const final=await state().catch(()=>null);await page.evaluate(()=>{window.__wbFlowOff?.forEach(off=>off());workbenchView?.pause();client?.disconnect();}).catch(()=>{});
 writeFileSync(out+'/result.json',JSON.stringify({status:failure?'FAIL':'PASS',failure,results,errors,clockRewinds,rewindEmptySamples,firstCloudOffsetMs,rewindRefillMs,samples,final,scope:'120s new-workbench capacity, one shared client; test-only in-client counters; rewind refill uses frozen recording start gap + 1s; not a replacement for historical 1800s transport benchmark'},null,2));await page.close();await browser.close();
}
if(failure)throw Error(failure);console.log(`PASS ${results.length} Peach replay groups; ${clockRewinds} observed clock rewinds; hardware/camera/Gazebo not started`);
