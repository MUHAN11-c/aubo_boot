/** Explicit Playwright CLI. Only read-only display/discovery operations are used. */
import assert from 'node:assert/strict';
import {showTool,showConnection} from './browser_controls.mjs';
const api=await import('../dist/workbench/index.js').catch(()=>({}));
assert.equal(typeof api.WorkbenchView,'function','workbench UI must be a reusable public component');
// The bounded native runner supplies an isolated URL and evidence directory.
if(!process.argv[2]){console.log('PASS WorkbenchView export; pass a URL for native browser flow');process.exit(0);}
const {chromium}=await import('playwright');
const {mkdirSync,readFileSync,writeFileSync,existsSync}=await import('node:fs');
const {dirname}=await import('node:path');
const out=process.env.WEB_LINK_EVIDENCE??'/tmp/ros2-web-workbench-browser';mkdirSync(out,{recursive:true});
const browser=await chromium.launch({headless:process.env.WORKBENCH_HEADLESS==='1',executablePath:process.env.CHROME_PATH??'/usr/bin/google-chrome',args:['--no-proxy-server']});
const page=await browser.newPage({viewport:{width:1440,height:1000}}),results=[],errors=[];
page.setDefaultTimeout(15000);page.on('pageerror',e=>errors.push(e.message));
await page.addInitScript(()=>{window.__workbenchGl=[];const original=HTMLCanvasElement.prototype.getContext;HTMLCanvasElement.prototype.getContext=function(...args){const context=original.apply(this,args);if(context&&/webgl/i.test(args[0])&&!window.__workbenchGl.includes(context))window.__workbenchGl.push(context);return context;};});
const wb=id=>page.locator(`#workbench-root [data-wb="${id}"]`);
const state=()=>page.evaluate(()=>workbenchView.stats());
const check=async(name,fn)=>{await fn();results.push(name);console.log('PASS',name);};
const enable=async id=>page.locator(`[data-display-id="${id}"] input[type=checkbox]`).check();
try{
 await page.goto(process.argv[2]+'/#workbench');await page.locator('#workbench-load').click();
 await check('offline mount has eight disabled templates and no spatial GL',async()=>{await wb('run').waitFor();assert(await wb('run').isDisabled());assert.equal(await page.locator('.wb-display').count(),8);assert.equal((await state()).workspace.displays.filter(d=>d.enabled).length,0);assert.equal(await page.evaluate(()=>__workbenchGl.length),0);});
 await showConnection(page);await page.locator('#connect').click();await page.waitForFunction(()=>document.getElementById('connection-state').textContent.includes('已连接'));
 await page.locator('#connection-close').click();
 await check('incremental spatial displays share exactly one real GL context',async()=>{
  await enable('cloud');await wb('run').click();await page.waitForFunction(()=>workbenchView.stats().controller.scene?.points>0);
  for(const id of ['robot','markers','path','tf'])await enable(id);
  await page.waitForFunction(()=>{const s=workbenchView.stats().controller.scene;return s?.robotLoaded&&s.joints>0&&s.markers>0&&s.pathPoints>0;});
  assert.equal(await page.evaluate(()=>__workbenchGl.length),1);assert.equal((await state()).controller.spatialDisplays,5);
 });
 await check('add a second cloud and stop one without disturbing its peer',async()=>{
  await wb('add-panel').locator(':scope > summary').click();await wb('plugin').selectOption('cloud');await wb('add-topic').fill('/web_link_demo/points');await wb('add').click();
  await page.waitForFunction(()=>workbenchView.stats().controller.spatialDisplays===6);
  const added=(await state()).workspace.displays.at(-1).id;assert.equal((await state()).workspace.displays.at(-1).plugin,'cloud');
  const editor=wb('editor');for(const summary of await editor.locator('details > summary').all())await summary.click();await editor.locator('[name="displayName"]').fill('点云对照');await editor.locator('[name="displayOptions"]').fill('{"maxPoints":50000}');await editor.locator('[name="qos"]').selectOption('custom');await editor.locator('[name="customQos"]').fill('{"history":"keep_last","depth":0,"reliability":"best_effort","durability":"volatile"}');await editor.locator('button[type=submit]').click();assert.equal((await state()).workspace.displays.at(-1).options.maxPoints,100000,'invalid edit must preserve the valid spec');
  await editor.locator('[name="qos"]').selectOption('sensor');await editor.locator('button[type=submit]').click();assert.equal((await state()).workspace.displays.at(-1).name,'点云对照');assert.equal((await state()).workspace.displays.at(-1).options.maxPoints,50000);
  await page.locator('[data-display-id="cloud"] input').uncheck();await page.waitForFunction(()=>workbenchView.stats().controller.scene.points>0);
  assert.equal((await state()).controller.spatialDisplays,5);assert.equal(await page.evaluate(()=>__workbenchGl.length),1);
  await page.locator(`[data-display-id="${added}"] [data-remove]`).click();assert.equal((await state()).controller.spatialDisplays,4);
  if(existsSync(dirname(out)+'/fixture-ready.json')){
   await wb('add-topic').fill('/workbench_fixture/points');await wb('add').click();await page.waitForFunction(()=>workbenchView.stats().controller.displays.some(d=>d.plugin==='cloud'&&d.stats.points>0));
   const independent=(await state()).workspace.displays.at(-1).id;assert.equal((await state()).workspace.displays.at(-1).inputs.data.topic,'/workbench_fixture/points');await page.locator(`[data-display-id="${independent}"] [data-remove]`).click();
  }
 });
 await check('image, depth, curve and arbitrary raw message adapters receive data',async()=>{
  for(const id of ['image','depth','plot'])await enable(id);
  await page.locator('[data-output-id="image"] img').waitFor({state:'visible'});await page.locator('[data-output-id="depth"] canvas').waitFor({state:'visible'});
  await page.waitForFunction(()=>workbenchView.stats().controller.displays.some(d=>d.id==='plot'&&d.stats.samples>2));
  if(!await wb('add-panel').evaluate(node=>node.open))await wb('add-panel').locator(':scope > summary').click();await wb('plugin').selectOption('raw');await wb('add-topic').fill('/web_link_demo/sensor');await wb('add').click();
  await page.waitForFunction(()=>workbenchView.stats().controller.displays.some(d=>d.plugin==='raw'&&d.stats.received>0));
 });
 await check('actual graph directions, filtering, providers and Action list',async()=>{
  await wb('tab-graph').click();await wb('graph-refresh').click();await page.waitForFunction(()=>!!workbenchView.stats().graphSnapshot);
  const s=(await state()).graphSnapshot;assert.notEqual(s.status,'failed');assert(s.edges.some(e=>e.kind==='publish'&&e.topic==='/web_link_demo/sensor'));assert(s.edges.some(e=>e.kind==='subscribe'&&e.topic==='/web_link_demo/sensor'));
  assert(s.services.some(v=>v.name==='/web_link_demo/add_two_ints'&&v.providers.includes('/web_link_demo/demo')));assert(s.actions.some(v=>v.name==='/web_link_demo/fibonacci'));
  await wb('graph-search').fill('/web_link_demo/sensor');assert((await state()).graph.edges>0);await wb('graph-mode').selectOption('nodes');assert.equal((await state()).graph.topics,0);await wb('graph-search').fill('');await wb('graph-mode').selectOption('topics');
  await wb('tab-interfaces').click();await wb('endpoint-kind').selectOption('service');await wb('endpoint').selectOption('/web_link_demo/add_two_ints');await wb('inspect').click();await page.waitForFunction(()=>document.querySelector('[data-wb="interface"]')?.textContent.includes('example_interfaces/srv/AddTwoInts'));
  assert((await wb('interface').textContent()).includes('int64'),'IDL fields must remain readable');
  await wb('tab-graph').click();await wb('graph-search').fill('std_msgs/msg/Float64');assert((await state()).graph.edges>0);await wb('graph-search').fill('/web_link_demo/sensor');await wb('graph').screenshot({path:out+'/graph.png'});await wb('graph-search').fill('');
  await wb('tab-space').click();await page.locator('#workbench-root [data-wb="grid"]').screenshot({path:out+'/displays.png'});
  if(existsSync(dirname(out)+'/fixture-ready.json')){
   assert(s.nodes.some(n=>n.name==='/workbench_fixture/source'));writeFileSync(dirname(out)+'/stop-fixture.request','stop\n');const deadline=Date.now()+5000;while(!existsSync(dirname(out)+'/fixture-stopped.json')){assert(Date.now()<deadline,'fixture stop deadline');await new Promise(resolve=>setTimeout(resolve,50));}
   await wb('tab-graph').click();await wb('graph-refresh').click();await page.waitForFunction(()=>!workbenchView.stats().graphRefreshing);assert(!(await state()).graphSnapshot.nodes.some(n=>n.name==='/workbench_fixture/source'),'departed node must disappear from new metadata snapshot');
  }
 });
 await wb('tab-space').click();
 await check('missing fixed-frame TF is visible on individual spatial display',async()=>{
  await page.locator('#workbench-root .wb-settings > summary').click();await wb('frame').fill('missing_frame');await wb('globals').locator('button[type=submit]').click();await wb('run').click();
  await page.waitForFunction(()=>Array.from(document.querySelectorAll('.wb-display-status')).some(node=>/TF|坐标/.test(node.textContent)));
  await wb('frame').fill('demo_base');await wb('globals').locator('button[type=submit]').click();await wb('run').click();
  await page.waitForFunction(()=>workbenchView.stats().controller.scene?.robotLoaded);await page.locator('#workbench-root .wb-settings > summary').click();
 });
 await check('save/export, invalid import rollback and valid import stays paused',async()=>{
  await page.locator('#workbench-root .wb-settings > summary').click();
  await wb('save').click();const download=page.waitForEvent('download');await wb('export').click();const file=out+'/workspace.json';await(await download).saveAs(file);const exported=JSON.parse(readFileSync(file,'utf8'));assert.equal(exported.schemaVersion,1);
  const before=(await state()).workspace;await wb('file').setInputFiles({name:'invalid.json',mimeType:'application/json',buffer:Buffer.from('{"schemaVersion":999}')});await page.waitForFunction(()=>document.querySelector('[data-wb="status"]').textContent.includes('失败'));assert.deepEqual((await state()).workspace,before);assert.equal((await state()).controller.running,true);
  await wb('file').setInputFiles(file);await page.waitForFunction(()=>!workbenchView.stats().controller.running);assert.deepEqual((await state()).workspace,exported);assert.equal((await state()).controller.activeDisplays,0);
  await wb('restore').click();assert.equal((await state()).controller.running,false);
 });
 await check('explicit resume, classic observation exclusion and disconnect cleanup',async()=>{
  await wb('run').click();await page.waitForFunction(()=>workbenchView.stats().controller.activeDisplays>0);
  await showTool(page,'media');await page.locator('#plot-start').click();assert.equal((await state()).controller.running,false);await page.locator('#plot-stop').click();
  await showTool(page,'workbench');await wb('run').click();await showConnection(page);await page.locator('#disconnect').click();await page.locator('#connection-close').click();await page.waitForFunction(()=>!workbenchView.stats().controller.running);
  assert.equal((await state()).controller.activeDisplays,0);assert.equal((await state()).graphRefreshing,false);
  assert.equal((await state()).graphStale,true);assert.match(await wb('graph-state').textContent(),/上一会话|过期/);
  const resources=await page.evaluate(()=>client.stats());assert.equal(resources.subscriptions,0);assert.equal(resources.pending,0);
  assert.equal(await page.evaluate(()=>__workbenchGl.filter(gl=>!gl.isContextLost()).length),0);
 });
 await page.locator('#workbench').screenshot({path:out+'/workbench.png'});assert.deepEqual(errors,[]);
 writeFileSync(out+'/result.json',JSON.stringify({status:'PASS',results,errors,state:await state()},null,2));
}catch(error){await page.screenshot({path:out+'/failure.png'}).catch(()=>{});writeFileSync(out+'/result.json',JSON.stringify({status:'FAIL',results,errors,error:String(error),uiStatus:await wb('status').textContent().catch(()=>null),state:await state().catch(()=>null)},null,2));throw error;}
finally{await page.close();await browser.close();}
