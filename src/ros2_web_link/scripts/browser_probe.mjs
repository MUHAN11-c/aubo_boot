// Explicit browser smoke CLI against the hardware-free example launch.
// All application interaction goes through the same controls a user clicks.
import assert from 'node:assert/strict';
import {mkdirSync, readFileSync, writeFileSync} from 'node:fs';
import {chromium} from 'playwright';
import {showTool,showConnection,revealControl} from './browser_controls.mjs';

const base=process.argv[2] || 'http://127.0.0.1:8088';
const out=process.env.WEB_LINK_EVIDENCE || '/tmp/ros2-web-link-browser';
mkdirSync(out,{recursive:true});
const browser=await chromium.launch({headless:process.env.WEB_LINK_HEADED!=='1',executablePath:process.env.CHROME_PATH || '/usr/bin/google-chrome',
  args:['--no-proxy-server','--host-resolver-rules=MAP ros2-web-link.lan 127.0.0.1']});
const page=await browser.newPage({viewport:{width:1280,height:900}});
const errors=[];const results=[];
page.on('pageerror',error=>errors.push(error.message));
const check=async(name,fn)=>{
  await fn();results.push(name);console.log(`PASS ${name}`);
};
const click=async id=>(await revealControl(page,id)).click();
const fill=async(id,value)=>(await revealControl(page,id)).fill(value);
const text=id=>page.locator(`#${id}`).textContent();
const contains=async(id,needle)=>{
  await page.waitForFunction(({id,needle})=>document.getElementById(id).textContent.includes(needle),{id,needle});
};
try {
  await page.goto(base);
  await check('offline controls / connection',async()=>{
    assert(await page.locator('#service-call').isDisabled());
    await click('connect');await contains('connection-state','已连接');
  });
  await check('one-click full self-check',async()=>{
    await click('self-check');
    await page.locator('#check-results[data-complete]').waitFor({timeout:45000});
    assert.equal(await page.locator('#check-results').getAttribute('data-complete'),'passed',await text('check-results'));
    assert.equal(await page.locator('#check-results .pass').count(),12);
  });
  await check('project selection / readonly run / complete report / fill panel',async()=>{
    await showConnection(page);await page.locator('#project-select').selectOption('manual');
    assert.equal(await page.locator('[data-audit-case]').count(),1);
    await click('project-select-none');
    assert.equal(await page.locator('[data-audit-case]:checked').count(),0);
    assert(await page.locator('#project-run').isDisabled());
    assert(await page.locator('#project-export').isDisabled());
    await showConnection(page);await page.locator('#project-select').selectOption('demo');
    await click('project-select-none');assert.equal(await page.locator('[data-audit-case]:checked').count(),0);
    assert(await page.locator('#project-run').isDisabled());
    await click('project-select-read');assert.equal(await page.locator('[data-audit-case]:checked').count(),6);
    await click('project-run-bottom');
    await page.locator('#project-cases[data-complete="passed"]').waitFor({timeout:45000});
    const download=page.waitForEvent('download');await click('project-export');
    await (await download).saveAs(`${out}/project-readonly.json`);
    const report=JSON.parse(readFileSync(`${out}/project-readonly.json`,'utf8'));
    assert.equal(report.projectId,'demo');assert.equal(report.selectedIds.length,6);
    assert.deepEqual(report.totals,{PASS:6,FAIL:0,CANCELLED:0,NOT_RUN:2});
    await page.locator('[data-case-id="demo.service"] .case-open').click();
    assert.equal(await page.locator('#service-name').inputValue(),'/web_link_demo/add_two_ints');
    assert.deepEqual(JSON.parse(await page.locator('#service-json').inputValue()),{a:3,b:4});
  });
  await check('project Action stop confirms terminal and keeps NOT_RUN visible',async()=>{
    await click('project-select-none');await page.locator('[data-audit-case="demo.action"]').check();
    await click('project-run');await page.locator('[data-case-id="demo.action"] .audit-status.running').waitFor();
    assert(await page.locator('#disconnect').isDisabled());
    // The demo step takes 200ms; allow feedback before stopping this 8-step goal.
    await page.waitForTimeout(350);await click('project-stop-bottom');
    await page.locator('#project-cases[data-complete="cancelled"]').waitFor({timeout:15000});
    const download=page.waitForEvent('download');await click('project-export');
    await (await download).saveAs(`${out}/project-cancelled.json`);
    const report=JSON.parse(readFileSync(`${out}/project-cancelled.json`,'utf8'));
    assert.deepEqual(report.totals,{PASS:0,FAIL:0,CANCELLED:1,NOT_RUN:7});
    const action=report.results.find(result=>result.id==='demo.action');
    assert.equal(action.evidence.cancelConfirmed,true);assert.equal(action.evidence.terminalStatus,5);
    await click('project-select-read');
  });
  await check('graph refresh',async()=>{await page.getByText('查看 ROS 图谱',{exact:true}).click();await click('graph-refresh');await contains('graph-result','/web_link_demo/demo');});
  await check('topic controls / actual echo / unsubscribe',async()=>{
    await click('topic-subscribe');await click('topic-publish');await contains('topic-result','hello from browser');
    await click('topic-unsubscribe');await contains('topic-result','已取消订阅');
  });
  await check('manual subscription followed by one-click self-check',async()=>{
    await click('topic-subscribe');await click('self-check');
    await page.locator('#check-results[data-complete]').waitFor({timeout:45000});
    assert.equal(await page.locator('#check-results').getAttribute('data-complete'),'passed',await text('check-results'));
    await click('topic-unsubscribe');
  });
  await check('service success / invalid JSON / wait abort',async()=>{
    await click('service-call');await contains('service-result','7');
    await fill('service-json','{bad');await click('service-call');await contains('service-result','SyntaxError');
    await fill('service-json','{}');await fill('service-name','/web_link_demo/slow');await fill('service-type','std_srvs/srv/Trigger');
    await click('service-call');await click('service-abort');await contains('service-result','WAIT_ABORTED');
  });
  await check('action button cancellation / terminal status',async()=>{
    await fill('action-json','{"order":20}');await click('action-send');await contains('action-feedback','sequence');
    await click('action-cancel');await contains('action-result','"status": 5');
  });
  await check('parameter list / describe / write / readback / rejected / atomic',async()=>{
    await click('parameter-list');await contains('parameter-result','gain');
    await fill('parameter-json','[{"name":"gain","type":3,"value":0.75}]');await click('parameter-set');await contains('parameter-write-result','true');
    await click('parameter-get');await contains('parameter-result','0.75');await contains('parameter-result','read_only');
    await fill('parameter-json','[{"name":"gain","type":3,"value":-1}]');await click('parameter-set');await contains('parameter-write-result','false');
    await fill('parameter-json','[{"name":"gain","type":3,"value":1}]');await click('parameter-atomic');await contains('parameter-write-result','true');
  });
  await check('lifecycle configure / active / inactive / cleanup',async()=>{
    await click('lifecycle-read');await contains('lifecycle-result','unconfigured');
    for(const [id,state] of [['1','inactive'],['3','active'],['4','inactive'],['2','unconfigured']]) {
      await page.locator('#lifecycle-transition').selectOption(id);await click('lifecycle-change');
      await page.waitForFunction(state=>JSON.parse(document.getElementById('lifecycle-result').textContent).state.label===state,state);
    }
  });
  await check('compressed image / TF / point cloud / diagnostics / stop',async()=>{
    await click('observe-start');await contains('image-state','demo_camera_optical');
    await contains('observation-result','point_step');await contains('observation-result','diagnostics');
    assert(await page.locator('#image-preview').evaluate(img=>img.complete&&img.naturalWidth===96));
    await page.screenshot({path:`${out}/desktop.png`,fullPage:true});
    await click('observe-stop');
  });
  await check('native action type discovery and fields',async()=>{
    await showTool(page,'overview');await page.getByText('原生类型发现与接口字段',{exact:true}).click();
    await page.locator('#interface-kind').selectOption('action');await fill('interface-name','/web_link_demo/fibonacci');
    await click('interface-resolve');await contains('interface-result','Fibonacci');await click('interface-describe');await contains('interface-result','sequence');
  });
  await check('3D URDF / TF / markers / points / path / layers',async()=>{
    await click('scene-start');await contains('scene-state','400 点');await contains('scene-state','2 活动');
    for(const layer of ['robot','tf','markers','cloud','path']){
      await page.locator(`[data-layer="${layer}"]`).uncheck();await page.locator(`[data-layer="${layer}"]`).check();
    }
    await fill('scene-fixed','missing_frame');await click('scene-frame');await contains('scene-state','缺少');
    await fill('scene-fixed','demo_base');await click('scene-frame');await click('scene-camera');
    await page.waitForFunction(()=>!document.querySelector('#scene-state').textContent.includes('缺少'));
    await page.locator('#scene-view').screenshot({path:`${out}/scene.png`});await click('scene-stop');
    assert.equal(await page.locator('#scene-view canvas').count(),0);
  });
  await check('raw depth / explicit units / time series / disposal',async()=>{
    await click('media-depth');await click('media-start');await contains('media-state','96 × 48');
    await fill('media-scale','');await click('media-start');await contains('media-state','requires depthScale');
    await fill('media-scale','0.001');await click('media-start');await contains('media-state','96 × 48');
    await click('plot-start');await contains('plot-state','个样本');
    await fill('plot-fields','missing');await click('plot-start');await contains('plot-state','缺少');
    await click('media-stop');await click('plot-stop');
    assert.equal(await page.locator('#media-view canvas').count(),0);assert.equal(await page.locator('#plot-view canvas').count(),0);
  });
  await check('disconnect / reconnect / disabled commands',async()=>{
    await click('disconnect');await contains('connection-state','已断开');assert(await page.locator('#service-call').isDisabled());
    await click('connect');await contains('connection-state','已连接');
  });
  await check('JSON evidence download',async()=>{
    const download=page.waitForEvent('download');await click('export-log');const file=await download;
    await file.saveAs(`${out}/operations.json`);
    assert(readFileSync(`${out}/operations.json`,'utf8').length>2);
    await click('clear-log');await contains('log','clear-log');
    // The shared command logger retains the clear operation's own OK entry.
    const cleared=page.waitForEvent('download');await click('export-log');
    await (await cleared).saveAs(`${out}/operations-cleared.json`);
    const remaining=JSON.parse(readFileSync(`${out}/operations-cleared.json`,'utf8'));
    assert.equal(remaining.length,1);assert.equal(remaining[0].operation,'clear-log');
  });
  for(const width of [768,390]) {
    await check(`responsive layout ${width}px (not Android device validation)`,async()=>{
      await page.setViewportSize({width,height:900});await page.goto(base);await page.waitForTimeout(400);
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'horizontal overflow');
      await page.screenshot({path:`${out}/layout-${width}.png`,fullPage:true});
      await click('connect');await contains('connection-state','已连接');
      await click('self-check');await page.locator('#check-results[data-complete]').waitFor({timeout:45000});
      assert.equal(await page.locator('#check-results').getAttribute('data-complete'),'passed',await text('check-results'));
    });
  }
  await check('plain LAN HTTP without secure-context APIs',async()=>{
    const port=new URL(base).port || '80';await page.goto(`http://ros2-web-link.lan:${port}`);
    assert.equal(await page.evaluate(()=>isSecureContext),false);
    await click('connect');await contains('connection-state','已连接');
    await click('self-check');await page.locator('#check-results[data-complete]').waitFor({timeout:45000});
    assert.equal(await page.locator('#check-results').getAttribute('data-complete'),'passed',await text('check-results'));
  });
  await check('no page errors',async()=>assert.deepEqual(errors,[]));
  writeFileSync(`${out}/summary.json`,JSON.stringify({base,results,errors,androidDeviceTested:false},null,2));
  console.log(`PASS ${results.length}/${results.length} browser scenarios; evidence: ${out}`);
} finally {await browser.close();}
