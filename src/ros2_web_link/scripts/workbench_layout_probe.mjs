/** Explicit desktop layout/keyboard smoke against the isolated no-hardware demo.
 * These checks complement data-flow probes; viewport sizing is not device testing. */
import assert from 'node:assert/strict';
import {mkdirSync,writeFileSync} from 'node:fs';
import {chromium} from 'playwright';
import {showTool,showConnection} from './browser_controls.mjs';

if(!process.argv[2])throw Error('Pass an isolated self_test.launch.py HTTP URL');
const out=process.env.WEB_LINK_EVIDENCE??'/tmp/ros2-web-layout';mkdirSync(out,{recursive:true});
const browser=await chromium.launch({headless:false,executablePath:process.env.CHROME_PATH??'/usr/bin/google-chrome',args:['--no-proxy-server']});
const page=await browser.newPage({viewport:{width:1440,height:1000}}),results=[],errors=[];
page.setDefaultTimeout(15000);page.on('pageerror',error=>errors.push(error.message));
const wb=id=>page.locator(`[data-wb="${id}"]`);
const check=async(name,fn)=>{await fn();results.push(name);console.log('PASS',name);};
const overflow=async()=>assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'viewport has horizontal overflow');
try{
  await page.goto(`${process.argv[2]}/#workbench`);await page.locator('#workbench-load').click();await wb('run').waitFor();
  await check('tool navigation and offline workbench are clear without connecting',async()=>{
    assert.equal(await page.locator('main > section:visible').count(),1);
    assert.equal(await page.locator('.rail nav [aria-current=page]').getAttribute('href'),'#workbench');
    assert(await wb('scene-empty').isVisible());assert(await wb('run').isDisabled());
    await page.screenshot({path:`${out}/offline.png`,fullPage:true});
  });
  await check('connection drawer is available from the workbench and Escape restores focus',async()=>{
    await showConnection(page);await page.locator('#ws-url').focus();await page.keyboard.press('Escape');
    assert(!await page.locator('#connection-panel').isVisible());assert(await page.locator('#connection-toggle').evaluate(node=>node===document.activeElement));
    await showConnection(page);await page.locator('#connect').click();await page.waitForFunction(()=>document.getElementById('connection-state').textContent.includes('已连接'));
    await page.locator('#connection-close').click();
  });
  await check('all display templates stay legible beside the live spatial viewport',async()=>{
    for(const id of ['robot','tf','markers','cloud','path','image','depth','plot'])await page.locator(`[data-display-id="${id}"] input`).check();
    await wb('run').click();await page.waitForFunction(()=>{const s=workbenchView.stats().controller.scene;return s?.robotLoaded&&s.points>0;});
    assert(!await wb('scene-empty').isVisible());const box=await wb('scene').boundingBox();assert(box.width>600&&box.height>400);
    const list=await wb('displays').evaluate(node=>({height:node.clientHeight,scroll:node.scrollHeight}));
    assert(list.scroll<=list.height+2,'default display list should fit without an extra scroll');
    await page.screenshot({path:`${out}/desktop-workbench.png`,fullPage:true});
  });
  await check('keyboard tabs preserve running displays and expose graph legend and details',async()=>{
    const before=await page.evaluate(()=>workbenchView.stats().controller.activeDisplays);
    await wb('tab-space').focus();await page.keyboard.press('ArrowRight');assert.equal(await wb('tab-graph').getAttribute('aria-selected'),'true');
    await wb('graph-refresh').click();await page.waitForFunction(()=>!!workbenchView.stats().graphSnapshot&&!workbenchView.stats().graphRefreshing);
    assert.equal(await page.evaluate(()=>workbenchView.stats().controller.activeDisplays),before);
    await wb('graph-search').fill('/__no_matching_entity__');assert(await wb('graph-empty').isVisible());
    await wb('graph-search').fill('/web_link_demo/sensor');assert.equal(await page.evaluate(()=>workbenchView.stats().graph.topics),1);
    const topic=await page.evaluate(()=>workbenchView.stats().graphSnapshot.topics.find(item=>item.name==='/web_link_demo/sensor').id);
    await wb('selection').selectOption(topic);await wb('focus').click();assert.match(await wb('selection-detail').textContent(),/发布节点/);assert.match(await wb('selection-detail').textContent(),/订阅节点/);
    await page.screenshot({path:`${out}/desktop-graph.png`,fullPage:true});
    await wb('use-topic').click();assert.equal(await wb('tab-space').getAttribute('aria-selected'),'true');assert.equal(await wb('add-topic').inputValue(),'/web_link_demo/sensor');
    assert(await wb('add-topic').isVisible());await wb('add-panel').locator(':scope > summary').click();
  });
  await check('interface tab presents read-only type definitions separately from the graph',async()=>{
    await wb('tab-interfaces').click();await wb('endpoint').selectOption('/web_link_demo/add_two_ints');await wb('inspect').click();
    await page.waitForFunction(()=>document.querySelector('[data-wb="interface"]').textContent.includes('int64'));
    assert(await wb('interface-state').isVisible());await page.screenshot({path:`${out}/desktop-interfaces.png`,fullPage:true});
  });
  await check('every tool page has one visible panel and a matching active navigation item',async()=>{
    const ids=await page.locator('.rail nav a').evaluateAll(nodes=>nodes.map(node=>node.hash.slice(1)));
    for(const id of ids){await showTool(page,id);assert.equal(await page.locator('main > section:visible').count(),1);assert.equal(await page.locator('main > section:visible').getAttribute('id'),id);await overflow();}
    await page.goBack();await page.locator('main > section#media').waitFor({state:'visible'});assert.equal(await page.locator('main > section:visible').getAttribute('id'),'media');
    await showTool(page,'workbench');await wb('tab-space').click();
  });
  for(const width of [1280,768,390])await check(`workbench panes and connection controls fit ${width}px`,async()=>{
    await page.setViewportSize({width,height:900});
    for(const pane of ['space','graph','interfaces']){await wb(`tab-${pane}`).click();await overflow();}
    await wb('tab-space').click();await page.screenshot({path:`${out}/workbench-${width}.png`,fullPage:true});
    await showConnection(page);await overflow();assert(await page.locator('#connect').isVisible());await page.locator('#connection-close').click();
  });
  await check('disconnect leaves a truthful paused state and marks interface data stale',async()=>{
    await showConnection(page);await page.locator('#disconnect').click();await page.locator('#connection-close').click();
    await page.waitForFunction(()=>!workbenchView.stats().controller.running);
    assert.match(await wb('status').textContent(),/暂停/);assert.equal(await page.locator('[data-wb="scene"] canvas').count(),0);
    await wb('tab-interfaces').click();assert.match(await wb('interface-state').textContent(),/过期/);assert(await wb('inspect').isDisabled());assert.deepEqual(errors,[]);
  });
  writeFileSync(`${out}/result.json`,JSON.stringify({status:'PASS',results,errors,physicalTabletTested:false},null,2));
}catch(error){await page.screenshot({path:`${out}/failure.png`,fullPage:true}).catch(()=>{});writeFileSync(`${out}/result.json`,JSON.stringify({status:'FAIL',results,errors,error:String(error)},null,2));throw error;}
finally{await page.close();await browser.close();}
