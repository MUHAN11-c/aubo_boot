/** Native viewport regression; the ROS fixture is the isolated self-test only. */
import assert from 'node:assert/strict';
import {mkdirSync,writeFileSync} from 'node:fs';
import {chromium} from 'playwright';
import {showConnection} from './browser_controls.mjs';
const out=process.env.WEB_LINK_EVIDENCE;mkdirSync(out,{recursive:true});
const browser=await chromium.launch({headless:false,executablePath:process.env.CHROME_PATH,args:['--no-proxy-server']});
const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];
page.on('pageerror',e=>errors.push(e.message));page.setDefaultTimeout(15000);
const wb=id=>page.locator(`[data-wb="${id}"]`);
let failure,initial,before,after,events;
try{
 await page.goto(process.argv[2]+'/#workbench');await page.locator('#workbench-load').click();await wb('run').waitFor();
 await showConnection(page);await page.locator('#connect').click();await page.waitForFunction(()=>client?.connected);await page.locator('#connection-close').click();
 await wb('tab-graph').click();await wb('graph-refresh').click();await page.waitForFunction(()=>!!workbenchView.stats().graphSnapshot&&!workbenchView.stats().graphRefreshing);
 await wb('graph-search').fill('/web_link_demo/sensor');await wb('graph').hover();await page.waitForTimeout(300);
 initial=await page.evaluate(()=>{window.__zoomEvents=[];workbenchView.graphView.cy.on('zoom pan',()=>{if(__zoomEvents.length<100)__zoomEvents.push({at:performance.now(),zoom:workbenchView.graphView.cy.zoom(),pan:{...workbenchView.graphView.cy.pan()}});});return {zoom:workbenchView.graphView.cy.zoom(),pan:workbenchView.graphView.cy.pan()};});
 for(let i=0;i<3;i++){await page.mouse.wheel(0,-120);await page.waitForTimeout(100);}await page.waitForTimeout(500);
 // Inspect the actual Cytoscape camera after trusted wheel input; do not set it
 // through internal state. Public stats expose zoom from the fixed version.
 before=await page.evaluate(()=>({zoom:workbenchView.graphView.cy.zoom(),pan:workbenchView.graphView.cy.pan()}));
 assert(before.zoom>initial.zoom*1.05,'wheel precondition: graph must actually zoom before checking preservation');
 await wb('tab-space').click();await page.waitForTimeout(200);await wb('tab-graph').click();await page.waitForTimeout(500);
 after=await page.evaluate(()=>({zoom:workbenchView.graphView.cy.zoom(),pan:workbenchView.graphView.cy.pan()}));
 assert(Math.abs(before.zoom-after.zoom)<.001,'tab change must preserve the user graph zoom');
 assert(Math.abs(before.pan.x-after.pan.x)<1&&Math.abs(before.pan.y-after.pan.y)<1,'tab change must preserve graph pan');
 if(await wb('graph-zoom-reset').count()){
  await wb('graph-zoom-reset').click();assert.equal(await page.evaluate(()=>workbenchView.stats().graph.zoom),1);
  await wb('graph-zoom-in').click();assert(await page.evaluate(()=>workbenchView.stats().graph.zoom)>1);
  await wb('graph-zoom-out').click();assert(Math.abs(await page.evaluate(()=>workbenchView.stats().graph.zoom)-1)<.001);
  await wb('graph-layout').selectOption('grid');await wb('graph-search').fill('');await page.screenshot({path:out+'/grid.png',fullPage:true});
 }
 assert.deepEqual(errors,[]);
}catch(error){failure=String(error);await page.screenshot({path:out+'/failure.png',fullPage:true}).catch(()=>{});}
finally{events=await page.evaluate(()=>window.__zoomEvents).catch(()=>null);writeFileSync(out+'/result.json',JSON.stringify({status:failure?'FAIL':'PASS',failure,initial,before,after,events,errors},null,2));await page.close();await browser.close();}
if(failure)throw Error(failure);console.log('PASS graph native zoom survives hidden pane and controls work');
