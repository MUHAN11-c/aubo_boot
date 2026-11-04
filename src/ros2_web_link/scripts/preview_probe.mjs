/** Explicit pure UI formatting probe; no browser, ROS, socket or DOM engine.
 * Load the page's actual classic-script functions in a VM. Only the text sink
 * is replaced; data traversal, formatting and record retention stay real.
 */
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
const source=readFileSync(process.env.WEB_LINK_PREVIEW_SOURCE??new URL('../web/app.js',import.meta.url),'utf8');
const start=source.indexOf('function streamPreview('),end=source.indexOf('function log(');
assert.ok(start>=0&&end>start,'locate actual page preview functions');
const show=source.match(/^const show = .*$/m)?.[0];assert.ok(show,'locate actual page text sink');
const log=source.slice(end,source.indexOf('function onlineControls(',end));
const projectSource=readFileSync(process.env.WEB_LINK_PROJECT_SOURCE??new URL('../web/project.js',import.meta.url),'utf8');
const projectStart=projectSource.indexOf('function projectSummarize(');
const projectEnd=projectSource.indexOf("\ncommand('project-run'",projectStart);
assert.ok(projectStart>=0&&projectEnd>projectStart,'locate actual project report summary');
const nodes=new Map(),records=[];
const sink=id=>{if(!nodes.has(id))nodes.set(id,{textContent:'',dataset:{}});return nodes.get(id);};
const context=vm.createContext({records,json:value=>JSON.stringify(value,null,2),Date,
  $:sink,document:{getElementById:sink}});
vm.runInContext(source.slice(start,end)+'\n'+show+'\n'+log+'\n'+projectSource.slice(projectStart,projectEnd)+'\nthis.api={streamPreview,show,log,projectSummarize};',context);
const api=context.api;
let passed=0;
function check(name,body){if(process.env.WEB_LINK_PREVIEW_CASE&&!name.includes(process.env.WEB_LINK_PREVIEW_CASE))return;body();passed++;console.log('PASS '+name);}
check('large service log retains a bounded snapshot instead of a raw byte payload',()=>{
  const payload={width:640,height:480,data:new Uint8Array(262144)};
  api.log('service-call','OK',payload);const detail=records.at(-1).detail;
  assert.ok(JSON.stringify(detail).length<=16384,'one retained log detail must stay bounded');
  assert.notEqual(detail,payload);assert.notEqual(detail.data,payload.data);
});
check('large result panel formats byte metadata before JSON serialization',()=>{
  api.show('result',{data:new Uint8Array(921600)});
  const text=nodes.get('result').textContent;assert.ok(text.length<=18000);assert.match(text,/bytes/);assert.match(text,/921600/);
});
check('wide object preview does not read every value before truncating',()=>{
  let reads=0;const object={};
  for(let i=0;i<1000;i++)Object.defineProperty(object,'field'+i,{enumerable:true,get(){reads++;return i;}});
  api.streamPreview(object);assert.ok(reads<=64,`preview read ${reads} fields`);
});
check('branching previews have a total traversal budget',()=>{
  let reads=0;
  const branch=depth=>{if(!depth)return 'leaf';const object={};for(let i=0;i<3;i++)Object.defineProperty(object,'n'+i,{enumerable:true,get(){reads++;return branch(depth-1);}});return object;};
  api.show('branch',branch(8));assert.ok(reads<=512,`preview visited ${reads} branches`);assert.ok(nodes.get('branch').textContent.length<=18000);
});
check('BigInt and circular data remain readable instead of breaking UI callbacks',()=>{
  const object={value:123n};object.self=object;
  assert.doesNotThrow(()=>api.show('circular',object));const text=nodes.get('circular').textContent;assert.match(text,/123/);assert.match(text,/circular|Circular|循环|reference/);
});
check('small standard results preserve their fields and values',()=>{
  const expected={status:4,successful:true,value:{sequence:[0,1,1,2]},note:'桃子 🍑'};
  api.show('small',expected);assert.deepEqual(JSON.parse(nodes.get('small').textContent),expected);
});
check('escaped and Unicode excerpts remain valid JSON within the text budget',()=>{
  const object={};for(let i=0;i<100;i++)object['field'+i]='\0"\\🍑桃'.repeat(200);
  api.show('escaped',object);const text=nodes.get('escaped').textContent;
  assert.ok(text.length<=18000);assert.doesNotThrow(()=>JSON.parse(text));
  api.log('escaped','OK',object);assert.ok(JSON.stringify(records.at(-1).detail,null,2).length<=4096);
});
check('ArrayBuffer and offset DataView report byte metadata',()=>{
  const buffer=new ArrayBuffer(1048576);api.show('buffers',{buffer,view:new DataView(buffer,16,32)});
  const detail=JSON.parse(nodes.get('buffers').textContent);assert.equal(detail.buffer.bytes,1048576);assert.equal(detail.view.bytes,32);
});
check('an unreadable property is isolated from later ordinary fields',()=>{
  const object={};Object.defineProperty(object,'bad',{enumerable:true,get(){throw null;}});object.ready=true;
  assert.doesNotThrow(()=>api.show('getter',object));const detail=JSON.parse(nodes.get('getter').textContent);assert.equal(detail.ready,true);assert.match(detail.bad,/unreadable/);
});
check('retained operation evidence is a snapshot when the original result changes',()=>{
  const result={sum:7,nested:{ready:true}};api.log('snapshot','OK',result);result.sum=999;result.nested.ready=false;
  assert.equal(records.at(-1).detail.sum,7);assert.equal(records.at(-1).detail.nested.ready,true);
});
check('record count and rendered text stay bounded with the newest event visible',()=>{
  records.length=0;
  for(let i=0;i<200;i++)records.push({at:new Date().toISOString(),operation:'event-'+i,result:'OK',detail:{value:i}});
  api.log('event-newest','OK',{value:200});
  assert.equal(records.length,200);assert.match(nodes.get('log').textContent,/event-newest/);assert.ok(nodes.get('log').textContent.length<=18000);
});
function report(evidence,kind='interface'){
  return {title:'preview probe',runId:'local-preview',totals:{PASS:1,FAIL:0,CANCELLED:0,NOT_RUN:0},selectedIds:['x'],elapsedMs:100,
    results:[{id:'x',kind,status:'PASS',interactive:false,label:'probe',evidence}]};
}
check('project report summary is bounded while original evidence remains complete',()=>{
  const original=report({definition:'x'.repeat(200000)});
  api.projectSummarize(original);
  const text=nodes.get('project-report-json').textContent;
  assert.ok(text.length<=18000,`project summary contains ${text.length} characters`);
  assert.doesNotThrow(()=>JSON.parse(text));
  assert.equal(original.results[0].evidence.definition.length,200000);
});
check('degraded Action analysis does not serialize the full evidence a second time',()=>{
  let reads=0;const evidence={result:{degraded:true}};
  Object.defineProperty(evidence,'large',{enumerable:true,get(){reads++;return 'x'.repeat(200000);}});
  api.projectSummarize(report(evidence,'action'));
  assert.match(nodes.get('project-analysis').textContent,/Action 返回 degraded=true/);
  assert.ok(reads<=1,`analysis read large evidence ${reads} times`);
});
check('only the Action result degraded flag controls the quality warning',()=>{
  api.projectSummarize(report({comparison:{degraded:true},result:{degraded:false}},'action'));
  assert.doesNotMatch(nodes.get('project-analysis').textContent,/Action 返回 degraded=true/);
});
console.log(`PASS ${passed} preview groups; actual page functions, no live UI or ROS`);
