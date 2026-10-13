const assert = require('node:assert/strict');
const test = require('node:test');
const fs = require('node:fs');
const file = require('node:path').join(__dirname, '../web/workbench.js');
const wb = fs.existsSync(file) ? require(file) : {};
const state = (epoch = 8, ids = ['a', 'b']) => ({debug: {enabled: true, motion_enabled: true}, perception: {targets: {scene_epoch: epoch, observations: ids.map(target_id => ({target_id}))}}});
test('manual target survives selected_target polling, epoch invalidates it', () => {
  const selection = wb.selectionFor(state(), {target_id: 'b', scene_epoch: 8});
  assert.equal(selection.target_id, 'b');
  assert.equal(wb.selectionFor(state(9), selection).target_id, '');
  assert.equal(wb.selectionFor(state(8, ['a']), selection).target_id, '');
});
test('confirmation guard rejects vanished targets and changed epochs', () => {
  const context = {target_id: 'a', scene_epoch: 8};
  assert.equal(wb.contextReason(state(), context), '');
  assert.match(wb.contextReason(state(9), context), /场景/);
  assert.match(wb.contextReason(state(8, ['b']), context), /目标/);
});
test('motion classification follows backend and allowed negative budgets remain diagnostic', () => {
  const s = state(); s.reconstruction = {grasp_decision: {allowed: true, radial_budget_m: -0.01}};
  assert.equal(wb.blockReason('execute_action', {mode: 'PREVIEW'}, s), '');
  s.debug.motion_enabled = false;
  assert.match(wb.blockReason('execute_action', {mode: 'FULL'}, s), /动臂/);
  assert.equal(wb.blockReason('cancel', {target: 'execute_action'}, s), '');
  assert.match(wb.blockReason('arm_service', {data: false}, s), /动臂/);
});
test('operation lock prevents duplicates and lets independent cancellation through', async () => {
  let resolve, sends = 0;
  const lock = wb.operationLock();
  const first = lock.run('execute_action', () => { sends++; return new Promise(r => {resolve = r;}); });
  assert.equal(lock.has('execute_action'), true);
  assert.equal(await lock.run('execute_action', () => {sends++;}), null);
  await lock.run('cancel:execute_action', async () => {sends++;});
  assert.equal(sends, 2);
  resolve(1); await first;
  assert.equal(lock.has('execute_action'), false);
});
test('new experiments generate safe unique request ids', () => {
  assert.match(wb.requestId(), /^web_\d+_[0-9a-f]{8}$/);
  assert.notEqual(wb.requestId(), wb.requestId());
});
test('stale state blocks operations while cancellation remains available', () => {
  const s = state(); s.ui = {fresh: false};
  assert.match(wb.blockReason('recon_query_service', {}, s), /陈旧/);
  assert.equal(wb.blockReason('cancel', {target:'build_action'}, s), '');
  assert.equal(wb.blockReason('control_service', {command:'CANCEL_NOW'}, s), '');
});
function harness(submit, onContextChange) {
  let current = state();
  const elements = new Map(), handlers = new Map();
  function element(selector) {
    if (!elements.has(selector)) elements.set(selector, {innerHTML:'',textContent:'',value:'',checked:true, dataset:{},setAttribute(){}});
    return elements.get(selector);
  }
  const container = {innerHTML:'', querySelector:element, querySelectorAll:()=>[], contains:()=>true,
    addEventListener:(name,fn)=>handlers.set(name,fn),removeEventListener:name=>handlers.delete(name)};
  const instance = wb.create(container,{getState:()=>current,submit,onContextChange});
  element('[data-wb-field="request"]').value = 'web_1_deadbeef';
  element('[data-wb-field="scene"]').value = 'lab';
  const click = async (dataset, confirm = false) => {
    const el = {dataset,disabled:false,hasAttribute:key=>key==='data-wb-step' ? 'wbStep' in dataset : key==='data-wb-new' ? 'wbNew' in dataset : false,querySelector:()=>({textContent:'执行'})};
    if (confirm) el.dataset.wbConfirm='1';
    await handlers.get('click')({target:{closest:()=>el}});
  };
  return {instance,elements,click,container,setState(next){current=next;},select(id){element('[data-wb-field="target"]').value=id;handlers.get('change')({target:element('[data-wb-field="target"]')});}};
}
test('step navigation sends no requests and preserves visualization host DOM', async () => {
  let sends=0;
  const h=harness(async()=>{sends++;});
  const initialHTML=h.container.innerHTML;
  for(let i=0;i<6;i++) await h.click({wbStep:String(i)});
  assert.equal(sends,0);
  assert.equal(h.container.innerHTML,initialHTML);
  h.instance.dispose();
});
test('submission captures target before confirmation and guard prevents stale send', async () => {
  let sends=0, guardReason=''; let h;
  h=harness(async(action,payload,confirmation,guard)=>{
    assert.equal(action,'execute_action'); assert.equal(payload.target_id,'b');
    assert.match(confirmation,/目标 b/);
    h.setState(state(9));
    guardReason=guard();
    if(!guardReason) sends++;
    return null;
  });
  h.select('b');
  await h.click({wbAction:'execute_action',wbPayload:'{"mode":"PREGRASP_ONLY"}'},true);
  assert.match(guardReason,/场景/);
  assert.equal(sends,0);
  h.instance.dispose();
});
test('polling preserves user input and does not select a new epoch target', () => {
  const h=harness(async()=>{}); h.select('b');
  h.elements.get('[data-wb-field="request"]').value='operator_input';
  h.instance.update(state(8));
  assert.equal(h.elements.get('[data-wb-field="target"]').value,'b');
  assert.equal(h.elements.get('[data-wb-field="request"]').value,'operator_input');
  h.instance.update(state(9));
  assert.equal(h.elements.get('[data-wb-field="target"]').value,'');
  assert.equal(h.elements.get('[data-wb-field="request"]').value,'operator_input');
  h.instance.dispose();
});
test('manual non-current target publishes immutable context, unchanged polling stays quiet', () => {
  let changes=0;
  const h=harness(async()=>{},()=>{changes++;});
  const s=state(); s.perception.targets.selected_target_id='a'; h.setState(s); h.instance.update(s);
  h.select('b');
  assert.equal(changes,1);
  const context=h.instance.context();
  assert.deepEqual(context,{target_id:'b',scene_epoch:8});
  context.target_id='tampered';
  assert.equal(h.instance.context().target_id,'b');
  h.instance.update(s); h.instance.update(s);
  assert.equal(changes,1);
  h.instance.dispose();
});
test('epoch invalidation publishes empty context once and new experiment refreshes it', async () => {
  let changes=0;
  const h=harness(async()=>{},()=>{changes++;}); h.select('b');
  const next=state(9); h.setState(next); h.instance.update(next);
  assert.deepEqual(h.instance.context(),{target_id:'',scene_epoch:9});
  assert.equal(changes,2);
  h.instance.update(next); assert.equal(changes,2);
  await h.click({wbNew:''}); assert.equal(changes,3);
  h.instance.dispose();
});
