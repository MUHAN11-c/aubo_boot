/** Read-only load probe assertions. This module never opens a socket. */
import assert from 'node:assert/strict';

/** Resource counters must be zero; configuration metadata is not a live resource. */
export function assertClientReleased(stats,wire) {
  for(const key of ['subscriptions','publishers','pending','listeners'])assert.equal(stats[key],0,`${key} not released`);
  assert.equal(wire.fragmentBytes,0,'fragment bytes not released');
  assert.equal(wire.fragmentMessages,0,'fragment IDs not released');
}

export function assertStreamProgress(session,now) {
  // Ten seconds covers discovery and the known gaps in this recorded dataset.
  // The same deadline applies to each generation, including before churn.
  for(const key of ['color','depth','cloud']){
    const received=session.receivedAt[key]??session.startedAt;
    assert.ok(now-received<=10000,`client ${session.slot}.${session.generation} ${key} stalled for >10s`);
  }
}
export function assertSlowProgress(state,now) {
  assert.ok(!state.closed&&state.readyState===1,'slow reader closed before test cleanup');
  if(state.paused)return;
  if(state.resumedAt!==null){
    if(state.frames>state.resumeBaseline){state.resumedAt=null;state.resumeChecks++;}
    else assert.ok(now-state.resumedAt<=8000,'slow reader resume stalled for >8s');
  }else assert.ok(now-state.lastFrameAt<=10000,'slow reader stalled for >10s');
}

/** Account for completed RPC attempts independently from the latency series. */
export function recordRpcResult(state,elapsedMs,successful) {
  // The run is <=3600s and every loop sleeps >=100ms. Retaining all <=36001
  // durations is <0.3MiB of numeric data per client and avoids a rolling-window
  // p95 being mistaken for the whole-run result. Reject instead of truncating.
  if(successful&&state.rpc.length>=36001)throw new RangeError('RPC latency sample budget exceeded');
  state.rpcCalls++;
  if(successful)state.rpc.push(elapsedMs);else state.rpcFailed++;
}

/** Return a diagnostic write failure instead of throwing out of a timer. */
export function diagnosticWriteError(write) {
  try{write();return undefined;}
  catch(error){try{return 'diagnostic write failed: '+String(error);}catch{return 'diagnostic write failed';}}
}
