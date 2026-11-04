/* Copyright 2026 ros2_web_link contributors; SPDX-License-Identifier: Apache-2.0 */
// CPU-only checks of our observer boundary and bounded summaries. Synthetic
// entries do not establish actual browser input or presentation latency.
import assert from 'node:assert/strict';
import {createEventTiming} from '../dist/scene/event-timing.js';

const previous=Object.getOwnPropertyDescriptor(globalThis,'PerformanceObserver');
const instances=[];
class Observer {
  static supportedEntryTypes=['event','longtask'];
  constructor(callback){this.callback=callback;this.disconnected=false;instances.push(this);}
  observe(options){this.options=options;}
  disconnect(){this.disconnected=true;}
  emit(entries){this.callback({getEntries:()=>entries},this);}
}
const entry=(duration=80,startTime=100,processingStart=125,processingEnd=165)=>({
  name:'click',entryType:'event',duration,startTime,processingStart,processingEnd,
  cancelable:true,interactionId:1,target:null,toJSON(){return {};},
});
try {
  Object.defineProperty(globalThis,'PerformanceObserver',{configurable:true,writable:true,value:Observer});
  const disabled=createEventTiming();
  assert.equal(disabled.snapshot().enabled,false);
  assert.equal(disabled.snapshot().supported,null);
  assert.equal(disabled.snapshot().reportedDurationP95Ms,null);
  assert.equal(instances.length,0,'disabled instrumentation must not allocate an observer');
  disabled.dispose();

  const active=createEventTiming(true),observer=instances.at(-1);
  assert.deepEqual(observer.options,{type:'event',durationThreshold:16,buffered:false});
  const message=entry();Object.defineProperty(message,'target',{get(){throw Error('Do not read DOM target');}});
  observer.emit([message,entry(120,1000,1050,1100)]);
  let stats=active.snapshot();
  assert.equal(stats.reportedEvents,2);assert.equal(stats.retainedSamples,2);
  assert.equal(stats.reportedDurationP95Ms,120);
  assert.equal(stats.maxInputDelayMs,50);assert.equal(stats.maxProcessingMs,50);
  assert.equal(stats.maxDurationMs,120);assert.equal(stats.requestedMinimumDurationMs,16);
  assert.equal(stats.specDurationResolutionMs,8);

  observer.emit([entry(NaN),entry(Infinity),entry(8),entry(80,-1),entry(80,100,90,120),entry(80,100,120,110),{...entry(),entryType:'longtask'}]);
  stats=active.snapshot();assert.equal(stats.reportedEvents,2);assert.equal(stats.ignoredEntries,6);

  observer.emit([entry(10000)]);
  observer.emit(Array.from({length:240},()=>entry(24,0,1,2)));
  stats=active.snapshot();
  assert.equal(stats.reportedEvents,243);assert.equal(stats.retainedSamples,120);
  assert.equal(stats.reportedDurationP95Ms,24,'p95 is the retained reported-event window');
  assert.equal(stats.maxDurationMs,10000,'historical maximum survives window rollover');
  active.dispose();active.dispose();
  observer.emit([entry(20000)]);
  assert.equal(active.snapshot().reportedEvents,243);
  assert.equal(active.snapshot().disposed,true);assert.equal(observer.disconnected,true);

  Object.defineProperty(globalThis,'PerformanceObserver',{configurable:true,writable:true,value:undefined});
  const unavailable=createEventTiming(true);
  assert.equal(unavailable.snapshot().supported,false);
  assert.equal(unavailable.snapshot().reportedDurationP95Ms,null);
  unavailable.dispose();

  class Legacy extends Observer {static supportedEntryTypes=undefined;}
  Object.defineProperty(globalThis,'PerformanceObserver',{configurable:true,writable:true,value:Legacy});
  let legacy;
  assert.doesNotThrow(()=>{legacy=createEventTiming(true);},'missing event capability list must degrade to unavailable');
  assert.equal(legacy.snapshot().supported,false);legacy.dispose();

  class Rejected extends Observer {observe(){throw Error('observer rejected');}}
  Object.defineProperty(globalThis,'PerformanceObserver',{configurable:true,writable:true,value:Rejected});
  const rejected=createEventTiming(true);
  assert.equal(rejected.snapshot().supported,false);
  assert.match(rejected.snapshot().observerError,/observer rejected/);
  assert.equal(instances.at(-1).disconnected,true);rejected.dispose();
} finally {
  if(previous)Object.defineProperty(globalThis,'PerformanceObserver',previous);
  else delete globalThis.PerformanceObserver;
}
console.log('PASS 7 Event Timing boundary groups: disabled, callback summaries, invalid entries, bounded window/disposal, unavailable/legacy and rejected observer');
