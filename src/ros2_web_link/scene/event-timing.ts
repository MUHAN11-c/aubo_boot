/** Optional page-level Event Timing. This is neither INP nor GPU/continuous
 * pointer latency. Browsers omit short events and quantize duration to 8ms.
 * Keep only numbers; never retain PerformanceEntry objects or DOM targets.
 */
import {errorText} from '../sdk/errors.js';

const MINIMUM_DURATION_MS=16,SAMPLE_LIMIT=120;
export function createEventTiming(enabled=false){
  let observer:PerformanceObserver|undefined,supported:boolean|null=null;
  let disposed=false,reportedEvents=0,ignoredEntries=0,observerError:string|undefined;
  let maxInputDelayMs:number|null=null,maxProcessingMs:number|null=null,maxDurationMs:number|null=null;
  const durations:number[]=[];
  if(enabled){
    supported=false;
    if(typeof PerformanceObserver!=='undefined'&&PerformanceObserver.supportedEntryTypes?.includes('event')){
      try{
        observer=new PerformanceObserver(list=>{
          if(disposed)return;
          for(const entry of list.getEntries()){
            if(entry.entryType!=='event')continue;
            const event=entry as PerformanceEventTiming;
            const {duration,startTime,processingStart,processingEnd}=event;
            if(![duration,startTime,processingStart,processingEnd].every(Number.isFinite)||
                duration<MINIMUM_DURATION_MS||startTime<0||processingStart<startTime||processingEnd<processingStart){
              ignoredEntries++;continue;
            }
            reportedEvents++;durations.push(duration);if(durations.length>SAMPLE_LIMIT)durations.shift();
            maxInputDelayMs=Math.max(maxInputDelayMs??0,processingStart-startTime);
            maxProcessingMs=Math.max(maxProcessingMs??0,processingEnd-processingStart);
            maxDurationMs=Math.max(maxDurationMs??0,duration);
          }
        });
        // Future events only: buffered history has a different default threshold.
        // Event Timing extends this dictionary; TS 7.0.2 omits that member.
        const options:PerformanceObserverInit&{durationThreshold:number}={
          type:'event',durationThreshold:MINIMUM_DURATION_MS,buffered:false};
        observer.observe(options);
        supported=true;
      }catch(error){
        observerError=errorText(error).slice(0,256);observer?.disconnect();observer=undefined;
      }
    }
  }
  return {
    snapshot(){
      const sorted=[...durations].sort((a,b)=>a-b);
      return {enabled,supported,disposed,scope:'page-reported-discrete-events',
        requestedMinimumDurationMs:MINIMUM_DURATION_MS,specDurationResolutionMs:8,sampleLimit:SAMPLE_LIMIT,
        limitations:'仅浏览器报告的离散事件；不含连续移动/滚轮，不是 INP 或 GPU 耗时',
        reportedEvents,retainedSamples:durations.length,ignoredEntries,
        reportedDurationP95Ms:sorted.length?sorted[Math.ceil(sorted.length*.95)-1]:null,
        maxInputDelayMs,maxProcessingMs,maxDurationMs,observerError};
    },
    dispose(){if(disposed)return;disposed=true;observer?.disconnect();observer=undefined;},
  };
}
