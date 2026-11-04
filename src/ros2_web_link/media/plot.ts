/** Bounded time series adapter around upstream uPlot, not a plotting framework. */
import uPlot from 'uplot';
import type {RosClient,TopicOptions} from '../sdk/index.js';
export interface PlotOptions {topic:string;type:string;fields:string[];maxPoints?:number;subscription?:TopicOptions;onStatus?:(message:string)=>void}
/** Plain field paths only (e.g. position.0); never evaluate an input as JS. */
export function numericField(message:unknown,path:string):number|undefined {
  if(!/^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z0-9_]+)*$/.test(path)) throw new TypeError('Use a plain field path such as position.0');
  let value:unknown=message;
  for(const key of path.split('.'))value=value && typeof value==='object'?(value as Record<string,unknown>)[key]:undefined;
  return typeof value==='number'&&Number.isFinite(value)?value:undefined;
}
export class SeriesPlot {
  private off?:()=>void;private plot:uPlot;private rows:number[][];private timer:ReturnType<typeof setInterval>;private resize:ResizeObserver;private dirty=false;
  private disposed=false;private offState:()=>void;
  constructor(private root:HTMLElement,private client:RosClient,private options:PlotOptions) {
    if(options.fields.length<1||options.fields.length>8)throw new RangeError('Choose one to eight numeric fields');
    if(!Number.isSafeInteger(options.maxPoints??600)||(options.maxPoints??600)<2||(options.maxPoints??600)>100000)throw new RangeError('Invalid plot capacity');
    this.rows=Array.from({length:options.fields.length+1},()=>[]);
    this.plot=new uPlot({width:Math.max(200,root.clientWidth),height:240,
      series:[{},...options.fields.map((field,i)=>({label:field,stroke:['#256444','#ba6f35','#306a9c','#9b436e','#70743c','#45624e','#a75943','#4c61a1'][i]}))],
      scales:{x:{time:true}},axes:[{stroke:'#67776c'},{stroke:'#67776c'}]},this.rows as uPlot.AlignedData,root);
    this.resize=new ResizeObserver(()=>this.plot.setSize({width:Math.max(200,root.clientWidth),height:240}));this.resize.observe(root);
    this.timer=setInterval(()=>{if(this.dirty){this.plot.setData(this.rows as uPlot.AlignedData);this.dirty=false;}},100);
    this.offState=client.onState(state=>{if(state!=='connected'){this.rows.forEach(row=>row.length=0);this.dirty=true;}});
  }
  start():void {
    if(this.disposed)throw new Error('Plot is disposed');if(this.off)return;
    this.off=this.client.subscribe<Record<string,unknown>>(this.options.topic,this.options.type,(message,info)=>{
      try {
        const values=this.options.fields.map(path=>numericField(message,path));if(values.some(v=>v===undefined)){this.options.onStatus?.('消息缺少所选数值字段');return;}
        const stamp=(message.header as {stamp?:{sec:number;nanosec:number}}|undefined)?.stamp;
        const time=stamp?stamp.sec+stamp.nanosec/1e9:info.receivedAt/1000;
        const last=this.rows[0].at(-1);if(last!==undefined && time<last)this.rows.forEach(row=>row.length=0);
        if(last===time)return;
        [time,...values as number[]].forEach((v,i)=>{this.rows[i].push(v);if(this.rows[i].length>(this.options.maxPoints??600))this.rows[i].shift();});
        this.dirty=true;this.options.onStatus?.(`${this.rows[0].length} 个样本 · ${stamp?'ROS header 时间':'浏览器接收时间'}`);
      } catch(error){this.options.onStatus?.((error as Error).message);}
    },{qos:'sensor',...this.options.subscription});
  }
  stats(){return {samples:this.rows[0].length,maxPoints:this.options.maxPoints??600,fields:[...this.options.fields]};}
  dispose():void {if(this.disposed)return;this.disposed=true;this.off?.();this.offState();clearInterval(this.timer);this.resize.disconnect();this.plot.destroy();this.rows.forEach(row=>row.length=0);}
}
