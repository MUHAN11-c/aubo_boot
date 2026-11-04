/** Owns one observation session and the reference-counted logical displays. */
import {SceneView} from '../scene/index.js';
import type {SceneOptions,SceneLayer} from '../scene/index.js';
import type {RosClient} from '../sdk/index.js';
import {errorText} from '../sdk/errors.js';
import {validateWorkspace} from './config.js';
import type {DisplaySpec,WorkspaceSpec} from './config.js';
import type {DisplayInstance,DisplayRegistry} from './registry.js';

export interface WorkbenchContext {
  getObservationClient():RosClient|undefined;
  /** Stop other consumers using incompatible Topic options before starting. */
  beforeResume?():void;
  getDisplayRoot?(id:string):HTMLElement;getSceneRoot?():HTMLElement;
  onDisplayStatus?(id:string,message:string):void;onSceneStatus?(message:string):void;
  onStateChange?():void;
}
type Active={spec:DisplaySpec;instance:DisplayInstance};
const clone=<T>(value:T):T=>JSON.parse(JSON.stringify(value)) as T;
export class WorkbenchController {
  private workspace:WorkspaceSpec;
  private active=new Map<string,Active>();private sceneHost?:SceneView;
  private activeClient?:RosClient;private stateOff?:()=>void;
  private running=false;private disposed=false;private lastError?:string;
  constructor(private context:WorkbenchContext,readonly registry:DisplayRegistry,initial:unknown){this.workspace=validateWorkspace(initial,registry);}
  private assertLive():void {if(this.disposed)throw new Error('Workbench is disposed');}
  private notify():void {try{this.context.onStateChange?.();}catch{/* Consumer observers do not own resources. */}}
  private status(id:string,message:string):void {try{this.context.onDisplayStatus?.(id,message);}catch{/* Isolate UI observers. */}}
  private scene(options:Partial<SceneOptions>&{layers:SceneLayer[]}):SceneView {
    if(!this.sceneHost){
      const root=this.context.getSceneRoot?.();if(!root)throw new Error('A shared scene root is required for spatial displays');
      const workspace=this.workspace;
      this.sceneHost=new SceneView(root,this.activeClient!,{fixedFrame:workspace.fixedFrame,tf:workspace.tf,tfStatic:workspace.tfStatic,useSimTime:workspace.useSimTime,
        robotDescription:'/robot_description',jointStates:'/joint_states',markers:'/visualization_marker_array',pointCloud:'/points',path:'/plan',layers:[],
        onStatus:message=>{try{this.context.onSceneStatus?.(message);}catch{/* UI observer. */}}});
      try{this.sceneHost.start();}catch(error){this.sceneHost.dispose();this.sceneHost=undefined;throw error;}
    }
    return this.sceneHost.createDisplay(options);
  }
  private create(spec:DisplaySpec):void {
    let instance:DisplayInstance|undefined;
    try{
      const definition=this.registry.get(spec.plugin);
      // Spatial factories never use a per-display DOM canvas. All requests route
      // through scene(), whose root owns the single renderer/TF/clock lifetime.
      const root=definition.spatial?this.context.getSceneRoot?.():this.context.getDisplayRoot?.(spec.id);
      instance=definition.create({client:this.activeClient!,root:root as HTMLElement,scene:options=>this.scene(options),onStatus:message=>this.status(spec.id,message)},clone(spec));
      instance.start();this.active.set(spec.id,{spec,instance});this.status(spec.id,'已启动，等待数据');
    }catch(error){try{instance?.dispose();}catch{/* Preserve the start failure. */}this.status(spec.id,`启动失败：${errorText(error)}`);throw error;}
  }
  private stop(id:string):void {
    const active=this.active.get(id);if(!active)return;this.active.delete(id);
    try{active.instance.dispose();}catch(error){this.lastError=`显示释放失败：${errorText(error)}`;}
    this.status(id,'已停止，订阅和数据已释放');
  }
  private releaseUnusedScene():void {
    if(![...this.active.values()].some(active=>this.registry.get(active.spec.plugin).spatial)){
      try{this.sceneHost?.dispose();}finally{this.sceneHost=undefined;}
    }
  }
  resume():void {
    this.assertLive();if(this.running)return;
    let client=this.context.getObservationClient();if(!client?.connected)throw new Error('Connect the observation client first');
    this.context.beforeResume?.();client=this.context.getObservationClient();
    if(!client?.connected)throw new Error('Observation connection changed');
    this.running=true;this.activeClient=client;this.lastError=undefined;
    try{
      this.stateOff=client.onState(state=>{if(state!=='connected')this.pause();});
      for(const spec of this.workspace.displays)if(spec.enabled)this.create(spec);
    }catch(error){this.lastError=errorText(error);this.pause();throw error;}
    this.notify();
  }
  /** Pause retains user checkboxes, but drops streams, temporal data and GL. */
  pause():void {
    this.running=false;this.stateOff?.();this.stateOff=undefined;
    for(const id of [...this.active.keys()])this.stop(id);
    this.releaseUnusedScene();this.activeClient=undefined;this.notify();
  }
  syncConnection():void {
    if(this.running&&(!this.activeClient?.connected||this.activeClient!==this.context.getObservationClient()))this.pause();
  }
  private apply(candidate:WorkspaceSpec):void {
    this.assertLive();const next=validateWorkspace(candidate,this.registry),previous=this.workspace;
    if(!this.running){this.workspace=next;this.notify();return;}
    const changed=[...this.active.values()].filter(active=>!next.displays.some(spec=>spec.enabled&&spec.id===active.spec.id&&JSON.stringify(spec)===JSON.stringify(active.spec)));
    for(const active of changed)this.stop(active.spec.id);
    const created:string[]=[];
    try{
      for(const spec of next.displays)if(spec.enabled&&!this.active.has(spec.id)){this.create(spec);created.push(spec.id);}
      this.workspace=next;
    }catch(error){
      for(const id of created)this.stop(id);
      // Keep unaffected peers alive. Restore replaced instances from the last
      // valid spec; if that fails, pause the complete observation session.
      try{for(const active of changed)this.create(active.spec);}catch{this.pause();}
      this.workspace=previous;this.lastError=errorText(error);throw error;
    }finally{this.releaseUnusedScene();this.notify();}
  }
  add(spec:DisplaySpec):void {const next=this.snapshot();next.displays.push(spec);this.apply(next);}
  remove(id:string):void {const next=this.snapshot();if(!next.displays.some(spec=>spec.id===id))throw new Error('Unknown display');next.displays=next.displays.filter(spec=>spec.id!==id);this.apply(next);}
  update(id:string,changes:Partial<Omit<DisplaySpec,'id'>>):void {
    const next=this.snapshot(),index=next.displays.findIndex(spec=>spec.id===id);if(index<0)throw new Error('Unknown display');
    next.displays[index]={...next.displays[index],...changes,id};this.apply(next);
  }
  setEnabled(id:string,enabled:boolean):void {this.update(id,{enabled});}
  /** Import/config replacement is atomic after validation and always pauses. */
  replace(value:unknown):void {this.assertLive();const next=validateWorkspace(value,this.registry);this.pause();this.workspace=next;this.notify();}
  snapshot():WorkspaceSpec {return clone(this.workspace);}
  private rendererStats(){
    const performance=this.sceneHost?.performanceStats();if(!performance)return;
    // Model ownership belongs to logical displays, not the empty shared host.
    const {robotParseMs:_parse,assetReadyMs:_ready,assetsLoaded:_loaded,assetsTotal:_total,jointStateCache:_joints,...renderer}=performance;
    return renderer;
  }
  private displayStats(active:Active){
    // A structural optional hook works across separately bundled public entries;
    // instanceof would fail for an equivalent SceneView from the scene bundle.
    return {id:active.spec.id,plugin:active.spec.plugin,stats:active.instance.stats?.(),assets:active.instance.assetStats?.()};
  }
  stats(){return {running:this.running,disposed:this.disposed,activeDisplays:this.active.size,spatialDisplays:[...this.active.values()].filter(active=>this.registry.get(active.spec.plugin).spatial).length,
    scene:this.sceneHost?.stats(),renderer:this.rendererStats(),lastError:this.lastError,
    displays:[...this.active.values()].map(active=>this.displayStats(active))};}
  resetCamera():void {this.sceneHost?.resetCamera();}
  dispose():void {if(this.disposed)return;this.pause();this.disposed=true;this.notify();}
}
