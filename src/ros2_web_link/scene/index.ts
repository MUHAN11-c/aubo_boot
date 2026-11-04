/** Optional read-only 3D component. Core transport never imports this module. */
import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import {OBJLoader} from 'three/examples/jsm/loaders/OBJLoader.js';
import URDFLoader from 'urdf-loader';
import type {URDFRobot,URDFVisual} from 'urdf-loader';
import type {RosClient,TopicOptions} from '../sdk/index.js';
import {errorText} from '../sdk/errors.js';
import {TfBuffer,nanoseconds} from './tf.js';
import type {RosStamp,FrameTransform} from './tf.js';
import {decodePointCloud} from './pointcloud.js';
import {createShaderChecks} from './shaders.js';
import {createEventTiming} from './event-timing.js';
import type {ShaderDiagnostics} from './shaders.js';
export {TfBuffer,decodePointCloud};
export type {RosStamp,FrameTransform,ShaderDiagnostics};

type Vec={x:number;y:number;z:number};
type Pose={position:Vec;orientation:Vec&{w:number}};
type Header={frame_id:string;stamp:RosStamp};
type Marker={header:Header;ns:string;id:number;type:number;action:number;pose:Pose;scale:Vec;color:Vec&{r:number;g:number;b:number;a:number};
  points:Vec[];colors:{r:number;g:number;b:number;a:number}[];text:string;mesh_resource:string;frame_locked:boolean;lifetime:{sec:number;nanosec:number}};
export type SceneLayer='robot'|'tf'|'markers'|'cloud'|'path';
/** Counts are summed across logical displays; shared TF frames are counted once. */
export interface SceneStats {
  useSimTime:boolean;clockReady:boolean;clockSeconds:number;clockNanoseconds:string;
  robotLoaded:boolean;joints:number;movableJoints:number;frames:number;markers:number;
  points:number;pathPoints:number;sharedDisplays:number;
}
export interface SceneOptions {
  fixedFrame:string;robotDescription:string;jointStates:string;tf:string;tfStatic:string;markers:string;pointCloud:string;path:string;
  /** Explicit ROS simulation clock. Default false follows system time; true
   * starts at ROS zero until /clock arrives and never extrapolates while paused.
   */
  useSimTime?:boolean;
  maxPoints?:number;maxMarkers?:number;assetBase?:string;onStatus?:(message:string)=>void;
  /** Reuse SDK subscription options; defaults preserve small-stream sharing. */
  pointCloudOptions?:TopicOptions;jointStatesOptions?:TopicOptions;
  robotDescriptionOptions?:TopicOptions;markersOptions?:TopicOptions;pathOptions?:TopicOptions;
  /** Opt-in CPU timing of public shader API calls; no extra GL queries. */
  profileShaders?:boolean;
  /** Opt-in page-level reported discrete-event timing; not GPU or INP. */
  profileInteractions?:boolean;
  /** Production mode validates failed links; verbose also queries all logs. */
  shaderDiagnostics?:ShaderDiagnostics;
  /** Initial subscription/display mask. Omitted retains the legacy five layers. */
  layers?:SceneLayer[];
}
type Resource={object:THREE.Object3D;header:Header;pose?:Pose;latest?:boolean;expires?:bigint;placedFrame?:string;visualKey?:string;namespace?:string};
// JointState may multiplex other models, and may arrive before the URDF. Keep
// only a bounded recent tail of unmapped names; loaded model values are scoped
// to that URDF. Name length also bounds retained string memory, not just count.
const MAX_PENDING_JOINTS=1024,MAX_PENDING_JOINT_NAME=1024;

/** Update GPU attributes in place; changing topology releases only geometry.
 * Materials own shader program references and must outlive ordinary samples.
 */
function updateGeometry(geometry:THREE.BufferGeometry,positions:Float32Array,colors?:Float32Array):THREE.BufferGeometry {
  const attribute=geometry.getAttribute('position'),color=geometry.getAttribute('color');
  if(!attribute||attribute.array.length!==positions.length||!!color!==!!colors){
    geometry.dispose();geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(positions,3).setUsage(THREE.DynamicDrawUsage));
    if(colors)geometry.setAttribute('color',new THREE.BufferAttribute(colors,3).setUsage(THREE.DynamicDrawUsage));
  }else{
    attribute.array.set(positions);attribute.needsUpdate=true;
    if(color&&colors){color.array.set(colors);color.needsUpdate=true;}
  }
  geometry.boundingSphere=null;geometry.boundingBox=null;
  return geometry;
}

/** Share a disposal set so retained or shared materials/textures survive once. */
function releaseMaterial(material:THREE.Material,disposed:Set<unknown>):void {
  if(disposed.has(material))return;disposed.add(material);
  for(const value of Object.values(material))if(value instanceof THREE.Texture&&!disposed.has(value)){disposed.add(value);value.dispose();}
  material.dispose();
}
/** Dispose GPU objects, including textures; removing from a Scene alone is insufficient. */
function release(object:THREE.Object3D):void {
  const disposed=new Set<unknown>();
  object.traverse(child=>{
    const mesh=child as THREE.Mesh;
    if(mesh.geometry&&!disposed.has(mesh.geometry)){disposed.add(mesh.geometry);mesh.geometry.dispose();}
    if(mesh.material)for(const material of Array.isArray(mesh.material)?mesh.material:[mesh.material]) {
      releaseMaterial(material,disposed);
    }
  });object.removeFromParent();
}
function position(object:THREE.Object3D,pose:Pose|FrameTransform):void {
  const p='translation' in pose?pose.translation:pose.position;
  const q='rotation' in pose?pose.rotation:pose.orientation;
  if(![p.x,p.y,p.z,q.x,q.y,q.z,q.w].every(Number.isFinite))throw new TypeError('Pose contains a non-finite value');
  if(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1e-12)throw new TypeError('Pose quaternion is zero');
  object.position.set(p.x,p.y,p.z);object.quaternion.set(q.x,q.y,q.z,q.w).normalize();
}

/** One DOM canvas with URDF/JointState, TF, MarkerArray, PointCloud2 and Path layers.
 * No joint command, FK/IK, planning or hardware action is issued by this viewer.
 */
export class SceneView {
  private scene=new THREE.Scene();private camera:THREE.PerspectiveCamera;private renderer:THREE.WebGLRenderer;
  private controls:OrbitControls;private resize:ResizeObserver;private tfBuffer:TfBuffer;
  private off:(()=>void)[]=[];private stateOff?:()=>void;private raf=0;private disposed=false;private dirty=true;
  private started=false;
  private children=new Set<SceneView>();
  private robot?:URDFRobot;private robotFrame='';private robotText='';private generation=0;
  private markerMap=new Map<string,Resource>();private axes=new Map<string,THREE.AxesHelper>();
  private retired:THREE.Object3D[]=[];
  private cloud?:Resource;private trace?:Resource;private joints:Record<string,number>=Object.create(null);
  private pendingJoints=new Map<string,number>();
  private lastReport=0;private clock?:bigint;private assetErrors=new Set<string>();private layerEnabled=new Map<string,boolean>();
  private lastTfUpdate=0;
  private renderSamples:number[]=[];private renderCount=0;private loopCount=0;
  private startedAt=performance.now();private previousLoop=0;private maxLoopGapMs=0;
  private pageLongTasks=0;private pageLongTaskMaxMs=0;private performanceObserver?:PerformanceObserver;
  private assetStartedAt=0;private robotParseMs=0;private assetReadyMs:number|null=null;
  private assetsLoaded=0;private assetsTotal=0;
  private renderDevice='';private programsSeen=new WeakSet<object>();private programCreations=0;
  private rendererInitMs=0;private constructorMs=0;private firstRenderMs:number|null=null;private renderMaxMs=0;
  private firstRenderCompleteAtMs:number|null=null;
  private shaderCalls:Record<string,{count:number;totalMs:number;maxMs:number}>={};
  private restoreShaderHooks:(()=>void)[]=[];private unavailableShaderHooks:string[]=[];
  private shaderChecks?:ReturnType<typeof createShaderChecks>;private renderErrors=0;
  private readonly onContextChange=()=>this.contextChanged();
  private eventTiming?:ReturnType<typeof createEventTiming>;
  private lastRenderError?:string;
  constructor(private root:HTMLElement,private client:RosClient,private options:SceneOptions,private readonly parent?:SceneView) {
    if(!options.fixedFrame)throw new TypeError('Choose a fixed frame');
    if(options.useSimTime!==undefined&&typeof options.useSimTime!=='boolean')throw new TypeError('useSimTime must be boolean');
    if(options.profileShaders!==undefined&&typeof options.profileShaders!=='boolean')throw new TypeError('profileShaders must be boolean');
    if(options.profileInteractions!==undefined&&typeof options.profileInteractions!=='boolean')throw new TypeError('profileInteractions must be boolean');
    if(options.shaderDiagnostics!==undefined&&!['errors-only','verbose'].includes(options.shaderDiagnostics))throw new TypeError('Unknown shaderDiagnostics mode');
    const layers:SceneLayer[]=['robot','tf','markers','cloud','path'];
    if(options.layers!==undefined&&(!Array.isArray(options.layers)||options.layers.some(layer=>!layers.includes(layer))))throw new TypeError('Unknown scene layer');
    for(const layer of layers)this.layerEnabled.set(layer,options.layers===undefined||options.layers.includes(layer));
    if(parent){
      if(parent.parent||parent.disposed||!parent.started||parent.client!==client)throw new Error('Choose a started root scene host');
      this.options={...options,fixedFrame:parent.options.fixedFrame,tf:parent.options.tf,tfStatic:parent.options.tfStatic,useSimTime:parent.options.useSimTime};
      this.camera=parent.camera;this.renderer=parent.renderer;this.controls=parent.controls;
      this.resize=parent.resize;this.tfBuffer=parent.tfBuffer;parent.scene.add(this.scene);
    }else{
    this.camera=new THREE.PerspectiveCamera(45,1,0.01,200);
    this.camera.up.set(0,0,1);this.camera.position.set(1.7,-1.8,1.2);
    this.scene.background=new THREE.Color('#edf0e7');
    const rendererAt=performance.now();this.renderer=new THREE.WebGLRenderer({antialias:true,alpha:false});
    this.rendererInitMs=performance.now()-rendererAt;this.renderer.setPixelRatio(Math.min(devicePixelRatio,2));
    const gl=this.renderer.getContext();this.renderDevice='未启用设备信息探针';
    if(options.profileShaders){const device=gl.getExtension('WEBGL_debug_renderer_info');this.renderDevice=String(gl.getParameter(device?.UNMASKED_RENDERER_WEBGL??gl.RENDERER));}
    if(options.profileShaders)this.profileShaderCalls(gl);
    this.shaderChecks=createShaderChecks(gl,this.renderer.debug,options.shaderDiagnostics??'errors-only');
    // Three owns resource restoration. This view only invalidates old shader
    // handles and requests a frame, including scenes with no incoming ROS data.
    this.renderer.domElement.addEventListener('webglcontextlost',this.onContextChange);
    this.renderer.domElement.addEventListener('webglcontextrestored',this.onContextChange);
    root.replaceChildren(this.renderer.domElement);
    this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.target.set(0,0,0.25);
    this.controls.enableDamping=true;this.controls.addEventListener('change',()=>{this.dirty=true;});
    const grid=new THREE.GridHelper(3,30,0x9bad95,0xd0d8c7);grid.rotation.x=Math.PI/2;this.scene.add(grid,new THREE.AxesHelper(0.3));
    this.scene.add(new THREE.HemisphereLight(0xffffff,0x64705a,2));
    const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(2,-2,5);this.scene.add(light);
    this.tfBuffer=new TfBuffer(client,{tfTopic:options.tf,tfStaticTopic:options.tfStatic,clockTopic:options.useSimTime?'/clock':false});
    this.resize=new ResizeObserver(()=>this.size());this.resize.observe(root);this.size();
    this.stateOff=client.onState(state=>{if(state!=='connected'){this.resetData();this.options.onStatus?.('已断线：已清除上一会话的模型状态和数据');}});
    // Page long tasks are diagnostic context, not GPU execution timings.
    if(typeof PerformanceObserver!=='undefined'&&PerformanceObserver.supportedEntryTypes.includes('longtask')){
      this.performanceObserver=new PerformanceObserver(list=>{for(const entry of list.getEntries()){this.pageLongTasks++;this.pageLongTaskMaxMs=Math.max(this.pageLongTaskMaxMs,entry.duration);}});
      this.performanceObserver.observe({type:'longtask'});
    }
    this.eventTiming=createEventTiming(options.profileInteractions??false);
    }
    this.constructorMs=performance.now()-this.startedAt;
  }
  /** Create a logical display with independent data/resources and one shared GL.
   * The root owns RAF, camera, controls, TF/clock and renderer destruction.
   * Start/dispose the child for subscription ownership; setLayer stays visual.
   */
  createDisplay(options:Partial<SceneOptions>&{layers:SceneLayer[]}):SceneView {
    if(this.disposed)throw new Error('Scene is disposed');
    if(this.parent)throw new Error('Only the root scene host creates displays');
    const child=new SceneView(this.root,this.client,{...this.options,...options,onStatus:options.onStatus,
      fixedFrame:this.options.fixedFrame,tf:this.options.tf,tfStatic:this.options.tfStatic,useSimTime:this.options.useSimTime},this);
    this.children.add(child);this.dirty=true;return child;
  }
  private contextChanged():void {
    if(this.disposed)return;
    this.shaderChecks?.resetPending();this.dirty=true;
  }
  private profileShaderCalls(gl:WebGLRenderingContext|WebGL2RenderingContext):void {
    const surface=gl as unknown as Record<string,(...args:unknown[])=>unknown>;
    for(const name of ['compileShader','linkProgram','getProgramParameter','getProgramInfoLog','getShaderParameter','getShaderInfoLog','getActiveUniform','getUniformLocation','getActiveAttrib','getAttribLocation']){
      const original=surface[name];if(typeof original!=='function')continue;
      const stats=this.shaderCalls[name]={count:0,totalMs:0,maxMs:0};
      try{
        surface[name]=(...args)=>{const at=performance.now();try{return original.apply(gl,args);}finally{const ms=performance.now()-at;stats.count++;stats.totalMs+=ms;stats.maxMs=Math.max(stats.maxMs,ms);}};
        this.restoreShaderHooks.push(()=>{surface[name]=original;});
      }catch{this.unavailableShaderHooks.push(name);}
    }
  }
  private size():void {
    const width=Math.max(1,this.root.clientWidth),height=Math.max(240,this.root.clientHeight);
    this.camera.aspect=width/height;this.camera.updateProjectionMatrix();this.renderer.setSize(width,height);this.dirty=true;
  }
  /** package:// resolves only through the HTTP package allowlist; no arbitrary remote URLs. */
  private asset(uri:string):string {
    let path=uri;
    if(uri.startsWith('package://'))path=(this.options.assetBase??'/assets/')+uri.slice(10);
    const url=new URL(path,location.href);
    if(url.origin!==location.origin || !url.pathname.startsWith(this.options.assetBase??'/assets/'))throw new TypeError(`Asset must use the local package resource route: ${uri}`);
    return url.href;
  }
  private loader(generation:number,robotAssets=false):URDFLoader {
    const manager=new THREE.LoadingManager();manager.setURLModifier(url=>url.startsWith('data:')?url:this.asset(url));
    manager.onError=url=>{if(generation===this.generation)this.assetErrors.add(url);};
    if(robotAssets){
      manager.onStart=(_url,loaded,total)=>{if(generation===this.generation){this.assetsLoaded=loaded;this.assetsTotal=total;}};
      manager.onProgress=(_url,loaded,total)=>{if(generation===this.generation){this.assetsLoaded=loaded;this.assetsTotal=total;}};
      manager.onLoad=()=>{if(generation===this.generation)this.assetReadyMs=performance.now()-this.assetStartedAt;};
    }
    const loader=new URDFLoader(manager);loader.packages=pkg=>(this.options.assetBase??'/assets/')+encodeURIComponent(pkg);
    const standard=loader.loadMeshCb.bind(loader);
    loader.loadMeshCb=(path,manager,material,done)=>{
      const complete:typeof done=(object,error)=>{
        if(this.disposed||generation!==this.generation){if(object)release(object);return;}
        if(error)this.assetErrors.add(path);if(object)this.dirty=true;done(object,error);
        if(robotAssets&&object&&!error)this.applyRobotMaterial(object,material);
      };
      if(/\.(glb|gltf)(\?|$)/i.test(path))new GLTFLoader(manager).load(path,gltf=>complete(gltf.scene),undefined,e=>complete(new THREE.Group(),e as Error));
      else if(/\.obj(\?|$)/i.test(path))new OBJLoader(manager).load(path,object=>complete(object),undefined,e=>complete(new THREE.Group(),e as Error));
      else standard(path,manager,material,complete);
    };
    return loader;
  }
  /** Match RViz RobotLink: an explicit URDF visual material overrides embedded
   * mesh materials; without that declaration, keep the asset's own appearance.
   * URDFLoader attaches the loaded object to its public URDFVisual in done().
   * Its DAE path otherwise ignores the supplied URDF material, which can make
   * an opaque link invisible when a legacy DAE declares transparency=0.
   */
  private applyRobotMaterial(object:THREE.Object3D,material:THREE.Material):void {
    const visual=object.parent as URDFVisual|null;
    if(!visual?.isURDFVisual||!visual.urdfNode||![...visual.urdfNode.children].some(node=>node.tagName.toLowerCase()==='material'))return;
    const retired=new Set<THREE.Material>();
    object.traverse(child=>{
      const mesh=child as THREE.Mesh;if(!mesh.isMesh)return;
      for(const old of Array.isArray(mesh.material)?mesh.material:[mesh.material])retired.add(old);
      mesh.material=material;
    });
    // A discarded DAE/GLTF material is no longer reachable by dispose(). Keep
    // any material/texture still used in this loaded asset, then release only
    // replaced resources. Geometry and the URDF material remain owned by it.
    const retained=new Set<unknown>();
    object.traverse(child=>{
      const current=(child as THREE.Mesh).material;if(!current)return;
      for(const value of Array.isArray(current)?current:[current]){
        retained.add(value);for(const property of Object.values(value))if(property instanceof THREE.Texture)retained.add(property);
      }
    });
    for(const old of retired)releaseMaterial(old,retained);
  }
  private robotDescription(xml:string):void {
    if(xml===this.robotText)return;if(xml.length>2000000)throw new RangeError('URDF XML budget exceeded');
    const parsed=new DOMParser().parseFromString(xml,'application/xml');
    if(parsed.querySelector('parsererror') || parsed.documentElement.tagName!=='robot')throw new TypeError('Invalid URDF XML');
    if(parsed.getElementsByTagName('*').length>20000)throw new RangeError('URDF element budget exceeded');
    const generation=++this.generation;this.assetErrors.clear();
    this.assetStartedAt=performance.now();this.assetReadyMs=null;this.assetsLoaded=this.assetsTotal=0;
    const next=this.loader(generation,true).parse(parsed);
    this.robotParseMs=performance.now()-this.assetStartedAt;
    if(this.assetsTotal===0)this.assetReadyMs=this.robotParseMs;
    if(this.robot)release(this.robot);this.robot=next;this.robotText=xml;
    this.robotFrame=Object.values(next.links).find(link=>link.parent===next || link===next)?.name || Object.keys(next.links)[0] || this.options.fixedFrame;
    // Observation shows measured values even slightly outside declared limits;
    // this does not relax any controller or planning limit.
    for(const joint of Object.values(next.joints))joint.ignoreLimits=true;
    this.applyCachedJoints(next);this.scene.add(next);this.dirty=true;
  }
  /** Preserve partial JointState updates only for joints in the new model. */
  private applyCachedJoints(robot:URDFRobot):void {
    const values:Record<string,number>=Object.create(null);
    for(const [name,value] of Object.entries(this.joints))if(Object.hasOwn(robot.joints,name))values[name]=value;
    for(const [name,value] of this.pendingJoints)if(Object.hasOwn(robot.joints,name)){
      values[name]=value;this.pendingJoints.delete(name);
    }
    this.joints=values;robot.setJointValues(values);
  }
  private bind<T>(topic:string,type:string,callback:(message:T)=>void,options:TopicOptions={qos:'reliable'}):void {
    this.off.push(this.client.subscribe<T>(topic,type,message=>{
      try {callback(message);this.dirty=true;}catch(error){this.options.onStatus?.((error as Error).message);}
    },options));
  }
  start():void {
    if(this.disposed)throw new Error('Scene is disposed');if(this.started)return;this.started=true;
    if(!this.parent)this.tfBuffer.start();
    // Subscription ownership is fixed by the constructor mask. The existing
    // setLayer API changes visibility only, including calls made before start.
    const enabled=(layer:SceneLayer)=>this.options.layers===undefined||this.options.layers.includes(layer);
    if(enabled('robot'))this.bind<{data:string}>(this.options.robotDescription,'std_msgs/msg/String',message=>this.robotDescription(message.data),{qos:'latched',...this.options.robotDescriptionOptions});
    if(enabled('robot'))this.bind<{name:unknown;position:unknown}>(this.options.jointStates,'sensor_msgs/msg/JointState',message=>{
      // Topic payloads are not schema-checked by rosbridge. A JSON object with
      // a huge .length must never become the loop bound; CBOR float arrays are
      // legitimate and remain supported without copying their data.
      if(!Array.isArray(message.name)||!(Array.isArray(message.position)||
        (ArrayBuffer.isView(message.position)&&!(message.position instanceof DataView)))){
        throw new TypeError('JointState name must be an array and position a numeric array');
      }
      const positions=message.position as ArrayLike<unknown>;let changed=false;
      for(let i=0;i<message.name.length;i++){
        const name=message.name[i],value=positions[i];
        if(typeof name!=='string'||typeof value!=='number'||!Number.isFinite(value))continue;
        if(this.robot&&Object.hasOwn(this.robot.joints,name)){
          if(this.joints[name]!==value){this.joints[name]=value;changed=true;}
          this.pendingJoints.delete(name);
        }else if(name.length<=MAX_PENDING_JOINT_NAME){
          // Refresh insertion order so early, active joints survive unrelated
          // names until their URDF arrives. No commands are sent by this cache.
          this.pendingJoints.delete(name);this.pendingJoints.set(name,value);
          if(this.pendingJoints.size>MAX_PENDING_JOINTS)this.pendingJoints.delete(this.pendingJoints.keys().next().value!);
        }
      }
      if(changed)this.robot?.setJointValues(this.joints);
    },{qos:'sensor',...this.options.jointStatesOptions});
    if(enabled('markers'))this.bind<{markers:Marker[]}>(this.options.markers,'visualization_msgs/msg/MarkerArray',message=>this.markers(message.markers),{qos:'reliable',...this.options.markersOptions});
    if(enabled('cloud'))this.bind<Record<string,unknown>>(this.options.pointCloud,'sensor_msgs/msg/PointCloud2',message=>{
      const cloud=decodePointCloud(message,{maxPoints:this.options.maxPoints??100000});
      if(!this.cloud){this.cloud={object:new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({size:0.025,color:cloud.colors?0xffffff:0x3c7f96,vertexColors:!!cloud.colors})),header:message.header as Header};this.scene.add(this.cloud.object);}
      const object=this.cloud.object as THREE.Points<THREE.BufferGeometry,THREE.PointsMaterial>;
      object.geometry=updateGeometry(object.geometry,cloud.positions,cloud.colors);
      if(object.material.vertexColors!==!!cloud.colors){object.material.vertexColors=!!cloud.colors;object.material.color.set(cloud.colors?0xffffff:0x3c7f96);object.material.needsUpdate=true;}
      this.cloud.header=message.header as Header;this.cloud.placedFrame=undefined;
    },{qos:'sensor',compression:'cbor',throttleMs:100,...this.options.pointCloudOptions});
    if(enabled('path'))this.bind<{header:Header;poses:{pose:Pose}[]}>(this.options.path,'nav_msgs/msg/Path',message=>{
      if(message.poses.length>100000)throw new RangeError('Path point budget exceeded');
      const positions=new Float32Array(message.poses.length*3);
      message.poses.forEach((p,i)=>{positions[i*3]=p.pose.position.x;positions[i*3+1]=p.pose.position.y;positions[i*3+2]=p.pose.position.z;});
      if(!this.trace){this.trace={object:new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xba6f35})),header:message.header};this.scene.add(this.trace.object);}
      const object=this.trace.object as THREE.Line;object.geometry=updateGeometry(object.geometry,positions);
      this.trace.header=message.header;this.trace.placedFrame=undefined;
    },{qos:'reliable',...this.options.pathOptions});
    if(this.options.useSimTime&&!this.parent)this.bind<{clock:RosStamp}>('/clock','rosgraph_msgs/msg/Clock',message=>{
      const now=nanoseconds(message.clock);
      if(now===undefined)throw new TypeError('Invalid ROS /clock stamp');
      if(this.clock!==undefined&&now<this.clock)this.clearTemporalData();
      this.clock=now;
    },{qos:'sensor'});
    if(!this.parent)this.animate();
  }
  /** Layer switches affect display only; they do not publish ROS commands. */
  setLayer(layer:'robot'|'tf'|'markers'|'cloud'|'path',enabled:boolean):void {this.layerEnabled.set(layer,enabled);this.dirty=true;}
  setFixedFrame(frame:string):void {
    if(!frame)throw new TypeError('Choose a fixed frame');
    if(this.parent){this.parent.setFixedFrame(frame);return;}
    this.options={...this.options,fixedFrame:frame};this.dirty=true;
    for(const child of this.children){child.options={...child.options,fixedFrame:frame};child.dirty=true;}
  }
  resetCamera():void {this.camera.position.set(1.7,-1.8,1.2);this.controls.target.set(0,0,0.25);this.controls.update();this.dirty=true;}
  private clearMarkers():void {for(const marker of this.markerMap.values())release(marker.object);this.markerMap.clear();}
  private clearTemporalData():void {
    this.clearMarkers();if(this.cloud)release(this.cloud.object);this.cloud=undefined;if(this.trace)release(this.trace.object);this.trace=undefined;this.dirty=true;
    for(const child of this.children)child.clearTemporalData();
  }
  /** Keep old shader references until replacements have rendered. Removing the
   * last material before that render makes Three destroy and recompile a shared
   * program. The queue is bounded even if many ROS samples arrive between RAFs.
   */
  private retire(object:THREE.Object3D):void {
    object.removeFromParent();this.retired.push(object);
    if(this.retired.length>2*(this.options.maxMarkers??1000)+4)release(this.retired.shift()!);
  }
  private flushRetired():void {
    for(const object of this.retired)release(object);this.retired=[];
    // Only the root renders. Child replacements must survive until that shared
    // render attempt, then release in the same frame instead of filling queues.
    for(const child of this.children)child.flushRetired();
  }
  private resetData():void {
    this.generation++;if(this.robot)release(this.robot);this.robot=undefined;this.robotText='';this.joints=Object.create(null);this.pendingJoints.clear();this.clearMarkers();
    if(this.cloud)release(this.cloud.object);this.cloud=undefined;if(this.trace)release(this.trace.object);this.trace=undefined;
    for(const axes of this.axes.values())release(axes);this.axes.clear();this.clock=undefined;this.dirty=true;
    for(const child of this.children)child.resetData();
  }
  private displayTime():bigint {
    // A ROS-time deadline must never contain a wall epoch, including before
    // the first clock tick. Rendering/performance and RPC deadlines stay steady.
    return this.parent?this.parent.displayTime():this.options.useSimTime?(this.clock??0n):BigInt(Date.now())*1000000n;
  }
  private markers(markers:Marker[]):void {
    if(markers.length>(this.options.maxMarkers??1000))throw new RangeError('Marker array budget exceeded');
    // MarkerArray is rendered as one snapshot. Apply ordered delete operations
    // to its final key set before touching GPU resources. Jazzy DELETEALL with
    // a namespace deletes only that namespace (same semantics as RViz).
    const updates=new Map<string,Marker>(),removed=new Set<string>();
    for(const marker of markers) {
      if(marker.action===3){
        for(const [key,resource] of this.markerMap)if(!marker.ns||resource.namespace===marker.ns)removed.add(key);
        for(const [key,update] of updates)if(!marker.ns||update.ns===marker.ns)updates.delete(key);
        continue;
      }
      const key=`${marker.ns}:${marker.id}`;
      if(marker.action===2){updates.delete(key);removed.add(key);continue;}
      if(marker.action!==0)throw new TypeError(`Unsupported Marker action ${marker.action}`);
      updates.set(key,marker);removed.delete(key);
    }
    for(const key of removed){const previous=this.markerMap.get(key);if(previous)this.retire(previous.object);this.markerMap.delete(key);}
    for(const marker of updates.values()) {
      const key=`${marker.ns}:${marker.id}`;const previous=this.markerMap.get(key);
      const duration=nanoseconds(marker.lifetime);
      if(duration===undefined)throw new TypeError('Invalid Marker lifetime');
      const expires=duration>0n?this.displayTime()+duration:undefined;
      if(marker.points.length>100000)throw new RangeError('Marker vertex budget exceeded');
      const visualKey=JSON.stringify([marker.type,marker.scale,marker.color,marker.points,marker.colors,marker.text,marker.mesh_resource]);
      // Header, pose and expiry change independently of the render shape.
      // Retain the material/texture when a publisher refreshes the same marker.
      if(previous?.visualKey===visualKey){
        Object.assign(previous,{header:marker.header,pose:marker.pose,latest:marker.frame_locked,expires,placedFrame:undefined});continue;
      }
      if(previous)this.retire(previous.object);this.markerMap.delete(key);
      if(this.markerMap.size>=(this.options.maxMarkers??1000))throw new RangeError('Marker capacity exceeded');
      const shape=this.markerObject(marker);const object=new THREE.Group();object.add(shape);
      const resource:Resource={object,header:marker.header,pose:marker.pose,latest:marker.frame_locked,expires,visualKey,namespace:marker.ns};
      this.markerMap.set(key,resource);this.scene.add(object);
    }
  }
  private markerObject(marker:Marker):THREE.Object3D {
    const color=new THREE.Color().setRGB(marker.color.r,marker.color.g,marker.color.b);
    const material=new THREE.MeshStandardMaterial({color,opacity:marker.color.a,transparent:marker.color.a<1,side:THREE.DoubleSide});
    const {x,y,z}=marker.scale;let geometry:THREE.BufferGeometry|undefined;
    if(marker.points.length>100000){material.dispose();throw new RangeError('Marker vertex budget exceeded');}
    switch(marker.type) {
      case 0:{
        material.dispose();const origin=marker.points.length>=2?new THREE.Vector3(...[marker.points[0].x,marker.points[0].y,marker.points[0].z] as [number,number,number]):new THREE.Vector3();
        const direction=marker.points.length>=2?new THREE.Vector3(marker.points[1].x,marker.points[1].y,marker.points[1].z).sub(origin):new THREE.Vector3(1,0,0);
        const length=marker.points.length>=2?direction.length():x;
        return new THREE.ArrowHelper(direction.normalize(),origin,length,color,marker.points.length>=2?z:length*0.25,marker.points.length>=2?y:y||length*0.1);
      }
      case 1:geometry=new THREE.BoxGeometry(x,y,z);break;
      case 2:geometry=new THREE.SphereGeometry(0.5,20,12);geometry.scale(x,y,z);break;
      case 3:geometry=new THREE.CylinderGeometry(x/2,x/2,z,20);geometry.rotateX(Math.PI/2);break;
      case 4:case 5:{
        material.dispose();geometry=new THREE.BufferGeometry().setFromPoints(marker.points.map(p=>new THREE.Vector3(p.x,p.y,p.z)));
        const line=new THREE.LineBasicMaterial({color,opacity:marker.color.a,transparent:marker.color.a<1});
        return marker.type===4?new THREE.Line(geometry,line):new THREE.LineSegments(geometry,line);
      }
      case 6:case 7:{
        geometry=marker.type===6?new THREE.BoxGeometry(x,y,z):new THREE.SphereGeometry(0.5,12,8);if(marker.type===7)geometry.scale(x,y,z);
        const instances=new THREE.InstancedMesh(geometry,material,marker.points.length);const matrix=new THREE.Matrix4();
        marker.points.forEach((p,i)=>{instances.setMatrixAt(i,matrix.makeTranslation(p.x,p.y,p.z));const c=marker.colors[i];if(c)instances.setColorAt(i,new THREE.Color(c.r,c.g,c.b));});return instances;
      }
      case 8:{
        material.dispose();geometry=new THREE.BufferGeometry().setFromPoints(marker.points.map(p=>new THREE.Vector3(p.x,p.y,p.z)));
        return new THREE.Points(geometry,new THREE.PointsMaterial({size:x,color,opacity:marker.color.a,transparent:marker.color.a<1}));
      }
      case 9:{
        material.dispose();const canvas=document.createElement('canvas');canvas.width=512;canvas.height=96;
        const context=canvas.getContext('2d')!;context.font='40px sans-serif';context.fillStyle=`#${color.getHexString()}`;context.fillText(marker.text.slice(0,100),6,60);
        const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(canvas),transparent:true,opacity:marker.color.a}));sprite.scale.set(z*5.33,z,1);return sprite;
      }
      case 10:{
        const group=new THREE.Group();const generation=this.generation;
        // Own material immediately: deleting a pending mesh must release it too.
        group.add(new THREE.Mesh(new THREE.BufferGeometry(),material));
        const loader=this.loader(generation);
        loader.loadMeshCb(this.asset(marker.mesh_resource),loader.manager,material,(object,error)=>{
          let ancestor:THREE.Object3D|null=group;
          while(ancestor&&ancestor!==this.scene)ancestor=ancestor.parent;
          if(this.disposed||generation!==this.generation||!ancestor){if(object)release(object);return;}
          if(error){if(object)release(object);this.assetErrors.add(marker.mesh_resource);return;}if(!object)return;
          object.scale.set(x,y,z);group.add(object);this.dirty=true;
        });return group;
      }
      case 11:geometry=new THREE.BufferGeometry().setFromPoints(marker.points.map(p=>new THREE.Vector3(p.x,p.y,p.z)));geometry.computeVertexNormals();break;
      default:material.dispose();throw new TypeError(`Unsupported Marker type ${marker.type}`);
    }
    return new THREE.Mesh(geometry,material);
  }
  private place(resource:Resource,issues:Set<string>):void {
    // RViz-style snapshot placement: non-frame-locked data stays at the
    // transform resolved for its stamp even after TF history expires.
    if(!resource.latest&&resource.placedFrame===this.options.fixedFrame){resource.object.visible=true;return;}
    const source=resource.header?.frame_id;const stamp=resource.header?.stamp;
    const zero=stamp?.sec===0&&stamp.nanosec===0;
    const transform=source?this.tfBuffer.lookup(this.options.fixedFrame,source,resource.latest||zero?undefined:stamp):undefined;
    if(!transform){resource.object.visible=false;issues.add(`缺少 ${source??'(empty frame)'} → ${this.options.fixedFrame} 的对应时间 TF`);return;}
    resource.object.visible=true;position(resource.object,transform);resource.placedFrame=this.options.fixedFrame;
    const child=resource.object.children[0];if(resource.pose&&child)position(child,resource.pose);
  }
  private animate=():void=>{this.renderFrame();};
  private renderFrame():void {
    if(this.disposed)return;
    if(!this.parent){
      this.raf=requestAnimationFrame(this.animate);this.controls.update();
      for(const child of this.children)try{child.renderFrame();}catch(error){child.lastRenderError=`显示更新失败：${errorText(error)}`;}
    }else this.clock=this.parent.clock;
    const loopAt=performance.now();this.loopCount++;
    if(this.previousLoop)this.maxLoopGapMs=Math.max(this.maxLoopGapMs,loopAt-this.previousLoop);
    this.previousLoop=loopAt;
    const tfUpdate=this.tfBuffer.stats().lastReceivedAt;if(tfUpdate!==this.lastTfUpdate){this.lastTfUpdate=tfUpdate;this.dirty=true;}
    const issues=new Set<string>();
    if(this.options.useSimTime&&(this.clock===undefined||this.clock===0n))issues.add('ROS 时间尚未初始化，等待 /clock');
    try {
      if(this.robot){const tf=this.tfBuffer.lookup(this.options.fixedFrame,this.robotFrame);this.robot.visible=!!tf&&this.layerEnabled.get('robot')!==false;if(tf)position(this.robot,tf);else issues.add(`URDF 根坐标 ${this.robotFrame} 不在固定坐标树中`);}
      const now=this.displayTime();
      for(const [key,marker] of this.markerMap) {
        if(marker.expires!==undefined&&now>=marker.expires){release(marker.object);this.markerMap.delete(key);this.dirty=true;continue;}
        this.place(marker,issues);marker.object.visible&&=this.layerEnabled.get('markers')!==false;
      }
      if(this.cloud){this.place(this.cloud,issues);this.cloud.object.visible&&=this.layerEnabled.get('cloud')!==false;}
      if(this.trace){this.place(this.trace,issues);this.trace.object.visible&&=this.layerEnabled.get('path')!==false;}
      const frames=this.layerEnabled.get('tf')===false?[]:this.tfBuffer.frames();
      for(const frame of frames) {
        let axis=this.axes.get(frame);if(!axis){axis=new THREE.AxesHelper(0.12);this.axes.set(frame,axis);this.scene.add(axis);}
        const tf=this.tfBuffer.lookup(this.options.fixedFrame,frame);axis.visible=!!tf&&this.layerEnabled.get('tf')!==false;if(tf)position(axis,tf);
      }
      for(const [frame,axis] of this.axes)if(!frames.includes(frame)){release(axis);this.axes.delete(frame);}
    }catch(error){issues.add((error as Error).message);}
    if(this.parent&&this.dirty){this.parent.dirty=true;this.dirty=false;}
    if(!this.parent&&this.dirty){const at=performance.now();let rendered=false;
      try{
        // Three returns normally when its context is lost. Neither that return
        // nor loss during submission is evidence of a completed frame.
        const gl=this.renderer.getContext();
        if(gl.isContextLost())throw new Error('WebGL 上下文已丢失，等待恢复或重新开始 3D');
        this.renderer.render(this.scene,this.camera);
        if(gl.isContextLost())throw new Error('WebGL 上下文在绘制期间丢失');
        rendered=true;this.lastRenderError=undefined;
      }
      catch(error){this.renderErrors++;this.lastRenderError=`绘制失败：${errorText(error)}`;}
      finally{this.shaderChecks?.flush();this.flushRetired();}const duration=performance.now()-at;
      if(rendered&&this.firstRenderMs===null){this.firstRenderMs=duration;this.firstRenderCompleteAtMs=performance.now()-this.startedAt;}this.renderMaxMs=Math.max(this.renderMaxMs,duration);
      this.renderSamples.push(duration);if(this.renderSamples.length>120)this.renderSamples.shift();if(rendered)this.renderCount++;this.dirty=false;
      for(const program of this.renderer.info.programs??[]){if(!this.programsSeen.has(program)){this.programsSeen.add(program);this.programCreations++;}}
    }
    if(this.lastRenderError)issues.add(this.lastRenderError);
    for(const error of this.shaderChecks?.errors??[])issues.add(error);
    if(performance.now()-this.lastReport>500){
      this.lastReport=performance.now();const stats=this.stats();
      const clockLabel=stats.useSimTime?`ROS 时间 ${stats.clockSeconds.toFixed(3)}`:'系统时间';
      this.options.onStatus?.(`${this.layerEnabled.get('robot')===false?'空间显示':stats.robotLoaded?'URDF 已加载':'等待 URDF'} · ${stats.joints} 关节（${stats.movableJoints} 活动） · ${stats.frames} TF 帧 · ${stats.markers} Marker · ${stats.points} 点 · ${stats.pathPoints} 轨迹点 · ${clockLabel}${issues.size?'\n'+[...issues].join('\n'):''}${this.assetErrors.size?'\n资源加载失败：'+[...this.assetErrors].join(', '):''}`);
    }
  }
  stats():SceneStats {const now=this.displayTime(),clock=this.parent?.clock??this.clock,children=[...this.children].map(child=>child.stats());return {useSimTime:!!this.options.useSimTime,clockReady:!this.options.useSimTime||(clock!==undefined&&clock!==0n),clockSeconds:Number(now)/1e9,clockNanoseconds:now.toString(),
    robotLoaded:!!this.robot||children.some(s=>s.robotLoaded),joints:Object.keys(this.robot?.joints??{}).length+children.reduce((n,s)=>n+s.joints,0),movableJoints:Object.values(this.robot?.joints??{}).filter(j=>j.jointType!=='fixed').length+children.reduce((n,s)=>n+s.movableJoints,0),frames:this.tfBuffer.stats().frames,markers:this.markerMap.size+children.reduce((n,s)=>n+s.markers,0),
    points:((this.cloud?.object as THREE.Points|undefined)?.geometry?.getAttribute('position')?.count??0)+children.reduce((n,s)=>n+s.points,0),
    pathPoints:((this.trace?.object as THREE.Line|undefined)?.geometry?.getAttribute('position')?.count??0)+children.reduce((n,s)=>n+s.pathPoints,0),sharedDisplays:this.children.size};}
  /** Model assets belong to this logical display, independently of its host GL. */
  assetStats(){return {robotParseMs:this.robotParseMs,assetReadyMs:this.assetReadyMs,assetsLoaded:this.assetsLoaded,assetsTotal:this.assetsTotal,
    jointStateCache:{modelValues:Object.keys(this.joints).length,unmappedValues:this.pendingJoints.size,unmappedLimit:MAX_PENDING_JOINTS}};}
  performanceStats(){
    const sorted=[...this.renderSamples].sort((a,b)=>a-b),seconds=Math.max(.001,(performance.now()-this.startedAt)/1000);
    return {renderCount:this.renderCount,renderFps:this.renderCount/seconds,loopFps:this.loopCount/seconds,maxLoopGapMs:this.maxLoopGapMs,
      cpuRenderP95Ms:sorted.length?sorted[Math.floor((sorted.length-1)*.95)]:null,
      pageLongTasks:this.pageLongTasks,pageLongTaskMaxMs:this.pageLongTaskMaxMs,
      ...this.assetStats(),
      renderDevice:this.renderDevice,programCreations:this.programCreations,
      rendererInitMs:this.rendererInitMs,constructorMs:this.constructorMs,firstRenderMs:this.firstRenderMs,renderMaxMs:this.renderMaxMs,firstRenderCompleteAtMs:this.firstRenderCompleteAtMs,
      shaderCalls:this.shaderCalls,unavailableShaderHooks:this.unavailableShaderHooks,
      shaderDiagnostics:this.shaderChecks?.mode,shaderErrors:this.shaderChecks?.errors??[],renderErrors:this.renderErrors,lastRenderError:this.lastRenderError,
      eventTiming:this.eventTiming?.snapshot()??{enabled:false},
      drawCalls:this.renderer.info.render.calls,triangles:this.renderer.info.render.triangles,
      geometries:this.renderer.info.memory.geometries,textures:this.renderer.info.memory.textures};
  }
  dispose():void {
    if(this.disposed)return;this.disposed=true;
    for(const child of [...this.children])child.dispose();
    if(!this.parent)cancelAnimationFrame(this.raf);this.off.forEach(off=>off());this.off=[];this.stateOff?.();
    if(this.parent){
      this.resetData();this.flushRetired();release(this.scene);this.parent.children.delete(this);this.parent.dirty=true;return;
    }
    this.renderer.domElement.removeEventListener('webglcontextlost',this.onContextChange);
    this.renderer.domElement.removeEventListener('webglcontextrestored',this.onContextChange);
    this.tfBuffer.dispose();this.resize.disconnect();this.controls.dispose();this.performanceObserver?.disconnect();this.resetData();this.flushRetired();release(this.scene);
    this.eventTiming?.dispose();
    this.shaderChecks?.dispose();this.shaderChecks=undefined;
    this.lastRenderError=undefined;
    for(const restore of this.restoreShaderHooks.reverse())restore();this.restoreShaderHooks=[];
    this.renderer.dispose();this.renderer.forceContextLoss();this.root.replaceChildren();
  }
}
