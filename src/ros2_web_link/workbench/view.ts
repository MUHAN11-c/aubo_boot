/** Reusable read-only observation workbench. ROS names/messages never enter HTML. */
import type {RosClient,TopicOptions} from '../sdk/index.js';
import {errorText} from '../sdk/errors.js';
import {discoverGraph,GraphView} from '../graph/index.js';
import type {GraphSnapshot,GraphSelection} from '../graph/index.js';
import {WorkbenchController} from './controller.js';
import {createDefaultRegistry,previewMessage} from './registry.js';
import {parseWorkspace,workspaceFromProfile,WORKSPACE_LIMITS} from './config.js';
import type {DisplaySpec,TopicBinding} from './config.js';

export interface WorkbenchViewOptions {
  getControlClient():RosClient|undefined;getObservationClient():RosClient|undefined;
  beforeResume?():void;profile?:Record<string,unknown>;storageKey?:string;onStatus?(message:string):void;
}
let nextView=0;
const element=<K extends keyof HTMLElementTagNameMap>(tag:K,text?:string,className?:string):HTMLElementTagNameMap[K]=>{
  const node=document.createElement(tag);if(text!==undefined)node.textContent=text;if(className)node.className=className;return node;
};
const option=(value:string,label=value)=>{const node=element('option',label);node.value=value;return node;};
export class WorkbenchView {
  readonly registry=createDefaultRegistry();readonly controller:WorkbenchController;
  private graphView:GraphView;private graphSnapshot?:GraphSnapshot;private graphWait?:AbortController;private inspectWait?:AbortController;
  private graphSequence=0;private importSequence=0;private wasRunning=false;private disposed=false;private selected?:string;
  private graphSource?:string;private graphStale=false;
  private lastControl?:RosClient;private autoTimer?:ReturnType<typeof setInterval>;private metricTimer:ReturnType<typeof setInterval>;
  private outputs=new Map<string,{article:HTMLElement;root:HTMLElement;title:HTMLElement;status:HTMLElement}>();
  private displayMessages=new Map<string,string>();private statusRows=new Map<string,HTMLElement>();
  private readonly listId=`ros2-workbench-topics-${++nextView}`;
  private readonly visibility=()=>{if(document.hidden){this.stopGraphRound();this.el('graph-state').textContent='页面已隐藏，本轮采集已取消；自动刷新在页面可见后继续。';}};
  constructor(private root:HTMLElement,private options:WorkbenchViewOptions){
    // Static markup only; all configuration, graph and ROS content is assigned
    // through textContent/value below, including labels passed to Cytoscape.
    root.classList.add('wb-shell');
    root.innerHTML=`
      <div class="wb-toolbar">
        <div class="inline"><button data-wb="run" class="primary">▶ 启动所选显示</button><button data-wb="pause">Ⅱ 暂停显示</button></div>
        <span data-wb="session" class="wb-session" role="status"></span>
        <span class="wb-frame-label">固定坐标系 <strong data-wb="frame-label"></strong></span>
      </div>
      <details class="wb-settings"><summary>工作区设置与保存 <span>TF · 时钟 · 配置文件</span></summary>
        <form data-wb="globals" class="fields"><label>固定坐标系<input data-wb="frame" required></label><label>TF Topic<input data-wb="tf" required></label><label>静态 TF Topic<input data-wb="static" required></label><label>布局<select data-wb="layout"><option value="split">并排</option><option value="stacked">上下</option></select></label><label class="wb-check"><input data-wb="clock" type="checkbox">使用 ROS /clock</label><button type="submit">应用并暂停</button></form>
        <div class="inline"><button data-wb="save">保存到本机</button><button data-wb="restore">恢复本机配置</button><button data-wb="export">下载配置 JSON</button><label class="wb-file">导入配置<input data-wb="file" type="file" accept=".json,application/json"></label></div>
        <p class="hint">导入、恢复与修改全局设置后保持暂停，需重新启动显示。</p>
      </details>
      <div class="wb-tabs" role="tablist" aria-label="工作台视图">
        <button data-wb="tab-space" role="tab">空间与数据</button><button data-wb="tab-graph" role="tab">节点关系</button><button data-wb="tab-interfaces" role="tab">接口信息</button>
      </div>
      <div data-wb="panel-space" role="tabpanel">
        <div class="wb-grid" data-wb="grid"><aside class="wb-manager" aria-label="显示列表与属性">
          <div class="wb-panel-title"><h3>显示列表 <span data-wb="display-count"></span></h3><span>勾选启用 · 点击编辑</span></div>
          <ul data-wb="displays" class="wb-displays"></ul>
          <details class="wb-add-panel" data-wb="add-panel"><summary>＋ 添加显示</summary><div class="wb-add"><label>显示类型<select data-wb="plugin"></select></label><label>Topic<input data-wb="add-topic" placeholder="选择或输入 /topic"></label><button data-wb="add">添加到显示列表</button></div></details>
          <datalist data-wb="topics"></datalist>
          <details class="wb-properties" open><summary>显示属性 <span data-wb="editor-title">未选择</span></summary><form data-wb="editor" class="wb-editor"></form></details>
        </aside><div class="wb-observation">
          <div class="wb-panel-title"><h3>3D 视图</h3><button data-wb="camera">重置视角</button></div>
          <div class="wb-viewport"><div data-wb="scene" class="scene-view wb-scene"></div><div data-wb="scene-empty" class="wb-empty"><span aria-hidden="true">◇</span><strong>尚未启动空间显示</strong><p>勾选左侧模型、TF 或点云，然后启动所选显示。</p></div><span class="wb-view-help">拖动旋转 · 右键平移 · 滚轮缩放</span></div>
          <p data-wb="scene-status" class="wb-scene-status">坐标单位 m · 等待空间数据</p>
          <div class="wb-panel-title wb-data-title"><h3>图像与数据</h3><span>图像 · 曲线 · 消息摘要</span></div><p data-wb="outputs-empty" class="wb-empty-note">启用图像、数值曲线或原始消息后，在此查看数据。</p><div data-wb="outputs" class="wb-outputs"></div>
          <details class="wb-diagnostics"><summary>资源与性能详情</summary><pre data-wb="metrics"></pre></details>
        </div></div>
      </div>
      <div data-wb="panel-graph" role="tabpanel" hidden>
        <div class="wb-relations">
          <div class="wb-graph-toolbar"><div class="inline"><button data-wb="graph-refresh" class="primary">刷新关系图</button><button data-wb="graph-stop">暂停刷新</button><button data-wb="fit">适应画布</button></div><label class="wb-check"><input data-wb="graph-auto" type="checkbox">每 5 秒刷新</label></div>
          <div class="wb-graph-filters"><label>搜索名称或消息类型<input data-wb="graph-search" placeholder="/namespace 或 sensor_msgs"></label><label>显示模式<select data-wb="graph-mode"><option value="topics">节点 + Topic</option><option value="nodes">仅节点（连线标注 Topic）</option></select></label><label class="wb-check"><input data-wb="graph-internal" type="checkbox" checked>隐藏内部 Topic</label></div>
          <div class="wb-graph-view-controls"><label>布局<select data-wb="graph-layout"><option value="breadthfirst">层级关系</option><option value="grid">紧凑网格</option></select></label><div class="inline" aria-label="关系图缩放"><button data-wb="graph-zoom-out" aria-label="缩小关系图">−</button><output data-wb="graph-zoom-level">100%</output><button data-wb="graph-zoom-in" aria-label="放大关系图">＋</button><button data-wb="graph-zoom-reset">1:1 字号</button></div><span data-wb="graph-zoom-hint">滚轮缩放，拖动画布平移。</span></div>
          <div class="wb-legend"><span><i class="wb-legend-node"></i>节点</span><span><i class="wb-legend-topic"></i>Topic</span><span>发布节点 → Topic → 订阅节点</span><span>滚轮缩放 · 拖动平移</span></div>
          <p data-wb="graph-state" class="wb-graph-state" role="status">尚未采集。连接后点击“刷新关系图”。</p>
          <div class="wb-graph-layout"><div class="wb-graph-viewport"><div data-wb="graph" class="wb-graph"></div><div data-wb="graph-empty" class="wb-empty"><span aria-hidden="true">⌘</span><strong data-wb="graph-empty-title">查看节点之间的连接</strong><p data-wb="graph-empty-body">刷新后显示发布、订阅关系；选择节点可突出其直接邻居。</p></div></div>
            <aside class="wb-graph-inspector"><h3>所选实体</h3><label>节点 / Topic<select data-wb="selection"></select></label><div class="inline"><button data-wb="focus">聚焦邻居</button><button data-wb="use-topic">添加为显示…</button></div><pre data-wb="selection-detail">点击图中节点或 Topic，查看名称、类型和关系详情。</pre><p class="hint">连线表示 ROS 声明关系，不代表数据已到达或 QoS 匹配。</p><button data-wb="graph-export">下载图谱 JSON</button></aside></div>
          <details class="wb-diagnostics"><summary>图谱范围、错误与能力</summary><pre data-wb="graph-errors"></pre></details>
        </div>
      </div>
      <div data-wb="panel-interfaces" role="tabpanel" hidden>
        <div class="wb-interface-head"><h3>服务与 Action</h3><p>先在“节点关系”刷新图谱，再选择接口查看定义。</p><span class="tag">只读查询</span></div>
        <p data-wb="interface-state" class="wb-graph-state" role="status">尚无图谱快照。</p><div class="wb-endpoints"><details open><summary>服务与提供节点</summary><pre data-wb="services">尚未采集</pre></details><details open><summary>Action 服务端</summary><pre data-wb="actions">尚未采集</pre></details></div>
        <div class="wb-interface-form"><label>接口种类<select data-wb="endpoint-kind"><option value="service">Service</option><option value="action">Action</option><option value="topic">Topic</option></select></label><label>接口名称<select data-wb="endpoint"></select></label><button data-wb="inspect" class="primary">读取类型与定义</button></div><pre data-wb="interface" class="wb-interface-result">读取本机已安装的标准或自定义 IDL；不调用业务服务或发送 Goal。</pre>
      </div>
      <p data-wb="status" class="wb-status" role="status">模板默认关闭。先连接 ROS，再勾选显示并启动。</p>`;
    this.configureTabs();
    this.el('topics').id=this.listId;this.input('add-topic').setAttribute('list',this.listId);
    for(const definition of this.registry.list())this.select('plugin').append(option(definition.id,definition.title));
    this.graphView=new GraphView(this.el('graph'),{onSelect:item=>this.selection(item),onViewportChange:zoom=>{
      this.el('graph-zoom-level').textContent=`${Math.round(zoom*100)}%`;
      this.el('graph-zoom-hint').textContent=zoom<.7?'全图文字较小，可切换紧凑网格、搜索 Topic 或点击 1:1 字号。':'滚轮缩放，拖动画布平移。';
    }});
    this.controller=new WorkbenchController({getObservationClient:options.getObservationClient,beforeResume:options.beforeResume,
      getSceneRoot:()=>this.el('scene'),getDisplayRoot:id=>this.output(id).root,
      onDisplayStatus:(id,message)=>{const bounded=message.slice(0,2048);this.displayMessages.set(id,bounded);const row=this.statusRows.get(id);if(row){row.textContent=bounded;row.title=bounded;row.parentElement!.dataset.issue=String(/缺少|失败|错误|invalid|error|timeout|超时/i.test(bounded));}const output=this.outputs.get(id);if(output)output.status.textContent=bounded;},
      onSceneStatus:message=>{this.el('scene-status').textContent=message;},onStateChange:()=>this.render()},this.registry,workspaceFromProfile(options.profile??{},this.registry));
    this.button('run',()=>{this.controller.resume();this.report('已启动所选显示。');});
    this.button('pause',()=>{this.controller.pause();this.report('显示已暂停，资源已释放。');});
    this.button('camera',()=>this.controller.resetCamera());
    this.button('add',()=>this.add());this.button('save',()=>{localStorage.setItem(options.storageKey??'ros2-web-link.workspace.v1',JSON.stringify(this.controller.snapshot()));this.report('已保存到本机浏览器。');});
    this.button('restore',()=>{const data=localStorage.getItem(options.storageKey??'ros2-web-link.workspace.v1');if(!data)throw Error('本机尚无保存配置');this.controller.replace(parseWorkspace(data,this.registry));this.renderEditor();this.report('已恢复配置，显示保持暂停。');});
    this.button('export',()=>this.download('ros2-workspace.json',this.controller.snapshot()));
    this.input('file').addEventListener('change',()=>void this.importFile());
    this.el('globals').addEventListener('submit',event=>{event.preventDefault();this.run(()=>{
      this.controller.replace({...this.controller.snapshot(),fixedFrame:this.input('frame').value.trim(),tf:this.input('tf').value.trim(),tfStatic:this.input('static').value.trim(),useSimTime:this.input('clock').checked,layout:this.select('layout').value});this.report('全局设置已更新，显示保持暂停。');
    });});
    this.el('editor').addEventListener('submit',event=>{event.preventDefault();this.run(()=>this.applyEditor());});
    this.button('graph-refresh',()=>void this.refreshGraph());this.button('graph-stop',()=>{this.stopGraph();this.renderGraphState();this.el('graph-state').textContent=this.graphSnapshot?`已暂停刷新 · ${this.el('graph-state').textContent}`:'已暂停刷新，尚无快照。';});
    this.button('graph-export',()=>{if(!this.graphSnapshot)throw Error('先刷新图谱');this.download('ros2-graph.json',{...this.graphSnapshot,source:this.graphSource,stale:this.graphStale});});
    for(const id of ['graph-search','graph-mode','graph-internal','graph-layout'])this.el(id).addEventListener(id==='graph-search'?'input':'change',()=>this.filterGraph());
    this.button('graph-zoom-in',()=>this.graphView.setZoom(this.graphView.stats().zoom*1.25));
    this.button('graph-zoom-out',()=>this.graphView.setZoom(this.graphView.stats().zoom/1.25));
    this.button('graph-zoom-reset',()=>this.graphView.setZoom(1));
    this.input('graph-auto').addEventListener('change',()=>{if(this.input('graph-auto').checked){this.autoTimer=setInterval(()=>{if(!document.hidden)void this.refreshGraph();},5000);void this.refreshGraph();}else{if(this.autoTimer)clearInterval(this.autoTimer);this.autoTimer=undefined;}});
    this.button('fit',()=>this.graphView.fit());this.button('focus',()=>this.graphView.focus(this.select('selection').value));
    this.select('selection').addEventListener('change',()=>this.graphView.focus(this.select('selection').value));
    this.button('use-topic',()=>{const topic=this.graphSnapshot?.topics.find(item=>item.id===this.select('selection').value);if(!topic)throw Error('先选择 Topic');this.input('add-topic').value=topic.name;this.showPane('space');(this.el('add-panel') as HTMLDetailsElement).open=true;this.input('add-topic').focus();this.report('已填入 Topic，可选择适配器后添加。');});
    this.select('endpoint-kind').addEventListener('change',()=>this.endpoints());this.button('inspect',()=>void this.inspect());
    document.addEventListener('visibilitychange',this.visibility);
    this.metricTimer=setInterval(()=>{if(!this.disposed&&!document.hidden){this.el('metrics').textContent=JSON.stringify(this.controller.stats(),null,2);this.syncConnection();}},1000);
    this.selected=this.controller.snapshot().displays[0]?.id;
    this.render();this.renderEditor();this.syncConnection();
  }
  private el(id:string):HTMLElement {return this.root.querySelector<HTMLElement>(`[data-wb="${id}"]`)!;}
  /** Tabs only reveal existing views: they never recreate subscriptions or GL.
   * Arrow keys follow the WAI-ARIA tab pattern; ids are unique per component. */
  private configureTabs():void {
    const panes=['space','graph','interfaces'] as const;
    for(const [index,pane] of panes.entries()){
      const tab=this.el(`tab-${pane}`),panel=this.el(`panel-${pane}`);
      tab.id=`${this.listId}-tab-${pane}`;panel.id=`${this.listId}-panel-${pane}`;
      tab.setAttribute('aria-controls',panel.id);panel.setAttribute('aria-labelledby',tab.id);
      tab.addEventListener('click',()=>this.showPane(pane));
      tab.addEventListener('keydown',event=>{
        const next=event.key==='ArrowRight'?(index+1)%panes.length:event.key==='ArrowLeft'?(index+panes.length-1)%panes.length:event.key==='Home'?0:event.key==='End'?panes.length-1:undefined;
        if(next===undefined)return;event.preventDefault();this.showPane(panes[next]);this.el(`tab-${panes[next]}`).focus();
      });
    }
    this.showPane('space');
  }
  private showPane(pane:'space'|'graph'|'interfaces'):void {
    for(const name of ['space','graph','interfaces']){
      const active=name===pane;this.el(`panel-${name}`).hidden=!active;
      this.el(`tab-${name}`).setAttribute('aria-selected',String(active));this.el(`tab-${name}`).tabIndex=active?0:-1;
    }
  }
  private input(id:string):HTMLInputElement {return this.el(id) as HTMLInputElement;}
  private select(id:string):HTMLSelectElement {return this.el(id) as HTMLSelectElement;}
  private button(id:string,callback:()=>void):void {this.el(id).addEventListener('click',()=>this.run(callback));}
  private run(callback:()=>void):void {try{if(this.disposed)return;callback();}catch(error){this.report(`失败：${errorText(error)}`);this.render();}finally{this.syncConnection();}}
  private report(message:string):void {this.el('status').textContent=message;this.el('status').dataset.level=/失败|错误/.test(message)?'error':'info';try{this.options.onStatus?.(message);}catch{/* Observer. */}}
  private output(id:string){
    let output=this.outputs.get(id);if(output)return output;
    const article=element('article',undefined,'wb-output');article.dataset.outputId=id;
    const title=element('h3',id),root=element('div',undefined,'wb-output-body'),status=element('p','等待数据','hint');
    article.append(title,root,status);this.el('outputs').append(article);output={article,title,root,status};this.outputs.set(id,output);return output;
  }
  private render():void {
    if(this.disposed)return;const workspace=this.controller.snapshot(),stats=this.controller.stats();
    if(this.wasRunning&&!stats.running)this.report('显示已暂停。启用选择已保留，重新启动后继续观测。');
    this.wasRunning=stats.running;
    this.el('frame-label').textContent=workspace.fixedFrame;
    this.el('display-count').textContent=String(workspace.displays.length);
    this.el('scene-empty').hidden=stats.spatialDisplays>0;
    this.el('outputs-empty').hidden=stats.activeDisplays>stats.spatialDisplays;
    this.el('session').dataset.running=String(stats.running);
    this.el('session').textContent=`${stats.running?'观测中':'已暂停'} · ${stats.activeDisplays} / ${workspace.displays.length} 显示 · ${stats.spatialDisplays} 路空间显示`;
    this.input('frame').value=workspace.fixedFrame;this.input('tf').value=workspace.tf;this.input('static').value=workspace.tfStatic;this.input('clock').checked=workspace.useSimTime;this.select('layout').value=workspace.layout;this.el('grid').dataset.layout=workspace.layout;
    const fragment=document.createDocumentFragment();this.statusRows.clear();
    for(const spec of workspace.displays){
      const li=element('li',undefined,'wb-display');li.dataset.displayId=spec.id;if(spec.id===this.selected)li.classList.add('selected');
      const enabled=element('input');enabled.type='checkbox';enabled.checked=spec.enabled;enabled.setAttribute('aria-label',`启用 ${spec.name}`);enabled.addEventListener('change',()=>this.run(()=>{this.controller.setEnabled(spec.id,enabled.checked);this.renderEditor();}));
      const name=element('button',spec.name,'wb-display-name');name.type='button';name.title=`${this.registry.get(spec.plugin).title} · ${spec.id}`;name.addEventListener('click',()=>{this.selected=spec.id;this.render();this.renderEditor();});
      const remove=element('button','×');remove.type='button';remove.dataset.remove='';remove.setAttribute('aria-label',`删除 ${spec.name}`);remove.addEventListener('click',()=>this.run(()=>{this.controller.remove(spec.id);if(this.selected===spec.id)this.selected=undefined;this.render();this.renderEditor();}));
      const status=element('small',this.displayMessages.get(spec.id)??(spec.enabled?'等待启动':'未启用'),'wb-display-status');this.statusRows.set(spec.id,status);li.dataset.issue=String(spec.enabled&&stats.running&&/缺少|失败|错误|invalid|error|timeout|超时/i.test(status.textContent??''));status.title=status.textContent??'';
      name.setAttribute('aria-pressed',String(spec.id===this.selected));
      const source=element('span',Object.values(spec.inputs).map(input=>input.topic).join(' · ')||'工作区 TF','wb-display-source');source.title=source.textContent??'';
      const badge=element('span',!spec.enabled?'已关闭':stats.running?'运行中':'待启动','wb-display-badge');badge.dataset.state=!spec.enabled?'off':stats.running?'active':'ready';
      li.append(enabled,name,badge,remove,source,status);fragment.append(li);
      const output=this.outputs.get(spec.id);if(output){output.title.textContent=spec.name;output.article.hidden=!spec.enabled||!stats.running;}
    }
    this.el('displays').replaceChildren(fragment);
    for(const [id,output] of this.outputs)if(!workspace.displays.some(spec=>spec.id===id)){output.article.remove();this.outputs.delete(id);}
    for(const id of this.displayMessages.keys())if(!workspace.displays.some(spec=>spec.id===id))this.displayMessages.delete(id);
    this.syncConnection();
  }
  private renderEditor():void {
    const root=this.el('editor');root.replaceChildren();const spec=this.controller.snapshot().displays.find(spec=>spec.id===this.selected);
    this.el('editor-title').textContent=spec?this.registry.get(spec.plugin).title:'未选择';
    if(!spec){root.append(element('p','点击显示名称编辑输入和属性。','hint'));return;}
    const field=(label:string,input:HTMLElement)=>{const node=element('label',label);node.append(input);root.append(node);};
    const name=element('input');name.name='displayName';name.value=spec.name;name.maxLength=120;field('显示名称',name);
    for(const [port,input] of Object.entries(spec.inputs)){
      const section=element('fieldset');section.dataset.port=port;section.append(element('legend',({description:'模型描述',joints:'关节状态',data:'消息输入'} as Record<string,string>)[port]??port));
      const add=(label:string,node:HTMLInputElement|HTMLSelectElement|HTMLTextAreaElement,key:string)=>{node.name=key;const field=element('label',label);field.append(node);section.append(field);};
      const topic=element('input');topic.value=input.topic;topic.required=true;topic.setAttribute('list',this.listId);add('Topic',topic,'topic');
      const type=element('input');type.value=input.type;type.required=true;add('消息类型',type,'type');
      topic.addEventListener('change',()=>{const types=this.graphSnapshot?.topics.find(item=>item.name===topic.value)?.types;if(types?.length===1)type.value=types[0];});
      const qos=element('select');for(const value of ['sensor','reliable','latched','staticTf','custom'])qos.append(option(value));qos.value=typeof input.qos==='string'?input.qos:'custom';add('QoS',qos,'qos');
      const transport=element('details',undefined,'wb-advanced');transport.append(element('summary','传输设置 · QoS / 编码'));
      const custom=element('textarea');custom.rows=3;custom.value=JSON.stringify(typeof input.qos==='object'?input.qos:{history:'keep_last',depth:10,reliability:'reliable',durability:'volatile'},null,2);add('自定义 QoS JSON',custom,'customQos');custom.parentElement!.hidden=qos.value!=='custom';qos.addEventListener('change',()=>{custom.parentElement!.hidden=qos.value!=='custom';});
      const throttle=element('input');throttle.type='number';throttle.min='0';throttle.max='60000';throttle.step='1';throttle.value=String(input.throttleMs??0);add('订阅间隔 / ms',throttle,'throttleMs');
      const compression=element('select');compression.append(option('none','JSON'),option('cbor','CBOR'));compression.value=input.compression??'none';add('传输',compression,'compression');
      for(const node of [qos,custom,throttle,compression])transport.append(node.parentElement!);section.append(transport);root.append(section);
    }
    if(spec.plugin==='tf')root.append(element('p','TF 输入、固定坐标系与时钟使用工作区全局设置。','hint'));
    const options=element('textarea');options.name='displayOptions';options.rows=4;options.value=JSON.stringify(spec.options,null,2);field('选项 JSON',options);const advanced=element('details',undefined,'wb-advanced');advanced.append(element('summary','显示选项 · JSON'),options.parentElement!);root.append(advanced);
    root.append(element('p',spec.plugin==='plot'?'fields: 1–8 个字段；maxPoints ≤ 600。':spec.plugin==='image'?'16UC1 必须填写 depthScale（米 / 原始单位）；maxPixels ≤ 4000000。':spec.plugin==='cloud'?'maxPoints ≤ 100000；总云点数预算为 400000。':'未知消息可选择“原始消息”适配器。','hint'));
    const submit=element('button','应用显示属性');submit.type='submit';root.append(submit);
  }
  private applyEditor():void {
    if(!this.selected)throw Error('先选择显示');const form=this.el('editor') as HTMLFormElement;
    const read=(root:ParentNode,name:string)=>(root.querySelector(`[name="${name}"]`) as HTMLInputElement).value;
    const inputs:Record<string,TopicBinding>={};
    for(const section of form.querySelectorAll<HTMLElement>('[data-port]'))inputs[section.dataset.port!]={topic:read(section,'topic').trim(),type:read(section,'type').trim(),qos:read(section,'qos')==='custom'?JSON.parse(read(section,'customQos')):read(section,'qos') as TopicOptions['qos'],throttleMs:Number(read(section,'throttleMs')),compression:read(section,'compression') as TopicOptions['compression']};
    this.controller.update(this.selected,{name:read(form,'displayName').trim(),inputs,options:JSON.parse(read(form,'displayOptions'))});this.renderEditor();this.report('显示属性已更新。');
  }
  private add():void {
    const definition=this.registry.get(this.select('plugin').value),inputs=JSON.parse(JSON.stringify(definition.defaults?.inputs??{})) as Record<string,TopicBinding>;
    const topic=this.input('add-topic').value.trim(),first=definition.ports[0];
    if(topic&&first){const types=this.graphSnapshot?.topics.find(item=>item.name===topic)?.types??[];inputs[first.id].topic=topic;
      const type=types.find(type=>first.types.includes('*')||first.types.includes(type));if(type)inputs[first.id].type=type;
      // A same-Topic display starts from its peer's exact options. This is SDK
      // sharing, not a second DDS reader with independently selectable QoS.
      const peer=this.controller.snapshot().displays.flatMap(spec=>Object.values(spec.inputs)).find(input=>input.topic===topic&&(first.types.includes('*')||first.types.includes(input.type)));if(peer)inputs[first.id]={...peer};
    }
    const existing=this.controller.snapshot().displays;let n=1;while(existing.some(spec=>spec.id===`${definition.id}_${n}`))n++;
    const spec:DisplaySpec={id:`${definition.id}_${n}`,name:`${definition.title} ${n}`,plugin:definition.id,enabled:true,inputs,options:definition.defaults?.options??{}};
    this.controller.add(spec);this.selected=spec.id;this.render();this.renderEditor();this.report('已添加显示；暂停状态下需点击“启动所选显示”。');
  }
  private async importFile():Promise<void> {
    const file=this.input('file').files?.[0],sequence=++this.importSequence;if(!file)return;
    try{
      if(file.size>WORKSPACE_LIMITS.jsonBytes)throw Error('配置 JSON 不能超过 1 MiB');const text=await file.text();
      if(this.disposed||sequence!==this.importSequence)return;const workspace=parseWorkspace(text,this.registry);this.controller.replace(workspace);this.selected=undefined;this.renderEditor();this.report('已导入配置，显示保持暂停。');
    }catch(error){if(!this.disposed)this.report(`导入失败：${errorText(error)}`);}finally{if(!this.disposed)this.input('file').value='';}
  }
  private download(name:string,value:unknown):void {
    const url=URL.createObjectURL(new Blob([JSON.stringify(value,null,2)],{type:'application/json'})),link=element('a');link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),0);
  }
  private filterGraph():void {this.graphView.filter({text:this.input('graph-search').value,hideInternal:this.input('graph-internal').checked,mode:this.select('graph-mode').value as 'topics'|'nodes',layout:this.select('graph-layout').value as 'breadthfirst'|'grid'});this.renderGraphState();}
  private renderGraphState():void {
    const snapshot=this.graphSnapshot;if(!snapshot)return;
    const visible=this.graphView.stats();
    this.el('graph-empty').hidden=(visible.nodes??0)+(visible.topics??0)>0;
    this.el('graph-empty-title').textContent='没有可显示的节点或 Topic';
    this.el('graph-empty-body').textContent='尝试修改搜索条件、显示内部 Topic，或重新刷新图谱。';
    const status={complete:'完整快照（rosapi 可见范围）',partial:'部分快照',failed:'采集失败'}[snapshot.status];
    this.el('graph-state').textContent=`${this.graphStale?'上一会话快照已过期，请重新刷新 · ':this.graphWait?'采集中，当前显示最近快照 · ':''}${status} · ${this.graphSource??'未知来源'} · ${new Date(snapshot.receivedAt).toLocaleTimeString()} · ${snapshot.nodes.length} 节点 / ${snapshot.topics.length} Topic / ${snapshot.edges.length} 声明边 · 当前视图 ${visible.nodes??0} 节点 / ${visible.topics??0} Topic · ${(snapshot.elapsedMs/1000).toFixed(2)} s${this.graphView.stats().projectionTruncated?' · 节点投影视图已截断，当前视图不完整':''}`;
    this.el('interface-state').textContent=this.el('graph-state').textContent;
  }
  private selection(item:GraphSelection):void {
    this.select('selection').value=item.id;
    const snapshot=this.graphSnapshot;if(!snapshot)return;
    const nodes=new Map(snapshot.nodes.map(node=>[node.id,node.name]));
    const lines=(values:string[])=>values.length?values.slice(0,32).join('\n')+(values.length>32?`\n… 共 ${values.length} 项`:''):'未发现';
    let detail:string;
    if(item.kind==='topic'){
      const topic=snapshot.topics.find(topic=>topic.id===item.id)!;
      const publishers=snapshot.edges.filter(edge=>edge.target===item.id&&edge.kind==='publish').map(edge=>nodes.get(edge.source)??edge.source);
      const subscribers=snapshot.edges.filter(edge=>edge.source===item.id&&edge.kind==='subscribe').map(edge=>nodes.get(edge.target)??edge.target);
      detail=`Topic\n${topic.name}\n\n消息类型\n${lines(topic.types)}\n\n发布节点 (${publishers.length})\n${lines(publishers)}\n\n订阅节点 (${subscribers.length})\n${lines(subscribers)}`;
    }else{
      const node=snapshot.nodes.find(node=>node.id===item.id)!;
      const publishing=snapshot.edges.filter(edge=>edge.source===item.id&&edge.kind==='publish').map(edge=>edge.topic);
      const subscribing=snapshot.edges.filter(edge=>edge.target===item.id&&edge.kind==='subscribe').map(edge=>edge.topic);
      detail=`节点\n${node.name}\n${node.duplicates>1?'重名节点：关系归属存在歧义':node.details==='known'?'已获取声明关系':'未获取完整关系'}\n\n发布 Topic (${publishing.length})\n${lines(publishing)}\n\n订阅 Topic (${subscribing.length})\n${lines(subscribing)}`;
    }
    // The inspector shows usable ROS names, not renderer-internal ids. Keep the
    // full graph in the explicit export while bounding the visible text.
    this.el('selection-detail').textContent=detail.length>8192?detail.slice(0,8180)+'\n…':detail;
  }
  private stopGraphRound():void {this.graphSequence++;this.graphWait?.abort();this.graphWait=undefined;this.inspectWait?.abort();this.inspectWait=undefined;this.syncGraphButtons();}
  private stopGraph():void {this.stopGraphRound();if(this.autoTimer)clearInterval(this.autoTimer);this.autoTimer=undefined;this.input('graph-auto').checked=false;}
  private syncGraphButtons():void {
    const online=!!this.options.getControlClient()?.connected;
    (this.el('graph-refresh') as HTMLButtonElement).disabled=!online||!!this.graphWait;
    (this.el('inspect') as HTMLButtonElement).disabled=!online||this.graphStale||!!this.inspectWait||!this.select('endpoint').value;
    (this.el('graph-export') as HTMLButtonElement).disabled=!this.graphSnapshot;
  }
  async refreshGraph():Promise<void> {
    if(this.disposed||this.graphWait||document.hidden)return;const client=this.options.getControlClient();if(!client?.connected)return;
    const sequence=++this.graphSequence,wait=new AbortController();this.graphWait=wait;this.syncGraphButtons();this.el('graph-state').textContent='采集中…（10 秒整体截止 / 并发 4）';
    try{
      const snapshot=await discoverGraph(client,{signal:wait.signal});if(this.disposed||sequence!==this.graphSequence||client!==this.options.getControlClient())return;
      const source=new URL(client.options.url);source.username='';source.password='';this.graphSource=source.origin+source.pathname;
      this.graphSnapshot=snapshot;this.graphStale=false;this.graphView.setSnapshot(snapshot);this.filterGraph();
      this.el('graph-errors').textContent=JSON.stringify({scope:snapshot.scope,capabilities:snapshot.capabilities,errors:snapshot.errors,omittedErrors:snapshot.omittedErrors},null,2);
      const selected=this.select('selection');selected.replaceChildren(...[...snapshot.nodes.map(item=>option(item.id,`节点 · ${item.name}${item.duplicates>1?'（重名）':''}`)),...snapshot.topics.map(item=>option(item.id,`Topic · ${item.name}`))]);
      this.el('topics').replaceChildren(...snapshot.topics.map(item=>option(item.name,item.types.join(' | '))));
      this.el('services').textContent=snapshot.services.map(item=>`${item.name}\n  提供者：${item.providers.join(', ')||'未取得证据'}`).join('\n');this.el('actions').textContent=snapshot.actions.map(item=>item.name).join('\n')||'未发现 Action 服务端';this.endpoints();
    }catch(error){if(!this.disposed&&sequence===this.graphSequence)this.el('graph-state').textContent=`采集失败：${errorText(error)}`;}
    finally{if(this.graphWait===wait)this.graphWait=undefined;if(!this.disposed){this.renderGraphState();this.syncGraphButtons();}}
  }
  private endpoints():void {
    const kind=this.select('endpoint-kind').value,snapshot=this.graphSnapshot;
    const values=kind==='service'?snapshot?.services:kind==='action'?snapshot?.actions:snapshot?.topics;
    this.select('endpoint').replaceChildren(...(values??[]).map(item=>option(item.name)));this.syncGraphButtons();
  }
  private async inspect():Promise<void> {
    const client=this.options.getControlClient(),kind=this.select('endpoint-kind').value,name=this.select('endpoint').value;
    if(!client?.connected||!name||this.inspectWait)return;const wait=new AbortController();this.inspectWait=wait;this.syncGraphButtons();
    try{
      const title=kind[0].toUpperCase()+kind.slice(1);
      const {type}=await client.callService<{type:string}>(`/rosapi/${kind}_type`,`rosapi_msgs/srv/${title}Type`,{[kind]:name},{timeoutMs:3000,signal:wait.signal});
      if(!/^[A-Za-z_][A-Za-z0-9_]*\/(msg|srv|action)\/[A-Za-z_][A-Za-z0-9_]*$/.test(type))throw Error('rosapi 未返回有效的 ROS 2 类型');
      const definitions=kind==='topic'?[['message','MessageDetails']]:kind==='service'?[['service_request','ServiceRequestDetails'],['service_response','ServiceResponseDetails']]:[['action_goal','ActionGoalDetails'],['action_feedback','ActionFeedbackDetails'],['action_result','ActionResultDetails']];
      const details=await Promise.all(definitions.map(async([endpoint,service])=>[endpoint,await client.callService(`/rosapi/${endpoint}_details`,`rosapi_msgs/srv/${service}`,{type},{timeoutMs:3000,signal:wait.signal})]));
      if(!this.disposed&&!wait.signal.aborted)this.el('interface').textContent=previewMessage({name,type,definitions:Object.fromEntries(details)},9);
    }catch(error){if(!this.disposed&&!wait.signal.aborted)this.el('interface').textContent=`读取失败：${errorText(error)}`;}
    finally{if(this.inspectWait===wait)this.inspectWait=undefined;if(!this.disposed)this.syncGraphButtons();}
  }
  syncConnection():void {
    if(this.disposed)return;this.controller.syncConnection();const client=this.options.getControlClient();
    if(client!==this.lastControl||!client?.connected){this.stopGraph();this.lastControl=client;if(this.graphSnapshot){this.graphStale=true;this.renderGraphState();}}
    (this.el('run') as HTMLButtonElement).disabled=!this.options.getObservationClient()?.connected||this.controller.stats().running;
    (this.el('pause') as HTMLButtonElement).disabled=!this.controller.stats().running;this.syncGraphButtons();
  }
  setProfile(profile:Record<string,unknown>):void {this.importSequence++;this.controller.replace(workspaceFromProfile(profile,this.registry));this.selected=undefined;this.renderEditor();this.report('已切换项目模板，显示默认关闭。');}
  pause():void {this.controller.pause();this.stopGraph();}
  stats(){return {workspace:this.controller.snapshot(),controller:this.controller.stats(),displayStatuses:Object.fromEntries(this.displayMessages),graph:this.graphView.stats(),graphSnapshot:this.graphSnapshot,graphSource:this.graphSource,graphStale:this.graphStale,graphRefreshing:!!this.graphWait};}
  dispose():void {
    if(this.disposed)return;this.disposed=true;this.importSequence++;this.stopGraph();clearInterval(this.metricTimer);document.removeEventListener('visibilitychange',this.visibility);this.controller.dispose();this.graphView.dispose();this.outputs.clear();this.statusRows.clear();this.displayMessages.clear();this.root.replaceChildren();
  }
}
