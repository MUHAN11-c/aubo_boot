/** Optional graph renderer. Cytoscape owns layout; ROS discovery stays separate. */
import cytoscape from 'cytoscape';
import {graphElements} from './model.js';
import type {GraphSnapshot,GraphElement,GraphNode,GraphTopic} from './model.js';
import {errorText} from '../sdk/errors.js';

export interface GraphFilter {text?:string;hideInternal?:boolean;mode?:'topics'|'nodes';layout?:'breadthfirst'|'grid'}
export interface GraphSelection {id:string;name:string;kind:'node'|'topic';details:GraphNode|GraphTopic}
const internal=(name:string)=>name==='/rosout'||name==='/parameter_events'||name.startsWith('/rosapi/')||name.split('/').some(part=>part.startsWith('_'));
const label=(value:string)=>value.length<=80?value:`${value.slice(0,24)}…${value.slice(-48)}`;

/** Pass no container for headless layout/contract use, not browser rendering.
 * One core instance is reused across refreshes. dispose removes this adapter's
 * event listener, observer, graph elements and the library's rendering loop.
 */
export class GraphView {
  private cy:cytoscape.Core;
  private snapshot?:GraphSnapshot;
  private selectedFilter:Required<GraphFilter>={text:'',hideInternal:false,mode:'topics',layout:'breadthfirst'};
  private resize?:ResizeObserver;
  private hasVisibleSize=false;
  private disposed=false;
  private truncated=false;
  private lastError?:string;
  private readonly onTap=(event:cytoscape.EventObject)=>this.select(event.target.id(),false);
  private readonly onZoom=()=>{try{this.options.onViewportChange?.(this.cy.zoom());}catch(error){this.lastError=errorText(error);}};
  constructor(private readonly root?:HTMLElement,private readonly options:{onSelect?:(item:GraphSelection)=>void;onViewportChange?:(zoom:number)=>void}={}){
    this.cy=cytoscape({container:root,headless:!root,styleEnabled:!!root,elements:[],
      minZoom:.05,maxZoom:4,pixelRatio:Math.min(globalThis.devicePixelRatio??1,2),
      layout:{name:'preset'},style:[
        {selector:'node',style:{label:(element:cytoscape.NodeSingular)=>label(String(element.data('label'))),'font-size':14,'text-wrap':'wrap','text-overflow-wrap':'anywhere','text-max-width':'180px','text-valign':'center','text-halign':'center','background-color':'#e2edf7','border-color':'#688dab','border-width':1,color:'#24465b',width:190,height:'label',padding:'10px'}},
        {selector:'node[kind="node"]',style:{shape:'round-rectangle'}},
        {selector:'node[kind="topic"]',style:{shape:'ellipse','background-color':'#e5f3e9','border-color':'#81a38b'}},
        {selector:'node[duplicates > 1]',style:{'border-color':'#bb6f31','border-width':3}},
        {selector:'edge',style:{label:(element:cytoscape.EdgeSingular)=>label(String(element.data('label')??'')),'font-size':9,'text-background-color':'#f7f5ed','text-background-opacity':.9,'curve-style':'bezier',width:1.4,'line-color':'#8097a8','target-arrow-color':'#8097a8','target-arrow-shape':'triangle','arrow-scale':.8}},
        {selector:'.muted',style:{opacity:.18}},
        {selector:'.focused',style:{'border-color':'#146b91','line-color':'#146b91','target-arrow-color':'#146b91',opacity:1}},
      ]});
    this.cy.on('tap','node',this.onTap);
    this.cy.on('zoom',this.onZoom);
    if(root){
      root.setAttribute('aria-label','ROS节点和Topic声明关系图；可缩放、平移及选择邻居');
      if(typeof ResizeObserver!=='undefined'){
        this.resize=new ResizeObserver(()=>{
          if(this.disposed||!root.clientWidth||!root.clientHeight)return;
          this.cy.resize();
          // A tab hides its container without ending the graph session. Fitting
          // on every resize silently discards the user's zoom and pan on return.
          if(!this.hasVisibleSize){this.hasVisibleSize=true;this.fit();}
        });this.resize.observe(root);
      }
    }
  }
  setSnapshot(snapshot:GraphSnapshot):void {
    if(this.disposed)throw new Error('Graph view is disposed');
    this.snapshot=snapshot;this.draw();
  }
  filter(filter:GraphFilter):void {
    if(this.disposed)throw new Error('Graph view is disposed');
    if(filter.mode!==undefined&&!['topics','nodes'].includes(filter.mode))throw new TypeError('Unknown graph mode');
    if(filter.text!==undefined&&(typeof filter.text!=='string'||filter.text.length>1024))throw new TypeError('Graph search must be a bounded string');
    if(filter.hideInternal!==undefined&&typeof filter.hideInternal!=='boolean')throw new TypeError('hideInternal must be boolean');
    if(filter.layout!==undefined&&!['breadthfirst','grid'].includes(filter.layout))throw new TypeError('Unknown graph layout');
    this.selectedFilter={...this.selectedFilter,...filter};this.draw();
  }
  private draw():void {
    if(!this.snapshot)return;
    const projected=graphElements(this.snapshot,{mode:this.selectedFilter.mode});this.truncated=projected.truncated;
    let elements:GraphElement[]=projected.elements;
    if(this.selectedFilter.hideInternal){
      const hidden=new Set(this.snapshot.topics.filter(t=>internal(t.name)).map(t=>t.id));
      const hiddenEdges=new Set(this.snapshot.edges.filter(e=>internal(e.topic)).map(e=>e.id));
      elements=elements.filter(e=>!hidden.has(e.data.id)&&!hiddenEdges.has(e.data.id)&&
        !(e.data.kind==='edge'&&e.data.label&&internal(e.data.label)));
    }
    const text=this.selectedFilter.text.trim().toLowerCase();
    if(text){
      const topics=new Set(this.snapshot.topics.filter(topic=>topic.name.toLowerCase().includes(text)||topic.types.some(type=>type.toLowerCase().includes(text))).map(topic=>topic.name));
      const nodes=new Set(elements.filter(e=>e.data.kind==='node'&&String(e.data.name??'').toLowerCase().includes(text)).map(e=>e.data.id));
      const selected=new Set(nodes),edges=new Set<string>();
      if(this.selectedFilter.mode==='nodes'){
        // Projected edges carry Topic names. Match only those edges or the
        // edges incident to a matched node, not every edge between its peers.
        for(const e of elements)if(e.data.source&&e.data.target&&(nodes.has(e.data.source)||nodes.has(e.data.target)||topics.has(e.data.label))){
          edges.add(e.data.id);selected.add(e.data.source);selected.add(e.data.target);
        }
      }else{
        const visibleTopics=new Set(elements.filter(e=>e.data.kind==='topic').map(e=>e.data.id));
        const relevantTopics=new Set(elements.filter(e=>e.data.kind==='topic'&&topics.has(e.data.name??'')).map(e=>e.data.id));
        // A Topic query needs one hop to its endpoints. A node query also
        // includes its Topics and their peers; never expand peers' other Topics.
        for(const e of elements)if(e.data.source&&e.data.target){
          if(nodes.has(e.data.source)&&visibleTopics.has(e.data.target))relevantTopics.add(e.data.target);
          if(nodes.has(e.data.target)&&visibleTopics.has(e.data.source))relevantTopics.add(e.data.source);
        }
        for(const id of relevantTopics)selected.add(id);
        for(const e of elements)if(e.data.source&&e.data.target&&(relevantTopics.has(e.data.source)||relevantTopics.has(e.data.target))){
          edges.add(e.data.id);selected.add(e.data.source);selected.add(e.data.target);
        }
      }
      elements=elements.filter(e=>e.data.kind==='edge'?edges.has(e.data.id):selected.has(e.data.id));
    }
    this.cy.batch(()=>{this.cy.elements().remove();this.cy.add(elements);});
    const bounds={x1:0,y1:0,w:Math.max(300,this.root?.clientWidth??1000),h:Math.max(300,this.root?.clientHeight??600)};
    this.cy.layout(this.selectedFilter.layout==='grid'
      ?{name:'grid',animate:false,padding:32,avoidOverlap:true,nodeDimensionsIncludeLabels:true,
        cols:Math.max(1,Math.floor(bounds.w/230)),boundingBox:bounds}
      :{name:'breadthfirst',directed:true,direction:'rightward',animate:false,padding:32,
        spacingFactor:1.4,nodeDimensionsIncludeLabels:true,boundingBox:bounds}).run();
    this.fit();
  }
  private select(id:string,fit:boolean):void {
    if(this.disposed||!this.snapshot)return;
    const node=this.cy.getElementById(id);if(node.empty()||!node.isNode())return;
    const kind=node.data('kind') as GraphSelection['kind'];
    const detail=kind==='node'?this.snapshot.nodes.find(n=>n.id===id):this.snapshot.topics.find(t=>t.id===id);
    if(!detail)return;
    this.cy.elements().addClass('muted').removeClass('focused');
    const neighbors=node.closedNeighborhood();neighbors.removeClass('muted').addClass('focused');
    if(fit)this.cy.fit(neighbors,50);
    try{this.options.onSelect?.({id,name:detail.name,kind,details:detail});}
    catch(error){this.lastError=errorText(error);}
  }
  focus(id:string):void {this.select(id,true);}
  fit():void {if(!this.disposed&&this.cy.elements().length)this.cy.fit(undefined,32);}
  /** Use Cytoscape's viewport API and limits; zoom about the visible centre. */
  setZoom(level:number):void {
    if(this.disposed)throw new Error('Graph view is disposed');
    if(!Number.isFinite(level)||level<=0)throw new TypeError('Zoom must be a finite positive number');
    this.cy.zoom({level:Math.max(this.cy.minZoom(),Math.min(this.cy.maxZoom(),level)),
      renderedPosition:{x:this.cy.width()/2,y:this.cy.height()/2}});
  }
  stats(){return {disposed:this.disposed,nodes:this.disposed?0:this.cy.nodes('[kind="node"]').length,
    topics:this.disposed?0:this.cy.nodes('[kind="topic"]').length,edges:this.disposed?0:this.cy.edges().length,
    zoom:this.cy.zoom(),layout:this.selectedFilter.layout,projectionTruncated:this.truncated,lastError:this.lastError};}
  dispose():void {
    if(this.disposed)return;this.disposed=true;this.resize?.disconnect();
    this.cy.off('tap','node',this.onTap);this.cy.off('zoom',this.onZoom);
    this.cy.elements().remove();this.cy.destroy();this.snapshot=undefined;
  }
}
