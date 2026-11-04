/** Real ImageView with controlled DOM/canvas boundaries. Count application RGBA
 * allocations only; this cannot measure browser canvas/GPU-internal copies.
 */
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {ImageView,decodeImage} from '../dist/media/image.js';

const originals=new Map(['document','ImageData','Uint8ClampedArray'].map(k=>[k,globalThis[k]]));
const NativeClamped=globalThis.Uint8ClampedArray;
let allocations=[],painted,callback,subscriptions=0,offs=0,stateOffs=0,paintCount=0;
let view;
const node=tag=>({tag,style:{},hidden:false,width:0,height:0,
  removeAttribute(){},getContext(){return {putImageData(data){painted=data;paintCount++;}};}});
const root={children:[],replaceChildren(...children){this.children=children;}};
const client={
  onState(){return()=>{stateOffs++;};},
  subscribe(_topic,_type,handler){callback=handler;subscriptions++;return()=>{offs++;};},
};
let groups=0;
try{
  globalThis.document={createElement:node};
  globalThis.ImageData=class {
    constructor(data,width,height){this.data=data;this.width=width;this.height=height;}
  };
  globalThis.Uint8ClampedArray=new Proxy(NativeClamped,{
    construct(target,args){const array=Reflect.construct(target,args);allocations.push(array);return array;},
  });
  view=new ImageView(root,client,{topic:'/image',type:'sensor_msgs/msg/Image',depthScale:.001});
  view.start();view.start();assert.equal(subscriptions,1);
  const cases=[
    {width:2,height:2,encoding:'bgr8',is_bigendian:0,step:8,
      data:new Uint8Array([1,2,3,4,5,6,99,99,7,8,9,10,11,12,88,88])},
    {width:640,height:480,encoding:'bgr8',is_bigendian:0,step:1920,
      data:new Uint8Array(640*480*3).map((_value,i)=>(i*31)%256)},
    {width:2,height:1,encoding:'16UC1',is_bigendian:1,step:4,
      data:new Uint8Array([3,232,0,0])},
  ];
  for(const message of cases){
    const expected=decodeImage(message,{depthScale:.001});
    allocations=[];painted=undefined;
    const before=paintCount;callback(message);
    assert.equal(paintCount,before+1,'actual ImageView did not paint');
    assert.equal(allocations.length,1,'raw frame must allocate only its decoded RGBA array');
    assert.equal(painted.data,allocations[0],'ImageData must receive the original decoded array');
    assert.equal(painted.width,message.width);assert.equal(painted.height,message.height);
    const hash=data=>createHash('sha256').update(data).digest('hex');
    assert.equal(hash(painted.data),hash(expected.pixels),'canvas payload differs from decoded pixels');
    assert.equal(root.children[1].hidden,false);assert.equal(root.children[0].hidden,true);
    groups++;console.log(`PASS ${message.encoding} ${message.width}x${message.height}: one ${painted.data.byteLength}B RGBA allocation, unchanged pixels`);
  }
  view.dispose();view.dispose();assert.equal(offs,1);assert.equal(stateOffs,1);
  assert.equal(root.children.length,0);assert.throws(()=>view.start(),/disposed/);
  groups++;console.log('PASS view owns one subscription/state listener and releases once');
}finally{
  view?.dispose();
  for(const [key,value] of originals)if(value===undefined)delete globalThis[key];else globalThis[key]=value;
}
console.log(`PASS ${groups} ImageView allocation/content/lifecycle groups; controlled DOM only`);
