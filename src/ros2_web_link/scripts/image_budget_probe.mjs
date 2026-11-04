/** Explicit boundary probe: no browser decoder or large pixel allocation.
 * Small header fixtures exercise the actual ImageView before Blob URL handoff.
 */
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {ImageView,decodeImage,imageBytes} from '../dist/media/image.js';
const savedDocument=globalThis.document;
const createUrl=URL.createObjectURL,revokeUrl=URL.revokeObjectURL;
let urlCalls=0,createdNodes=0,callback,reports=[],view;
const root={replaceChildren(){}};
const client={onState:()=>()=>{},subscribe(_topic,_type,handler){callback=handler;return()=>{};}};
const png=(width,height)=>{
  const data=new Uint8Array(33);data.set([137,80,78,71,13,10,26,10,0,0,0,13,73,72,68,82]);
  const header=new DataView(data.buffer);header.setUint32(16,width);header.setUint32(20,height);return data;
};
const jpeg=new Uint8Array([255,216,255,192,0,17,8,0,1,0,1,3,1,17,0,2,17,0,3,17,0,255,217]);
function make(options={}){
  view=new ImageView(root,client,{topic:'/image',type:'sensor_msgs/msg/CompressedImage',onStatus:text=>reports.push(text),...options});view.start();return view;
}
const cases={
  rawEncodingCannotResolveObjectPrototype(){
    for(const encoding of ['constructor','__proto__','toString','unknown'])assert.throws(()=>decodeImage({width:1,height:1,encoding,step:3,is_bigendian:0,data:new Uint8Array([1,2,3])}),/Unsupported image encoding/);
    assert.deepEqual([...decodeImage({width:1,height:1,encoding:'rgb8',step:3,is_bigendian:0,data:new Uint8Array([1,2,3])}).pixels],[1,2,3,255]);
  },
  numericArraysMustContainUint8Bytes(){
    for(const value of [-1,256,1.5,NaN,Infinity,'1',null,undefined])assert.throws(()=>imageBytes([value]),/uint8/);
    assert.throws(()=>imageBytes(new Array(1)),/uint8/,'sparse byte array silently filled with zero');
    const inherited=Object.create(Array.prototype);inherited[0]=42;const sparse=new Array(1);Object.setPrototypeOf(sparse,inherited);
    assert.throws(()=>imageBytes(sparse),/uint8/,'sparse byte array resolved an inherited element');
    assert.deepEqual([...imageBytes([0,1,128,255],4)],[0,1,128,255]);
  },
  invalidDataKindsDoNotAllocate(){
    assert.throws(()=>imageBytes(10,1),/data|uint8|array/i,'numeric data was treated as an allocation length');
    for(const data of [true,null,undefined,{length:10},new ArrayBuffer(2),new DataView(new ArrayBuffer(2)),new Uint16Array([1])])assert.throws(()=>imageBytes(data),/data|uint8|array/i);
  },
  uint8CrossRealmAndIntrinsicBrand(){
    const foreign=vm.runInNewContext('new Uint8Array([99,1,2,88]).subarray(1,3)');assert.deepEqual([...imageBytes(foreign,2)],[1,2]);assert.throws(()=>imageBytes(foreign,1),/byte budget/);
    const spoofed=new Uint16Array([1]);Object.defineProperty(spoofed,Symbol.toStringTag,{value:'Uint8Array'});assert.throws(()=>imageBytes(spoofed),/data|uint8|array/i);
  },
  unrelatedFormatStopsBeforeHeaderReader(){
    make();const data=new Uint8Array(64);new DataView(data.buffer).setUint32(0,64);data.set([102,116,121,112,104,101,105,99],4);
    let calls=0;const original=String.fromCodePoint;String.fromCodePoint=(...args)=>{calls++;return original(...args);};
    try{callback({format:'png',data});}finally{String.fromCodePoint=original;}
    assert.equal(urlCalls,0);assert.equal(calls,0,'non-PNG/JPEG bytes entered unrelated header parsers');assert.match(reports.at(-1),/header|Invalid|invalid/);
  },
  compressedJpegPixelLimit(){make({maxPixels:4});const data=jpeg.slice();new DataView(data.buffer).setUint16(7,3);new DataView(data.buffer).setUint16(9,2);callback({format:'jpeg',data});assert.equal(urlCalls,0);assert.match(reports.at(-1),/pixel budget/);},
  compressedPixelLimitBeforeBrowser(){make({maxPixels:4});callback({format:'png',data:png(3,2)});assert.equal(urlCalls,0,'oversized compressed header reached browser decode');assert.match(reports.at(-1),/pixel budget/);},
  compressedDepthIsNotOrdinaryPng(){make();callback({format:'16UC1; compressedDepth png',data:png(1,1)});assert.equal(urlCalls,0);assert.match(reports.at(-1),/compressedDepth/);},
  invalidHeaderNeverReachesBrowser(){make();callback({format:'png',data:new Uint8Array([137,80,78,71])});assert.equal(urlCalls,0);assert.match(reports.at(-1),/header|Invalid|invalid/);},
  codecMustMatchPayload(){make();callback({format:'jpeg',data:png(1,1)});assert.equal(urlCalls,0);assert.match(reports.at(-1),/format|codec|match/);},
  standardEmptyJpegFormatFallback(){make();callback({format:'',data:jpeg});assert.equal(urlCalls,1,'standard empty format JPEG fallback rejected');},
  validPngAndSubview(){make({maxPixels:1});const parent=new Uint8Array(45);parent.set(png(1,1),7);callback({format:'rgb8; png compressed rgb8',data:parent.subarray(7,40)});assert.equal(urlCalls,1);},
  invalidRawPixelBudgets(){const image={width:1,height:1,encoding:'rgb8',step:3,is_bigendian:0,data:new Uint8Array(3)};for(const maxPixels of [NaN,Infinity,1.5,0,-1])assert.throws(()=>decodeImage(image,{maxPixels}),/maxPixels|pixel budget/);},
  invalidByteBudgets(){for(const budget of [NaN,Infinity,1.5,0,-1])assert.throws(()=>imageBytes(new Uint8Array(1),budget),/maxBytes|byte budget/);},
  invalidViewBudgetBeforeDom(){const before=createdNodes;assert.throws(()=>make({maxPixels:Infinity}),/maxPixels|pixel budget/);assert.equal(createdNodes,before);},
};
let failed=0;
try{
  globalThis.document={createElement:tag=>{createdNodes++;return {tag,style:{},hidden:false,removeAttribute(){},getContext:()=>({})};}};
  URL.createObjectURL=()=>{urlCalls++;return `blob:fixture-${urlCalls}`;};URL.revokeObjectURL=()=>{};
  for(const [name,check] of Object.entries(cases)){
    urlCalls=0;reports=[];callback=undefined;
    try{check();console.log(`PASS ${name}`);}catch(error){failed++;console.error(`FAIL ${name}: ${error.message}`);}finally{view?.dispose();view=undefined;}
  }
}finally{
  if(savedDocument===undefined)delete globalThis.document;else globalThis.document=savedDocument;
  URL.createObjectURL=createUrl;URL.revokeObjectURL=revokeUrl;
}
assert.equal(failed,0,`${failed} image boundary groups failed`);
console.log(`PASS ${Object.keys(cases).length} image header/budget groups; no browser decoding or decompression test claim`);
