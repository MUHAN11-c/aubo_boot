/** Standard image boundary: no cv_bridge/numpy dependency in the web package. */
import type {RosClient,TopicOptions} from '../sdk/index.js';
import {imageDimensionsFromData} from 'image-dimensions';
export interface ImageOptions {topic:string;type:'sensor_msgs/msg/Image'|'sensor_msgs/msg/CompressedImage';depthScale?:number;maxDepth?:number;maxPixels?:number;subscription?:TopicOptions;onStatus?:(message:string)=>void}
export interface RawImage {width:number;height:number;encoding:string;is_bigendian:boolean|0|1;step:number;data:string|number[]|Uint8Array}
// Invoke the standard intrinsic getter, not a spoofable instance tag. This
// accepts Uint8Array views from another realm without accepting other views.
const typedArrayName=Object.getOwnPropertyDescriptor(Object.getPrototypeOf(Uint8Array.prototype),Symbol.toStringTag)!.get!;
// A Map cannot accidentally treat an inherited Object property as an encoding.
const encodingBytes=new Map<string,number>([
  ['rgb8',3],['bgr8',3],['rgba8',4],['bgra8',4],['mono8',1],['8uc1',1],['16uc1',2],['32fc1',4],
]);

function pixelBudget(value=4000000):number {
  if(!Number.isSafeInteger(value)||value<1)throw new TypeError('maxPixels must be a positive safe integer');
  return value;
}
function checkSize(width:number,height:number,limit:number):void {
  if(!Number.isSafeInteger(width)||!Number.isSafeInteger(height)||width<1||height<1||width*height>limit)throw new RangeError('Invalid image size or pixel budget exceeded');
}

/** Inspect encoded headers before the browser allocates a decoded image.
 * image-dimensions owns PNG/JPEG parsing; it does not decode pixel content.
 * compressedDepth has an additional transport header and explicit units, so
 * it must never enter the ordinary color PNG path merely by matching "png".
 */
function compressedImage(message:Record<string,unknown>,maxPixels:number){
  const format=String(message.format??'');
  if(/compressedDepth/i.test(format))throw new TypeError('compressedDepth is not an ordinary PNG/JPEG image; use raw depth with explicit units');
  if(/\btiff\b/i.test(format))throw new TypeError('Only PNG/JPEG compressed images are supported');
  const png=/\bpng\b/i.test(format),jpeg=/\b(?:jpeg|jpg)\b/i.test(format);
  if(png&&jpeg)throw new TypeError('Compressed image format has ambiguous codecs');
  // sensor_msgs/CompressedImage specifies JPEG for empty/unknown ordinary
  // compressed_image_transport format strings. Actual bytes must still match.
  const expected=png?'png':'jpeg';
  const bytes=imageBytes(message.data as RawImage['data']);
  // Keep non-target formats out of the general header reader altogether. This
  // is a fixed signature gate; dimensions still belong to the mature library.
  const pngSignature=[137,80,78,71,13,10,26,10];
  const encoded=pngSignature.every((byte,index)=>bytes[index]===byte)?'png':
    bytes[0]===255&&bytes[1]===216&&bytes[2]===255?'jpeg':undefined;
  if(!encoded)throw new TypeError('Invalid PNG/JPEG compressed image header');
  if(encoded!==expected)throw new TypeError('Compressed image format does not match the encoded header');
  const size=imageDimensionsFromData(bytes);
  if(!size||!['png','jpeg'].includes(size.type))throw new TypeError('Invalid PNG/JPEG compressed image header');
  if(size.type!==expected)throw new TypeError('Compressed image format does not match the encoded header');
  checkSize(size.width,size.height,maxPixels);
  return {bytes,mime:expected==='png'?'image/png':'image/jpeg'};
}

/** Share ImageView's encoded-byte/header/pixel boundary with custom previews.
 * Call before creating a Blob URL. Header checks do not decode pixel content.
 */
export function compressedImagePayload(message:Record<string,unknown>,options:Pick<ImageOptions,'maxPixels'>={}){
  return compressedImage(message,pixelBudget(options.maxPixels));
}

/** Decode rosbridge uint8[] without allowing unbounded browser allocation. */
export function imageBytes(data:string|number[]|Uint8Array,maxBytes=16000000):Uint8Array {
  if(!Number.isSafeInteger(maxBytes)||maxBytes<1)throw new TypeError('maxBytes must be a positive safe integer');
  const byteView=ArrayBuffer.isView(data)&&typedArrayName.call(data)==='Uint8Array';
  if(typeof data!=='string'&&!Array.isArray(data)&&!byteView)throw new TypeError('Image data must be a Base64 string, number array or Uint8Array');
  const length=typeof data==='string' ? Math.floor(data.length*3/4)-(data.endsWith('==')?2:data.endsWith('=')?1:0) : data.length;
  if(length>maxBytes) throw new RangeError('Image byte budget exceeded');
  if(typeof data==='string') return Uint8Array.from(atob(data),c=>c.charCodeAt(0));
  if(Array.isArray(data)){
    const bytes=new Uint8Array(data.length);
    for(let i=0;i<data.length;i++){
      if(!Object.hasOwn(data,i))throw new TypeError('Image data must contain uint8 bytes');
      const value:unknown=data[i];
      if(typeof value!=='number'||!Number.isInteger(value)||value<0||value>255)throw new TypeError('Image data must contain uint8 bytes');
      bytes[i]=value;
    }
    return bytes;
  }
  // A cross-realm Uint8Array shares bytes through a normalized local view.
  const {buffer,byteOffset,byteLength}=data;
  return data instanceof Uint8Array?data:new Uint8Array(buffer,byteOffset,byteLength);
}
/** Respect row padding, RGB/BGR order, endianness and explicit depth units.
 * 16UC1 has no universally valid unit: callers must specify meters per unit.
 */
export function decodeImage(message:RawImage,options:Pick<ImageOptions,'depthScale'|'maxDepth'|'maxPixels'>={}):{width:number;height:number;pixels:Uint8ClampedArray<ArrayBuffer>} {
  const {width,height,encoding,step}=message;
  checkSize(width,height,pixelBudget(options.maxPixels));
  const format=encoding.toLowerCase();
  const bpp=encodingBytes.get(format);
  if(bpp===undefined) throw new TypeError(`Unsupported image encoding: ${encoding}`);
  if(!Number.isSafeInteger(step)||step<width*bpp) throw new RangeError('Image row step is too small');
  const bytes=imageBytes(message.data);if(bytes.byteLength<step*height) throw new RangeError('Image data is truncated');
  const depth=format==='16uc1'||format==='32fc1';
  if(format==='16uc1' && (!Number.isFinite(options.depthScale)||Number(options.depthScale)<=0)) throw new TypeError('16UC1 requires depthScale in meters per unit');
  const maxDepth=options.maxDepth??5;if(!Number.isFinite(maxDepth)||maxDepth<=0)throw new TypeError('maxDepth must be positive meters');
  const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);const pixels=new Uint8ClampedArray(width*height*4);
  for(let y=0;y<height;y++)for(let x=0;x<width;x++) {
    const src=y*step+x*bpp,dst=(y*width+x)*4;pixels[dst+3]=255;
    if(depth) {
      const meters=format==='16uc1'?view.getUint16(src,!message.is_bigendian)*options.depthScale!:view.getFloat32(src,!message.is_bigendian);
      if(!Number.isFinite(meters)||meters<=0)continue;
      const t=Math.max(0,Math.min(1,meters/maxDepth));
      pixels[dst]=255*(1-t);pixels[dst+1]=255*(1-Math.abs(2*t-1));pixels[dst+2]=255*t;
    } else if(bpp===1) pixels[dst]=pixels[dst+1]=pixels[dst+2]=bytes[src];
    else {const bgr=format.startsWith('bgr');pixels[dst]=bytes[src+(bgr?2:0)];pixels[dst+1]=bytes[src+1];pixels[dst+2]=bytes[src+(bgr?0:2)];if(bpp===4)pixels[dst+3]=bytes[src+3];}
  }
  return {width,height,pixels};
}

/** Mount into any framework's DOM container. dispose() owns subscription/Blob cleanup. */
export class ImageView {
  private off?:()=>void;private url?:string;private image:HTMLImageElement;private canvas:HTMLCanvasElement;private disposed=false;private offState:()=>void;
  constructor(private root:HTMLElement,private client:RosClient,private options:ImageOptions) {
    pixelBudget(options.maxPixels);
    this.image=document.createElement('img');this.image.alt='ROS image';this.canvas=document.createElement('canvas');
    this.image.style.maxWidth=this.canvas.style.maxWidth='100%';this.image.hidden=this.canvas.hidden=true;root.replaceChildren(this.image,this.canvas);
    this.image.onload=()=>this.options.onStatus?.(`已显示 ${this.options.topic} · ${this.image.naturalWidth} × ${this.image.naturalHeight}`);
    this.image.onerror=()=>this.options.onStatus?.('图像内容解码失败');
    this.offState=client.onState(state=>{if(state!=='connected'){this.image.removeAttribute('src');this.image.hidden=this.canvas.hidden=true;if(this.url)URL.revokeObjectURL(this.url);this.url=undefined;this.options.onStatus?.('已断线，等待新图像');}});
  }
  start():void {
    if(this.disposed)throw new Error('Image view is disposed');if(this.off)return;
    this.off=this.client.subscribe<Record<string,unknown>>(this.options.topic,this.options.type,message=>{
      try {
        if(this.options.type==='sensor_msgs/msg/CompressedImage') {
          const {bytes,mime}=compressedImage(message,pixelBudget(this.options.maxPixels));
          const url=URL.createObjectURL(new Blob([new Uint8Array(bytes).buffer],{type:mime}));
          this.image.src=url;if(this.url)URL.revokeObjectURL(this.url);this.url=url;
          this.image.hidden=false;this.canvas.hidden=true;
        } else {
          const decoded=decodeImage(message as unknown as RawImage,this.options);
          if(this.canvas.width!==decoded.width)this.canvas.width=decoded.width;if(this.canvas.height!==decoded.height)this.canvas.height=decoded.height;
          // decodeImage owns a fresh ArrayBuffer. ImageData retains that view;
          // cloning it here would copy a complete RGBA frame on every update.
          this.canvas.getContext('2d')!.putImageData(new ImageData(decoded.pixels,decoded.width,decoded.height),0,0);
          this.canvas.hidden=false;this.image.hidden=true;
        }
        if(this.options.type==='sensor_msgs/msg/Image')this.options.onStatus?.(`已显示 ${this.options.topic} · ${this.canvas.width} × ${this.canvas.height}`);
      } catch(error) {this.options.onStatus?.((error as Error).message);}
    },{qos:'sensor',throttleMs:500,compression:'cbor',...this.options.subscription});
  }
  dispose():void {if(this.disposed)return;this.disposed=true;this.off?.();this.offState();this.image.onload=this.image.onerror=null;this.off=undefined;if(this.url)URL.revokeObjectURL(this.url);this.url=undefined;this.root.replaceChildren();}
}
