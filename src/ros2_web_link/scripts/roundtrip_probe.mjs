/** Browser-compatible SDK -> rosbridge -> ROS DDS -> CBOR SDK data roundtrip. */
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {RosClient} from '../dist/sdk/index.js';
import {imageBytes} from '../dist/media/image.js';
import {decodePointCloud} from '../dist/scene/pointcloud.js';
const hash=value=>createHash('sha256').update(value).digest('hex');
const client=new RosClient({url:process.argv[2]??'ws://127.0.0.1:19290',reconnect:false});
const topic='/web_link_demo/roundtrip_image';let off,publisher;
const pending=new Map();
try{
  await client.connect();
  off=client.subscribe(topic,'sensor_msgs/msg/Image',message=>{
    const wait=pending.get(message.header.frame_id);if(!wait)return;
    try{assert.equal(message.width,wait.width);assert.equal(message.height,wait.height);assert.equal(hash(imageBytes(message.data)),wait.hash);assert.ok(message.data instanceof Uint8Array);wait.resolve();}
    catch(error){wait.reject(error);}
  },{qos:'reliable',compression:'cbor'});
  publisher=client.publisher(topic,'sensor_msgs/msg/Image',{qos:'reliable'});
  await new Promise(resolve=>setTimeout(resolve,600));
  for(const [width,height] of [[640,480],[1280,720]]){
    const raw=Buffer.alloc(width*height*3);for(let i=0;i<raw.length;i++)raw[i]=i%251;
    const id=`roundtrip-${width}-${height}-${Date.now()}`;let timer;
    try{
      const result=new Promise((resolve,reject)=>{pending.set(id,{width,height,hash:hash(raw),resolve,reject});timer=setTimeout(()=>reject(Error('native DDS image roundtrip timeout')),5000);});
      publisher.publish({header:{stamp:{sec:Math.floor(Date.now()/1000),nanosec:0},frame_id:id},height,width,encoding:'rgb8',is_bigendian:0,step:width*3,data:raw.toString('base64')});
      await result;console.log(`PASS ${width}x${height} ${raw.length} bytes JSON/Base64 publish -> native ROS -> CBOR receive; exact SHA256`);
    }finally{clearTimeout(timer);pending.delete(id);}
  }
  // Exercise the full registered-cloud size seen in the actual Peach capture.
  // These are deterministic synthetic coordinates/padding, not camera evidence.
  off();publisher.dispose();off=undefined;publisher=undefined;
  const width=172372,pointStep=24,raw=Buffer.alloc(width*pointStep);
  for(let i=0;i<raw.length;i++)raw[i]=i%251;
  for(let i=0;i<width;i++){
    raw.writeFloatLE(i/1000,i*pointStep);
    raw.writeFloatLE(-.3,i*pointStep+4);
    raw.writeFloatLE(.8,i*pointStep+8);
    raw.writeUInt32LE(0x00ab8030,i*pointStep+16);
  }
  const fields=[['x',0],['y',4],['z',8],['rgb',16]].map(([name,offset])=>({name,offset,datatype:7,count:1}));
  const cloudTopic='/web_link_demo/roundtrip_cloud',expectedHash=hash(raw);
  for(const compression of ['none','cbor']){
    const id=`cloud-${compression}-${Date.now()}`;
    let timer;
    try{
      const result=new Promise((resolve,reject)=>{
        off=client.subscribe(cloudTopic,'sensor_msgs/msg/PointCloud2',message=>{
          if(message.header.frame_id!==id)return;
          try{
            assert.equal(message.width,width);assert.equal(message.height,1);
            assert.equal(message.point_step,pointStep);assert.equal(message.row_step,raw.length);
            assert.equal(message.is_bigendian,false);assert.equal(message.is_dense,true);
            assert.deepEqual(message.fields,fields);
            assert.equal(typeof message.data==='string',compression==='none');
            if(compression==='cbor')assert.ok(message.data instanceof Uint8Array);
            assert.equal(hash(imageBytes(message.data)),expectedHash);
            const cloud=decodePointCloud(message);
            assert.equal(cloud.count,Math.ceil(width/Math.ceil(width/100000)));
            assert.ok(cloud.colors);
            const stride=Math.ceil(width/100000);
            for(const index of [0,Math.floor(cloud.count/2),cloud.count-1]){
              const source=index*stride;
              assert.deepEqual(Array.from(cloud.positions.subarray(index*3,index*3+3)),
                [Math.fround(source/1000),Math.fround(-.3),Math.fround(.8)]);
              assert.deepEqual(Array.from(cloud.colors.subarray(index*3,index*3+3)),
                [0xab,0x80,0x30].map(value=>Math.fround(value/255)));
            }
            resolve();
          }catch(error){reject(error);}
        },{qos:'reliable',compression});
        timer=setTimeout(()=>reject(Error(`native DDS ${compression} full-cloud timeout`)),5000);
      });
      // Observe rejection immediately even if advertise/publish throws before
      // await result. The original promise still rejects at the awaited boundary.
      void result.catch(()=>{});
      // Discovery wait counts within the same absolute result timer. Never retry
      // a missing delivery, and never publish to an application/robot namespace.
      publisher=client.publisher(cloudTopic,'sensor_msgs/msg/PointCloud2',{qos:'reliable'});
        await new Promise(resolve=>setTimeout(resolve,600));
        publisher.publish({header:{stamp:{sec:Math.floor(Date.now()/1000),nanosec:0},frame_id:id},
          height:1,width,fields,is_bigendian:false,point_step:pointStep,row_step:raw.length,
          data:raw.toString('base64'),is_dense:true});
        await result;
        console.log(`PASS ${raw.length} bytes full PointCloud2 JSON/Base64 publish -> native ROS -> ${compression} receive; exact SHA256 ${expectedHash}, layout and sampled geometry`);
    }finally{clearTimeout(timer);off?.();publisher?.dispose();off=undefined;publisher=undefined;}
  }
}finally{off?.();publisher?.dispose();assert.equal(client.stats().subscriptions,0);assert.equal(client.stats().publishers,0);assert.equal(client.stats().pending,0);client.dispose();}
console.log('PASS data publishing is explicit; two images and full clouds preserved exactly; client resources zero');
