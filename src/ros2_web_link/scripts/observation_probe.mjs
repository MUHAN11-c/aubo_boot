// Hardware-free CLI acceptance: npm run build, then node scripts/observation_probe.mjs.
// Uses the shipped pure decoders; no bridge, ROS session, DOM, ports or robot commands.
import assert from 'node:assert/strict';
import {TfBuffer} from '../dist/scene/tf.js';
import {QOS} from '../dist/sdk/index.js';
import {decodePointCloud} from '../dist/scene/pointcloud.js';
import {decodeImage, imageBytes} from '../dist/media/image.js';

let checks = 0;
const buffers = new Set();
function check(name, verify) {
  verify();
  checks++;
  console.log(`PASS ${name}`);
}
function near(actual, expected) {
  assert.ok(Math.abs(actual - expected) < 1e-5, `${actual} != ${expected}`);
}
function buffer(options = {}, client = {}) {
  const value = new TfBuffer(client, options);
  buffers.add(value);
  return value;
}

const field = (name, offset, datatype = 7, count = 1) => ({name, offset, datatype, count});
function cloud({
  width = 2, height = 1, point_step = 16, row_step = width * point_step,
  fields = [field('x', 0), field('y', 4), field('z', 8), field('rgb', 12)],
  is_bigendian = false, data,
} = {}) {
  return {width, height, point_step, row_step, fields, is_bigendian,
    data: data ?? new Uint8Array(row_step * height)};
}
function writePoints(message, records) {
  const view = new DataView(message.data.buffer, message.data.byteOffset, message.data.byteLength);
  for (let i = 0; i < records.length; i++) {
    const at = Math.floor(i / message.width) * message.row_step + i % message.width * message.point_step;
    for (let j = 0; j < 3; j++) view.setFloat32(at + j * 4, records[i][j], !message.is_bigendian);
    if (message.point_step >= 16) view.setUint32(at + 12, records[i][3] ?? 0, !message.is_bigendian);
  }
  return message;
}
const stamp = (sec, nanosec = 0) => ({sec, nanosec});
const transform = (parent, child, sec, x = 0, y = 0, z = 0, rotation = {x: 0, y: 0, z: 0, w: 1}) => ({
  header: {frame_id: parent, stamp: stamp(sec)}, child_frame_id: child,
  transform: {translation: {x, y, z}, rotation},
});
const ingest = (tf, ...items) => tf.ingest({transforms: items});
const stat = (tf, ...items) => tf.ingest({transforms: items}, true);
function image({width = 1, height = 1, encoding = 'rgb8', is_bigendian = false, step = 3, data = [1, 2, 3]} = {}) {
  return {width, height, encoding, is_bigendian, step, data};
}

try {
  check('PointCloud2 row padding / Base64 / FLOAT32 packed RGB with NaN alpha', () => {
    const message = writePoints(cloud({width: 2, height: 2, row_step: 40}), [
      [1, 2, 3, 0xffff0000], [4, 5, 6, 0xff00ff00], [7, 8, 9, 0xff0000ff], [NaN, 0, 0],
    ]);
    message.data = Buffer.from(message.data).toString('base64');
    const decoded = decodePointCloud(message);
    assert.equal(decoded.count, 3);
    assert.deepEqual([...decoded.positions], [1, 2, 3, 4, 5, 6, 7, 8, 9]);
    assert.deepEqual([...decoded.colors], [1, 0, 0, 0, 1, 0, 0, 0, 1]);
  });
  check('PointCloud2 big endian / UINT32 packed RGB / numeric byte arrays', () => {
    const message = writePoints(cloud({is_bigendian: true,
      fields: [field('x', 0), field('y', 4), field('z', 8), field('rgba', 12, 6)]}), [
      [-1, 2.5, 3, 0xff804020], [4, 5, 6, 0xff0000ff],
    ]);
    message.data = [...message.data];
    const decoded = decodePointCloud(message);
    near(decoded.positions[0], -1);
    near(decoded.colors[0], 128 / 255);
    near(decoded.colors[1], 64 / 255);
    near(decoded.colors[2], 32 / 255);
  });
  check('PointCloud2 all eight numeric datatypes / actual field offsets', () => {
    const setters = ['', 'setInt8', 'setUint8', 'setInt16', 'setUint16', 'setInt32', 'setUint32', 'setFloat32', 'setFloat64'];
    for (let datatype = 1; datatype <= 8; datatype++) {
      const message = cloud({width: 1, point_step: 32,
        fields: [field('z', 16, datatype), field('x', 0, datatype), field('y', 8, datatype)]});
      const view = new DataView(message.data.buffer);
      [0, 8, 16].forEach((at, i) => view[setters[datatype]](at, i + 1, true));
      assert.deepEqual([...decodePointCloud(message).positions], [1, 2, 3]);
    }
  });
  check('PointCloud2 separate RGB / Uint8Array byteOffset', () => {
    const message = cloud({width: 1, fields: [field('x', 0), field('y', 4), field('z', 8),
      field('r', 12, 2), field('g', 13, 2), field('b', 14, 2)]});
    message.data = new Uint8Array(new ArrayBuffer(24), 4, 16);
    writePoints(message, [[1, 2, 3]]);
    message.data.set([255, 128, 0], 12);
    const decoded = decodePointCloud(message);
    near(decoded.colors[0], 1);
    near(decoded.colors[1], 128 / 255);
    near(decoded.colors[2], 0);
  });
  check('PointCloud2 NaN / Infinity / Float64 overflow omitted', () => {
    const message = writePoints(cloud({width: 4}), [[1, 2, 3], [Infinity, 2, 3], [NaN, 2, 3], [1, 2, -Infinity]]);
    assert.equal(decodePointCloud(message).count, 1);
    const wide = cloud({width: 1, point_step: 24,
      fields: [field('x', 0, 8), field('y', 8, 8), field('z', 16, 8)]});
    new DataView(wide.data.buffer).setFloat64(0, 1e300, true);
    assert.equal(decodePointCloud(wide).count, 0);
  });
  check('PointCloud2 bounded uniform sampling', () => {
    const message = writePoints(cloud({width: 10}), Array.from({length: 10}, (_, i) => [i, 0, 0]));
    const decoded = decodePointCloud(message, {maxPoints: 3});
    assert.equal(decoded.count, 3);
    assert.deepEqual([...decoded.positions], [0, 0, 0, 4, 0, 0, 8, 0, 0]);
  });
  check('PointCloud2 byte budgets reject before allocation', () => {
    assert.throws(() => decodePointCloud(cloud(), {maxBytes: 31}), /maxBytes/);
    assert.throws(() => decodePointCloud({...cloud({width: 0, height: 0}), data: 'AAAA'}, {maxBytes: 1}), /maxBytes/);
    assert.throws(() => decodePointCloud(cloud({width: 0xffffffff, height: 0xffffffff,
      row_step: 0xffffffff, data: new Uint8Array()})), /safe integer|maxBytes|uint32/);
  });
  check('PointCloud2 malformed dimensions / strides / fields / bytes rejected', () => {
    const variants = [{row_step: 31}, {data: [0]}, {width: -1}, {width: 1.5}, {is_bigendian: 'false'}, {point_step: 0},
      {fields: [field('x', 15)]}, {fields: [field('x', 0, 9)]}, {fields: [field('x', 0, 7, 0)]},
      {fields: [field('x', 0), field('x', 4)]}, {data: '????'}, {data: Array(32).fill(256)},
      {fields: [field('x', 0), field('y', 4)]}];
    for (const variant of variants) assert.throws(() => decodePointCloud({...cloud(), ...variant}));
    assert.throws(() => decodePointCloud(cloud(), {maxPoints: 0}));
    assert.throws(() => decodePointCloud(cloud(), {maxBytes: Infinity}));
  });
  check('PointCloud2 valid empty cloud', () => {
    assert.equal(decodePointCloud(cloud({width: 0, height: 0, point_step: 0, row_step: 0, fields: []})).count, 0);
  });

  check('TF identity needs no history and still validates frame IDs / exact timestamps', () => {
    const tf = buffer();
    const identity = {translation: {x: 0, y: 0, z: 0}, rotation: {x: 0, y: 0, z: 0, w: 1}};
    assert.deepEqual(tf.lookup('map', 'map', stamp(123, 456)), identity);
    assert.deepEqual(tf.lookup('/map', 'map', stamp(0)), identity);
    assert.deepEqual(tf.lookup('map', 'map'), identity);
    assert.equal(tf.lookup('map', 'map', {sec: 0, nanosec: 1e9}), undefined);
    assert.equal(tf.lookup('map', 'map', {sec: 1.5, nanosec: 0}), undefined);
    assert.equal(tf.lookup('', '', stamp(0)), undefined);
    assert.equal(tf.lookup('/', '/', stamp(0)), undefined);
    assert.equal(tf.lookup('bad frame', 'bad frame', stamp(0)), undefined);
    assert.deepEqual(tf.stats(), {frames: 0, samples: 0, lastReceivedAt: 0});
  });
  check('TF static valid at arbitrary ROS times / normalized slash IDs', () => {
    const tf = buffer();
    stat(tf, transform('/map', '/base', 0, 1));
    near(tf.lookup('map', 'base', stamp(-2147483648)).translation.x, 1);
    near(tf.lookup('map', 'base', stamp(2147483647, 999999999)).translation.x, 1);
    near(tf.lookup('map', 'base').translation.x, 1);
  });
  check('TF dynamic interpolation and strict extrapolation rejection', () => {
    const tf = buffer({historySeconds: 30});
    ingest(tf, transform('map', 'tool', 10), transform('map', 'tool', 20, 10));
    near(tf.lookup('map', 'tool', stamp(15)).translation.x, 5);
    assert.equal(tf.lookup('map', 'tool', stamp(0)), undefined);
    assert.equal(tf.lookup('map', 'tool', stamp(9)), undefined);
    assert.equal(tf.lookup('map', 'tool', stamp(21)), undefined);
    near(tf.lookup('map', 'tool').translation.x, 10);
  });
  check('TF single dynamic sample allows only its exact timestamp', () => {
    const tf = buffer();
    ingest(tf, transform('map', 'tool', 10, 3));
    assert.equal(tf.lookup('map', 'tool', stamp(9)), undefined);
    assert.equal(tf.lookup('map', 'tool', stamp(11)), undefined);
    near(tf.lookup('map', 'tool', stamp(10)).translation.x, 3);
  });
  check('TF upstream quaternion composition / inverse / spherical interpolation', () => {
    const tf = buffer({historySeconds: 30});
    const half = Math.SQRT1_2;
    stat(tf, transform('map', 'base', 0, 1, 0, 0, {x: 0, y: 0, z: half, w: half}));
    ingest(tf, transform('base', 'tool', 10, 2));
    const pose = tf.lookup('map', 'tool', stamp(10));
    near(pose.translation.x, 1);
    near(pose.translation.y, 2);
    const inverse = tf.lookup('tool', 'map', stamp(10));
    near(inverse.translation.x, -2);
    near(inverse.translation.y, 1);
    const rotating = buffer({historySeconds: 30});
    ingest(rotating, transform('a', 'b', 10), transform('a', 'b', 20, 0, 0, 0, {x: 0, y: 0, z: 1, w: 0}));
    near(Math.abs(rotating.lookup('a', 'b', stamp(15)).rotation.z), half);
  });
  check('TF latest common time across a dynamic chain', () => {
    const tf = buffer({historySeconds: 30});
    ingest(tf, transform('a', 'b', 10), transform('a', 'b', 20, 10), transform('b', 'c', 12), transform('b', 'c', 18, 6));
    near(tf.lookup('a', 'c').translation.x, 14);
    assert.equal(tf.lookup('a', 'c', stamp(19)), undefined);
  });
  check('TF irrelevant ancestors do not invalidate the shortest path', () => {
    const tf = buffer({historySeconds: 100});
    ingest(tf, transform('world', 'base', 1, 99));
    stat(tf, transform('base', 'left', 0, 1), transform('base', 'right', 0, 2));
    near(tf.lookup('left', 'right', stamp(100)).translation.x, 1);
  });
  check('TF out-of-order samples do not reset the time epoch', () => {
    const tf = buffer({historySeconds: 30});
    ingest(tf, transform('map', 'tool', 20, 10));
    ingest(tf, transform('map', 'tool', 10));
    near(tf.lookup('map', 'tool', stamp(15)).translation.x, 5);
    assert.equal(tf.stats().samples, 2);
  });
  check('TF explicit Clock rollback clears dynamics and retains statics', () => {
    const tf = buffer();
    stat(tf, transform('map', 'base', 0, 1));
    ingest(tf, transform('base', 'tool', 10, 2));
    tf.ingestClock({clock: stamp(10)});
    tf.ingestClock({clock: stamp(9)});
    assert.equal(tf.lookup('map', 'tool'), undefined);
    near(tf.lookup('map', 'base', stamp(100)).translation.x, 1);
    assert.equal(tf.stats().samples, 1);
    assert.equal(tf.stats().lastReceivedAt, 0);
    ingest(tf, transform('base', 'tool', 9, 3));
    near(tf.lookup('map', 'tool', stamp(9)).translation.x, 4);
  });
  check('TF history duration / sample capacity / frame budgets', () => {
    const tf = buffer({historySeconds: 2, maxSamplesPerFrame: 1, maxFrames: 2});
    for (let second = 0; second < 100; second++) ingest(tf, transform('map', 'tool', second, second));
    assert.equal(tf.stats().samples, 1);
    assert.equal(tf.stats().frames, 2);
    ingest(tf, transform('map', 'extra', 99));
    assert.equal(tf.stats().frames, 2);
    assert.equal(tf.lookup('map', 'tool', stamp(98)), undefined);
    const history = buffer({historySeconds: 2, maxSamplesPerFrame: 100});
    for (let second = 0; second <= 5; second++) ingest(history, transform('a', 'b', second, second));
    assert.equal(history.stats().samples, 3);
    assert.equal(history.lookup('a', 'b', stamp(2)), undefined);
    near(history.lookup('a', 'b', stamp(3)).translation.x, 3);
  });
  check('TF reparent / static-dynamic takeover / cycle and invalid-data rejection', () => {
    const tf = buffer();
    stat(tf, transform('a', 'b', 0, 1));
    ingest(tf, transform('a', 'b', 2, 2));
    assert.equal(tf.stats().samples, 1);
    assert.equal(tf.lookup('a', 'b', stamp(1)), undefined);
    stat(tf, transform('a', 'b', 0, 3));
    near(tf.lookup('a', 'b', stamp(100)).translation.x, 3);
    ingest(tf, transform('b', 'a', 3));
    assert.equal(tf.stats().samples, 1);
    ingest(tf, transform('a', 'c', 3, NaN));
    assert.equal(tf.stats().frames, 2);
    ingest(tf, transform('a', 'd', 3, 0, 0, 0, {x: 0, y: 0, z: 0, w: 0}));
    assert.equal(tf.stats().frames, 2);
    ingest(tf, transform('new', 'b', 4, 4));
    assert.equal(tf.lookup('a', 'b'), undefined);
    near(tf.lookup('new', 'b', stamp(4)).translation.x, 4);
  });
  check('TF read-only QoS subscriptions / reconnect reset / resource cleanup', () => {
    assert.equal(QOS.staticTf?.depth,100,'Multiple static writers need bounded history beyond depth 1');
    const subscriptions = [];
    let state;
    let stopped = 0;
    // Minimal transport boundary; all tree and wire processing is production code.
    const client = {
      onState(fn) { state = fn; fn('connected'); return () => { stopped++; }; },
      subscribe(topic, type, fn, options) {
        subscriptions.push({topic, type, fn, options});
        return () => { stopped++; };
      },
    };
    const tf = buffer({clockTopic: '/clock'}, client);
    tf.start();
    tf.start();
    assert.deepEqual(subscriptions.map(item => [item.topic, item.type, item.options]), [
      ['/tf', 'tf2_msgs/msg/TFMessage', {qos: 'reliable'}],
      ['/tf_static', 'tf2_msgs/msg/TFMessage', {qos: 'staticTf'}],
      ['/clock', 'rosgraph_msgs/msg/Clock', {qos: 'sensor'}],
    ]);
    subscriptions[1].fn({transforms: [transform('a', 'b', 0)]});
    assert.equal(tf.stats().samples, 1);
    state('reconnecting');
    assert.deepEqual(tf.stats(), {frames: 0, samples: 0, lastReceivedAt: 0});
    tf.dispose();
    tf.dispose();
    assert.equal(stopped, 4);
    assert.throws(() => tf.start(), /disposed/);
    const noClock = buffer({clockTopic: false}, client);
    noClock.start();
    assert.equal(subscriptions.length, 5);
    noClock.dispose();
  });
  check('TF switching full dynamic history to static retains the new static sample', () => {
    const tf = buffer({historySeconds: 10, maxSamplesPerFrame: 4});
    for (let second = 95; second <= 100; second++) ingest(tf, transform('a', 'b', second, second));
    stat(tf, transform('a', 'b', 0, 3));
    near(tf.lookup('a', 'b', stamp(200)).translation.x, 3);
    assert.equal(tf.stats().samples, 1);
  });
  check('TF invalid timestamps / unknown frames / explicit clear', () => {
    const tf = buffer();
    stat(tf, transform('a', 'b', 0));
    assert.equal(tf.lookup('a', 'b', {sec: 0, nanosec: 1e9}), undefined);
    assert.equal(tf.lookup('a', 'missing'), undefined);
    near(tf.lookup('a', 'a').translation.x, 0);
    tf.clear();
    assert.deepEqual(tf.stats(), {frames: 0, samples: 0, lastReceivedAt: 0});
  });

  check('Image byte decoding supports Base64 / numeric arrays / Uint8Array views', () => {
    assert.deepEqual([...imageBytes('AQIDBA==')], [1, 2, 3, 4]);
    assert.deepEqual([...imageBytes('AQI=', 2)], [1, 2]);
    assert.throws(() => imageBytes('AQI=', 1), /byte budget/);
    assert.deepEqual([...imageBytes([1, 2, 3, 4])], [1, 2, 3, 4]);
    const view = new Uint8Array(new Uint8Array([99, 1, 2, 3, 4, 88]).buffer, 1, 4);
    assert.deepEqual([...imageBytes(view)], [1, 2, 3, 4]);
  });
  check('Image RGB8 / BGR8 / RGBA8 / BGRA8 channel order and alpha', () => {
    for (const [encoding, data, expected] of [
      ['rgb8', [10, 20, 30], [10, 20, 30, 255]],
      ['bgr8', [30, 20, 10], [10, 20, 30, 255]],
      ['rgba8', [10, 20, 30, 40], [10, 20, 30, 40]],
      ['bgra8', [30, 20, 10, 40], [10, 20, 30, 40]],
    ]) {
      const decoded = decodeImage(image({encoding, step: data.length, data}));
      assert.deepEqual({width: decoded.width, height: decoded.height}, {width: 1, height: 1});
      assert.deepEqual([...decoded.pixels], expected);
    }
  });
  check('Image mono8 / 8UC1 and row padding', () => {
    for (const encoding of ['mono8', '8UC1']) {
      const decoded = decodeImage(image({width: 2, height: 2, encoding, step: 3, data: [5, 6, 99, 7, 8, 88]}));
      assert.deepEqual([...decoded.pixels], [5, 5, 5, 255, 6, 6, 6, 255, 7, 7, 7, 255, 8, 8, 8, 255]);
    }
  });
  check('Image RGB row padding is skipped for every row', () => {
    const data = Buffer.from([1, 2, 3, 99, 88, 4, 5, 6, 77, 66]).toString('base64');
    const decoded = decodeImage(image({height: 2, step: 5, data}));
    assert.deepEqual([...decoded.pixels], [1, 2, 3, 255, 4, 5, 6, 255]);
  });
  check('Image big-endian 16UC1 requires explicit depthScale in meters', () => {
    const message = image({width: 2, encoding: '16UC1', is_bigendian: true, step: 6,
      data: new Uint8Array([0x03, 0xe8, 0, 0, 99, 88])});
    assert.throws(() => decodeImage(message), /depthScale/);
    for (const depthScale of [0, -1, NaN, Infinity]) assert.throws(() => decodeImage(message, {depthScale}), /depthScale/);
    const decoded = decodeImage(message, {depthScale: 0.001, maxDepth: 2});
    assert.deepEqual([...decoded.pixels], [128, 255, 128, 255, 0, 0, 0, 255]);
  });
  check('Image Float32 depth handles endianness / NaN / Infinity / byteOffset', () => {
    for (const is_bigendian of [false, true]) {
      const data = new Uint8Array(new ArrayBuffer(20), 4, 12);
      const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
      [2, NaN, Infinity].forEach((value, index) => view.setFloat32(index * 4, value, !is_bigendian));
      const decoded = decodeImage(image({width: 3, encoding: '32FC1', is_bigendian, step: 12, data}), {maxDepth: 2});
      assert.deepEqual([...decoded.pixels], [0, 0, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255]);
    }
  });
  check('Image pixel and byte budgets reject oversized inputs', () => {
    assert.throws(() => decodeImage(image({width: 2, data: [1, 2, 3, 4, 5, 6], step: 6}), {maxPixels: 1}), /pixel budget/);
    assert.throws(() => imageBytes([1, 2, 3], 2), /byte budget/);
    assert.throws(() => imageBytes(new Uint8Array(3), 2), /byte budget/);
    assert.throws(() => imageBytes('AQID', 2), /byte budget/);
    assert.throws(() => decodeImage(image({data: new Uint8Array(16_000_001)})), /byte budget/);
  });
  check('Image malformed dimensions / row step / truncation / unsupported encoding rejected', () => {
    for (const width of [0, -1, 1.5, Number.MAX_SAFE_INTEGER]) assert.throws(() => decodeImage(image({width})), /size|pixel budget/);
    assert.throws(() => decodeImage(image({step: 2})), /row step/);
    assert.throws(() => decodeImage(image({step: 4})), /truncated/);
    assert.throws(() => decodeImage(image({encoding: 'invalid'})), /Unsupported image encoding/);
    assert.throws(() => decodeImage(image(), {maxDepth: 0}), /maxDepth/);
  });
  console.log(`PASS ${checks} observation boundary groups; no ROS connection or browser was opened`);
} finally {
  for (const tf of buffers) tf.dispose();
  buffers.clear();
}
