// Read-only CLI smoke check against an explicitly selected bridge and existing
// /tf_static writers. Uses retained DDS samples; never publishes robot commands.
import assert from 'node:assert/strict';
import {RosClient, QOS} from '../dist/sdk/index.js';

const url = process.argv[2];
assert(url, 'Usage: node scripts/static_tf_probe.mjs ws://HOST:PORT [frame,parent>child,...]');
const required = (process.argv[3] ||
  'world,camera_depth_optical_frame,tcp,wrist3_Link>camera_link').split(',');
const clients = [];
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));

async function subscribe(client, label) {
  const frames = new Set();
  const edges = new Set();
  const missing = () => required.filter(item => !(item.includes('>') ? edges : frames).has(item));
  let messages = 0;
  const off = client.subscribe('/tf_static', 'tf2_msgs/msg/TFMessage', message => {
    messages++;
    for (const transform of message.transforms) {
      const parent = transform.header.frame_id.replace(/^\//, '');
      const child = transform.child_frame_id.replace(/^\//, '');
      frames.add(parent);
      frames.add(child);
      edges.add(`${parent}>${child}`);
    }
  }, {qos: QOS.staticTf});
  try {
    const deadline = Date.now() + 8000;
    while (missing().length && Date.now() < deadline) await pause(30);
    assert.equal(missing().length, 0,
      `${label}: missing ${missing()}; received edges ${[...edges].sort()}`);
    console.log(`PASS ${label}: ${messages} retained messages, ${frames.size} frames, ${required.join(', ')}`);
    return off;
  } catch (error) {
    off();
    throw error;
  }
}

try {
  const stops = [];
  for (let index = 0; index < 3; index++) {
    const client = new RosClient({url, reconnect: false});
    clients.push(client);
    await client.connect();
    stops.push(await subscribe(client, `sequential client ${index + 1}`));
  }
  stops[0]();
  clients[0].dispose();
  stops[1]();
  await pause(200);
  stops[1] = await subscribe(clients[1], 'client 2 re-subscribes after original client leaves');
  stops[2]();
  clients[2].disconnect();
  await clients[2].connect();
  stops[2] = await subscribe(clients[2], 'client 3 reconnects its WebSocket');
  for (const stop of stops) stop();
  console.log('PASS static TF multi-client replay and unsubscribe smoke check');
} finally {
  for (const client of clients) client.dispose();
}
