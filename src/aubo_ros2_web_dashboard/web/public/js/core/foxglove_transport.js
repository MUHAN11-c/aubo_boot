// foxglove_transport.js — 向后兼容层
//
// 原有 foxglove_transport.js 已被重构到 transport/foxglove/client.js。
// 本文件保持 re-export, 确保现有引用 (pointcloud_viewer.js 等) 无需修改即可工作。
//
// @deprecated 新代码请使用:
//   import { foxgloveClient } from '../transport/foxglove/client.js';
//   import { CdrReader } from '../transport/foxglove/vendor/foxglove_cdr.js';

import { foxgloveClient, FoxgloveClient } from '../transport/foxglove/client.js';
import { CdrReader } from '../transport/foxglove/vendor/foxglove_cdr.js';

const g = globalThis;

// ── 向后兼容: 保留旧的 parsePointCloud2Cdr ─────────────────────────────────────

function parsePointCloud2Cdr(buf) {
  const r = new CdrReader(buf);
  r.uint32(); r.uint32();  // stamp sec, nanosec
  r.string();              // frame_id
  const height = r.uint32();
  const width = r.uint32();
  const n = height * width;
  const nFields = r.uint32();
  const fields = [];
  for (let i = 0; i < nFields; i++) {
    const name = r.string();
    const offset = r.uint32();
    const datatype = r.uint8();
    const count = r.uint32();
    fields.push({ name, offset, datatype, count });
  }
  r.uint8();               // is_bigendian
  const pointStep = r.uint32();
  r.uint32();              // row_step
  const dataLen = r.uint32();
  const dataStart = r.offset;
  r.uint8();               // is_dense

  const xf = fields.find(f => f.name === 'x');
  const yf = fields.find(f => f.name === 'y');
  const zf = fields.find(f => f.name === 'z');

  const bufDV = new DataView(buf.buffer, buf.byteOffset + dataStart, dataLen);
  const positions = new Float32Array(n * 3);

  const readFloat = (ptOff, field) => {
    const off = ptOff + field.offset;
    if (field.datatype === 7) return bufDV.getFloat32(off, true);
    return 0;
  };

  for (let i = 0; i < n; i++) {
    const ptBase = i * pointStep;
    positions[i * 3]     = xf ? readFloat(ptBase, xf) : 0;
    positions[i * 3 + 1] = yf ? readFloat(ptBase, yf) : 0;
    positions[i * 3 + 2] = zf ? readFloat(ptBase, zf) : 0;
  }
  return { positions, count: n, width, height };
}

// ── 单例 (向后兼容) ─────────────────────────────────────────────────────────

const foxgloveTransport = foxgloveClient;
g.foxgloveTransport = foxgloveTransport;

export { FoxgloveClient as FoxgloveTransport, foxgloveTransport, CdrReader, parsePointCloud2Cdr };
