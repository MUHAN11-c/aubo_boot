/** Bounded PointCloud2 wire decoding; coordinates remain in the message frame. */
export interface DecodedPointCloud {
  positions: Float32Array;
  /** RGB channels in [0, 1], in the same point order as positions. */
  colors?: Float32Array;
  count: number;
}

interface Field {name: string; offset: number; datatype: number; count: number}
type Wire = Record<string, unknown>;
const FIELD_BYTES = [0, 1, 1, 2, 2, 4, 4, 4, 8] as const;

function record(value: unknown, label: string): Wire {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${label} must be an object`);
  return value as Wire;
}

function uint32(value: unknown, label: string): number {
  if (typeof value !== 'number' || !Number.isInteger(value) || value < 0 || value > 0xffff_ffff) {
    throw new TypeError(`${label} must be a uint32`);
  }
  return value;
}

function budget(value: number, label: string): number {
  if (!Number.isSafeInteger(value) || value < 1) throw new TypeError(`${label} must be a positive safe integer`);
  return value;
}

function bytes(value: unknown, limit: number): Uint8Array {
  if (value instanceof Uint8Array) {
    if (value.byteLength > limit) throw new RangeError('PointCloud2 data exceeds maxBytes');
    return value;
  }
  if (Array.isArray(value)) {
    if (value.length > limit) throw new RangeError('PointCloud2 data exceeds maxBytes');
    const output = new Uint8Array(value.length);
    for (let i = 0; i < value.length; i++) {
      const item: unknown = value[i];
      if (typeof item !== 'number' || !Number.isInteger(item) || item < 0 || item > 255) {
        throw new TypeError('PointCloud2 data must contain uint8 bytes');
      }
      output[i] = item;
    }
    return output;
  }
  if (typeof value === 'string') {
    // Check the encoded size before atob allocates its decoded binary string.
    if (value.length > 4 * Math.ceil(limit / 3)) throw new RangeError('PointCloud2 Base64 exceeds maxBytes');
    if (value.length % 4 !== 0 || !/^[A-Za-z0-9+/]*={0,2}$/.test(value)) throw new TypeError('Invalid PointCloud2 Base64');
    const padding = value.endsWith('==') ? 2 : value.endsWith('=') ? 1 : 0;
    const length = value.length / 4 * 3 - padding;
    if (length > limit) throw new RangeError('PointCloud2 Base64 exceeds maxBytes');
    const binary = atob(value);
    const output = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) output[i] = binary.charCodeAt(i);
    return output;
  }
  throw new TypeError('PointCloud2 data must be Base64, a number array or Uint8Array');
}

function scalar(view: DataView, offset: number, datatype: number, littleEndian: boolean): number {
  switch (datatype) {
    case 1: return view.getInt8(offset);
    case 2: return view.getUint8(offset);
    case 3: return view.getInt16(offset, littleEndian);
    case 4: return view.getUint16(offset, littleEndian);
    case 5: return view.getInt32(offset, littleEndian);
    case 6: return view.getUint32(offset, littleEndian);
    case 7: return view.getFloat32(offset, littleEndian);
    case 8: return view.getFloat64(offset, littleEndian);
    default: throw new TypeError('Unsupported PointField datatype');
  }
}

/**
 * Decode fields using their actual byte offsets and row stride, including padding.
 * maxBytes is a hard input budget. Clouds above maxPoints are uniformly sampled;
 * at most maxPoints records are read and at most that many points are allocated.
 * Malformed metadata and byte-budget overflow are reported as errors.
 */
export function decodePointCloud(message: unknown, options: {maxPoints?: number; maxBytes?: number} = {}): DecodedPointCloud {
  const maxPoints = budget(options.maxPoints ?? 100_000, 'maxPoints');
  const maxBytes = budget(options.maxBytes ?? 16_000_000, 'maxBytes');
  const m = record(message, 'PointCloud2');
  const width = uint32(m.width, 'width');
  const height = uint32(m.height, 'height');
  const pointStep = uint32(m.point_step, 'point_step');
  const rowStep = uint32(m.row_step, 'row_step');
  const total = width * height;
  const byteLength = rowStep * height;
  if (!Number.isSafeInteger(total) || !Number.isSafeInteger(byteLength)) throw new RangeError('PointCloud2 dimensions exceed safe integer range');
  if (byteLength > maxBytes) throw new RangeError('PointCloud2 dimensions exceed maxBytes');
  if (total > 0 && pointStep === 0) throw new TypeError('PointCloud2 point_step must be positive for a nonempty cloud');
  if (rowStep < width * pointStep) throw new TypeError('PointCloud2 row_step is shorter than one row');
  if (typeof m.is_bigendian !== 'boolean') throw new TypeError('PointCloud2 is_bigendian must be boolean');
  const littleEndian = !m.is_bigendian;
  const data = bytes(m.data, maxBytes);
  if (data.byteLength !== byteLength) throw new TypeError('PointCloud2 data length must equal row_step * height');
  if (!Array.isArray(m.fields)) throw new TypeError('PointCloud2 fields must be an array');
  const fields = new Map<string, Field>();
  for (const entry of m.fields) {
    const item = record(entry, 'PointField');
    if (typeof item.name !== 'string' || fields.has(item.name)) throw new TypeError('PointField names must be strings and unique');
    const offset = uint32(item.offset, 'PointField offset');
    const datatype = uint32(item.datatype, 'PointField datatype');
    const count = uint32(item.count, 'PointField count');
    const size = FIELD_BYTES[datatype];
    if (!size || count === 0 || offset + size * count > pointStep) throw new TypeError(`PointField ${item.name} exceeds point_step or has an unsupported layout`);
    fields.set(item.name, {name: item.name, offset, datatype, count});
  }
  if (total === 0) return {positions: new Float32Array(), count: 0};
  const xyz = ['x', 'y', 'z'].map(name => {
    const field = fields.get(name);
    if (!field || field.count !== 1) throw new TypeError(`PointCloud2 requires scalar ${name}`);
    return field;
  });
  const packed = fields.get('rgba') ?? fields.get('rgb');
  if (packed && (packed.count !== 1 || ![6, 7].includes(packed.datatype))) throw new TypeError('Packed rgb/rgba must be UINT32 or FLOAT32');
  const channels = ['r', 'g', 'b'].map(name => fields.get(name));
  if (!packed && channels.some(Boolean) && channels.some(field => !field || field.count !== 1)) {
    throw new TypeError('Separate PointCloud2 colors require scalar r, g and b');
  }
  const hasColors = !!packed || channels.every(Boolean);
  const stride = Math.max(1, Math.ceil(total / maxPoints));
  const capacity = Math.ceil(total / stride);
  const positions = new Float32Array(capacity * 3);
  const colors = hasColors ? new Float32Array(capacity * 3) : undefined;
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  let count = 0;
  for (let index = 0; index < total; index += stride) {
    const offset = Math.floor(index / width) * rowStep + index % width * pointStep;
    const x = Math.fround(scalar(view, offset + xyz[0]!.offset, xyz[0]!.datatype, littleEndian));
    const y = Math.fround(scalar(view, offset + xyz[1]!.offset, xyz[1]!.datatype, littleEndian));
    const z = Math.fround(scalar(view, offset + xyz[2]!.offset, xyz[2]!.datatype, littleEndian));
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    if (colors) {
      if (packed) {
        // Read raw bits even for FLOAT32: a packed rgba value can itself be NaN.
        const rgb = view.getUint32(offset + packed.offset, littleEndian);
        colors[count * 3] = ((rgb >>> 16) & 255) / 255;
        colors[count * 3 + 1] = ((rgb >>> 8) & 255) / 255;
        colors[count * 3 + 2] = (rgb & 255) / 255;
      } else {
        const rgb = channels.map(field => scalar(view, offset + field!.offset, field!.datatype, littleEndian));
        if (!rgb.every(Number.isFinite)) continue;
        for (let i = 0; i < 3; i++) colors[count * 3 + i] = Math.max(0, Math.min(255, rgb[i]!)) / 255;
      }
    }
    positions[count * 3] = x;
    positions[count * 3 + 1] = y;
    positions[count * 3 + 2] = z;
    count++;
  }
  return {positions: positions.subarray(0, count * 3), ...(colors ? {colors: colors.subarray(0, count * 3)} : {}), count};
}
