/** Read-only ROS TF transport and bounded policy around Lichtblick's TransformTree. */
import type {RosClient} from '../sdk/index.js';
import {CoordinateFrame} from '../third_party/lichtblick_transforms/CoordinateFrame.js';
import {Transform} from '../third_party/lichtblick_transforms/Transform.js';
import {AddTransformResult, TransformTree} from '../third_party/lichtblick_transforms/TransformTree.js';
import {ObjectPool} from '../third_party/lichtblick_transforms/den/ObjectPool.js';
import {makePose} from '../third_party/lichtblick_transforms/geometry.js';

export interface RosStamp {sec: number; nanosec: number}
export interface FrameTransform {
  translation: {x: number; y: number; z: number};
  rotation: {x: number; y: number; z: number; w: number};
}
export interface TfBufferOptions {
  tfTopic?: string;
  tfStaticTopic?: string;
  /** Explicit simulation clock, disabled by default. TF publisher timestamps
   * may legitimately arrive out of order and do not select a clock source.
   */
  clockTopic?: string | false;
  historySeconds?: number;
  maxFrames?: number;
  maxSamplesPerFrame?: number;
}

type Wire = Record<string, unknown>;
type TimeAndTransform = [bigint, Transform];
// These encompass every valid builtin_interfaces/Time (signed int32 seconds).
const MIN_TIME = -(1n << 63n);
const MAX_TIME = (1n << 63n) - 1n;
const UNLIMITED_DELTA = 1n << 64n;

function object(value: unknown): Wire | undefined {
  return value !== null && typeof value === 'object' && !Array.isArray(value) ? value as Wire : undefined;
}

function frameId(value: unknown): string | undefined {
  if (typeof value !== 'string' || value.length > 1024 || /\s|\0/.test(value)) return undefined;
  const id = value.replace(/^\/+/, '');
  return id.length > 0 ? id : undefined;
}

/** Shared canonical Time/Duration wire conversion; not a public SDK entry. */
export function nanoseconds(value: unknown): bigint | undefined {
  const stamp = object(value);
  if (!stamp || typeof stamp.sec !== 'number' || !Number.isInteger(stamp.sec) || stamp.sec < -0x8000_0000 || stamp.sec > 0x7fff_ffff ||
    typeof stamp.nanosec !== 'number' || !Number.isInteger(stamp.nanosec) || stamp.nanosec < 0 || stamp.nanosec >= 1_000_000_000) return undefined;
  return BigInt(stamp.sec) * 1_000_000_000n + BigInt(stamp.nanosec);
}

function positiveInteger(value: number, label: string): number {
  if (!Number.isSafeInteger(value) || value < 1) throw new TypeError(`${label} must be a positive safe integer`);
  return value;
}

/**
 * TF values, parent relations, sorted history and interpolation belong exclusively
 * to the vendored upstream tree. This class only adapts wire input and policies.
 * A missing timestamp means latest common observation time, for display only.
 * An explicit timestamp is never extrapolated beyond any dynamic edge's history.
 */
export class TfBuffer {
  private readonly history: bigint;
  private readonly maxFrames: number;
  private readonly maxSamples: number;
  private readonly options: Required<TfBufferOptions>;
  private readonly lower: TimeAndTransform = [0n, Transform.Identity()];
  private readonly upper: TimeAndTransform = [0n, Transform.Identity()];
  private pool: ObjectPool<Transform>;
  private tree: TransformTree;
  private staticFrames = new Set<string>();
  private cleanups: (() => void)[] = [];
  private started = false;
  private disposed = false;
  private lastReceivedAt = 0;
  private latestDynamic?: bigint;
  private clock?: bigint;

  constructor(private readonly client: RosClient, options: TfBufferOptions = {}) {
    const historySeconds = options.historySeconds ?? 10;
    if (!Number.isFinite(historySeconds) || historySeconds <= 0 || !Number.isSafeInteger(Math.round(historySeconds * 1e9))) {
      throw new TypeError('historySeconds must be positive and representable in nanoseconds');
    }
    this.maxFrames = positiveInteger(options.maxFrames ?? 256, 'maxFrames');
    this.maxSamples = positiveInteger(options.maxSamplesPerFrame ?? 500, 'maxSamplesPerFrame');
    this.history = BigInt(Math.round(historySeconds * 1e9));
    this.options = {tfTopic: '/tf', tfStaticTopic: '/tf_static', clockTopic: false, ...options,
      historySeconds, maxFrames: this.maxFrames, maxSamplesPerFrame: this.maxSamples};
    this.pool = this.makePool();
    this.tree = new TransformTree(this.pool, this.history, this.maxSamples);
  }

  private makePool(): ObjectPool<Transform> {
    // Pool retention is independently bounded even when many frames are pruned.
    return new ObjectPool(Transform.Empty, {maxCapacity: Math.min(this.maxSamples, 4096)});
  }

  start(): void {
    if (this.disposed) throw new Error('TfBuffer is disposed');
    if (this.started) return;
    this.started = true;
    try {
      // onState reports its initial state as well. Every WebSocket session starts
      // with fresh TF data and receive timestamps; old static TF must be replayed.
      this.cleanups.push(this.client.onState(() => this.clear()));
      this.cleanups.push(this.client.subscribe(this.options.tfTopic, 'tf2_msgs/msg/TFMessage', message => this.ingest(message), {qos: 'reliable'}));
      this.cleanups.push(this.client.subscribe(this.options.tfStaticTopic, 'tf2_msgs/msg/TFMessage', message => this.ingest(message, true), {qos: 'staticTf'}));
      if (this.options.clockTopic !== false) {
        this.cleanups.push(this.client.subscribe(this.options.clockTopic, 'rosgraph_msgs/msg/Clock', message => this.ingestClock(message), {qos: 'sensor'}));
      }
    } catch (error) {
      for (const cleanup of this.cleanups.splice(0).reverse()) cleanup();
      this.started = false;
      this.clear();
      throw error;
    }
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    for (const cleanup of this.cleanups.splice(0).reverse()) cleanup();
    this.started = false;
    this.clear();
  }

  clear(): void {
    this.tree.clear();
    this.pool.clear();
    this.staticFrames.clear();
    this.latestDynamic = undefined;
    this.clock = undefined;
    this.lastReceivedAt = 0;
  }

  frames(): string[] { return [...this.tree.frames().keys()].sort(); }

  /** Only explicit Clock rollback denotes a new time epoch. TF may be out of order. */
  ingestClock(message: unknown): void {
    if (this.disposed) return;
    const time = nanoseconds(object(message)?.clock);
    if (time === undefined) return;
    if (this.clock !== undefined && time < this.clock) this.clearDynamic();
    this.clock = time;
  }

  private clearDynamic(): void {
    // Copy references directly from the upstream tree only while rebuilding it.
    // No second transform cache or parallel parent/child graph is maintained.
    const retained: {child: string; parent: string; transform: Transform}[] = [];
    for (const child of this.staticFrames) {
      const frame = this.tree.frame(child);
      const parent = frame?.parent();
      if (frame && parent && frame.findClosestTransforms(this.lower, this.upper, 0n, UNLIMITED_DELTA)) {
        retained.push({child, parent: parent.id, transform: this.lower[1]});
      }
    }
    this.pool = this.makePool();
    this.tree = new TransformTree(this.pool, this.history, this.maxSamples);
    this.staticFrames.clear();
    for (const item of retained) {
      this.tree.addTransform(item.child, item.parent, 0n, item.transform);
      this.staticFrames.add(item.child);
    }
    this.latestDynamic = undefined;
    this.lastReceivedAt = 0;
  }

  /** Ignore malformed individual transforms; keep accepted history bounded. */
  ingest(message: unknown, isStatic = false): void {
    if (this.disposed) return;
    const transforms = object(message)?.transforms;
    if (!Array.isArray(transforms)) return;
    for (const raw of transforms) {
      const item = object(raw);
      const header = object(item?.header);
      const child = frameId(item?.child_frame_id);
      const parent = frameId(header?.frame_id);
      const time = isStatic ? 0n : nanoseconds(header?.stamp);
      const transform = object(item?.transform);
      const p = object(transform?.translation);
      const q = object(transform?.rotation);
      if (!child || !parent || child === parent || time === undefined || !p || !q) continue;
      const components = [p.x, p.y, p.z, q.x, q.y, q.z, q.w];
      if (!components.every(value => typeof value === 'number' && Number.isFinite(value))) continue;
      const rotation = [q.x, q.y, q.z, q.w] as [number, number, number, number];
      // Match tf2's near-unit input requirement; normalization remains upstream.
      if (Math.abs(Math.hypot(...rotation) - 1) > 0.01) continue;
      const addedFrames = Number(!this.tree.hasFrame(child)) + Number(!this.tree.hasFrame(parent));
      if (this.tree.frames().size + addedFrames > this.maxFrames) continue;
      const wasStatic = this.staticFrames.has(child);
      const previous = this.tree.frame(child);
      if (previous && wasStatic !== isStatic) {
        // Use the upstream ancestor query to reject a cycle before replacing the
        // existing cache; failed reparenting must leave accepted data intact.
        if (this.tree.frame(parent)?.findAncestor(child)) continue;
        // Reset before insertion: otherwise upstream history-window eviction
        // could immediately discard the new static sample at timestamp zero.
        while (previous.transformsSize() > 0) previous.removeTransformAt(this.range(previous)!.first);
      }
      const tf = this.pool.acquire().setPositionRotation([p.x as number, p.y as number, p.z as number], rotation);
      const result = this.tree.addTransform(child, parent, time, tf);
      if (result === AddTransformResult.CYCLE_DETECTED) {
        this.pool.release(tf);
        continue;
      }
      const frame = this.tree.frame(child)!;
      if (isStatic) this.staticFrames.add(child);
      else {
        this.staticFrames.delete(child);
        this.latestDynamic = this.latestDynamic === undefined || time > this.latestDynamic ? time : this.latestDynamic;
      }
      // Upstream amortizes capacity trimming; enforce small capacities (1..3)
      // too, for which its quarter-capacity eviction rounds down to zero.
      while (frame.transformsSize() > this.maxSamples) frame.removeTransformAt(this.range(frame)!.first);
      this.lastReceivedAt = Date.now();
    }
    this.prune();
  }

  private range(frame: CoordinateFrame): {first: bigint; last: bigint} | undefined {
    // Ask the upstream history for its extrema; keep no duplicate timestamp index.
    if (!frame.findClosestTransforms(this.lower, this.upper, MIN_TIME, UNLIMITED_DELTA)) return undefined;
    const first = this.lower[0];
    if (!frame.findClosestTransforms(this.lower, this.upper, MAX_TIME, UNLIMITED_DELTA)) return undefined;
    return {first, last: this.upper[0]};
  }

  private prune(): void {
    if (this.latestDynamic === undefined) return;
    const cutoff = this.latestDynamic - this.history;
    for (const frame of [...this.tree.frames().values()]) {
      if (this.staticFrames.has(frame.id)) continue;
      const parent = frame.parent();
      if (!parent) continue;
      for (let range = this.range(frame); range && range.first < cutoff; range = this.range(frame)) {
        this.tree.removeTransform(frame.id, parent.id, range.first);
      }
    }
  }

  lookup(target: string, source: string, stamp?: RosStamp): FrameTransform | undefined {
    const targetId = frameId(target);
    const sourceId = frameId(source);
    if (!targetId || !sourceId) return undefined;
    const requested = stamp === undefined ? undefined : nanoseconds(stamp);
    if (stamp !== undefined && requested === undefined) return undefined;
    if (targetId === sourceId) {
      // A pose already in the requested frame needs no TF publisher or history.
      const pose = makePose();
      return {translation: pose.position, rotation: pose.orientation};
    }
    const destination = this.tree.frame(targetId);
    const origin = this.tree.frame(sourceId);
    if (!destination || !origin) return undefined;

    // Inspect the shortest path using upstream parent relations. Ancestors above
    // the common frame are irrelevant and must not falsely reject the lookup.
    const targetAncestors = new Set<CoordinateFrame>();
    for (let frame: CoordinateFrame | undefined = destination; frame; frame = frame.parent()) targetAncestors.add(frame);
    let common: CoordinateFrame | undefined = origin;
    const path: CoordinateFrame[] = [];
    while (common && !targetAncestors.has(common)) { path.push(common); common = common.parent(); }
    if (!common) return undefined;
    for (let frame = destination; frame !== common; frame = frame.parent()!) path.push(frame);
    let earliest = MIN_TIME;
    let latest = MAX_TIME;
    let dynamic = false;
    for (const frame of path) {
      if (this.staticFrames.has(frame.id)) continue;
      const range = this.range(frame);
      if (!range) return undefined;
      dynamic = true;
      if (range.first > earliest) earliest = range.first;
      if (range.last < latest) latest = range.last;
    }
    const time = requested ?? (dynamic ? latest : 0n);
    if (dynamic && (time < earliest || time > latest)) return undefined;
    const pose = destination.applyLocal(makePose(), makePose(), origin, time, UNLIMITED_DELTA);
    if (!pose) return undefined;
    return {translation: pose.position, rotation: pose.orientation};
  }

  stats(): {frames: number; samples: number; lastReceivedAt: number} {
    let samples = 0;
    for (const frame of this.tree.frames().values()) samples += frame.transformsSize();
    return {frames: this.tree.frames().size, samples, lastReceivedAt: this.lastReceivedAt};
  }
}
