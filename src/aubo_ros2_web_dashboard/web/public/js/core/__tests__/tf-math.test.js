import { describe, it, expect } from 'vitest';
import {
  quatToRpy, rpyToQuat, rpyDegToQuat, quatToRpyDeg,
  ivgComposeTransforms, ivgInvertTransform, ivgIdentityTransform,
  ivgFindRelativeTransform, normalizeFrameId,
} from '../tf-math.js';

describe('quatToRpy', () => {
  it('identity quaternion → zero RPY', () => {
    const rpy = quatToRpy({ x: 0, y: 0, z: 0, w: 1 });
    expect(rpy.roll).toBeCloseTo(0, 5);
    expect(rpy.pitch).toBeCloseTo(0, 5);
    expect(rpy.yaw).toBeCloseTo(0, 5);
  });

  it('90 deg yaw (π/2 around Z)', () => {
    // RPY(0, 0, π/2) → quat z=sin(π/4), w=cos(π/4)
    const s = Math.sin(Math.PI / 4);
    const c = Math.cos(Math.PI / 4);
    const rpy = quatToRpy({ x: 0, y: 0, z: s, w: c });
    expect(rpy.yaw).toBeCloseTo(Math.PI / 2, 5);
  });

  it('does not throw on bad input', () => {
    expect(() => quatToRpy({})).not.toThrow();
  });
});

describe('rpyToQuat + quatToRpy roundtrip', () => {
  it('roundtrip: (0.1, -0.2, 0.3)', () => {
    const q = rpyToQuat(0.1, -0.2, 0.3);
    const rpy = quatToRpy(q);
    expect(rpy.roll).toBeCloseTo(0.1, 5);
    expect(rpy.pitch).toBeCloseTo(-0.2, 5);
    expect(rpy.yaw).toBeCloseTo(0.3, 5);
  });

  it('deg variants work', () => {
    const q = rpyDegToQuat(0, 0, 90);
    const rpy = quatToRpyDeg(q);
    expect(rpy.yaw).toBeCloseTo(90, 1);
  });
});

describe('ivgIdentityTransform', () => {
  it('returns identity', () => {
    const tf = ivgIdentityTransform();
    expect(tf.translation).toEqual({ x: 0, y: 0, z: 0 });
    expect(tf.rotation).toEqual({ x: 0, y: 0, z: 0, w: 1 });
  });
});

describe('ivgInvertTransform', () => {
  it('inverting identity yields identity', () => {
    const inv = ivgInvertTransform(ivgIdentityTransform());
    expect(inv.translation.x).toBeCloseTo(0, 5);
    expect(inv.translation.y).toBeCloseTo(0, 5);
    expect(inv.translation.z).toBeCloseTo(0, 5);
    expect(inv.rotation.w).toBeCloseTo(1, 5);
  });

  it('inverting a translation negates it', () => {
    const tf = {
      translation: { x: 1, y: 2, z: 3 },
      rotation: { x: 0, y: 0, z: 0, w: 1 },
    };
    const inv = ivgInvertTransform(tf);
    expect(inv.translation.x).toBeCloseTo(-1, 5);
    expect(inv.translation.y).toBeCloseTo(-2, 5);
    expect(inv.translation.z).toBeCloseTo(-3, 5);
  });
});

describe('ivgComposeTransforms', () => {
  it('identity compose identity = identity', () => {
    const result = ivgComposeTransforms(ivgIdentityTransform(), ivgIdentityTransform());
    expect(result.translation).toEqual({ x: 0, y: 0, z: 0 });
    expect(result.rotation).toEqual({ x: 0, y: 0, z: 0, w: 1 });
  });

  it('compose translations', () => {
    const a = { translation: { x: 1, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } };
    const b = { translation: { x: 0, y: 2, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } };
    const result = ivgComposeTransforms(a, b);
    expect(result.translation.x).toBeCloseTo(1, 5);
    expect(result.translation.y).toBeCloseTo(2, 5);
  });
});

describe('ivgFindRelativeTransform', () => {
  it('same frame = identity', () => {
    const result = ivgFindRelativeTransform('base', 'base', {});
    expect(result).not.toBeNull();
    expect(result.rotation.w).toBeCloseTo(1, 5);
  });

  it('returns null for unknown frames', () => {
    expect(ivgFindRelativeTransform('a', 'b', {})).toBeNull();
  });

  it('finds transform from child to base (inverse direction)', () => {
    // child is at +1 in base frame; child→base = inverse = -1
    const edges = {
      'child': {
        parent: 'base',
        transform: { translation: { x: 1, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } },
      },
    };
    const result = ivgFindRelativeTransform('child', 'base', edges);
    expect(result).not.toBeNull();
    expect(result.translation.x).toBeCloseTo(-1, 5);
  });
});

describe('normalizeFrameId', () => {
  it('removes leading slashes', () => {
    expect(normalizeFrameId('/base_link')).toBe('base_link');
  });
  it('handles empty', () => {
    expect(normalizeFrameId('')).toBe('');
    expect(normalizeFrameId(null)).toBe('');
  });
});
