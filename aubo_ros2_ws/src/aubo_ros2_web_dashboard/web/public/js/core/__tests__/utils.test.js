import { describe, it, expect } from 'vitest';
import { escapeHtml, canonicalRosTopic, encodeTopicQueryValue, rosMsgArrayField } from '../utils.js';

describe('escapeHtml', () => {
  it('escapes < > & " \'', () => {
    expect(escapeHtml('<script>alert("xss")</script>'))
      .toBe('&lt;script&gt;alert(&quot;xss&quot;)&lt;/script&gt;');
  });

  it('escapes single quote', () => {
    expect(escapeHtml("it's")).toBe('it&#39;s');
  });

  it('returns empty for empty string', () => {
    expect(escapeHtml('')).toBe('');
  });

  it('converts non-string to string', () => {
    expect(escapeHtml(123)).toBe('123');
    expect(escapeHtml(null)).toBe('null');
  });
});

describe('canonicalRosTopic', () => {
  it('adds leading slash', () => {
    expect(canonicalRosTopic('robot_status')).toBe('/robot_status');
  });

  it('keeps leading slash', () => {
    expect(canonicalRosTopic('/robot_status')).toBe('/robot_status');
  });

  it('trims whitespace', () => {
    expect(canonicalRosTopic('  /joint_states  ')).toBe('/joint_states');
  });

  it('returns empty for empty input', () => {
    expect(canonicalRosTopic('')).toBe('');
    expect(canonicalRosTopic(null)).toBe('');
  });
});

describe('encodeTopicQueryValue', () => {
  it('encodes segment content, keeping / as separator', () => {
    // Leading / splits to ['', 'camera', 'color', 'image_raw']
    // Each segment is URI-encoded, then re-joined with /
    expect(encodeTopicQueryValue('/camera/color/image_raw'))
      .toBe('/camera/color/image_raw');
  });

  it('encodes special chars in segments', () => {
    expect(encodeTopicQueryValue('/topic/with spaces'))
      .toBe('/topic/with%20spaces');
  });
});

describe('rosMsgArrayField', () => {
  it('returns array from data field', () => {
    expect(rosMsgArrayField({ name: { data: ['j1', 'j2'] } }, 'name'))
      .toEqual(['j1', 'j2']);
  });

  it('returns array directly', () => {
    expect(rosMsgArrayField({ name: ['j1', 'j2'] }, 'name'))
      .toEqual(['j1', 'j2']);
  });

  it('returns empty for missing key', () => {
    expect(rosMsgArrayField({}, 'name')).toEqual([]);
    expect(rosMsgArrayField(null, 'name')).toEqual([]);
  });
});
