/** ROS 2 ParameterValue codec. Types are the rcl_interfaces/ParameterType IDs. */
export type ParameterType = 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9;
export type ParameterValue = boolean | number | string | boolean[] | number[] | string[] | undefined;
export interface ParameterWrite { name: string; type: ParameterType; value: ParameterValue }
export interface DecodedParameter { type: ParameterType; value: ParameterValue }

const fields = ['','bool_value','integer_value','double_value','string_value',
  'byte_array_value','bool_array_value','integer_array_value','double_array_value','string_array_value'];

/** Validate JSON numeric values, without guessing whether a number is int64.
 * Finite large doubles (e.g. 1e20) are valid. Safe-integer checks belong in the
 * typed integer parameter codec below; generic ROS messages need their schema.
 */
export function assertSafeJson(value: unknown): void {
  if (typeof value === 'number' && !Number.isFinite(value)) {
    throw new TypeError('JSON number is non-finite');
  }
  if (typeof value === 'bigint') throw new TypeError('JSON/rosbridge cannot transmit bigint losslessly');
  if (Array.isArray(value)) value.forEach(assertSafeJson);
  else if (value && typeof value === 'object') Object.values(value).forEach(assertSafeJson);
}

/** Encode all nine concrete ROS 2 types; type 0 explicitly requests NOT_SET. */
export function encodeParameter(type: ParameterType, value: ParameterValue): Record<string, unknown> {
  if (!Number.isInteger(type) || type < 0 || type > 9) throw new TypeError('Unknown ROS parameter type');
  const scalarType = type > 4 ? [0,0,0,0,0,2,1,2,3,4][type] : type;
  const valid = (v: unknown) => scalarType === 1 ? typeof v === 'boolean' :
    scalarType === 4 ? typeof v === 'string' : typeof v === 'number' && Number.isFinite(v) &&
      (scalarType !== 2 || Number.isSafeInteger(v)) && (type !== 5 || (v >= 0 && v <= 255));
  if (type === 0) {
    if (value !== undefined) throw new TypeError('NOT_SET requires undefined');
    return {type: 0};
  }
  if (type > 4 ? !Array.isArray(value) || !value.every(valid) : !valid(value)) {
    throw new TypeError(`Value does not match ROS parameter type ${type}`);
  }
  assertSafeJson(value);
  return {type, [fields[type]]: value};
}

/** rosbridge represents uint8[] as Base64; accept that and ordinary JSON arrays. */
export function decodeParameter(raw: Record<string, unknown>): DecodedParameter {
  const type = raw.type as ParameterType;
  let value = raw[fields[type]] as ParameterValue;
  if (type === 5 && typeof value === 'string') value = Array.from(atob(value), c => c.charCodeAt(0));
  if (type === 0) value = undefined;
  encodeParameter(type, value);
  return {type, value};
}
