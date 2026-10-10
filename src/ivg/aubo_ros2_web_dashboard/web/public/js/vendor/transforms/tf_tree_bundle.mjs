var __defProp = Object.defineProperty;
var __export = (target, all) => {
  for (var name in all)
    __defProp(target, name, { get: all[name], enumerable: true });
};

// node_modules/gl-matrix/esm/common.js
var EPSILON = 1e-6;
var ARRAY_TYPE = typeof Float32Array !== "undefined" ? Float32Array : Array;
var RANDOM = Math.random;
var degree = Math.PI / 180;
if (!Math.hypot) Math.hypot = function() {
  var y = 0, i = arguments.length;
  while (i--) {
    y += arguments[i] * arguments[i];
  }
  return Math.sqrt(y);
};

// node_modules/gl-matrix/esm/mat3.js
function create() {
  var out = new ARRAY_TYPE(9);
  if (ARRAY_TYPE != Float32Array) {
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
  }
  out[0] = 1;
  out[4] = 1;
  out[8] = 1;
  return out;
}

// node_modules/gl-matrix/esm/mat4.js
var mat4_exports = {};
__export(mat4_exports, {
  add: () => add,
  adjoint: () => adjoint,
  clone: () => clone,
  copy: () => copy,
  create: () => create2,
  determinant: () => determinant,
  equals: () => equals,
  exactEquals: () => exactEquals,
  frob: () => frob,
  fromQuat: () => fromQuat,
  fromQuat2: () => fromQuat2,
  fromRotation: () => fromRotation,
  fromRotationTranslation: () => fromRotationTranslation,
  fromRotationTranslationScale: () => fromRotationTranslationScale,
  fromRotationTranslationScaleOrigin: () => fromRotationTranslationScaleOrigin,
  fromScaling: () => fromScaling,
  fromTranslation: () => fromTranslation,
  fromValues: () => fromValues,
  fromXRotation: () => fromXRotation,
  fromYRotation: () => fromYRotation,
  fromZRotation: () => fromZRotation,
  frustum: () => frustum,
  getRotation: () => getRotation,
  getScaling: () => getScaling,
  getTranslation: () => getTranslation,
  identity: () => identity,
  invert: () => invert,
  lookAt: () => lookAt,
  mul: () => mul,
  multiply: () => multiply,
  multiplyScalar: () => multiplyScalar,
  multiplyScalarAndAdd: () => multiplyScalarAndAdd,
  ortho: () => ortho,
  orthoNO: () => orthoNO,
  orthoZO: () => orthoZO,
  perspective: () => perspective,
  perspectiveFromFieldOfView: () => perspectiveFromFieldOfView,
  perspectiveNO: () => perspectiveNO,
  perspectiveZO: () => perspectiveZO,
  rotate: () => rotate,
  rotateX: () => rotateX,
  rotateY: () => rotateY,
  rotateZ: () => rotateZ,
  scale: () => scale,
  set: () => set,
  str: () => str,
  sub: () => sub,
  subtract: () => subtract,
  targetTo: () => targetTo,
  translate: () => translate,
  transpose: () => transpose
});
function create2() {
  var out = new ARRAY_TYPE(16);
  if (ARRAY_TYPE != Float32Array) {
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 0;
    out[6] = 0;
    out[7] = 0;
    out[8] = 0;
    out[9] = 0;
    out[11] = 0;
    out[12] = 0;
    out[13] = 0;
    out[14] = 0;
  }
  out[0] = 1;
  out[5] = 1;
  out[10] = 1;
  out[15] = 1;
  return out;
}
function clone(a) {
  var out = new ARRAY_TYPE(16);
  out[0] = a[0];
  out[1] = a[1];
  out[2] = a[2];
  out[3] = a[3];
  out[4] = a[4];
  out[5] = a[5];
  out[6] = a[6];
  out[7] = a[7];
  out[8] = a[8];
  out[9] = a[9];
  out[10] = a[10];
  out[11] = a[11];
  out[12] = a[12];
  out[13] = a[13];
  out[14] = a[14];
  out[15] = a[15];
  return out;
}
function copy(out, a) {
  out[0] = a[0];
  out[1] = a[1];
  out[2] = a[2];
  out[3] = a[3];
  out[4] = a[4];
  out[5] = a[5];
  out[6] = a[6];
  out[7] = a[7];
  out[8] = a[8];
  out[9] = a[9];
  out[10] = a[10];
  out[11] = a[11];
  out[12] = a[12];
  out[13] = a[13];
  out[14] = a[14];
  out[15] = a[15];
  return out;
}
function fromValues(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33) {
  var out = new ARRAY_TYPE(16);
  out[0] = m00;
  out[1] = m01;
  out[2] = m02;
  out[3] = m03;
  out[4] = m10;
  out[5] = m11;
  out[6] = m12;
  out[7] = m13;
  out[8] = m20;
  out[9] = m21;
  out[10] = m22;
  out[11] = m23;
  out[12] = m30;
  out[13] = m31;
  out[14] = m32;
  out[15] = m33;
  return out;
}
function set(out, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33) {
  out[0] = m00;
  out[1] = m01;
  out[2] = m02;
  out[3] = m03;
  out[4] = m10;
  out[5] = m11;
  out[6] = m12;
  out[7] = m13;
  out[8] = m20;
  out[9] = m21;
  out[10] = m22;
  out[11] = m23;
  out[12] = m30;
  out[13] = m31;
  out[14] = m32;
  out[15] = m33;
  return out;
}
function identity(out) {
  out[0] = 1;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = 1;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 1;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}
function transpose(out, a) {
  if (out === a) {
    var a01 = a[1], a02 = a[2], a03 = a[3];
    var a12 = a[6], a13 = a[7];
    var a23 = a[11];
    out[1] = a[4];
    out[2] = a[8];
    out[3] = a[12];
    out[4] = a01;
    out[6] = a[9];
    out[7] = a[13];
    out[8] = a02;
    out[9] = a12;
    out[11] = a[14];
    out[12] = a03;
    out[13] = a13;
    out[14] = a23;
  } else {
    out[0] = a[0];
    out[1] = a[4];
    out[2] = a[8];
    out[3] = a[12];
    out[4] = a[1];
    out[5] = a[5];
    out[6] = a[9];
    out[7] = a[13];
    out[8] = a[2];
    out[9] = a[6];
    out[10] = a[10];
    out[11] = a[14];
    out[12] = a[3];
    out[13] = a[7];
    out[14] = a[11];
    out[15] = a[15];
  }
  return out;
}
function invert(out, a) {
  var a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
  var a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
  var a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
  var a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];
  var b00 = a00 * a11 - a01 * a10;
  var b01 = a00 * a12 - a02 * a10;
  var b02 = a00 * a13 - a03 * a10;
  var b03 = a01 * a12 - a02 * a11;
  var b04 = a01 * a13 - a03 * a11;
  var b05 = a02 * a13 - a03 * a12;
  var b06 = a20 * a31 - a21 * a30;
  var b07 = a20 * a32 - a22 * a30;
  var b08 = a20 * a33 - a23 * a30;
  var b09 = a21 * a32 - a22 * a31;
  var b10 = a21 * a33 - a23 * a31;
  var b11 = a22 * a33 - a23 * a32;
  var det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
  if (!det) {
    return null;
  }
  det = 1 / det;
  out[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
  out[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
  out[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
  out[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
  out[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
  out[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
  out[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
  out[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
  out[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
  out[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
  out[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
  out[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
  out[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
  out[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
  out[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
  out[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;
  return out;
}
function adjoint(out, a) {
  var a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
  var a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
  var a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
  var a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];
  out[0] = a11 * (a22 * a33 - a23 * a32) - a21 * (a12 * a33 - a13 * a32) + a31 * (a12 * a23 - a13 * a22);
  out[1] = -(a01 * (a22 * a33 - a23 * a32) - a21 * (a02 * a33 - a03 * a32) + a31 * (a02 * a23 - a03 * a22));
  out[2] = a01 * (a12 * a33 - a13 * a32) - a11 * (a02 * a33 - a03 * a32) + a31 * (a02 * a13 - a03 * a12);
  out[3] = -(a01 * (a12 * a23 - a13 * a22) - a11 * (a02 * a23 - a03 * a22) + a21 * (a02 * a13 - a03 * a12));
  out[4] = -(a10 * (a22 * a33 - a23 * a32) - a20 * (a12 * a33 - a13 * a32) + a30 * (a12 * a23 - a13 * a22));
  out[5] = a00 * (a22 * a33 - a23 * a32) - a20 * (a02 * a33 - a03 * a32) + a30 * (a02 * a23 - a03 * a22);
  out[6] = -(a00 * (a12 * a33 - a13 * a32) - a10 * (a02 * a33 - a03 * a32) + a30 * (a02 * a13 - a03 * a12));
  out[7] = a00 * (a12 * a23 - a13 * a22) - a10 * (a02 * a23 - a03 * a22) + a20 * (a02 * a13 - a03 * a12);
  out[8] = a10 * (a21 * a33 - a23 * a31) - a20 * (a11 * a33 - a13 * a31) + a30 * (a11 * a23 - a13 * a21);
  out[9] = -(a00 * (a21 * a33 - a23 * a31) - a20 * (a01 * a33 - a03 * a31) + a30 * (a01 * a23 - a03 * a21));
  out[10] = a00 * (a11 * a33 - a13 * a31) - a10 * (a01 * a33 - a03 * a31) + a30 * (a01 * a13 - a03 * a11);
  out[11] = -(a00 * (a11 * a23 - a13 * a21) - a10 * (a01 * a23 - a03 * a21) + a20 * (a01 * a13 - a03 * a11));
  out[12] = -(a10 * (a21 * a32 - a22 * a31) - a20 * (a11 * a32 - a12 * a31) + a30 * (a11 * a22 - a12 * a21));
  out[13] = a00 * (a21 * a32 - a22 * a31) - a20 * (a01 * a32 - a02 * a31) + a30 * (a01 * a22 - a02 * a21);
  out[14] = -(a00 * (a11 * a32 - a12 * a31) - a10 * (a01 * a32 - a02 * a31) + a30 * (a01 * a12 - a02 * a11));
  out[15] = a00 * (a11 * a22 - a12 * a21) - a10 * (a01 * a22 - a02 * a21) + a20 * (a01 * a12 - a02 * a11);
  return out;
}
function determinant(a) {
  var a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
  var a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
  var a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
  var a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];
  var b00 = a00 * a11 - a01 * a10;
  var b01 = a00 * a12 - a02 * a10;
  var b02 = a00 * a13 - a03 * a10;
  var b03 = a01 * a12 - a02 * a11;
  var b04 = a01 * a13 - a03 * a11;
  var b05 = a02 * a13 - a03 * a12;
  var b06 = a20 * a31 - a21 * a30;
  var b07 = a20 * a32 - a22 * a30;
  var b08 = a20 * a33 - a23 * a30;
  var b09 = a21 * a32 - a22 * a31;
  var b10 = a21 * a33 - a23 * a31;
  var b11 = a22 * a33 - a23 * a32;
  return b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
}
function multiply(out, a, b) {
  var a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
  var a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
  var a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
  var a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];
  var b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];
  out[0] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
  out[1] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
  out[2] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
  out[3] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
  b0 = b[4];
  b1 = b[5];
  b2 = b[6];
  b3 = b[7];
  out[4] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
  out[5] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
  out[6] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
  out[7] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
  b0 = b[8];
  b1 = b[9];
  b2 = b[10];
  b3 = b[11];
  out[8] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
  out[9] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
  out[10] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
  out[11] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
  b0 = b[12];
  b1 = b[13];
  b2 = b[14];
  b3 = b[15];
  out[12] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
  out[13] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
  out[14] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
  out[15] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
  return out;
}
function translate(out, a, v) {
  var x = v[0], y = v[1], z = v[2];
  var a00, a01, a02, a03;
  var a10, a11, a12, a13;
  var a20, a21, a22, a23;
  if (a === out) {
    out[12] = a[0] * x + a[4] * y + a[8] * z + a[12];
    out[13] = a[1] * x + a[5] * y + a[9] * z + a[13];
    out[14] = a[2] * x + a[6] * y + a[10] * z + a[14];
    out[15] = a[3] * x + a[7] * y + a[11] * z + a[15];
  } else {
    a00 = a[0];
    a01 = a[1];
    a02 = a[2];
    a03 = a[3];
    a10 = a[4];
    a11 = a[5];
    a12 = a[6];
    a13 = a[7];
    a20 = a[8];
    a21 = a[9];
    a22 = a[10];
    a23 = a[11];
    out[0] = a00;
    out[1] = a01;
    out[2] = a02;
    out[3] = a03;
    out[4] = a10;
    out[5] = a11;
    out[6] = a12;
    out[7] = a13;
    out[8] = a20;
    out[9] = a21;
    out[10] = a22;
    out[11] = a23;
    out[12] = a00 * x + a10 * y + a20 * z + a[12];
    out[13] = a01 * x + a11 * y + a21 * z + a[13];
    out[14] = a02 * x + a12 * y + a22 * z + a[14];
    out[15] = a03 * x + a13 * y + a23 * z + a[15];
  }
  return out;
}
function scale(out, a, v) {
  var x = v[0], y = v[1], z = v[2];
  out[0] = a[0] * x;
  out[1] = a[1] * x;
  out[2] = a[2] * x;
  out[3] = a[3] * x;
  out[4] = a[4] * y;
  out[5] = a[5] * y;
  out[6] = a[6] * y;
  out[7] = a[7] * y;
  out[8] = a[8] * z;
  out[9] = a[9] * z;
  out[10] = a[10] * z;
  out[11] = a[11] * z;
  out[12] = a[12];
  out[13] = a[13];
  out[14] = a[14];
  out[15] = a[15];
  return out;
}
function rotate(out, a, rad, axis) {
  var x = axis[0], y = axis[1], z = axis[2];
  var len3 = Math.hypot(x, y, z);
  var s, c, t;
  var a00, a01, a02, a03;
  var a10, a11, a12, a13;
  var a20, a21, a22, a23;
  var b00, b01, b02;
  var b10, b11, b12;
  var b20, b21, b22;
  if (len3 < EPSILON) {
    return null;
  }
  len3 = 1 / len3;
  x *= len3;
  y *= len3;
  z *= len3;
  s = Math.sin(rad);
  c = Math.cos(rad);
  t = 1 - c;
  a00 = a[0];
  a01 = a[1];
  a02 = a[2];
  a03 = a[3];
  a10 = a[4];
  a11 = a[5];
  a12 = a[6];
  a13 = a[7];
  a20 = a[8];
  a21 = a[9];
  a22 = a[10];
  a23 = a[11];
  b00 = x * x * t + c;
  b01 = y * x * t + z * s;
  b02 = z * x * t - y * s;
  b10 = x * y * t - z * s;
  b11 = y * y * t + c;
  b12 = z * y * t + x * s;
  b20 = x * z * t + y * s;
  b21 = y * z * t - x * s;
  b22 = z * z * t + c;
  out[0] = a00 * b00 + a10 * b01 + a20 * b02;
  out[1] = a01 * b00 + a11 * b01 + a21 * b02;
  out[2] = a02 * b00 + a12 * b01 + a22 * b02;
  out[3] = a03 * b00 + a13 * b01 + a23 * b02;
  out[4] = a00 * b10 + a10 * b11 + a20 * b12;
  out[5] = a01 * b10 + a11 * b11 + a21 * b12;
  out[6] = a02 * b10 + a12 * b11 + a22 * b12;
  out[7] = a03 * b10 + a13 * b11 + a23 * b12;
  out[8] = a00 * b20 + a10 * b21 + a20 * b22;
  out[9] = a01 * b20 + a11 * b21 + a21 * b22;
  out[10] = a02 * b20 + a12 * b21 + a22 * b22;
  out[11] = a03 * b20 + a13 * b21 + a23 * b22;
  if (a !== out) {
    out[12] = a[12];
    out[13] = a[13];
    out[14] = a[14];
    out[15] = a[15];
  }
  return out;
}
function rotateX(out, a, rad) {
  var s = Math.sin(rad);
  var c = Math.cos(rad);
  var a10 = a[4];
  var a11 = a[5];
  var a12 = a[6];
  var a13 = a[7];
  var a20 = a[8];
  var a21 = a[9];
  var a22 = a[10];
  var a23 = a[11];
  if (a !== out) {
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
    out[12] = a[12];
    out[13] = a[13];
    out[14] = a[14];
    out[15] = a[15];
  }
  out[4] = a10 * c + a20 * s;
  out[5] = a11 * c + a21 * s;
  out[6] = a12 * c + a22 * s;
  out[7] = a13 * c + a23 * s;
  out[8] = a20 * c - a10 * s;
  out[9] = a21 * c - a11 * s;
  out[10] = a22 * c - a12 * s;
  out[11] = a23 * c - a13 * s;
  return out;
}
function rotateY(out, a, rad) {
  var s = Math.sin(rad);
  var c = Math.cos(rad);
  var a00 = a[0];
  var a01 = a[1];
  var a02 = a[2];
  var a03 = a[3];
  var a20 = a[8];
  var a21 = a[9];
  var a22 = a[10];
  var a23 = a[11];
  if (a !== out) {
    out[4] = a[4];
    out[5] = a[5];
    out[6] = a[6];
    out[7] = a[7];
    out[12] = a[12];
    out[13] = a[13];
    out[14] = a[14];
    out[15] = a[15];
  }
  out[0] = a00 * c - a20 * s;
  out[1] = a01 * c - a21 * s;
  out[2] = a02 * c - a22 * s;
  out[3] = a03 * c - a23 * s;
  out[8] = a00 * s + a20 * c;
  out[9] = a01 * s + a21 * c;
  out[10] = a02 * s + a22 * c;
  out[11] = a03 * s + a23 * c;
  return out;
}
function rotateZ(out, a, rad) {
  var s = Math.sin(rad);
  var c = Math.cos(rad);
  var a00 = a[0];
  var a01 = a[1];
  var a02 = a[2];
  var a03 = a[3];
  var a10 = a[4];
  var a11 = a[5];
  var a12 = a[6];
  var a13 = a[7];
  if (a !== out) {
    out[8] = a[8];
    out[9] = a[9];
    out[10] = a[10];
    out[11] = a[11];
    out[12] = a[12];
    out[13] = a[13];
    out[14] = a[14];
    out[15] = a[15];
  }
  out[0] = a00 * c + a10 * s;
  out[1] = a01 * c + a11 * s;
  out[2] = a02 * c + a12 * s;
  out[3] = a03 * c + a13 * s;
  out[4] = a10 * c - a00 * s;
  out[5] = a11 * c - a01 * s;
  out[6] = a12 * c - a02 * s;
  out[7] = a13 * c - a03 * s;
  return out;
}
function fromTranslation(out, v) {
  out[0] = 1;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = 1;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 1;
  out[11] = 0;
  out[12] = v[0];
  out[13] = v[1];
  out[14] = v[2];
  out[15] = 1;
  return out;
}
function fromScaling(out, v) {
  out[0] = v[0];
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = v[1];
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = v[2];
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}
function fromRotation(out, rad, axis) {
  var x = axis[0], y = axis[1], z = axis[2];
  var len3 = Math.hypot(x, y, z);
  var s, c, t;
  if (len3 < EPSILON) {
    return null;
  }
  len3 = 1 / len3;
  x *= len3;
  y *= len3;
  z *= len3;
  s = Math.sin(rad);
  c = Math.cos(rad);
  t = 1 - c;
  out[0] = x * x * t + c;
  out[1] = y * x * t + z * s;
  out[2] = z * x * t - y * s;
  out[3] = 0;
  out[4] = x * y * t - z * s;
  out[5] = y * y * t + c;
  out[6] = z * y * t + x * s;
  out[7] = 0;
  out[8] = x * z * t + y * s;
  out[9] = y * z * t - x * s;
  out[10] = z * z * t + c;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}
function fromXRotation(out, rad) {
  var s = Math.sin(rad);
  var c = Math.cos(rad);
  out[0] = 1;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = c;
  out[6] = s;
  out[7] = 0;
  out[8] = 0;
  out[9] = -s;
  out[10] = c;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}
function fromYRotation(out, rad) {
  var s = Math.sin(rad);
  var c = Math.cos(rad);
  out[0] = c;
  out[1] = 0;
  out[2] = -s;
  out[3] = 0;
  out[4] = 0;
  out[5] = 1;
  out[6] = 0;
  out[7] = 0;
  out[8] = s;
  out[9] = 0;
  out[10] = c;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}
function fromZRotation(out, rad) {
  var s = Math.sin(rad);
  var c = Math.cos(rad);
  out[0] = c;
  out[1] = s;
  out[2] = 0;
  out[3] = 0;
  out[4] = -s;
  out[5] = c;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 1;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}
function fromRotationTranslation(out, q, v) {
  var x = q[0], y = q[1], z = q[2], w = q[3];
  var x2 = x + x;
  var y2 = y + y;
  var z2 = z + z;
  var xx = x * x2;
  var xy = x * y2;
  var xz = x * z2;
  var yy = y * y2;
  var yz = y * z2;
  var zz = z * z2;
  var wx = w * x2;
  var wy = w * y2;
  var wz = w * z2;
  out[0] = 1 - (yy + zz);
  out[1] = xy + wz;
  out[2] = xz - wy;
  out[3] = 0;
  out[4] = xy - wz;
  out[5] = 1 - (xx + zz);
  out[6] = yz + wx;
  out[7] = 0;
  out[8] = xz + wy;
  out[9] = yz - wx;
  out[10] = 1 - (xx + yy);
  out[11] = 0;
  out[12] = v[0];
  out[13] = v[1];
  out[14] = v[2];
  out[15] = 1;
  return out;
}
function fromQuat2(out, a) {
  var translation = new ARRAY_TYPE(3);
  var bx = -a[0], by = -a[1], bz = -a[2], bw = a[3], ax = a[4], ay = a[5], az = a[6], aw = a[7];
  var magnitude = bx * bx + by * by + bz * bz + bw * bw;
  if (magnitude > 0) {
    translation[0] = (ax * bw + aw * bx + ay * bz - az * by) * 2 / magnitude;
    translation[1] = (ay * bw + aw * by + az * bx - ax * bz) * 2 / magnitude;
    translation[2] = (az * bw + aw * bz + ax * by - ay * bx) * 2 / magnitude;
  } else {
    translation[0] = (ax * bw + aw * bx + ay * bz - az * by) * 2;
    translation[1] = (ay * bw + aw * by + az * bx - ax * bz) * 2;
    translation[2] = (az * bw + aw * bz + ax * by - ay * bx) * 2;
  }
  fromRotationTranslation(out, a, translation);
  return out;
}
function getTranslation(out, mat) {
  out[0] = mat[12];
  out[1] = mat[13];
  out[2] = mat[14];
  return out;
}
function getScaling(out, mat) {
  var m11 = mat[0];
  var m12 = mat[1];
  var m13 = mat[2];
  var m21 = mat[4];
  var m22 = mat[5];
  var m23 = mat[6];
  var m31 = mat[8];
  var m32 = mat[9];
  var m33 = mat[10];
  out[0] = Math.hypot(m11, m12, m13);
  out[1] = Math.hypot(m21, m22, m23);
  out[2] = Math.hypot(m31, m32, m33);
  return out;
}
function getRotation(out, mat) {
  var scaling = new ARRAY_TYPE(3);
  getScaling(scaling, mat);
  var is1 = 1 / scaling[0];
  var is2 = 1 / scaling[1];
  var is3 = 1 / scaling[2];
  var sm11 = mat[0] * is1;
  var sm12 = mat[1] * is2;
  var sm13 = mat[2] * is3;
  var sm21 = mat[4] * is1;
  var sm22 = mat[5] * is2;
  var sm23 = mat[6] * is3;
  var sm31 = mat[8] * is1;
  var sm32 = mat[9] * is2;
  var sm33 = mat[10] * is3;
  var trace = sm11 + sm22 + sm33;
  var S = 0;
  if (trace > 0) {
    S = Math.sqrt(trace + 1) * 2;
    out[3] = 0.25 * S;
    out[0] = (sm23 - sm32) / S;
    out[1] = (sm31 - sm13) / S;
    out[2] = (sm12 - sm21) / S;
  } else if (sm11 > sm22 && sm11 > sm33) {
    S = Math.sqrt(1 + sm11 - sm22 - sm33) * 2;
    out[3] = (sm23 - sm32) / S;
    out[0] = 0.25 * S;
    out[1] = (sm12 + sm21) / S;
    out[2] = (sm31 + sm13) / S;
  } else if (sm22 > sm33) {
    S = Math.sqrt(1 + sm22 - sm11 - sm33) * 2;
    out[3] = (sm31 - sm13) / S;
    out[0] = (sm12 + sm21) / S;
    out[1] = 0.25 * S;
    out[2] = (sm23 + sm32) / S;
  } else {
    S = Math.sqrt(1 + sm33 - sm11 - sm22) * 2;
    out[3] = (sm12 - sm21) / S;
    out[0] = (sm31 + sm13) / S;
    out[1] = (sm23 + sm32) / S;
    out[2] = 0.25 * S;
  }
  return out;
}
function fromRotationTranslationScale(out, q, v, s) {
  var x = q[0], y = q[1], z = q[2], w = q[3];
  var x2 = x + x;
  var y2 = y + y;
  var z2 = z + z;
  var xx = x * x2;
  var xy = x * y2;
  var xz = x * z2;
  var yy = y * y2;
  var yz = y * z2;
  var zz = z * z2;
  var wx = w * x2;
  var wy = w * y2;
  var wz = w * z2;
  var sx = s[0];
  var sy = s[1];
  var sz = s[2];
  out[0] = (1 - (yy + zz)) * sx;
  out[1] = (xy + wz) * sx;
  out[2] = (xz - wy) * sx;
  out[3] = 0;
  out[4] = (xy - wz) * sy;
  out[5] = (1 - (xx + zz)) * sy;
  out[6] = (yz + wx) * sy;
  out[7] = 0;
  out[8] = (xz + wy) * sz;
  out[9] = (yz - wx) * sz;
  out[10] = (1 - (xx + yy)) * sz;
  out[11] = 0;
  out[12] = v[0];
  out[13] = v[1];
  out[14] = v[2];
  out[15] = 1;
  return out;
}
function fromRotationTranslationScaleOrigin(out, q, v, s, o) {
  var x = q[0], y = q[1], z = q[2], w = q[3];
  var x2 = x + x;
  var y2 = y + y;
  var z2 = z + z;
  var xx = x * x2;
  var xy = x * y2;
  var xz = x * z2;
  var yy = y * y2;
  var yz = y * z2;
  var zz = z * z2;
  var wx = w * x2;
  var wy = w * y2;
  var wz = w * z2;
  var sx = s[0];
  var sy = s[1];
  var sz = s[2];
  var ox = o[0];
  var oy = o[1];
  var oz = o[2];
  var out0 = (1 - (yy + zz)) * sx;
  var out1 = (xy + wz) * sx;
  var out2 = (xz - wy) * sx;
  var out4 = (xy - wz) * sy;
  var out5 = (1 - (xx + zz)) * sy;
  var out6 = (yz + wx) * sy;
  var out8 = (xz + wy) * sz;
  var out9 = (yz - wx) * sz;
  var out10 = (1 - (xx + yy)) * sz;
  out[0] = out0;
  out[1] = out1;
  out[2] = out2;
  out[3] = 0;
  out[4] = out4;
  out[5] = out5;
  out[6] = out6;
  out[7] = 0;
  out[8] = out8;
  out[9] = out9;
  out[10] = out10;
  out[11] = 0;
  out[12] = v[0] + ox - (out0 * ox + out4 * oy + out8 * oz);
  out[13] = v[1] + oy - (out1 * ox + out5 * oy + out9 * oz);
  out[14] = v[2] + oz - (out2 * ox + out6 * oy + out10 * oz);
  out[15] = 1;
  return out;
}
function fromQuat(out, q) {
  var x = q[0], y = q[1], z = q[2], w = q[3];
  var x2 = x + x;
  var y2 = y + y;
  var z2 = z + z;
  var xx = x * x2;
  var yx = y * x2;
  var yy = y * y2;
  var zx = z * x2;
  var zy = z * y2;
  var zz = z * z2;
  var wx = w * x2;
  var wy = w * y2;
  var wz = w * z2;
  out[0] = 1 - yy - zz;
  out[1] = yx + wz;
  out[2] = zx - wy;
  out[3] = 0;
  out[4] = yx - wz;
  out[5] = 1 - xx - zz;
  out[6] = zy + wx;
  out[7] = 0;
  out[8] = zx + wy;
  out[9] = zy - wx;
  out[10] = 1 - xx - yy;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}
function frustum(out, left, right, bottom, top, near, far) {
  var rl = 1 / (right - left);
  var tb = 1 / (top - bottom);
  var nf = 1 / (near - far);
  out[0] = near * 2 * rl;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = near * 2 * tb;
  out[6] = 0;
  out[7] = 0;
  out[8] = (right + left) * rl;
  out[9] = (top + bottom) * tb;
  out[10] = (far + near) * nf;
  out[11] = -1;
  out[12] = 0;
  out[13] = 0;
  out[14] = far * near * 2 * nf;
  out[15] = 0;
  return out;
}
function perspectiveNO(out, fovy, aspect, near, far) {
  var f = 1 / Math.tan(fovy / 2), nf;
  out[0] = f / aspect;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = f;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[11] = -1;
  out[12] = 0;
  out[13] = 0;
  out[15] = 0;
  if (far != null && far !== Infinity) {
    nf = 1 / (near - far);
    out[10] = (far + near) * nf;
    out[14] = 2 * far * near * nf;
  } else {
    out[10] = -1;
    out[14] = -2 * near;
  }
  return out;
}
var perspective = perspectiveNO;
function perspectiveZO(out, fovy, aspect, near, far) {
  var f = 1 / Math.tan(fovy / 2), nf;
  out[0] = f / aspect;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = f;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[11] = -1;
  out[12] = 0;
  out[13] = 0;
  out[15] = 0;
  if (far != null && far !== Infinity) {
    nf = 1 / (near - far);
    out[10] = far * nf;
    out[14] = far * near * nf;
  } else {
    out[10] = -1;
    out[14] = -near;
  }
  return out;
}
function perspectiveFromFieldOfView(out, fov, near, far) {
  var upTan = Math.tan(fov.upDegrees * Math.PI / 180);
  var downTan = Math.tan(fov.downDegrees * Math.PI / 180);
  var leftTan = Math.tan(fov.leftDegrees * Math.PI / 180);
  var rightTan = Math.tan(fov.rightDegrees * Math.PI / 180);
  var xScale = 2 / (leftTan + rightTan);
  var yScale = 2 / (upTan + downTan);
  out[0] = xScale;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = yScale;
  out[6] = 0;
  out[7] = 0;
  out[8] = -((leftTan - rightTan) * xScale * 0.5);
  out[9] = (upTan - downTan) * yScale * 0.5;
  out[10] = far / (near - far);
  out[11] = -1;
  out[12] = 0;
  out[13] = 0;
  out[14] = far * near / (near - far);
  out[15] = 0;
  return out;
}
function orthoNO(out, left, right, bottom, top, near, far) {
  var lr = 1 / (left - right);
  var bt = 1 / (bottom - top);
  var nf = 1 / (near - far);
  out[0] = -2 * lr;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = -2 * bt;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 2 * nf;
  out[11] = 0;
  out[12] = (left + right) * lr;
  out[13] = (top + bottom) * bt;
  out[14] = (far + near) * nf;
  out[15] = 1;
  return out;
}
var ortho = orthoNO;
function orthoZO(out, left, right, bottom, top, near, far) {
  var lr = 1 / (left - right);
  var bt = 1 / (bottom - top);
  var nf = 1 / (near - far);
  out[0] = -2 * lr;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = -2 * bt;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = nf;
  out[11] = 0;
  out[12] = (left + right) * lr;
  out[13] = (top + bottom) * bt;
  out[14] = near * nf;
  out[15] = 1;
  return out;
}
function lookAt(out, eye, center, up) {
  var x0, x1, x2, y0, y1, y2, z0, z1, z2, len3;
  var eyex = eye[0];
  var eyey = eye[1];
  var eyez = eye[2];
  var upx = up[0];
  var upy = up[1];
  var upz = up[2];
  var centerx = center[0];
  var centery = center[1];
  var centerz = center[2];
  if (Math.abs(eyex - centerx) < EPSILON && Math.abs(eyey - centery) < EPSILON && Math.abs(eyez - centerz) < EPSILON) {
    return identity(out);
  }
  z0 = eyex - centerx;
  z1 = eyey - centery;
  z2 = eyez - centerz;
  len3 = 1 / Math.hypot(z0, z1, z2);
  z0 *= len3;
  z1 *= len3;
  z2 *= len3;
  x0 = upy * z2 - upz * z1;
  x1 = upz * z0 - upx * z2;
  x2 = upx * z1 - upy * z0;
  len3 = Math.hypot(x0, x1, x2);
  if (!len3) {
    x0 = 0;
    x1 = 0;
    x2 = 0;
  } else {
    len3 = 1 / len3;
    x0 *= len3;
    x1 *= len3;
    x2 *= len3;
  }
  y0 = z1 * x2 - z2 * x1;
  y1 = z2 * x0 - z0 * x2;
  y2 = z0 * x1 - z1 * x0;
  len3 = Math.hypot(y0, y1, y2);
  if (!len3) {
    y0 = 0;
    y1 = 0;
    y2 = 0;
  } else {
    len3 = 1 / len3;
    y0 *= len3;
    y1 *= len3;
    y2 *= len3;
  }
  out[0] = x0;
  out[1] = y0;
  out[2] = z0;
  out[3] = 0;
  out[4] = x1;
  out[5] = y1;
  out[6] = z1;
  out[7] = 0;
  out[8] = x2;
  out[9] = y2;
  out[10] = z2;
  out[11] = 0;
  out[12] = -(x0 * eyex + x1 * eyey + x2 * eyez);
  out[13] = -(y0 * eyex + y1 * eyey + y2 * eyez);
  out[14] = -(z0 * eyex + z1 * eyey + z2 * eyez);
  out[15] = 1;
  return out;
}
function targetTo(out, eye, target, up) {
  var eyex = eye[0], eyey = eye[1], eyez = eye[2], upx = up[0], upy = up[1], upz = up[2];
  var z0 = eyex - target[0], z1 = eyey - target[1], z2 = eyez - target[2];
  var len3 = z0 * z0 + z1 * z1 + z2 * z2;
  if (len3 > 0) {
    len3 = 1 / Math.sqrt(len3);
    z0 *= len3;
    z1 *= len3;
    z2 *= len3;
  }
  var x0 = upy * z2 - upz * z1, x1 = upz * z0 - upx * z2, x2 = upx * z1 - upy * z0;
  len3 = x0 * x0 + x1 * x1 + x2 * x2;
  if (len3 > 0) {
    len3 = 1 / Math.sqrt(len3);
    x0 *= len3;
    x1 *= len3;
    x2 *= len3;
  }
  out[0] = x0;
  out[1] = x1;
  out[2] = x2;
  out[3] = 0;
  out[4] = z1 * x2 - z2 * x1;
  out[5] = z2 * x0 - z0 * x2;
  out[6] = z0 * x1 - z1 * x0;
  out[7] = 0;
  out[8] = z0;
  out[9] = z1;
  out[10] = z2;
  out[11] = 0;
  out[12] = eyex;
  out[13] = eyey;
  out[14] = eyez;
  out[15] = 1;
  return out;
}
function str(a) {
  return "mat4(" + a[0] + ", " + a[1] + ", " + a[2] + ", " + a[3] + ", " + a[4] + ", " + a[5] + ", " + a[6] + ", " + a[7] + ", " + a[8] + ", " + a[9] + ", " + a[10] + ", " + a[11] + ", " + a[12] + ", " + a[13] + ", " + a[14] + ", " + a[15] + ")";
}
function frob(a) {
  return Math.hypot(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15]);
}
function add(out, a, b) {
  out[0] = a[0] + b[0];
  out[1] = a[1] + b[1];
  out[2] = a[2] + b[2];
  out[3] = a[3] + b[3];
  out[4] = a[4] + b[4];
  out[5] = a[5] + b[5];
  out[6] = a[6] + b[6];
  out[7] = a[7] + b[7];
  out[8] = a[8] + b[8];
  out[9] = a[9] + b[9];
  out[10] = a[10] + b[10];
  out[11] = a[11] + b[11];
  out[12] = a[12] + b[12];
  out[13] = a[13] + b[13];
  out[14] = a[14] + b[14];
  out[15] = a[15] + b[15];
  return out;
}
function subtract(out, a, b) {
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
  out[3] = a[3] - b[3];
  out[4] = a[4] - b[4];
  out[5] = a[5] - b[5];
  out[6] = a[6] - b[6];
  out[7] = a[7] - b[7];
  out[8] = a[8] - b[8];
  out[9] = a[9] - b[9];
  out[10] = a[10] - b[10];
  out[11] = a[11] - b[11];
  out[12] = a[12] - b[12];
  out[13] = a[13] - b[13];
  out[14] = a[14] - b[14];
  out[15] = a[15] - b[15];
  return out;
}
function multiplyScalar(out, a, b) {
  out[0] = a[0] * b;
  out[1] = a[1] * b;
  out[2] = a[2] * b;
  out[3] = a[3] * b;
  out[4] = a[4] * b;
  out[5] = a[5] * b;
  out[6] = a[6] * b;
  out[7] = a[7] * b;
  out[8] = a[8] * b;
  out[9] = a[9] * b;
  out[10] = a[10] * b;
  out[11] = a[11] * b;
  out[12] = a[12] * b;
  out[13] = a[13] * b;
  out[14] = a[14] * b;
  out[15] = a[15] * b;
  return out;
}
function multiplyScalarAndAdd(out, a, b, scale5) {
  out[0] = a[0] + b[0] * scale5;
  out[1] = a[1] + b[1] * scale5;
  out[2] = a[2] + b[2] * scale5;
  out[3] = a[3] + b[3] * scale5;
  out[4] = a[4] + b[4] * scale5;
  out[5] = a[5] + b[5] * scale5;
  out[6] = a[6] + b[6] * scale5;
  out[7] = a[7] + b[7] * scale5;
  out[8] = a[8] + b[8] * scale5;
  out[9] = a[9] + b[9] * scale5;
  out[10] = a[10] + b[10] * scale5;
  out[11] = a[11] + b[11] * scale5;
  out[12] = a[12] + b[12] * scale5;
  out[13] = a[13] + b[13] * scale5;
  out[14] = a[14] + b[14] * scale5;
  out[15] = a[15] + b[15] * scale5;
  return out;
}
function exactEquals(a, b) {
  return a[0] === b[0] && a[1] === b[1] && a[2] === b[2] && a[3] === b[3] && a[4] === b[4] && a[5] === b[5] && a[6] === b[6] && a[7] === b[7] && a[8] === b[8] && a[9] === b[9] && a[10] === b[10] && a[11] === b[11] && a[12] === b[12] && a[13] === b[13] && a[14] === b[14] && a[15] === b[15];
}
function equals(a, b) {
  var a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
  var a4 = a[4], a5 = a[5], a6 = a[6], a7 = a[7];
  var a8 = a[8], a9 = a[9], a10 = a[10], a11 = a[11];
  var a12 = a[12], a13 = a[13], a14 = a[14], a15 = a[15];
  var b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];
  var b4 = b[4], b5 = b[5], b6 = b[6], b7 = b[7];
  var b8 = b[8], b9 = b[9], b10 = b[10], b11 = b[11];
  var b12 = b[12], b13 = b[13], b14 = b[14], b15 = b[15];
  return Math.abs(a0 - b0) <= EPSILON * Math.max(1, Math.abs(a0), Math.abs(b0)) && Math.abs(a1 - b1) <= EPSILON * Math.max(1, Math.abs(a1), Math.abs(b1)) && Math.abs(a2 - b2) <= EPSILON * Math.max(1, Math.abs(a2), Math.abs(b2)) && Math.abs(a3 - b3) <= EPSILON * Math.max(1, Math.abs(a3), Math.abs(b3)) && Math.abs(a4 - b4) <= EPSILON * Math.max(1, Math.abs(a4), Math.abs(b4)) && Math.abs(a5 - b5) <= EPSILON * Math.max(1, Math.abs(a5), Math.abs(b5)) && Math.abs(a6 - b6) <= EPSILON * Math.max(1, Math.abs(a6), Math.abs(b6)) && Math.abs(a7 - b7) <= EPSILON * Math.max(1, Math.abs(a7), Math.abs(b7)) && Math.abs(a8 - b8) <= EPSILON * Math.max(1, Math.abs(a8), Math.abs(b8)) && Math.abs(a9 - b9) <= EPSILON * Math.max(1, Math.abs(a9), Math.abs(b9)) && Math.abs(a10 - b10) <= EPSILON * Math.max(1, Math.abs(a10), Math.abs(b10)) && Math.abs(a11 - b11) <= EPSILON * Math.max(1, Math.abs(a11), Math.abs(b11)) && Math.abs(a12 - b12) <= EPSILON * Math.max(1, Math.abs(a12), Math.abs(b12)) && Math.abs(a13 - b13) <= EPSILON * Math.max(1, Math.abs(a13), Math.abs(b13)) && Math.abs(a14 - b14) <= EPSILON * Math.max(1, Math.abs(a14), Math.abs(b14)) && Math.abs(a15 - b15) <= EPSILON * Math.max(1, Math.abs(a15), Math.abs(b15));
}
var mul = multiply;
var sub = subtract;

// node_modules/gl-matrix/esm/quat.js
var quat_exports = {};
__export(quat_exports, {
  add: () => add4,
  calculateW: () => calculateW,
  clone: () => clone4,
  conjugate: () => conjugate,
  copy: () => copy4,
  create: () => create5,
  dot: () => dot3,
  equals: () => equals4,
  exactEquals: () => exactEquals4,
  exp: () => exp,
  fromEuler: () => fromEuler,
  fromMat3: () => fromMat3,
  fromValues: () => fromValues4,
  getAngle: () => getAngle,
  getAxisAngle: () => getAxisAngle,
  identity: () => identity2,
  invert: () => invert2,
  len: () => len2,
  length: () => length3,
  lerp: () => lerp3,
  ln: () => ln,
  mul: () => mul3,
  multiply: () => multiply3,
  normalize: () => normalize3,
  pow: () => pow,
  random: () => random2,
  rotateX: () => rotateX3,
  rotateY: () => rotateY3,
  rotateZ: () => rotateZ3,
  rotationTo: () => rotationTo,
  scale: () => scale4,
  set: () => set4,
  setAxes: () => setAxes,
  setAxisAngle: () => setAxisAngle,
  slerp: () => slerp,
  sqlerp: () => sqlerp,
  sqrLen: () => sqrLen2,
  squaredLength: () => squaredLength3,
  str: () => str3
});

// node_modules/gl-matrix/esm/vec3.js
var vec3_exports = {};
__export(vec3_exports, {
  add: () => add2,
  angle: () => angle,
  bezier: () => bezier,
  ceil: () => ceil,
  clone: () => clone2,
  copy: () => copy2,
  create: () => create3,
  cross: () => cross,
  dist: () => dist,
  distance: () => distance,
  div: () => div,
  divide: () => divide,
  dot: () => dot,
  equals: () => equals2,
  exactEquals: () => exactEquals2,
  floor: () => floor,
  forEach: () => forEach,
  fromValues: () => fromValues2,
  hermite: () => hermite,
  inverse: () => inverse,
  len: () => len,
  length: () => length,
  lerp: () => lerp,
  max: () => max,
  min: () => min,
  mul: () => mul2,
  multiply: () => multiply2,
  negate: () => negate,
  normalize: () => normalize,
  random: () => random,
  rotateX: () => rotateX2,
  rotateY: () => rotateY2,
  rotateZ: () => rotateZ2,
  round: () => round,
  scale: () => scale2,
  scaleAndAdd: () => scaleAndAdd,
  set: () => set2,
  sqrDist: () => sqrDist,
  sqrLen: () => sqrLen,
  squaredDistance: () => squaredDistance,
  squaredLength: () => squaredLength,
  str: () => str2,
  sub: () => sub2,
  subtract: () => subtract2,
  transformMat3: () => transformMat3,
  transformMat4: () => transformMat4,
  transformQuat: () => transformQuat,
  zero: () => zero
});
function create3() {
  var out = new ARRAY_TYPE(3);
  if (ARRAY_TYPE != Float32Array) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
  }
  return out;
}
function clone2(a) {
  var out = new ARRAY_TYPE(3);
  out[0] = a[0];
  out[1] = a[1];
  out[2] = a[2];
  return out;
}
function length(a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  return Math.hypot(x, y, z);
}
function fromValues2(x, y, z) {
  var out = new ARRAY_TYPE(3);
  out[0] = x;
  out[1] = y;
  out[2] = z;
  return out;
}
function copy2(out, a) {
  out[0] = a[0];
  out[1] = a[1];
  out[2] = a[2];
  return out;
}
function set2(out, x, y, z) {
  out[0] = x;
  out[1] = y;
  out[2] = z;
  return out;
}
function add2(out, a, b) {
  out[0] = a[0] + b[0];
  out[1] = a[1] + b[1];
  out[2] = a[2] + b[2];
  return out;
}
function subtract2(out, a, b) {
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
  return out;
}
function multiply2(out, a, b) {
  out[0] = a[0] * b[0];
  out[1] = a[1] * b[1];
  out[2] = a[2] * b[2];
  return out;
}
function divide(out, a, b) {
  out[0] = a[0] / b[0];
  out[1] = a[1] / b[1];
  out[2] = a[2] / b[2];
  return out;
}
function ceil(out, a) {
  out[0] = Math.ceil(a[0]);
  out[1] = Math.ceil(a[1]);
  out[2] = Math.ceil(a[2]);
  return out;
}
function floor(out, a) {
  out[0] = Math.floor(a[0]);
  out[1] = Math.floor(a[1]);
  out[2] = Math.floor(a[2]);
  return out;
}
function min(out, a, b) {
  out[0] = Math.min(a[0], b[0]);
  out[1] = Math.min(a[1], b[1]);
  out[2] = Math.min(a[2], b[2]);
  return out;
}
function max(out, a, b) {
  out[0] = Math.max(a[0], b[0]);
  out[1] = Math.max(a[1], b[1]);
  out[2] = Math.max(a[2], b[2]);
  return out;
}
function round(out, a) {
  out[0] = Math.round(a[0]);
  out[1] = Math.round(a[1]);
  out[2] = Math.round(a[2]);
  return out;
}
function scale2(out, a, b) {
  out[0] = a[0] * b;
  out[1] = a[1] * b;
  out[2] = a[2] * b;
  return out;
}
function scaleAndAdd(out, a, b, scale5) {
  out[0] = a[0] + b[0] * scale5;
  out[1] = a[1] + b[1] * scale5;
  out[2] = a[2] + b[2] * scale5;
  return out;
}
function distance(a, b) {
  var x = b[0] - a[0];
  var y = b[1] - a[1];
  var z = b[2] - a[2];
  return Math.hypot(x, y, z);
}
function squaredDistance(a, b) {
  var x = b[0] - a[0];
  var y = b[1] - a[1];
  var z = b[2] - a[2];
  return x * x + y * y + z * z;
}
function squaredLength(a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  return x * x + y * y + z * z;
}
function negate(out, a) {
  out[0] = -a[0];
  out[1] = -a[1];
  out[2] = -a[2];
  return out;
}
function inverse(out, a) {
  out[0] = 1 / a[0];
  out[1] = 1 / a[1];
  out[2] = 1 / a[2];
  return out;
}
function normalize(out, a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  var len3 = x * x + y * y + z * z;
  if (len3 > 0) {
    len3 = 1 / Math.sqrt(len3);
  }
  out[0] = a[0] * len3;
  out[1] = a[1] * len3;
  out[2] = a[2] * len3;
  return out;
}
function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
function cross(out, a, b) {
  var ax = a[0], ay = a[1], az = a[2];
  var bx = b[0], by = b[1], bz = b[2];
  out[0] = ay * bz - az * by;
  out[1] = az * bx - ax * bz;
  out[2] = ax * by - ay * bx;
  return out;
}
function lerp(out, a, b, t) {
  var ax = a[0];
  var ay = a[1];
  var az = a[2];
  out[0] = ax + t * (b[0] - ax);
  out[1] = ay + t * (b[1] - ay);
  out[2] = az + t * (b[2] - az);
  return out;
}
function hermite(out, a, b, c, d, t) {
  var factorTimes2 = t * t;
  var factor1 = factorTimes2 * (2 * t - 3) + 1;
  var factor2 = factorTimes2 * (t - 2) + t;
  var factor3 = factorTimes2 * (t - 1);
  var factor4 = factorTimes2 * (3 - 2 * t);
  out[0] = a[0] * factor1 + b[0] * factor2 + c[0] * factor3 + d[0] * factor4;
  out[1] = a[1] * factor1 + b[1] * factor2 + c[1] * factor3 + d[1] * factor4;
  out[2] = a[2] * factor1 + b[2] * factor2 + c[2] * factor3 + d[2] * factor4;
  return out;
}
function bezier(out, a, b, c, d, t) {
  var inverseFactor = 1 - t;
  var inverseFactorTimesTwo = inverseFactor * inverseFactor;
  var factorTimes2 = t * t;
  var factor1 = inverseFactorTimesTwo * inverseFactor;
  var factor2 = 3 * t * inverseFactorTimesTwo;
  var factor3 = 3 * factorTimes2 * inverseFactor;
  var factor4 = factorTimes2 * t;
  out[0] = a[0] * factor1 + b[0] * factor2 + c[0] * factor3 + d[0] * factor4;
  out[1] = a[1] * factor1 + b[1] * factor2 + c[1] * factor3 + d[1] * factor4;
  out[2] = a[2] * factor1 + b[2] * factor2 + c[2] * factor3 + d[2] * factor4;
  return out;
}
function random(out, scale5) {
  scale5 = scale5 || 1;
  var r = RANDOM() * 2 * Math.PI;
  var z = RANDOM() * 2 - 1;
  var zScale = Math.sqrt(1 - z * z) * scale5;
  out[0] = Math.cos(r) * zScale;
  out[1] = Math.sin(r) * zScale;
  out[2] = z * scale5;
  return out;
}
function transformMat4(out, a, m) {
  var x = a[0], y = a[1], z = a[2];
  var w = m[3] * x + m[7] * y + m[11] * z + m[15];
  w = w || 1;
  out[0] = (m[0] * x + m[4] * y + m[8] * z + m[12]) / w;
  out[1] = (m[1] * x + m[5] * y + m[9] * z + m[13]) / w;
  out[2] = (m[2] * x + m[6] * y + m[10] * z + m[14]) / w;
  return out;
}
function transformMat3(out, a, m) {
  var x = a[0], y = a[1], z = a[2];
  out[0] = x * m[0] + y * m[3] + z * m[6];
  out[1] = x * m[1] + y * m[4] + z * m[7];
  out[2] = x * m[2] + y * m[5] + z * m[8];
  return out;
}
function transformQuat(out, a, q) {
  var qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  var x = a[0], y = a[1], z = a[2];
  var uvx = qy * z - qz * y, uvy = qz * x - qx * z, uvz = qx * y - qy * x;
  var uuvx = qy * uvz - qz * uvy, uuvy = qz * uvx - qx * uvz, uuvz = qx * uvy - qy * uvx;
  var w2 = qw * 2;
  uvx *= w2;
  uvy *= w2;
  uvz *= w2;
  uuvx *= 2;
  uuvy *= 2;
  uuvz *= 2;
  out[0] = x + uvx + uuvx;
  out[1] = y + uvy + uuvy;
  out[2] = z + uvz + uuvz;
  return out;
}
function rotateX2(out, a, b, rad) {
  var p = [], r = [];
  p[0] = a[0] - b[0];
  p[1] = a[1] - b[1];
  p[2] = a[2] - b[2];
  r[0] = p[0];
  r[1] = p[1] * Math.cos(rad) - p[2] * Math.sin(rad);
  r[2] = p[1] * Math.sin(rad) + p[2] * Math.cos(rad);
  out[0] = r[0] + b[0];
  out[1] = r[1] + b[1];
  out[2] = r[2] + b[2];
  return out;
}
function rotateY2(out, a, b, rad) {
  var p = [], r = [];
  p[0] = a[0] - b[0];
  p[1] = a[1] - b[1];
  p[2] = a[2] - b[2];
  r[0] = p[2] * Math.sin(rad) + p[0] * Math.cos(rad);
  r[1] = p[1];
  r[2] = p[2] * Math.cos(rad) - p[0] * Math.sin(rad);
  out[0] = r[0] + b[0];
  out[1] = r[1] + b[1];
  out[2] = r[2] + b[2];
  return out;
}
function rotateZ2(out, a, b, rad) {
  var p = [], r = [];
  p[0] = a[0] - b[0];
  p[1] = a[1] - b[1];
  p[2] = a[2] - b[2];
  r[0] = p[0] * Math.cos(rad) - p[1] * Math.sin(rad);
  r[1] = p[0] * Math.sin(rad) + p[1] * Math.cos(rad);
  r[2] = p[2];
  out[0] = r[0] + b[0];
  out[1] = r[1] + b[1];
  out[2] = r[2] + b[2];
  return out;
}
function angle(a, b) {
  var ax = a[0], ay = a[1], az = a[2], bx = b[0], by = b[1], bz = b[2], mag1 = Math.sqrt(ax * ax + ay * ay + az * az), mag2 = Math.sqrt(bx * bx + by * by + bz * bz), mag = mag1 * mag2, cosine = mag && dot(a, b) / mag;
  return Math.acos(Math.min(Math.max(cosine, -1), 1));
}
function zero(out) {
  out[0] = 0;
  out[1] = 0;
  out[2] = 0;
  return out;
}
function str2(a) {
  return "vec3(" + a[0] + ", " + a[1] + ", " + a[2] + ")";
}
function exactEquals2(a, b) {
  return a[0] === b[0] && a[1] === b[1] && a[2] === b[2];
}
function equals2(a, b) {
  var a0 = a[0], a1 = a[1], a2 = a[2];
  var b0 = b[0], b1 = b[1], b2 = b[2];
  return Math.abs(a0 - b0) <= EPSILON * Math.max(1, Math.abs(a0), Math.abs(b0)) && Math.abs(a1 - b1) <= EPSILON * Math.max(1, Math.abs(a1), Math.abs(b1)) && Math.abs(a2 - b2) <= EPSILON * Math.max(1, Math.abs(a2), Math.abs(b2));
}
var sub2 = subtract2;
var mul2 = multiply2;
var div = divide;
var dist = distance;
var sqrDist = squaredDistance;
var len = length;
var sqrLen = squaredLength;
var forEach = function() {
  var vec = create3();
  return function(a, stride, offset, count, fn, arg) {
    var i, l;
    if (!stride) {
      stride = 3;
    }
    if (!offset) {
      offset = 0;
    }
    if (count) {
      l = Math.min(count * stride + offset, a.length);
    } else {
      l = a.length;
    }
    for (i = offset; i < l; i += stride) {
      vec[0] = a[i];
      vec[1] = a[i + 1];
      vec[2] = a[i + 2];
      fn(vec, vec, arg);
      a[i] = vec[0];
      a[i + 1] = vec[1];
      a[i + 2] = vec[2];
    }
    return a;
  };
}();

// node_modules/gl-matrix/esm/vec4.js
function create4() {
  var out = new ARRAY_TYPE(4);
  if (ARRAY_TYPE != Float32Array) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
  }
  return out;
}
function clone3(a) {
  var out = new ARRAY_TYPE(4);
  out[0] = a[0];
  out[1] = a[1];
  out[2] = a[2];
  out[3] = a[3];
  return out;
}
function fromValues3(x, y, z, w) {
  var out = new ARRAY_TYPE(4);
  out[0] = x;
  out[1] = y;
  out[2] = z;
  out[3] = w;
  return out;
}
function copy3(out, a) {
  out[0] = a[0];
  out[1] = a[1];
  out[2] = a[2];
  out[3] = a[3];
  return out;
}
function set3(out, x, y, z, w) {
  out[0] = x;
  out[1] = y;
  out[2] = z;
  out[3] = w;
  return out;
}
function add3(out, a, b) {
  out[0] = a[0] + b[0];
  out[1] = a[1] + b[1];
  out[2] = a[2] + b[2];
  out[3] = a[3] + b[3];
  return out;
}
function scale3(out, a, b) {
  out[0] = a[0] * b;
  out[1] = a[1] * b;
  out[2] = a[2] * b;
  out[3] = a[3] * b;
  return out;
}
function length2(a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  var w = a[3];
  return Math.hypot(x, y, z, w);
}
function squaredLength2(a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  var w = a[3];
  return x * x + y * y + z * z + w * w;
}
function normalize2(out, a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  var w = a[3];
  var len3 = x * x + y * y + z * z + w * w;
  if (len3 > 0) {
    len3 = 1 / Math.sqrt(len3);
  }
  out[0] = x * len3;
  out[1] = y * len3;
  out[2] = z * len3;
  out[3] = w * len3;
  return out;
}
function dot2(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}
function lerp2(out, a, b, t) {
  var ax = a[0];
  var ay = a[1];
  var az = a[2];
  var aw = a[3];
  out[0] = ax + t * (b[0] - ax);
  out[1] = ay + t * (b[1] - ay);
  out[2] = az + t * (b[2] - az);
  out[3] = aw + t * (b[3] - aw);
  return out;
}
function exactEquals3(a, b) {
  return a[0] === b[0] && a[1] === b[1] && a[2] === b[2] && a[3] === b[3];
}
function equals3(a, b) {
  var a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
  var b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];
  return Math.abs(a0 - b0) <= EPSILON * Math.max(1, Math.abs(a0), Math.abs(b0)) && Math.abs(a1 - b1) <= EPSILON * Math.max(1, Math.abs(a1), Math.abs(b1)) && Math.abs(a2 - b2) <= EPSILON * Math.max(1, Math.abs(a2), Math.abs(b2)) && Math.abs(a3 - b3) <= EPSILON * Math.max(1, Math.abs(a3), Math.abs(b3));
}
var forEach2 = function() {
  var vec = create4();
  return function(a, stride, offset, count, fn, arg) {
    var i, l;
    if (!stride) {
      stride = 4;
    }
    if (!offset) {
      offset = 0;
    }
    if (count) {
      l = Math.min(count * stride + offset, a.length);
    } else {
      l = a.length;
    }
    for (i = offset; i < l; i += stride) {
      vec[0] = a[i];
      vec[1] = a[i + 1];
      vec[2] = a[i + 2];
      vec[3] = a[i + 3];
      fn(vec, vec, arg);
      a[i] = vec[0];
      a[i + 1] = vec[1];
      a[i + 2] = vec[2];
      a[i + 3] = vec[3];
    }
    return a;
  };
}();

// node_modules/gl-matrix/esm/quat.js
function create5() {
  var out = new ARRAY_TYPE(4);
  if (ARRAY_TYPE != Float32Array) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
  }
  out[3] = 1;
  return out;
}
function identity2(out) {
  out[0] = 0;
  out[1] = 0;
  out[2] = 0;
  out[3] = 1;
  return out;
}
function setAxisAngle(out, axis, rad) {
  rad = rad * 0.5;
  var s = Math.sin(rad);
  out[0] = s * axis[0];
  out[1] = s * axis[1];
  out[2] = s * axis[2];
  out[3] = Math.cos(rad);
  return out;
}
function getAxisAngle(out_axis, q) {
  var rad = Math.acos(q[3]) * 2;
  var s = Math.sin(rad / 2);
  if (s > EPSILON) {
    out_axis[0] = q[0] / s;
    out_axis[1] = q[1] / s;
    out_axis[2] = q[2] / s;
  } else {
    out_axis[0] = 1;
    out_axis[1] = 0;
    out_axis[2] = 0;
  }
  return rad;
}
function getAngle(a, b) {
  var dotproduct = dot3(a, b);
  return Math.acos(2 * dotproduct * dotproduct - 1);
}
function multiply3(out, a, b) {
  var ax = a[0], ay = a[1], az = a[2], aw = a[3];
  var bx = b[0], by = b[1], bz = b[2], bw = b[3];
  out[0] = ax * bw + aw * bx + ay * bz - az * by;
  out[1] = ay * bw + aw * by + az * bx - ax * bz;
  out[2] = az * bw + aw * bz + ax * by - ay * bx;
  out[3] = aw * bw - ax * bx - ay * by - az * bz;
  return out;
}
function rotateX3(out, a, rad) {
  rad *= 0.5;
  var ax = a[0], ay = a[1], az = a[2], aw = a[3];
  var bx = Math.sin(rad), bw = Math.cos(rad);
  out[0] = ax * bw + aw * bx;
  out[1] = ay * bw + az * bx;
  out[2] = az * bw - ay * bx;
  out[3] = aw * bw - ax * bx;
  return out;
}
function rotateY3(out, a, rad) {
  rad *= 0.5;
  var ax = a[0], ay = a[1], az = a[2], aw = a[3];
  var by = Math.sin(rad), bw = Math.cos(rad);
  out[0] = ax * bw - az * by;
  out[1] = ay * bw + aw * by;
  out[2] = az * bw + ax * by;
  out[3] = aw * bw - ay * by;
  return out;
}
function rotateZ3(out, a, rad) {
  rad *= 0.5;
  var ax = a[0], ay = a[1], az = a[2], aw = a[3];
  var bz = Math.sin(rad), bw = Math.cos(rad);
  out[0] = ax * bw + ay * bz;
  out[1] = ay * bw - ax * bz;
  out[2] = az * bw + aw * bz;
  out[3] = aw * bw - az * bz;
  return out;
}
function calculateW(out, a) {
  var x = a[0], y = a[1], z = a[2];
  out[0] = x;
  out[1] = y;
  out[2] = z;
  out[3] = Math.sqrt(Math.abs(1 - x * x - y * y - z * z));
  return out;
}
function exp(out, a) {
  var x = a[0], y = a[1], z = a[2], w = a[3];
  var r = Math.sqrt(x * x + y * y + z * z);
  var et = Math.exp(w);
  var s = r > 0 ? et * Math.sin(r) / r : 0;
  out[0] = x * s;
  out[1] = y * s;
  out[2] = z * s;
  out[3] = et * Math.cos(r);
  return out;
}
function ln(out, a) {
  var x = a[0], y = a[1], z = a[2], w = a[3];
  var r = Math.sqrt(x * x + y * y + z * z);
  var t = r > 0 ? Math.atan2(r, w) / r : 0;
  out[0] = x * t;
  out[1] = y * t;
  out[2] = z * t;
  out[3] = 0.5 * Math.log(x * x + y * y + z * z + w * w);
  return out;
}
function pow(out, a, b) {
  ln(out, a);
  scale4(out, out, b);
  exp(out, out);
  return out;
}
function slerp(out, a, b, t) {
  var ax = a[0], ay = a[1], az = a[2], aw = a[3];
  var bx = b[0], by = b[1], bz = b[2], bw = b[3];
  var omega, cosom, sinom, scale0, scale1;
  cosom = ax * bx + ay * by + az * bz + aw * bw;
  if (cosom < 0) {
    cosom = -cosom;
    bx = -bx;
    by = -by;
    bz = -bz;
    bw = -bw;
  }
  if (1 - cosom > EPSILON) {
    omega = Math.acos(cosom);
    sinom = Math.sin(omega);
    scale0 = Math.sin((1 - t) * omega) / sinom;
    scale1 = Math.sin(t * omega) / sinom;
  } else {
    scale0 = 1 - t;
    scale1 = t;
  }
  out[0] = scale0 * ax + scale1 * bx;
  out[1] = scale0 * ay + scale1 * by;
  out[2] = scale0 * az + scale1 * bz;
  out[3] = scale0 * aw + scale1 * bw;
  return out;
}
function random2(out) {
  var u1 = RANDOM();
  var u2 = RANDOM();
  var u3 = RANDOM();
  var sqrt1MinusU1 = Math.sqrt(1 - u1);
  var sqrtU1 = Math.sqrt(u1);
  out[0] = sqrt1MinusU1 * Math.sin(2 * Math.PI * u2);
  out[1] = sqrt1MinusU1 * Math.cos(2 * Math.PI * u2);
  out[2] = sqrtU1 * Math.sin(2 * Math.PI * u3);
  out[3] = sqrtU1 * Math.cos(2 * Math.PI * u3);
  return out;
}
function invert2(out, a) {
  var a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
  var dot4 = a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3;
  var invDot = dot4 ? 1 / dot4 : 0;
  out[0] = -a0 * invDot;
  out[1] = -a1 * invDot;
  out[2] = -a2 * invDot;
  out[3] = a3 * invDot;
  return out;
}
function conjugate(out, a) {
  out[0] = -a[0];
  out[1] = -a[1];
  out[2] = -a[2];
  out[3] = a[3];
  return out;
}
function fromMat3(out, m) {
  var fTrace = m[0] + m[4] + m[8];
  var fRoot;
  if (fTrace > 0) {
    fRoot = Math.sqrt(fTrace + 1);
    out[3] = 0.5 * fRoot;
    fRoot = 0.5 / fRoot;
    out[0] = (m[5] - m[7]) * fRoot;
    out[1] = (m[6] - m[2]) * fRoot;
    out[2] = (m[1] - m[3]) * fRoot;
  } else {
    var i = 0;
    if (m[4] > m[0]) i = 1;
    if (m[8] > m[i * 3 + i]) i = 2;
    var j = (i + 1) % 3;
    var k = (i + 2) % 3;
    fRoot = Math.sqrt(m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + 1);
    out[i] = 0.5 * fRoot;
    fRoot = 0.5 / fRoot;
    out[3] = (m[j * 3 + k] - m[k * 3 + j]) * fRoot;
    out[j] = (m[j * 3 + i] + m[i * 3 + j]) * fRoot;
    out[k] = (m[k * 3 + i] + m[i * 3 + k]) * fRoot;
  }
  return out;
}
function fromEuler(out, x, y, z) {
  var halfToRad = 0.5 * Math.PI / 180;
  x *= halfToRad;
  y *= halfToRad;
  z *= halfToRad;
  var sx = Math.sin(x);
  var cx = Math.cos(x);
  var sy = Math.sin(y);
  var cy = Math.cos(y);
  var sz = Math.sin(z);
  var cz = Math.cos(z);
  out[0] = sx * cy * cz - cx * sy * sz;
  out[1] = cx * sy * cz + sx * cy * sz;
  out[2] = cx * cy * sz - sx * sy * cz;
  out[3] = cx * cy * cz + sx * sy * sz;
  return out;
}
function str3(a) {
  return "quat(" + a[0] + ", " + a[1] + ", " + a[2] + ", " + a[3] + ")";
}
var clone4 = clone3;
var fromValues4 = fromValues3;
var copy4 = copy3;
var set4 = set3;
var add4 = add3;
var mul3 = multiply3;
var scale4 = scale3;
var dot3 = dot2;
var lerp3 = lerp2;
var length3 = length2;
var len2 = length3;
var squaredLength3 = squaredLength2;
var sqrLen2 = squaredLength3;
var normalize3 = normalize2;
var exactEquals4 = exactEquals3;
var equals4 = equals3;
var rotationTo = function() {
  var tmpvec3 = create3();
  var xUnitVec3 = fromValues2(1, 0, 0);
  var yUnitVec3 = fromValues2(0, 1, 0);
  return function(out, a, b) {
    var dot4 = dot(a, b);
    if (dot4 < -0.999999) {
      cross(tmpvec3, xUnitVec3, a);
      if (len(tmpvec3) < 1e-6) cross(tmpvec3, yUnitVec3, a);
      normalize(tmpvec3, tmpvec3);
      setAxisAngle(out, tmpvec3, Math.PI);
      return out;
    } else if (dot4 > 0.999999) {
      out[0] = 0;
      out[1] = 0;
      out[2] = 0;
      out[3] = 1;
      return out;
    } else {
      cross(tmpvec3, a, b);
      out[0] = tmpvec3[0];
      out[1] = tmpvec3[1];
      out[2] = tmpvec3[2];
      out[3] = 1 + dot4;
      return normalize3(out, out);
    }
  };
}();
var sqlerp = function() {
  var temp1 = create5();
  var temp2 = create5();
  return function(out, a, b, c, d, t) {
    slerp(temp1, a, d, t);
    slerp(temp2, b, c, t);
    slerp(out, temp1, temp2, 2 * t * (1 - t));
    return out;
  };
}();
var setAxes = function() {
  var matr = create();
  return function(out, view, right, up) {
    matr[0] = right[0];
    matr[3] = right[1];
    matr[6] = right[2];
    matr[1] = up[0];
    matr[4] = up[1];
    matr[7] = up[2];
    matr[2] = -view[0];
    matr[5] = -view[1];
    matr[8] = -view[2];
    return normalize3(out, fromMat3(out, matr));
  };
}();

// src/den/collection/ArrayMap.ts
var ArrayMap = class {
  #list = [];
  // eslint-disable-next-line no-restricted-syntax
  get size() {
    return this.#list.length;
  }
  /** Clears array and returns removed elements */
  clear() {
    return this.#list.splice(0);
  }
  [Symbol.iterator]() {
    return this.#list[Symbol.iterator]();
  }
  /** Retrieve the key/value tuple at the given index, if it exists. */
  at(index) {
    return this.#list[index];
  }
  /**
   * Store a key/value tuple in the sorted list. If the key already exists, the
   * previous entry is overwritten.
   * Returns replaced value if it exists.
   */
  set(key, value) {
    const index = this.binarySearch(key);
    if (index >= 0) {
      const existingEntry = this.#list[index][1];
      this.#list[index][1] = value;
      return existingEntry;
    } else {
      const newEntry = [key, value];
      const greaterThanIndex = ~index;
      if (greaterThanIndex >= this.#list.length) {
        this.#list.push(newEntry);
      } else {
        this.#list.splice(greaterThanIndex, 0, newEntry);
      }
    }
    return void 0;
  }
  /** Removes the first element and returns it, if available. */
  shift() {
    return this.#list.shift();
  }
  /** Removes the last element and returns it, if available. */
  pop() {
    return this.#list.pop();
  }
  /** Removes the element with the given key, if it exists.
   * Returns element removed.
   */
  remove(key) {
    const index = this.binarySearch(key);
    if (index >= 0) {
      return this.#list.splice(index, 1)[0];
    }
    return void 0;
  }
  /** Removes all elements with keys greater than the given key.
   * Returns elements removed.
   */
  removeAfter(key) {
    const index = this.binarySearch(key);
    const greaterThanIndex = index >= 0 ? index + 1 : ~index;
    const removed = this.#list.splice(greaterThanIndex);
    return removed;
  }
  /** Removes all elements with keys less than the given key.
   * Returns elements removed.
   */
  removeBefore(key) {
    const index = this.binarySearch(key);
    const lessThanIndex = index >= 0 ? index : ~index;
    return this.#list.splice(0, lessThanIndex);
  }
  /** Access the first key/value tuple in the list, without modifying the list. */
  minEntry() {
    return this.#list[0];
  }
  /** Access the last key/value tuple in the list, without modifying the list. */
  maxEntry() {
    return this.#list[this.#list.length - 1];
  }
  /** Access the first key in the list, without modifying the list. */
  minKey() {
    return this.#list[0]?.[0];
  }
  /** Access the last key in the list, without modifying the list. */
  maxKey() {
    return this.#list[this.#list.length - 1]?.[0];
  }
  /**
   * Performs a binary search on the sorted list to find the index of the entry
   * with the given key.
   * @param key Key to search for.
   * @returns The index of the key/value tuple if an exact match is found;
   * otherwise, a negative number. If the key is not found and the key is less
   * than one or more keys in the list, the negative number returned is the
   * bitwise complement of the index of the first element with a larger key. If
   * the key is not found and is greater than all keys in the list, the negative
   * number returned is the bitwise complement of the index of the last element
   * plus 1.
   */
  binarySearch(key) {
    const list = this.#list;
    if (list.length === 0) {
      return -1;
    }
    let left = 0;
    let right = list.length - 1;
    if (key < list[left][0]) {
      return ~left;
    } else if (key > list[right][0]) {
      return ~(right + 1);
    }
    while (left <= right) {
      const mid = left + right >> 1;
      const midKey = list[mid][0];
      if (midKey === key) {
        return mid;
      } else if (key < midKey) {
        right = mid - 1;
      } else {
        left = mid + 1;
      }
    }
    return ~left;
  }
};

// src/den/collection/ObjectPool.ts
var ObjectPool = class {
  #init;
  #maxCapacity;
  #objects = [];
  /**
   *
   * @param init - A function that returns a new object.
   * @param options.maxCapacity - Limits the number of elements in the pool at a given time.
   */
  constructor(init, options = {}) {
    this.#init = init;
    this.#maxCapacity = options.maxCapacity;
  }
  /** Returns an object from the pool or instantiates and returns a new one if
   * there are none.
   */
  acquire() {
    return this.#objects.pop() ?? this.#init();
  }
  /** Release a object back to the pool to be reused.
   * If the maxCapacity is defined and has been reached it will be dropped.
   */
  release(obj) {
    if (this.#maxCapacity == void 0 || this.#objects.length < this.#maxCapacity) {
      this.#objects.push(obj);
    }
  }
  /**
   * Clears all objects in the pool.
   * Returns the objects that were cleared; this can be helpful if they have
   * custom dispose logic.
   */
  clear() {
    return this.#objects.splice(0);
  }
};

// src/transforms/geometry.ts
function makePose() {
  return { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
}
function xyzrpyToPose(xyz, rpy) {
  const o = quat_exports.fromEuler([0, 0, 0, 1], rpy[0], rpy[1], rpy[2]);
  return {
    position: { x: xyz[0], y: xyz[1], z: xyz[2] },
    orientation: { x: o[0], y: o[1], z: o[2], w: o[3] }
  };
}
function vec3Identity() {
  return [0, 0, 0];
}
function vec3FromValues(x, y, z) {
  return [x, y, z];
}
function vec3Clone(a) {
  return [a[0], a[1], a[2]];
}
function quatIdentity() {
  return [0, 0, 0, 1];
}
function quatFromValues(x, y, z, w) {
  return [x, y, z, w];
}
function quatClone(q) {
  return [q[0], q[1], q[2], q[3]];
}
function mat4Identity() {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}
function mat4FromValues(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33) {
  return [m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33];
}
function mat4Clone(m) {
  return [
    m[0],
    m[1],
    m[2],
    m[3],
    m[4],
    m[5],
    m[6],
    m[7],
    m[8],
    m[9],
    m[10],
    m[11],
    m[12],
    m[13],
    m[14],
    m[15]
  ];
}
function approxEq(v1, v2, epsilon = 1e-5) {
  return Math.abs(v1 - v2) <= epsilon;
}
function quatAproxEq(q1, q2) {
  return approxEq(q1.x, q2.x) && approxEq(q1.y, q2.y) && approxEq(q1.z, q2.z) && approxEq(q1.w, q2.w);
}
function poseApproxEq(p1, p2) {
  return approxEq(p1.position.x, p2.position.x) && approxEq(p1.position.y, p2.position.y) && approxEq(p1.position.z, p2.position.z) && quatAproxEq(p1.orientation, p2.orientation);
}
function getRotationNoScaling(out, mat) {
  const m11 = mat[0];
  const m12 = mat[1];
  const m13 = mat[2];
  const m21 = mat[4];
  const m22 = mat[5];
  const m23 = mat[6];
  const m31 = mat[8];
  const m32 = mat[9];
  const m33 = mat[10];
  const trace = m11 + m22 + m33;
  let S = 0;
  if (trace > 0) {
    S = Math.sqrt(trace + 1) * 2;
    out[3] = 0.25 * S;
    out[0] = (m23 - m32) / S;
    out[1] = (m31 - m13) / S;
    out[2] = (m12 - m21) / S;
  } else if (m11 > m22 && m11 > m33) {
    S = Math.sqrt(1 + m11 - m22 - m33) * 2;
    out[3] = (m23 - m32) / S;
    out[0] = 0.25 * S;
    out[1] = (m12 + m21) / S;
    out[2] = (m31 + m13) / S;
  } else if (m22 > m33) {
    S = Math.sqrt(1 + m22 - m11 - m33) * 2;
    out[3] = (m31 - m13) / S;
    out[0] = (m12 + m21) / S;
    out[1] = 0.25 * S;
    out[2] = (m23 + m32) / S;
  } else {
    S = Math.sqrt(1 + m33 - m11 - m22) * 2;
    out[3] = (m12 - m21) / S;
    out[0] = (m31 + m13) / S;
    out[1] = (m23 + m32) / S;
    out[2] = 0.25 * S;
  }
  return out;
}

// src/transforms/Transform.ts
var Transform = class _Transform {
  #position;
  #rotation;
  #matrix;
  constructor(matrixOrPosition, rotation) {
    if (matrixOrPosition == void 0) {
      this.#position = [0, 0, 0];
      this.#rotation = [0, 0, 0, 1];
      quat_exports.normalize(this.#rotation, this.#rotation);
      this.#matrix = mat4_exports.fromRotationTranslation(mat4Identity(), this.#rotation, this.#position);
    } else if (matrixOrPosition.length === 16) {
      this.#matrix = matrixOrPosition;
      this.#position = [0, 0, 0];
      this.#rotation = [0, 0, 0, 1];
      mat4_exports.getTranslation(this.#position, this.#matrix);
      getRotationNoScaling(this.#rotation, this.#matrix);
    } else if (matrixOrPosition.length === 3 && rotation != void 0) {
      this.#position = matrixOrPosition;
      this.#rotation = rotation;
      quat_exports.normalize(this.#rotation, this.#rotation);
      this.#matrix = mat4_exports.fromRotationTranslation(mat4Identity(), this.#rotation, this.#position);
    } else {
      throw new Error(`new Transform() expected either mat4 or vec3 + quat`);
    }
  }
  position() {
    return this.#position;
  }
  rotation() {
    return this.#rotation;
  }
  matrix() {
    return this.#matrix;
  }
  setPosition(position) {
    vec3_exports.copy(this.#position, position);
    mat4_exports.fromRotationTranslation(this.#matrix, this.#rotation, this.#position);
    return this;
  }
  setRotation(rotation) {
    quat_exports.normalize(this.#rotation, rotation);
    mat4_exports.fromRotationTranslation(this.#matrix, this.#rotation, this.#position);
    return this;
  }
  /**
   * Update position and rotation simultaneously. This is more efficient than
   * calling setPosition and setRotation separately, since we only need to
   * update the matrix once
   */
  setPositionRotation(position, rotation) {
    vec3_exports.copy(this.#position, position);
    quat_exports.normalize(this.#rotation, rotation);
    mat4_exports.fromRotationTranslation(this.#matrix, this.#rotation, this.#position);
    return this;
  }
  /**
   * Update position and rotation from a Pose object
   */
  setPose(pose) {
    vec3_exports.set(this.#position, pose.position.x, pose.position.y, pose.position.z);
    quat_exports.set(
      this.#rotation,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
    );
    quat_exports.normalize(this.#rotation, this.#rotation);
    mat4_exports.fromRotationTranslation(this.#matrix, this.#rotation, this.#position);
    return this;
  }
  /**
   * Update position and rotation from a matrix with unit scale
   */
  setMatrixUnscaled(matrix) {
    mat4_exports.copy(this.#matrix, matrix);
    mat4_exports.getTranslation(this.#position, matrix);
    getRotationNoScaling(this.#rotation, matrix);
    return this;
  }
  /**
   * Copy the values in another transform into this one
   */
  copy(other) {
    vec3_exports.copy(this.#position, other.#position);
    quat_exports.copy(this.#rotation, other.#rotation);
    mat4_exports.copy(this.#matrix, other.#matrix);
    return this;
  }
  toPose(out) {
    out.position.x = this.#position[0];
    out.position.y = this.#position[1];
    out.position.z = this.#position[2];
    out.orientation.x = this.#rotation[0];
    out.orientation.y = this.#rotation[1];
    out.orientation.z = this.#rotation[2];
    out.orientation.w = this.#rotation[3];
  }
  static Identity() {
    return new _Transform(vec3Identity(), quatIdentity());
  }
  static Empty() {
    return new _Transform();
  }
  /**
   * Interpolate between two rigid body transforms using linear interpolation on
   * the translation vector and spherical linear interpolation on the rotation
   * quaternion.
   *
   * @param out Output transform to store the result in
   * @param a Start transform
   * @param b End transform
   * @param t Interpolant in the range [0, 1]
   * @returns A reference to `out`
   */
  static Interpolate(out, a, b, t) {
    vec3_exports.lerp(out.#position, a.position(), b.position(), t);
    quat_exports.slerp(out.#rotation, a.rotation(), b.rotation(), t);
    out.setPositionRotation(out.#position, out.#rotation);
    return out;
  }
};

// src/transforms/time.ts
function compareTime(a, b) {
  return a < b ? -1 : a > b ? 1 : 0;
}
function percentOf(start, end, target) {
  const totalDuration = end - start;
  const targetDuration = target - start;
  return Number(targetDuration) / Number(totalDuration);
}
function interpolate(start, end, fraction) {
  const duration = Number(end - start);
  return start + BigInt(Math.round(duration * fraction));
}

// src/transforms/CoordinateFrame.ts
var MAX_DURATION = 4294967295n * BigInt(1e9);
var MAX_CAPACITY_EVICT_PORTION = 0.25;
var DEG2RAD = Math.PI / 180;
var tempLower = [0n, Transform.Identity()];
var tempUpper = [0n, Transform.Identity()];
var tempVec4 = [0, 0, 0, 0];
var temp2Vec4 = [0, 0, 0, 0];
var tempTransform = Transform.Identity();
var tempMatrix = mat4Identity();
var FALLBACK_FRAME_ID = Symbol("FALLBACK_FRAME_ID");
var CoordinateFrame = class _CoordinateFrame {
  static FALLBACK_FRAME_ID = FALLBACK_FRAME_ID;
  id;
  maxStorageTime;
  maxCapacity;
  offsetPosition;
  offsetEulerDegrees;
  #transformPool;
  #parent;
  #transforms;
  constructor(id, parent, maxStorageTime, maxCapacity, transformPool) {
    if (parent) {
      this.#parent = parent;
    }
    this.id = id;
    this.maxStorageTime = maxStorageTime;
    this.maxCapacity = maxCapacity;
    this.#transforms = new ArrayMap();
    this.#transformPool = transformPool;
  }
  static assertUserFrame(frame) {
    if (frame.id === FALLBACK_FRAME_ID) {
      throw new Error("Expected user frame");
    }
  }
  parent() {
    return this.#parent;
  }
  /**
   * Returns the top-most frame by walking up each parent frame. If the current
   * frame does not have a parent, the current frame is returned.
   */
  root() {
    if (this.id === FALLBACK_FRAME_ID) {
      return this;
    }
    _CoordinateFrame.assertUserFrame(this);
    let root = this;
    while (root.#parent) {
      root = root.#parent;
    }
    return root;
  }
  /**
   * Returns true if this frame has no parent frame.
   */
  isRoot() {
    return this.#parent == void 0;
  }
  /**
   * Returns the number of transforms stored in the transform history.
   */
  transformsSize() {
    return this.#transforms.size;
  }
  /**
   * Set the parent frame for this frame. If the parent frame is already set to
   * a different frame, the transform history is cleared.
   */
  setParent(parent) {
    if (this.#parent && this.#parent !== parent) {
      const removed = this.#transforms.clear();
      for (const [, tf] of removed) {
        this.#transformPool.release(tf);
      }
    }
    this.#parent = parent;
  }
  /**
   * Search for an ancestor frame with the given ID by walking up the chain of
   * parent frames, starting at the current frame.
   * @param id Frame ID to search for
   * @returns The ancestor frame, or undefined if not found
   */
  findAncestor(id) {
    let ancestor = this.#parent;
    while (ancestor) {
      if (ancestor.id === id) {
        return ancestor;
      }
      ancestor = ancestor.#parent;
    }
    return void 0;
  }
  /**
   * Add a transform to the transform history maintained by this frame. When the maximum capacity
   * has been reached, the history is trimmed by removing the larger portion of either
   * frames that are outside of the `maxStorageTime` or the last quarter of oldest frames.
   * This is to amortize the cost of trimming the history ever time a new transform is added.
   *
   * If a transform with an identical timestamp already exists, it is replaced.
   */
  addTransform(time, transform) {
    const oldTf = this.#transforms.set(time, transform);
    if (oldTf) {
      this.#transformPool.release(oldTf);
    }
    const transformsFull = this.#transforms.size >= this.maxCapacity;
    if (transformsFull) {
      const removeBeforeIndex = Math.floor(this.maxCapacity * MAX_CAPACITY_EVICT_PORTION);
      let removeBeforeTime = this.#transforms.at(removeBeforeIndex)[0];
      const endTime = this.#transforms.maxKey();
      const startTime = endTime - this.maxStorageTime;
      removeBeforeTime = startTime > removeBeforeTime ? startTime : removeBeforeTime;
      const entriesRemoved = this.#transforms.removeBefore(removeBeforeTime);
      for (const [, tf] of entriesRemoved) {
        this.#transformPool.release(tf);
      }
    }
  }
  /** Remove all transforms with timestamps greater than the given timestamp. */
  removeTransformsAfter(time) {
    const removed = this.#transforms.removeAfter(time);
    for (const [, tf] of removed) {
      this.#transformPool.release(tf);
    }
  }
  /** Removes a transform with a specific timestamp */
  removeTransformAt(time) {
    const tf = this.#transforms.remove(time);
    if (tf?.[1]) {
      this.#transformPool.release(tf[1]);
    }
  }
  /**
   * Find the closest transform(s) in the transform history to the given time.
   * Note that if an exact match is found, both `outLower` and `outUpper` will
   * be set to the same transform.
   * @param outLower This will be set to the found transform with the closest
   *   timestamp <= the given time
   * @param outUpper This will be set to the found transform with the closest
   *   timestamp >= the given time
   * @param time Time to search for
   * @param maxDelta The time parameter can exceed the bounds of the transform
   *   history by up to this amount and still clamp to the oldest or newest
   *   transform
   * @returns True if the search was successful
   */
  findClosestTransforms(outLower, outUpper, time, maxDelta) {
    const transformCount = this.#transforms.size;
    if (transformCount === 0) {
      return false;
    } else if (transformCount === 1) {
      const [latestTime, latestTf] = this.#transforms.maxEntry();
      if (time <= latestTime + maxDelta) {
        outLower[0] = outUpper[0] = latestTime;
        outLower[1] = outUpper[1] = latestTf;
        return true;
      }
      return false;
    }
    const index = this.#transforms.binarySearch(time);
    if (index >= 0) {
      const [, tf] = this.#transforms.at(index);
      outLower[0] = outUpper[0] = time;
      outLower[1] = outUpper[1] = tf;
      return true;
    }
    const greaterThanIndex = ~index;
    if (greaterThanIndex >= this.#transforms.size) {
      const [latestTime, latestTf] = this.#transforms.maxEntry();
      if (time <= latestTime + maxDelta) {
        outLower[0] = outUpper[0] = latestTime;
        outLower[1] = outUpper[1] = latestTf;
        return true;
      }
      return false;
    }
    const lessThanIndex = greaterThanIndex - 1;
    if (lessThanIndex < 0) {
      const [earliestTime, earliestTf] = this.#transforms.minEntry();
      if (earliestTime + maxDelta >= time) {
        outLower[0] = outUpper[0] = earliestTime;
        outLower[1] = outUpper[1] = earliestTf;
        return true;
      }
      return false;
    }
    const [lteTime, lteTf] = this.#transforms.at(lessThanIndex);
    const [gtTime, gtTf] = this.#transforms.at(greaterThanIndex);
    outLower[0] = lteTime;
    outLower[1] = lteTf;
    outUpper[0] = gtTime;
    outUpper[1] = gtTf;
    return true;
  }
  /**
   * Transform a pose from the coordinate frame `srcFrame` to this coordinate
   * frame at the given time. The transform will be done using the shortest path
   * from `srcFrame` to this frame
   *
   * Transforms can go up through multiple parents, down through one or more
   * children, or both as long as the transforms share a common ancestor.
   *
   * A common variable naming convention for the returned pose is
   * `thisFrame_T_srcFrame` which is read right-to-left as "the translation that
   * moves a point from `srcFrame` to `thisFrame`".
   * @param out Output pose, this will be modified with the result on success
   * @param input Input pose that exists in `srcFrame`
   * @param srcFrame Coordinate frame we are transforming from
   * @param time Time to compute the transform at
   * @param maxDelta The time parameter can exceed the bounds of the transform
   *   history by up to this amount and still clamp to the oldest or newest
   *   transform
   * @returns A reference to `out` on success, otherwise undefined
   */
  applyLocal(out, input, srcFrame, time, maxDelta = MAX_DURATION) {
    if (this.id === FALLBACK_FRAME_ID || srcFrame.id === FALLBACK_FRAME_ID) {
      return out;
    }
    _CoordinateFrame.assertUserFrame(this);
    _CoordinateFrame.assertUserFrame(srcFrame);
    if (srcFrame === this) {
      copyPose(out, input);
      return out;
    } else if (srcFrame.findAncestor(this.id)) {
      return _CoordinateFrame.Apply(out, input, this, srcFrame, false, time, maxDelta) ? out : void 0;
    } else if (this.findAncestor(srcFrame.id)) {
      return _CoordinateFrame.Apply(out, input, srcFrame, this, true, time, maxDelta) ? out : void 0;
    }
    let curSrcFrame = srcFrame;
    while (curSrcFrame) {
      const commonAncestor = this.findAncestor(curSrcFrame.id);
      if (commonAncestor) {
        if (!_CoordinateFrame.Apply(out, input, commonAncestor, srcFrame, false, time, maxDelta)) {
          return void 0;
        }
        return _CoordinateFrame.Apply(out, out, commonAncestor, this, true, time, maxDelta) ? out : void 0;
      }
      curSrcFrame = curSrcFrame.#parent;
    }
    return void 0;
  }
  /**
   * Transform a pose from the coordinate frame `srcFrame` to rootFrame at
   * `srcTime`, then from `rootFrame` to this coordinate frame at `dstTime`. The
   * transform will be done using the shortest path from `srcFrame` to the root
   * frame, then from the root frame to this frame.
   *
   * Transforms can go up through multiple parents, down through one or more
   * children, or both as long as the transforms share a common ancestor.
   * @param out Output pose, this will be modified with the result on success
   * @param input Input pose that exists in `srcFrame`
   * @param rootFrame Reference coordinate frame to transform from srcFrame into as srcTime
   * @param srcFrame Coordinate frame we are transforming from
   * @param dstTime Time to transform from rootFrome into this frame
   * @param srcTime Time to transform from srcFrame into rootFrame
   * @param maxDelta The time parameter can exceed the bounds of the transform
   *   history by up to this amount and still clamp to the oldest or newest
   *   transform
   * @returns A reference to `out` on success, otherwise undefined
   */
  apply(out, input, rootFrame, srcFrame, dstTime, srcTime, maxDelta = MAX_DURATION) {
    if (rootFrame.applyLocal(out, input, srcFrame, srcTime, maxDelta) == void 0) {
      return void 0;
    }
    return this.applyLocal(out, out, rootFrame, dstTime, maxDelta);
  }
  /**
   * Returns a display-friendly rendition of `id`, quoting the frame id if it is
   * an empty string or starts or ends with whitespace.
   */
  displayName() {
    return _CoordinateFrame.DisplayName(this.id);
  }
  /**
   * Interpolate between two [time, transform] pairs.
   * @param output Output parameter for the interpolated time and transform
   * @param lower Start [time, transform]
   * @param upper End [time, transform]
   * @param time Interpolant in the range [lower[0], upper[0]]
   */
  static Interpolate(output, lower, upper, time) {
    const [lowerTime, lowerTf] = lower;
    const [upperTime, upperTf] = upper;
    if (lowerTime === upperTime) {
      output[0] = upperTime;
      output[1].copy(upperTf);
      return;
    }
    const fraction = Math.max(0, Math.min(1, percentOf(lowerTime, upperTime, time)));
    output[0] = interpolate(lowerTime, upperTime, fraction);
    Transform.Interpolate(output[1], lowerTf, upperTf, fraction);
  }
  /**
   * Interpolate the transform between two [time, transform] pairs.
   * @param output Output parameter for the interpolated transform
   * @param lower Start [time, transform]
   * @param upper End [time, transform]
   * @param time Interpolant in the range [lower[0], upper[0]]
   */
  static InterpolateTransform(output, lower, upper, time) {
    const [lowerTime, lowerTf] = lower;
    const [upperTime, upperTf] = upper;
    if (lowerTime === upperTime) {
      output.copy(upperTf);
      return;
    }
    const fraction = Math.max(0, Math.min(1, percentOf(lowerTime, upperTime, time)));
    Transform.Interpolate(output, lowerTf, upperTf, fraction);
  }
  /**
   * Get the transform `parentFrame_T_childFrame` (from child to parent) at the
   * given time.
   * @param out Output transform matrix
   * @param parentFrame Parent destination frame
   * @param childFrame Child source frame
   * @param time Time to transform at
   * @param maxDelta The time parameter can exceed the bounds of the transform
   *   history by up to this amount and still clamp to the oldest or newest
   *   transform
   * @returns True on success
   */
  static GetTransformMatrix(out, parentFrame, childFrame, time, maxDelta) {
    mat4_exports.identity(out);
    let curFrame = childFrame;
    while (curFrame !== parentFrame) {
      if (!curFrame.findClosestTransforms(tempLower, tempUpper, time, maxDelta)) {
        return false;
      }
      _CoordinateFrame.InterpolateTransform(tempTransform, tempLower, tempUpper, time);
      if (curFrame.offsetEulerDegrees) {
        const quaternion = tempTransform.rotation();
        const multByRotation = quaternionFromEuler(tempVec4, curFrame.offsetEulerDegrees);
        quat_exports.multiply(temp2Vec4, quaternion, multByRotation);
        tempTransform.setRotation(temp2Vec4);
      }
      if (curFrame.offsetPosition) {
        const p = tempTransform.position();
        vec3_exports.add(p, p, curFrame.offsetPosition);
        tempTransform.setPosition(p);
      }
      mat4_exports.multiply(out, tempTransform.matrix(), out);
      if (curFrame.#parent == void 0) {
        throw new Error(
          `Frame "${parentFrame.displayName()}" is not a parent of "${childFrame.displayName()}"`
        );
      }
      curFrame = curFrame.#parent;
    }
    return true;
  }
  /**
   * Apply the transform from `child` to `parent` at the given time to the given
   * input pose. The transform can optionally be inverted, to go from `parent`
   * to `child`.
   * @param out Output pose, this will be modified with the result on success
   * @param input Input pose that exists in `child`, or `parent` if `invert` is
   *   true
   * @param parent Parent frame
   * @param child Child frame
   * @invert Whether to invert the transform (go from parent to child)
   * @param time Time to compute the transform at
   * @param maxDelta The time parameter can exceed the bounds of the transform
   *   history by up to this amount and still clamp to the oldest or newest
   *   transform
   * @returns True on success
   */
  static Apply(out, input, parent, child, invert3, time, maxDelta) {
    if (!_CoordinateFrame.GetTransformMatrix(tempMatrix, parent, child, time, maxDelta)) {
      return false;
    }
    if (invert3) {
      mat4_exports.invert(tempMatrix, tempMatrix);
    }
    mat4_exports.multiply(tempMatrix, tempMatrix, tempTransform.setPose(input).matrix());
    tempTransform.setMatrixUnscaled(tempMatrix).toPose(out);
    return true;
  }
  /**
   * Returns a display-friendly rendition of `frameId`, quoting the id if it is
   * an empty string or starts or ends with whitespace.
   */
  static DisplayName(frameId) {
    if (frameId === FALLBACK_FRAME_ID) {
      return "(none)";
    }
    return frameId === "" || frameId.startsWith(" ") || frameId.endsWith(" ") ? `"${frameId}"` : frameId;
  }
};
function copyPose(out, pose) {
  const p = pose.position;
  const o = pose.orientation;
  out.position.x = p.x;
  out.position.y = p.y;
  out.position.z = p.z;
  out.orientation.x = o.x;
  out.orientation.y = o.y;
  out.orientation.z = o.z;
  out.orientation.w = o.w;
}
function quaternionFromEuler(out, euler) {
  const x = euler[0] * DEG2RAD;
  const y = euler[1] * DEG2RAD;
  const z = euler[2] * DEG2RAD;
  const c1 = Math.cos(x / 2);
  const c2 = Math.cos(y / 2);
  const c3 = Math.cos(z / 2);
  const s1 = Math.sin(x / 2);
  const s2 = Math.sin(y / 2);
  const s3 = Math.sin(z / 2);
  out[0] = s1 * c2 * c3 + c1 * s2 * s3;
  out[1] = c1 * s2 * c3 - s1 * c2 * s3;
  out[2] = c1 * c2 * s3 + s1 * s2 * c3;
  out[3] = c1 * c2 * c3 - s1 * s2 * s3;
  return out;
}

// src/transforms/TransformTree.ts
var DEFAULT_MAX_CAPACITY_PER_FRAME = 1e4;
var AddTransformResult = /* @__PURE__ */ ((AddTransformResult2) => {
  AddTransformResult2[AddTransformResult2["NOT_UPDATED"] = 0] = "NOT_UPDATED";
  AddTransformResult2[AddTransformResult2["UPDATED"] = 1] = "UPDATED";
  AddTransformResult2[AddTransformResult2["CYCLE_DETECTED"] = 2] = "CYCLE_DETECTED";
  return AddTransformResult2;
})(AddTransformResult || {});
var DEFAULT_FRAME_IDS = ["base_link", "odom", "map", "earth"];
var TransformTree = class _TransformTree {
  #frames = /* @__PURE__ */ new Map();
  #maxStorageTime;
  #maxCapacityPerFrame;
  #transformPool;
  defaultRootFrame;
  constructor(transformPool, maxStorageTime = MAX_DURATION, maxCapacityPerFrame = DEFAULT_MAX_CAPACITY_PER_FRAME) {
    this.#transformPool = transformPool;
    this.#maxStorageTime = maxStorageTime;
    this.#maxCapacityPerFrame = maxCapacityPerFrame;
    this.defaultRootFrame = new CoordinateFrame(
      CoordinateFrame.FALLBACK_FRAME_ID,
      void 0,
      this.#maxStorageTime,
      this.#maxCapacityPerFrame,
      this.#transformPool
    );
    this.defaultRootFrame.addTransform(0n, Transform.Identity());
  }
  addTransform(frameId, parentFrameId, time, transform) {
    let updated = !this.hasFrame(frameId);
    let cycleDetected = false;
    const frame = this.getOrCreateFrame(frameId);
    const curParentFrame = frame.parent();
    if (curParentFrame?.id !== parentFrameId) {
      cycleDetected = this.#checkParentForCycle(frameId, parentFrameId);
      if (!cycleDetected) {
        frame.setParent(this.getOrCreateFrame(parentFrameId));
        updated = true;
      }
    }
    if (!cycleDetected) {
      frame.addTransform(time, transform);
    }
    return cycleDetected ? 2 /* CYCLE_DETECTED */ : updated ? 1 /* UPDATED */ : 0 /* NOT_UPDATED */;
  }
  /**
   * Removes transform data from a particular parent-child link at the given timestamp. Does nothing
   * if the child does not exist or has a different parent.
   */
  removeTransform(childFrameId, parentFrameId, stamp) {
    const child = this.frame(childFrameId);
    if (!child) {
      return;
    }
    if (child.parent()?.id !== parentFrameId) {
      return;
    }
    child.removeTransformAt(stamp);
    this.#removeEmptyAncestors(child);
  }
  /**
   * Walk up the tree starting from `candidate` and prune frames with no history entries and no
   * children.
   */
  #removeEmptyAncestors(candidate) {
    if (candidate.transformsSize() > 0) {
      return;
    }
    const childrenByParentId = /* @__PURE__ */ new Map();
    for (const frame of this.#frames.values()) {
      childrenByParentId.set(frame.id, /* @__PURE__ */ new Set());
    }
    for (const frame of this.#frames.values()) {
      const parent = frame.parent();
      if (parent === candidate) {
        return;
      }
      if (parent == void 0) {
        continue;
      }
      const children = childrenByParentId.get(parent.id);
      if (!children) {
        throw new Error("invariant: should have children array");
      }
      children.add(frame.id);
    }
    for (let current = candidate; current; current = current.parent()) {
      if (current.transformsSize() > 0) {
        return;
      }
      const children = childrenByParentId.get(current.id);
      if (children && children.size > 0) {
        return;
      }
      this.#frames.delete(current.id);
      childrenByParentId.delete(current.id);
      const parentId = current.parent()?.id;
      if (parentId) {
        childrenByParentId.get(parentId)?.delete(current.id);
      }
    }
  }
  clear() {
    this.#frames.clear();
  }
  clearAfter(time) {
    for (const frame of this.#frames.values()) {
      frame.removeTransformsAfter(time);
    }
  }
  hasFrame(id) {
    if (id === CoordinateFrame.FALLBACK_FRAME_ID) {
      return true;
    }
    return this.#frames.has(id);
  }
  frame(id) {
    if (id === CoordinateFrame.FALLBACK_FRAME_ID) {
      return this.defaultRootFrame;
    }
    return this.#frames.get(id);
  }
  getOrCreateFrame(id) {
    let frame = this.#frames.get(id);
    if (!frame) {
      frame = new CoordinateFrame(
        id,
        void 0,
        this.#maxStorageTime,
        this.#maxCapacityPerFrame,
        this.#transformPool
      );
      this.#frames.set(id, frame);
    }
    return frame;
  }
  frames() {
    return this.#frames;
  }
  apply(output, input, frameId, rootFrameId, srcFrameId, dstTime, srcTime, maxDelta) {
    const frame = this.frame(frameId);
    const srcFrame = this.frame(srcFrameId);
    if (!frame || !srcFrame) {
      return void 0;
    }
    const rootFrame = (rootFrameId != void 0 ? this.frame(rootFrameId) : frame.root()) ?? frame.root();
    return frame.apply(output, input, rootFrame, srcFrame, dstTime, srcTime, maxDelta);
  }
  frameList() {
    const frames = Array.from(this.#frames.values());
    const frameMap = new Map(
      frames.map((frame) => [frame.id, { id: frame.id, children: [] }])
    );
    const rootFrames = [];
    for (const frame of frames) {
      const frameEntry = frameMap.get(frame.id);
      const parentId = frame.parent()?.id;
      if (parentId == void 0) {
        rootFrames.push(frameEntry);
      } else {
        const parent = frameMap.get(parentId);
        if (parent == void 0) {
          continue;
        }
        parent.children.push(frameEntry);
      }
    }
    const output = [];
    function addFrame(frame, depth) {
      const displayName = CoordinateFrame.DisplayName(frame.id);
      output.push({
        value: frame.id,
        label: `${"\xA0\xA0".repeat(depth)}${displayName}`
      });
      frame.children.sort((a, b) => a.id.localeCompare(b.id));
      for (const child of frame.children) {
        addFrame(child, depth + 1);
      }
    }
    rootFrames.sort((a, b) => a.id.localeCompare(b.id));
    for (const entry of rootFrames) {
      addFrame(entry, 0);
    }
    return output;
  }
  /** Get heuristically most valid follow frame Id */
  getDefaultFollowFrameId() {
    const allFrames = this.frames();
    if (allFrames.size === 0) {
      return void 0;
    }
    for (const frameId of DEFAULT_FRAME_IDS) {
      const frame = this.frame(frameId);
      if (frame) {
        return frame.id;
      }
    }
    const rootsToCounts = /* @__PURE__ */ new Map();
    for (const frame of allFrames.values()) {
      const root = frame.root();
      const rootId2 = root.id;
      rootsToCounts.set(rootId2, (rootsToCounts.get(rootId2) ?? 0) + 1);
    }
    const rootsArray = Array.from(rootsToCounts.entries());
    const rootId = rootsArray.sort((a, b) => b[1] - a[1])[0]?.[0];
    return rootId;
  }
  #checkParentForCycle(frameId, parentFrameId) {
    if (frameId === parentFrameId) {
      return true;
    }
    let frame = this.frame(parentFrameId);
    while (frame?.parent()) {
      if (frame.parent()?.id === frameId) {
        return true;
      }
      frame = frame.parent();
    }
    return false;
  }
  static Clone(tree) {
    const newTree = new _TransformTree(
      tree.#transformPool,
      tree.#maxStorageTime,
      tree.#maxCapacityPerFrame
    );
    newTree.#frames = tree.#frames;
    return newTree;
  }
};
export {
  AddTransformResult,
  ArrayMap,
  CoordinateFrame,
  DEFAULT_MAX_CAPACITY_PER_FRAME,
  MAX_CAPACITY_EVICT_PORTION,
  MAX_DURATION,
  ObjectPool,
  Transform,
  TransformTree,
  approxEq,
  compareTime,
  getRotationNoScaling,
  interpolate,
  makePose,
  mat4Clone,
  mat4FromValues,
  mat4Identity,
  percentOf,
  poseApproxEq,
  quatAproxEq,
  quatClone,
  quatFromValues,
  quatIdentity,
  vec3Clone,
  vec3FromValues,
  vec3Identity,
  xyzrpyToPose
};
