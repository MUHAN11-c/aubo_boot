// pointcloud_viewer.js — 点云 Three.js 渲染 (v3: RGB颜色 + TF支持)
// 不自行解析 CDR，由 FoxgloveAdapter 的 MessageReader 解码
//
// 用法:
//   var viewer = createPointCloudViewer({ scene: viewer3d.scene, maxPoints: 300000 });
//   viewer.updatePoints(decodedPC2);  // decodedPC2 = MessageReader 解码后的 PointCloud2 对象
//   viewer.points.position.copy(...)  // 应用 TF 变换
//   viewer.dispose();

var g = globalThis;
var TAG = '[pc-viewer]';

// 使用 THREE.PointsMaterial + vertexColors (利用 THREE.js 内置顶点颜色管线, 避坑自定义 ShaderMaterial 属性绑定)

export function createPointCloudViewer(options) {
  var THREE = g.THREE;
  if (!THREE) throw new Error('THREE 未加载');

  var opts = Object.assign({
    maxPoints: 300000,
    pointSize: 0.003,
    visible: true,
  }, options);

  if (!opts.scene) throw new Error('options.scene 是必填项');

  var scene = opts.scene;
  var maxPoints = opts.maxPoints;

  // ── 几何体: position + color ──
  var positionArr = new Float32Array(maxPoints * 3);
  var colorArr = new Float32Array(maxPoints * 3);

  var geometry = new THREE.BufferGeometry();
  var posAttr = new THREE.BufferAttribute(positionArr, 3);
  var colAttr = new THREE.BufferAttribute(colorArr, 3);

  if (typeof geometry.setAttribute === 'function') {
    geometry.setAttribute('position', posAttr);
    geometry.setAttribute('color', colAttr);
  } else {
    geometry.addAttribute('position', posAttr);
    geometry.addAttribute('pcVertexColor', colAttr);
  }
  geometry.setDrawRange(0, 0);

  var material = new THREE.ShaderMaterial({
    attributes: { pcVertexColor: { type: 'c', value: null } },
    uniforms: { uPointSize: { value: opts.pointSize } },
    vertexShader: [
      'attribute vec3 pcVertexColor;',
      'varying vec3 vColor;',
      'uniform float uPointSize;',
      'void main() {',
      '  vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);',
      '  gl_PointSize = uPointSize * (300.0 / -mvPosition.z);',
      '  gl_Position = projectionMatrix * mvPosition;',
      '  vColor = pcVertexColor;',
      '}',
    ].join('\n'),
    fragmentShader: [
      'varying vec3 vColor;',
      'void main() {',
      '  float d = length(gl_PointCoord - vec2(0.5));',
      '  if (d > 0.5) discard;',
      '  float alpha = 1.0 - smoothstep(0.0, 0.5, d);',
      '  gl_FragColor = vec4(vColor, alpha * 0.85);',
      '}',
    ].join('\n'),
    transparent: true,
    depthWrite: false,
    blending: THREE.NormalBlending,
  });

  var points = new THREE.Points(geometry, material);
  points.visible = opts.visible;
  points.frustumCulled = false;
  points.name = 'pointcloud';
  scene.add(points);
  console.log(TAG, 'ready, maxPoints:', maxPoints);

  var pointCount = 0;
  var _frameCount = 0;
  var _lastLogTime = 0;
  var _hasRgb = false;

  function updatePoints(pc2) {
    if (!pc2 || !pc2.data) return;
    _frameCount++;

    var fields = pc2.fields || [];
    var pointStep = pc2.point_step || 0;
    var total = (pc2.height || 1) * (pc2.width || 0);
    var n = Math.min(total, maxPoints);
    var rawData = pc2.data;

    // 扫描字段
    var xOff = -1, yOff = -1, zOff = -1, iOff = -1;
    var hasIntensity = false;
    for (var fi = 0; fi < fields.length; fi++) {
      var f = fields[fi];
      if (f.name === 'x') xOff = f.offset;
      else if (f.name === 'y') yOff = f.offset;
      else if (f.name === 'z') zOff = f.offset;
      else if (f.name === 'rgb') { iOff = f.offset; _hasRgb = true; }
      else if (f.name === 'intensity') { iOff = f.offset; hasIntensity = true; }
    }

    if (xOff < 0 || yOff < 0 || zOff < 0) {
      if (_frameCount === 1) console.warn(TAG, 'no xyz fields');
      return;
    }

    if (_frameCount === 1) {
      console.log(TAG, '首帧 fields:', fields.map(function(f) { return f.name; }),
        'rgb:', _hasRgb, 'intensity:', hasIntensity, 'point_step:', pointStep);
      console.log(TAG, 'rawData typeof:', typeof rawData, 'isUint8:', rawData instanceof Uint8Array,
        'len:', rawData.length, 'buf:', !!rawData.buffer, 'bo:', rawData.byteOffset);
      // 检查原始数据前 40 字节
      var sample = [];
      for (var si = 0; si < Math.min(40, rawData.length); si++) { sample.push(rawData[si]); }
      console.log(TAG, 'rawData 前40字节:', sample);
    }

    // 从 Uint8Array 安全读取 float32 (避免 DataView byteOffset 嵌套问题)
    function readFloatLE(arr, offset) {
      var b0 = arr[offset], b1 = arr[offset+1], b2 = arr[offset+2], b3 = arr[offset+3];
      var sign = (b3 & 0x80) ? -1 : 1;
      var exp = ((b3 & 0x7F) << 1) | ((b2 & 0x80) >> 7);
      var mant = ((b2 & 0x7F) << 16) | (b1 << 8) | b0;
      if (exp === 0) return sign * Math.pow(2, -126) * (mant / 0x800000);
      if (exp === 255) return mant ? NaN : sign * Infinity;
      return sign * Math.pow(2, exp - 127) * (1 + mant / 0x800000);
    }

    for (var i = 0; i < n; i++) {
      var base = i * pointStep;
      positionArr[i * 3]     = readFloatLE(rawData, base + xOff);
      positionArr[i * 3 + 1] = readFloatLE(rawData, base + yOff);
      positionArr[i * 3 + 2] = readFloatLE(rawData, base + zOff);

      if (iOff >= 0) {
        if (_hasRgb) {
          // RGB packed float → 手动读取4字节并重解释为 uint32
          var b0 = rawData[base + iOff], b1 = rawData[base + iOff + 1],
              b2 = rawData[base + iOff + 2], b3 = rawData[base + iOff + 3];
          var packed = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0;
          colorArr[i * 3]     = (packed & 0xFF) / 255;
          colorArr[i * 3 + 1] = ((packed >> 8) & 0xFF) / 255;
          colorArr[i * 3 + 2] = ((packed >> 16) & 0xFF) / 255;
        } else {
          var v = readFloatLE(rawData, base + iOff);
          colorArr[i * 3] = v;
          colorArr[i * 3 + 1] = v;
          colorArr[i * 3 + 2] = v;
        }
      } else {
        // 无颜色字段 → Z 高度着色 (在 JS 侧预计算)
        var z = positionArr[i * 3 + 2];
        var t = (z + 0.5) / 1.5;  // [-0.5, 1.0] → [0,1]
        if (t < 0) t = 0; if (t > 1) t = 1;
        colorArr[i * 3]     = 0.2 + 0.3 * t;
        colorArr[i * 3 + 1] = 0.5 + 0.5 * (1 - t);
        colorArr[i * 3 + 2] = 1.0 - 0.5 * t;
      }
    }

    pointCount = n;
    geometry.attributes.position.needsUpdate = true;
    geometry.attributes.pcVertexColor.needsUpdate = true;
    geometry.setDrawRange(0, n);

    if (_frameCount === 1) {
      console.log(TAG, '首帧:', n, 'points');
      console.log(TAG, '首帧前3点 xyz:', positionArr.slice(0, 9));
      console.log(TAG, '首帧前3点 rgb:', colorArr.slice(0, 9));
    } else if (Date.now() - _lastLogTime > 10000) {
      _lastLogTime = Date.now();
      console.log(TAG, _frameCount + ' 帧, ' + n + ' points');
    }
  }

  function setVisible(v) { points.visible = !!v; }
  function setPointSize(s) { material.uniforms.uPointSize.value = s; }

  function dispose() {
    scene.remove(points);
    geometry.dispose();
    material.dispose();
    console.log(TAG, 'disposed');
  }

  var api = {
    updatePoints: updatePoints,
    setVisible: setVisible,
    setPointSize: setPointSize,
    dispose: dispose,
    get pointCount() { return pointCount; },
    get points() { return points; },
  };

  g.__pcViewer = api;
  return api;
}
