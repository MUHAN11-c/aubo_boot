/**
 * RWT 仪表板脚本（demo.html / dev.html 共用）。
 *
 * - dev：可编辑话题、显示 WebSocket URL、话题列表；需用户点击连接。
 * - demo：隐藏话题输入，用 /api/config 预设；加载配置后自动 connectRos。
 *
 * 依赖：页面先加载 roslib（全局 ROSLIB）；Three 与 OrbitControls 自 jsDelivr ESM。
 */
import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js";
import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js";

const ROSLIB = globalThis.ROSLIB;

/** @type {"demo"|"dev"} */
const MODE = document.documentElement.dataset.rwtMode === "demo" ? "demo" : "dev";
const IS_DEMO = MODE === "demo";

/** 话题输入防抖，减少 rosbridge 订阅抖动。 */
function debounce(fn, ms) {
  let t;
  return (...args) => {
    clearTimeout(t);
    t = setTimeout(() => fn.apply(null, args), ms);
  };
}

/**
 * 拉取 JSON；超时用 AbortController；非 2xx 时解析网关统一 error 结构。
 * @param {string} url
 * @param {number} [timeoutMs]
 */
async function fetchJsonWithTimeout(url, timeoutMs = 12000) {
  const ctrl = new AbortController();
  const to = setTimeout(() => ctrl.abort(), timeoutMs);
  try {
    const r = await fetch(url, {
      signal: ctrl.signal,
      headers: { Accept: "application/json" },
    });
    const rid = r.headers.get("X-Request-ID");
    if (!r.ok) {
      let msg = r.statusText || `HTTP ${r.status}`;
      try {
        const j = await r.json();
        if (j?.error?.message) msg = j.error.message;
      } catch {
        /* ignore */
      }
      const err = new Error(rid ? `${msg}（请求 ID: ${rid}）` : msg);
      err.requestId = rid;
      throw err;
    }
    return await r.json();
  } finally {
    clearTimeout(to);
  }
}

function b64ToU8(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

/** 将 sensor_msgs/Image（rgb8/bgr8/rgba8）画到 2D canvas。 */
function drawImageMsg(msg, canvas) {
  if (!canvas || !msg?.data) return;
  const w = msg.width;
  const h = msg.height;
  const enc = (msg.encoding || "").toLowerCase();
  const raw = typeof msg.data === "string" ? b64ToU8(msg.data) : new Uint8Array(msg.data);
  canvas.width = w;
  canvas.height = h;
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(w, h);
  const step = msg.step || Math.floor(raw.length / h);
  if (enc === "rgb8" || enc === "rgb888") {
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = y * step + x * 3;
        const j = (y * w + x) * 4;
        img.data[j] = raw[i];
        img.data[j + 1] = raw[i + 1];
        img.data[j + 2] = raw[i + 2];
        img.data[j + 3] = 255;
      }
    }
  } else if (enc === "bgr8" || enc === "bgr888") {
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = y * step + x * 3;
        const j = (y * w + x) * 4;
        img.data[j] = raw[i + 2];
        img.data[j + 1] = raw[i + 1];
        img.data[j + 2] = raw[i];
        img.data[j + 3] = 255;
      }
    }
  } else if (enc === "rgba8") {
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = y * step + x * 4;
        const j = (y * w + x) * 4;
        img.data[j] = raw[i];
        img.data[j + 1] = raw[i + 1];
        img.data[j + 2] = raw[i + 2];
        img.data[j + 3] = raw[i + 3];
      }
    }
  } else {
    ctx.fillStyle = "#333";
    ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = "#fff";
    ctx.font = "14px sans-serif";
    ctx.fillText(`不支持的 encoding: ${enc}`, 8, 24);
    return;
  }
  ctx.putImageData(img, 0, 0);
}

function findField(msg, name) {
  return (msg.fields || []).find((f) => f.name === name);
}

/** 从 PointCloud2 解析 x/y/z 到 Float32Array（最多 maxPoints 点）。 */
function parsePointCloud2XYZ(msg, maxPoints) {
  const raw = typeof msg.data === "string" ? b64ToU8(msg.data) : new Uint8Array(msg.data);
  const fx = findField(msg, "x");
  const fy = findField(msg, "y");
  const fz = findField(msg, "z");
  if (!fx || !fy || !fz) return new Float32Array(0);
  const pointStep = msg.point_step || 0;
  const width = msg.width || 0;
  const height = msg.height || 0;
  const n =
    width && height
      ? width * height
      : Math.floor(raw.length / (pointStep || 1));
  const cap = Math.min(n, maxPoints);
  const out = new Float32Array(cap * 3);
  const dv = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
  let o = 0;
  for (let i = 0; i < cap && o < cap * 3; i++) {
    const base = i * pointStep;
    if (base + fz.offset + 4 > raw.byteLength) break;
    out[o++] = dv.getFloat32(base + fx.offset, true);
    out[o++] = dv.getFloat32(base + fy.offset, true);
    out[o++] = dv.getFloat32(base + fz.offset, true);
  }
  return out.subarray(0, o);
}

function main() {
  // —— DOM 与 rosbridge / Three 状态均在闭包内 ——
  const elBadge = document.getElementById("status-badge");
  const elWsUrl = document.getElementById("ws-url");
  const elTopicColor = document.getElementById("topic-color");
  const elTopicPc = document.getElementById("topic-pc");
  const elCanvas = document.getElementById("img-canvas");
  const elThreeRoot = document.getElementById("three-root");
  const elTopicsPre = document.getElementById("topics-pre");
  const btnConnect = document.getElementById("btn-connect");
  const btnDisconnect = document.getElementById("btn-disconnect");
  const btnTopics = document.getElementById("btn-topics");
  const elAlert = document.getElementById("app-alert");
  const elAlertText = document.getElementById("app-alert-text");
  const btnAlertDismiss = document.getElementById("app-alert-dismiss");

  function showAppAlert(msg) {
    if (!elAlert || !elAlertText) return;
    elAlertText.textContent = msg;
    elAlert.hidden = false;
  }

  function hideAppAlert() {
    if (!elAlert || !elAlertText) return;
    elAlert.hidden = true;
    elAlertText.textContent = "";
  }

  btnAlertDismiss?.addEventListener("click", hideAppAlert);

  let rosbridgePort = 9090;
  let publicHost = "";
  let ros = null;
  let imageTopic = null;
  let pcListener = null;
  let threeCleanup = null;
  let threeState = null;
  let connected = false;
  let errorText = "";

  function wsUrl() {
    const host = publicHost || window.location.hostname;
    const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
    return `${proto}//${host}:${rosbridgePort}`;
  }

  function updateBadge() {
    elBadge.classList.remove("ok", "err");
    if (connected) {
      elBadge.textContent = "已连接 rosbridge";
      elBadge.classList.add("ok");
    } else if (errorText) {
      elBadge.textContent = errorText;
      elBadge.classList.add("err");
    } else {
      elBadge.textContent = "未连接";
    }
  }

  function updateWsField() {
    if (elWsUrl) elWsUrl.value = wsUrl();
  }

  function updateButtons() {
    btnConnect.disabled = connected;
    if (btnTopics) btnTopics.disabled = !connected;
  }

  function teardownThree() {
    if (threeCleanup) {
      threeCleanup();
      threeCleanup = null;
      threeState = null;
    }
  }

  /** 在容器中创建 Three 场景、OrbitControls、点云 BufferGeometry，并挂 ResizeObserver / 横竖屏。 */
  function mountThree(el) {
    teardownThree();
    if (!el || el.clientWidth === 0) return;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);
    const camera = new THREE.PerspectiveCamera(50, el.clientWidth / el.clientHeight, 0.01, 100);
    camera.position.set(1.2, 0.8, 1.2);
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(el.clientWidth, el.clientHeight);
    el.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    if (THREE.TOUCH) {
      controls.touches = {
        ONE: THREE.TOUCH.ROTATE,
        TWO: THREE.TOUCH.DOLLY_PAN,
      };
    }
    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const dir = new THREE.DirectionalLight(0xffffff, 0.5);
    dir.position.set(2, 3, 2);
    scene.add(dir);
    scene.add(new THREE.GridHelper(2, 20, 0x444444, 0x222222));
    scene.add(new THREE.AxesHelper(0.5));

    // 手机略减点；平板触控（pointer: coarse）略减以保 Orbit 流畅
    const maxPts =
      window.innerWidth < 768
        ? 12000
        : window.matchMedia("(pointer: coarse)").matches
          ? 28000
          : 40000;
    const geom = new THREE.BufferGeometry();
    const pos = new Float32Array(maxPts * 3);
    geom.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    geom.setDrawRange(0, 0);
    const mat = new THREE.PointsMaterial({
      size: 0.01,
      color: 0x3d8bfd,
      sizeAttenuation: true,
    });
    scene.add(new THREE.Points(geom, mat));

    function applyCanvasSize() {
      if (!el.clientWidth || !el.clientHeight) return;
      camera.aspect = el.clientWidth / el.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(el.clientWidth, el.clientHeight);
    }

    const ro = new ResizeObserver(() => applyCanvasSize());
    ro.observe(el);

    let orientTimer = 0;
    const onOrientationChange = () => {
      window.clearTimeout(orientTimer);
      orientTimer = window.setTimeout(applyCanvasSize, 320);
    };
    window.addEventListener("orientationchange", onOrientationChange);

    const vv = window.visualViewport;
    const onVisualViewportChange = () => applyCanvasSize();
    vv?.addEventListener("resize", onVisualViewportChange);
    vv?.addEventListener("scroll", onVisualViewportChange);

    let rid = 0;
    const loop = () => {
      rid = requestAnimationFrame(loop);
      controls.update();
      renderer.render(scene, camera);
    };
    loop();

    threeCleanup = () => {
      cancelAnimationFrame(rid);
      ro.disconnect();
      window.removeEventListener("orientationchange", onOrientationChange);
      window.clearTimeout(orientTimer);
      vv?.removeEventListener("resize", onVisualViewportChange);
      vv?.removeEventListener("scroll", onVisualViewportChange);
      controls.dispose();
      geom.dispose();
      mat.dispose();
      renderer.dispose();
      if (renderer.domElement.parentNode === el) el.removeChild(renderer.domElement);
    };

    threeState = { geom, maxPts };
  }

  function disconnectRos() {
    if (imageTopic) {
      try {
        imageTopic.unsubscribe();
      } catch {
        /* ignore */
      }
      imageTopic = null;
    }
    if (pcListener) {
      try {
        pcListener.unsubscribe();
      } catch {
        /* ignore */
      }
      pcListener = null;
    }
    if (ros) {
      try {
        ros.close();
      } catch {
        /* ignore */
      }
      ros = null;
    }
    connected = false;
    teardownThree();
    updateBadge();
    updateButtons();
  }

  function subscribeImage() {
    if (!ros || !elTopicColor.value.trim()) return;
    if (imageTopic) {
      try {
        imageTopic.unsubscribe();
      } catch {
        /* ignore */
      }
    }
    imageTopic = new ROSLIB.Topic({
      ros,
      name: elTopicColor.value.trim(),
      messageType: "sensor_msgs/msg/Image",
      throttle_rate: 200,
      queue_length: 1,
    });
    imageTopic.subscribe((msg) => drawImageMsg(msg, elCanvas));
  }

  function subscribePc() {
    if (!ros || !elTopicPc.value.trim() || !threeState) return;
    if (pcListener) {
      try {
        pcListener.unsubscribe();
      } catch {
        /* ignore */
      }
    }
    pcListener = new ROSLIB.Topic({
      ros,
      name: elTopicPc.value.trim(),
      messageType: "sensor_msgs/msg/PointCloud2",
      throttle_rate: 500,
      queue_length: 1,
    });
    pcListener.subscribe((msg) => {
      if (!threeState) return;
      const { geom, maxPts } = threeState;
      const arr = parsePointCloud2XYZ(msg, maxPts);
      const attr = geom.getAttribute("position");
      const cap = Math.min(arr.length, attr.array.length);
      for (let i = 0; i < cap; i++) attr.array[i] = arr[i];
      attr.needsUpdate = true;
      geom.setDrawRange(0, cap / 3);
    });
  }

  function connectRos() {
    disconnectRos();
    errorText = "";
    hideAppAlert();
    updateBadge();
    if (!ROSLIB) {
      errorText = "roslib 未加载";
      showAppAlert(errorText);
      updateBadge();
      return;
    }
    updateWsField();
    btnConnect.setAttribute("aria-busy", "true");
    ros = new ROSLIB.Ros({ url: wsUrl() });
    ros.on("connection", () => {
      connected = true;
      errorText = "";
      btnConnect.setAttribute("aria-busy", "false");
      updateBadge();
      updateButtons();
      queueMicrotask(() => {
        mountThree(elThreeRoot);
        subscribeImage();
        subscribePc();
      });
    });
    ros.on("error", (e) => {
      errorText = `rosbridge 错误: ${e?.message || e}`;
      connected = false;
      btnConnect.setAttribute("aria-busy", "false");
      updateBadge();
      updateButtons();
    });
    ros.on("close", () => {
      connected = false;
      btnConnect.setAttribute("aria-busy", "false");
      updateBadge();
      updateButtons();
    });
  }

  function refreshTopics() {
    if (!ros || !elTopicsPre) return;
    ros.getTopics(
      (res) => {
        elTopicsPre.textContent = (res.topics || []).join("\n");
      },
      (err) => {
        elTopicsPre.textContent = `获取话题失败: ${err}`;
      }
    );
  }

  async function loadConfig() {
    try {
      const j = await fetchJsonWithTimeout("/api/config", 12000);
      hideAppAlert();
      rosbridgePort = j.rosbridge_port ?? 9090;
      publicHost = j.public_host || "";
      elTopicColor.value = j.ivg_presets?.color_image_topic || "/camera/color/image_raw";
      elTopicPc.value = j.ivg_presets?.graspnet_pointcloud_topic || "/graspnet_pointcloud";
    } catch (e) {
      const msg =
        e?.name === "AbortError"
          ? "加载配置超时，请检查网络或网关是否可达。"
          : `加载 /api/config 失败: ${e?.message || e}`;
      errorText = msg;
      showAppAlert(msg);
      updateBadge();
    }
    updateWsField();
  }

  const debouncedSubscribeImage = debounce(() => {
    if (connected) subscribeImage();
  }, 350);
  const debouncedSubscribePc = debounce(() => {
    if (connected && threeState) subscribePc();
  }, 350);

  btnConnect.addEventListener("click", connectRos);
  btnDisconnect.addEventListener("click", disconnectRos);
  btnTopics?.addEventListener("click", refreshTopics);
  if (elTopicColor && elTopicColor.type !== "hidden") {
    elTopicColor.addEventListener("input", debouncedSubscribeImage);
    elTopicColor.addEventListener("change", debouncedSubscribeImage);
  }
  if (elTopicPc && elTopicPc.type !== "hidden") {
    elTopicPc.addEventListener("input", debouncedSubscribePc);
    elTopicPc.addEventListener("change", debouncedSubscribePc);
  }

  // 演示页：配置就绪后自动连接；开发页仅填充表单项，由用户点「连接」
  void loadConfig().then(() => {
    if (IS_DEMO && ROSLIB) connectRos();
  });
  updateBadge();
  updateButtons();

  window.addEventListener("beforeunload", () => disconnectRos());
}

main();
