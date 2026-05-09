// ivg_web_video — MJPEG 视频流 <img> 标签管理，含自动重连
// 优先走网关代理 (ivgPorts.webVideoProxyOriginPrefix)，
// 回退到直连 web_video_server（旧版兼容）
const g = globalThis;

function hostname() {
    return (typeof window !== 'undefined' && window.location && window.location.hostname) || '127.0.0.1';
}

import { encodeTopicQueryValue } from './core/utils.js';

// ── 直连模式（旧版兼容）─────────────────────────────────────────────────────

function legacyPort(opts) {
    let p = opts.port != null && opts.port > 0 ? Number(opts.port) : NaN;
    if (!isNaN(p) && p > 0) return p;
    if (g.location && g.location.port) {
        const lp = parseInt(String(g.location.port), 10);
        if (!isNaN(lp) && lp > 0) return lp;
    }
    return (g.location && g.location.protocol === 'https:') ? 443 : 80;
}

function buildQuery(topic, opts, defaultType) {
    const parts = ['topic=' + encodeTopicQueryValue(topic)];
    parts.push('type=' + encodeURIComponent(opts.type != null ? String(opts.type) : defaultType));
    if (opts.width > 0) parts.push('width=' + opts.width);
    if (opts.height > 0) parts.push('height=' + opts.height);
    if (opts.quality != null && opts.quality > 0) parts.push('quality=' + opts.quality);
    if (opts.qos_profile) parts.push('qos_profile=' + opts.qos_profile);
    if (opts.default_transport) parts.push('default_transport=' + opts.default_transport);
    const cid = opts.client_id != null ? String(opts.client_id) : '';
    if (cid !== '') parts.push('client_id=' + encodeURIComponent(cid));
    return parts.join('&');
}

// ── URL 构建 ────────────────────────────────────────────────────────────────

function streamUrl(topic, opts) {
    opts = opts || {};
    // 优先使用网关代理
    if (typeof g.ivgPorts !== 'undefined' && typeof g.ivgPorts.webVideoProxyOriginPrefix === 'function') {
        const base = g.ivgPorts.webVideoProxyOriginPrefix();
        if (base && base.indexOf('/api/v1/camera/stream') !== -1) {
            const q = new URLSearchParams();
            q.set('topic', String(topic));
            q.set('stream_id', opts.client_id != null ? String(opts.client_id) : 'ivg_web_video');
            q.set('quality', String(opts.quality != null && opts.quality > 0 ? opts.quality : 85));
            return `${base}?${q}`;
        }
        if (base) return `${base}/stream?${buildQuery(topic, opts, 'mjpeg')}`;
    }
    // 回退：直连 web_video_server
    const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
    return `http://${h}:${legacyPort(opts)}/stream?${buildQuery(topic, opts, 'mjpeg')}`;
}

function snapshotUrl(topic, opts) {
    opts = opts || {};
    if (typeof g.ivgPorts !== 'undefined' && typeof g.ivgPorts.webVideoProxyOriginPrefix === 'function') {
        const base = g.ivgPorts.webVideoProxyOriginPrefix();
        if (base && base.indexOf('/api/v1/camera/stream') !== -1) return streamUrl(topic, opts);
        if (base) return `${base}/snapshot?${buildQuery(topic, opts, 'jpeg')}`;
    }
    const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
    return `http://${h}:${legacyPort(opts)}/snapshot?${buildQuery(topic, opts, 'jpeg')}`;
}

function viewerUrl(topic, opts) {
    opts = opts || {};
    if (typeof g.ivgPorts !== 'undefined' && typeof g.ivgPorts.webVideoProxyOriginPrefix === 'function') {
        const base = g.ivgPorts.webVideoProxyOriginPrefix();
        if (base && base.indexOf('/api/v1/camera/stream') !== -1) return streamUrl(topic, opts);
        if (base) return `${base}/stream_viewer?topic=${encodeTopicQueryValue(topic)}`;
    }
    const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
    return `http://${h}:${legacyPort(opts)}/stream_viewer?topic=${encodeTopicQueryValue(topic)}`;
}

// ── MJPEG <img> 自动重连 ───────────────────────────────────────────────────

function mjpegStreamAttachAutoReload(img, getUrl) {
    if (!img || typeof getUrl !== 'function') return;
    if (img._ivgMjpegRecoverCleanup) {
        img._ivgMjpegRecoverCleanup();
        img._ivgMjpegRecoverCleanup = null;
    }

    let attempts = 0;
    let timer = null;
    const maxAttempts = 15;

    function clearTimer() {
        if (timer) { clearTimeout(timer); timer = null; }
    }

    function reload() {
        if (attempts >= maxAttempts) return;
        attempts++;
        const delay = Math.min(30000, 2000 * Math.pow(2, attempts - 1));
        clearTimer();
        timer = setTimeout(() => {
            timer = null;
            const u = getUrl();
            const sep = u.indexOf('?') >= 0 ? '&' : '?';
            img.src = `${u}${sep}_ivgRecover=${Date.now()}`;
        }, delay);
    }

    function onLoad() { attempts = 0; clearTimer(); }
    function onError() { reload(); }

    img.addEventListener('load', onLoad);
    img.addEventListener('error', onError);
    img._ivgMjpegRecoverCleanup = () => {
        img.removeEventListener('load', onLoad);
        img.removeEventListener('error', onError);
        clearTimer();
    };
}

// ── 全局导出 ────────────────────────────────────────────────────────────────

g.ivgWebVideo = { streamUrl, snapshotUrl, viewerUrl, mjpegStreamAttachAutoReload };
