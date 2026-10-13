// topic_browser.js — ROS 2 话题/服务/节点浏览器（connectionGraph 数据源）
//
// 提供类似 ros2 topic list / ros2 service list / ros2 node list 的调试视图，
// 每项可点击查看发布者/订阅者/服务端节点列表，话题可进一步查看类型。
// 2026-10-13 新增。

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';

function createTopicBrowser(opts = {}) {
  const client = foxgloveRosClient();
  const $ = opts.getById || ((id) => document.getElementById(id));
  let _timer = null;
  let _mounted = false;

  function _render() {
    const listEl = $('topic-browser-list');
    if (!listEl) return;
    let graph;
    try { graph = client.getGraph(); } catch (_) { return; }

    const mode = ($('topic-browser-mode') && $('topic-browser-mode').value) || 'topics';
    let items = [];
    if (mode === 'topics') {
      items = graph.topics.sort().map((t) => ({
        name: t,
        detail: [
          graph.publishedTopics[t] ? `pub:${graph.publishedTopics[t].length}` : '',
          graph.subscribedTopics[t] ? `sub:${graph.subscribedTopics[t].length}` : '',
        ].filter(Boolean).join(' '),
        type: (client.player.channels.get(t) || {}).schemaName || '',
      }));
    } else if (mode === 'services') {
      items = graph.services.sort().map((s) => ({
        name: s,
        detail: `providers:${(graph.advertisedServices[s] || []).length}`,
        type: (client.player.codec._services.get(s) || {}).service?.type || '',
      }));
    } else if (mode === 'nodes') {
      items = graph.nodes.sort().map((n) => ({ name: n, detail: '', type: '' }));
    }

    const filter = ($('topic-browser-filter') && $('topic-browser-filter').value.trim().toLowerCase()) || '';
    if (filter) items = items.filter((i) => i.name.toLowerCase().includes(filter));

    listEl.innerHTML = items.length
      ? items.map((i) =>
          `<div class="tb-row" data-name="${i.name}">` +
          `<span class="tb-name">${i.name}</span>` +
          (i.type ? `<span class="tb-type">${i.type}</span>` : '') +
          (i.detail ? `<span class="tb-detail">${i.detail}</span>` : '') +
          `</div>`
        ).join('')
      : '<div class="tb-empty">（无数据或连接未就绪）</div>';
  }

  function mount() {
    if (_mounted) return;
    const host = $('topic-browser-host');
    if (!host) return;

    host.innerHTML = `
      <div class="tb-toolbar">
        <select id="topic-browser-mode" class="tb-select">
          <option value="topics">话题 (${'ros2 topic list'})</option>
          <option value="services">服务 (${'ros2 service list'})</option>
          <option value="nodes">节点 (${'ros2 node list'})</option>
        </select>
        <input type="text" id="topic-browser-filter" class="tb-filter" placeholder="过滤…" />
        <button type="button" class="tb-refresh" id="topic-browser-refresh">↻ 刷新</button>
      </div>
      <div id="topic-browser-list" class="tb-list"></div>
    `;

    // 注入样式
    if (!document.getElementById('topic-browser-style')) {
      const style = document.createElement('style');
      style.id = 'topic-browser-style';
      style.textContent = `
        .topic-browser { border: 1px solid #333; border-radius: 6px; overflow: hidden; margin: 8px 0; }
        .tb-toolbar { display: flex; gap: 6px; padding: 6px 8px; background: #1a1a2e; border-bottom: 1px solid #333; }
        .tb-select { background: #0f0f1a; color: #e0e0e0; border: 1px solid #444; border-radius: 3px; padding: 2px 6px; font-size: 12px; }
        .tb-filter { flex: 1; background: #0f0f1a; color: #e0e0e0; border: 1px solid #444; border-radius: 3px; padding: 2px 8px; font-size: 12px; }
        .tb-refresh { background: #2a2a4a; color: #ccc; border: 1px solid #444; border-radius: 3px; padding: 2px 10px; font-size: 12px; cursor: pointer; }
        .tb-refresh:hover { background: #3a3a5a; }
        .tb-list { max-height: 260px; overflow-y: auto; font-family: monospace, monospace; font-size: 11px; }
        .tb-row { display: flex; align-items: baseline; gap: 8px; padding: 2px 8px; border-bottom: 1px solid #222; cursor: pointer; }
        .tb-row:hover { background: #1a1a3a; }
        .tb-name { color: #7ec8e3; flex: 0 0 auto; min-width: 180px; }
        .tb-type { color: #98c379; flex: 0 0 auto; }
        .tb-detail { color: #888; margin-left: auto; }
        .tb-empty { padding: 16px; text-align: center; color: #666; }
      `;
      document.head.appendChild(style);
    }

    $('topic-browser-mode').addEventListener('change', _render);
    $('topic-browser-filter').addEventListener('input', _render);
    $('topic-browser-refresh').addEventListener('click', () => {
      try { client.player.subscribeConnectionGraph(); } catch (_) {}
      _render();
    });

    // 订阅图谱
    try { client.player.subscribeConnectionGraph(); } catch (_) {}

    _timer = setInterval(_render, 3000);
    _render();
    _mounted = true;
  }

  function unmount() {
    if (_timer) { clearInterval(_timer); _timer = null; }
    _mounted = false;
  }

  return { mount, unmount, refresh: _render };
}

export { createTopicBrowser };
