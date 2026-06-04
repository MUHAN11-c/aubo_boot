/**
 * 从 full_analysis.json 的 connectionGraphUpdates 中提取拓扑数据
 * 生成: 1) DOT 文件 (Graphviz)  2) 交互式 HTML (vis-network)
 */
import fs from 'node:fs';

const data = JSON.parse(fs.readFileSync('./full_analysis.json', 'utf8'));
const updates = data.connectionGraphUpdates || [];

if (updates.length === 0) {
  console.log('无拓扑数据');
  process.exit(0);
}

// 聚合所有更新 → 最终拓扑快照
const pubMap = new Map(); // topic → Set<nodeId>
const subMap = new Map(); // topic → Set<nodeId>

for (const u of updates) {
  for (const pt of u.raw.publishedTopics || []) {
    if (!pubMap.has(pt.name)) pubMap.set(pt.name, new Set());
    for (const id of (pt.publisherIds || [])) pubMap.get(pt.name).add(id);
  }
  for (const st of u.raw.subscribedTopics || []) {
    if (!subMap.has(st.name)) subMap.set(st.name, new Set());
    for (const id of (st.subscriberIds || [])) subMap.get(st.name).add(id);
  }
}

// 收集所有 topic/节点
const allTopics = new Set([...pubMap.keys(), ...subMap.keys()]);
const allNodeIds = new Set();
for (const ids of pubMap.values()) for (const id of ids) allNodeIds.add(id);
for (const ids of subMap.values()) for (const id of ids) allNodeIds.add(id);

console.log(`Topics: ${allTopics.size}, Node IDs: ${allNodeIds.size}, Edges: ${pubMap.size + subMap.size}`);

// ═══════════════════════════════════════════════════════
// 1. 生成 DOT 文件
// ═══════════════════════════════════════════════════════
function safeId(s) { return s.replace(/[^a-zA-Z0-9_]/g, '_').replace(/^_+/, ''); }

let dot = 'digraph ROS2_Topology {\n';
dot += '  rankdir=LR;\n  bgcolor="#1a1a2e";\n';
dot += '  node [fontname="sans-serif" fontsize=11];\n';
dot += '  edge [fontname="sans-serif" fontsize=9];\n\n';

for (const t of allTopics) {
  const label = t.split('/').filter(Boolean).slice(-2).join('/');
  dot += `  "${safeId(t)}" [label="${label}" shape=ellipse style=filled fillcolor="#16213e" fontcolor="#e94560" color="#0f3460"];\n`;
}

const pubNodes = new Set();
for (const [topic, ids] of pubMap) {
  for (const id of ids) {
    const nodeName = `pub_${id}`;
    pubNodes.add(nodeName);
    dot += `  "${nodeName}" [label="pub#${id}" shape=box style=filled fillcolor="#0f3460" fontcolor="#16c79a" color="#16c79a"];\n`;
    dot += `  "${nodeName}" -> "${safeId(topic)}" [color="#16c79a44"];\n`;
  }
}

const subNodes = new Set();
for (const [topic, ids] of subMap) {
  for (const id of ids) {
    const nodeName = `sub_${id}`;
    subNodes.add(nodeName);
    dot += `  "${nodeName}" [label="sub#${id}" shape=box style=filled fillcolor="#0f3460" fontcolor="#f5c518" color="#f5c518"];\n`;
    dot += `  "${safeId(topic)}" -> "${nodeName}" [color="#f5c51844"];\n`;
  }
}

dot += '}\n';
fs.writeFileSync('./topology.dot', dot);
console.log('DOT 文件已生成: topology.dot');

// ═══════════════════════════════════════════════════════
// 2. 生成交互式 HTML (vis-network)
// ═══════════════════════════════════════════════════════
const visNodes = [];
const visEdges = [];
const nodeSet = new Set();

for (const t of allTopics) {
  const short = t.split('/').filter(Boolean).slice(-2).join('/');
  const pubCount = pubMap.get(t)?.size || 0;
  const subCount = subMap.get(t)?.size || 0;
  const size = Math.min(50, 15 + (pubCount + subCount) * 2);
  visNodes.push({
    id: `t_${safeId(t)}`,
    label: `${short}\n(pub:${pubCount} sub:${subCount})`,
    title: t,
    shape: 'ellipse',
    size,
    color: { background: '#16213e', border: '#e94560' },
    font: { color: '#e94560', size: 10 },
    group: 'topic',
  });
  nodeSet.add(`t_${safeId(t)}`);
}

for (const [topic, ids] of pubMap) {
  let i = 0;
  for (const id of ids) {
    const nid = `p_${id}`;
    if (!nodeSet.has(nid)) {
      visNodes.push({
        id: nid, label: `pub#${id}`, shape: 'box', size: 8,
        color: { background: '#0f3460', border: '#16c79a' },
        font: { color: '#16c79a', size: 8 }, group: 'pub',
      });
      nodeSet.add(nid);
    }
    visEdges.push({ from: nid, to: `t_${safeId(topic)}`, arrows: 'to', color: { color: '#16c79a44' }, width: 0.5 });
    i++; if (i > 5) break;
  }
}

for (const [topic, ids] of subMap) {
  let i = 0;
  for (const id of ids) {
    const nid = `s_${id}`;
    if (!nodeSet.has(nid)) {
      visNodes.push({
        id: nid, label: `sub#${id}`, shape: 'box', size: 8,
        color: { background: '#0f3460', border: '#f5c518' },
        font: { color: '#f5c518', size: 8 }, group: 'sub',
      });
      nodeSet.add(nid);
    }
    visEdges.push({ from: `t_${safeId(topic)}`, to: nid, arrows: 'to', color: { color: '#f5c51844' }, width: 0.5 });
    i++; if (i > 10) break;
  }
}

const html = `<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ROS2 拓扑图 — foxglove_bridge connectionGraphUpdate</title>
<script src="https://unpkg.com/vis-network/standalone/umd/vis-network.min.js"></script>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { background: #1a1a2e; color: #eee; font-family: monospace; overflow: hidden; }
  #header { height: 40px; display: flex; align-items: center; padding: 0 16px;
            background: #0f3460; border-bottom: 2px solid #e94560; gap: 16px; }
  #header h1 { font-size: 14px; color: #e94560; white-space: nowrap; }
  #header .stat { font-size: 12px; color: #16c79a; }
  #graph { width: 100vw; height: calc(100vh - 40px); }
  #legend { position: fixed; bottom: 16px; left: 16px; background: #000000aa;
            padding: 10px 14px; border-radius: 6px; font-size: 11px; line-height: 1.8; }
  .pub-color { color: #16c79a; } .sub-color { color: #f5c518; } .topic-color { color: #e94560; }
</style>
</head>
<body>
<div id="header">
  <h1>⚡ ROS2 Topology — foxglove_bridge connectionGraphUpdate</h1>
  <span class="stat">Topics: ${allTopics.size}</span>
  <span class="stat">Publishers: ${allNodeIds.size}</span>
  <span class="stat">Edges: ${pubMap.size + subMap.size}</span>
  <span class="stat" style="margin-left:auto">${data.summary.totalConnectionGraphUpdates} updates | ${data.exportedAt}</span>
</div>
<div id="graph"></div>
<div id="legend">
  <div><span class="topic-color">●</span> Topic (椭圆)</div>
  <div><span class="pub-color">■</span> Publisher</div>
  <div><span class="sub-color">■</span> Subscriber</div>
  <div style="margin-top:4px;font-size:10px;color:#888">滚轮缩放 | 拖拽平移 | 点击选点</div>
</div>
<script>
const nodes = new vis.DataSet(${JSON.stringify(visNodes)});
const edges = new vis.DataSet(${JSON.stringify(visEdges)});

const container = document.getElementById('graph');
new vis.Network(container, { nodes, edges }, {
  physics: {
    solver: 'forceAtlas2Based',
    forceAtlas2Based: { gravitationalConstant: -80, centralGravity: 0.005, springLength: 150, springConstant: 0.02 },
    stabilization: { iterations: 300 },
  },
  interaction: { hover: true, tooltipDelay: 100, zoomView: true, dragView: true },
  edges: { smooth: { type: 'continuous' } },
});
</script>
</body>
</html>`;

fs.writeFileSync('./topology.html', html);
console.log('HTML 拓扑图已生成: topology.html (visNodes: ' + visNodes.length + ', visEdges: ' + visEdges.length + ')');
