<script setup lang="ts">
/**
 * TfMonitorView — TF 监控面板
 *
 * 替代旧版: tf_monitor_panel.html + tf_monitor_panel.js (280行)
 *
 * 功能:
 *   - 驱动模式检测 (real/simulation) — /aubo/mode
 *   - 状态概览 (在线/使能/运动/规划)
 *   - 末端位姿表 (XYZ + 四元数 + RPY)
 *   - 关节角表 (6 关节 × 度/弧度)
 *   - 快照记录 / JSON 导出 / 复制
 */
import { useRos } from '@/composables/useRos'
import { ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, MODE_TOPIC, MODE_TYPE, TF_TOPIC, TF_STATIC_TOPIC, TF_TYPE } from '@/constants/ros'

const { isConnected, subscribe, onRosJson, onControlJson } = useRos()

/** AUBO E5 关节名 (固定顺序) */
const JOINT_NAMES = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']

// ═══════════════════════ 状态 ═══════════════════════

const mode = ref<string>('unknown')
const statusOk = reactive({ is_online: false, enable: false, in_motion: false, planning_status: '—' })
const pose = reactive({ x: '—', y: '—', z: '—', qx: '—', qy: '—', qz: '—', qw: '—', roll: '—', pitch: '—', yaw: '—' })
const joints = ref<Array<{ name: string; deg: string; rad: string }>>(JOINT_NAMES.map(n => ({ name: n, deg: '—', rad: '—' })))
const latestMsg = ref<any>(null)
const records = ref<any[]>([])
const toast = ref('')
let toastTimer: ReturnType<typeof setTimeout>

function showToast(text: string) {
  toast.value = text; clearTimeout(toastTimer)
  toastTimer = setTimeout(() => { toast.value = '' }, 2000)
}

// ═══════════════════════ 数据更新 ═══════════════════════

onRosJson(MODE_TOPIC, (msg: any) => {
  const raw = msg?.data ? String(msg.data) : ''
  mode.value = raw === 'simulation' ? 'simulation' : raw === 'real' ? 'real' : 'unknown'
})

onRosJson(ROBOT_STATUS_TOPIC, (msg: any) => {
  if (!msg) return; latestMsg.value = msg
  statusOk.is_online = !!msg.is_online; statusOk.enable = !!msg.enable; statusOk.in_motion = !!msg.in_motion
  statusOk.planning_status = msg.planning_status ?? '—'
  const p = msg.cartesian_position_xyz || {}; const o = msg.cartesian_position?.orientation || {}; const r = msg.cartesian_rpy || {}
  Object.assign(pose, {
    x: p.x?.toFixed(4) ?? '—', y: p.y?.toFixed(4) ?? '—', z: p.z?.toFixed(4) ?? '—',
    qx: o.x?.toFixed(4) ?? '—', qy: o.y?.toFixed(4) ?? '—', qz: o.z?.toFixed(4) ?? '—', qw: o.w?.toFixed(4) ?? '—',
    roll: r.x != null ? (r.x * 180 / Math.PI).toFixed(1) + '°' : '—', pitch: r.y != null ? (r.y * 180 / Math.PI).toFixed(1) + '°' : '—', yaw: r.z != null ? (r.z * 180 / Math.PI).toFixed(1) + '°' : '—',
  })
  const degs = msg.joint_position_deg || []; const rads = msg.joint_position_rad || []
  joints.value = JOINT_NAMES.map((n, i) => ({ name: n, deg: degs[i]?.toFixed(3) ?? '—', rad: rads[i]?.toFixed(6) ?? '—' }))
})

// ═══════════════════════ 订阅 ═══════════════════════

function setupSubs() {
  if (!isConnected()) return
  subscribe(ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, 10); subscribe(MODE_TOPIC, MODE_TYPE, 1)
  subscribe(TF_TOPIC, TF_TYPE, 30); subscribe(TF_STATIC_TOPIC, TF_TYPE, 1)
}
onControlJson((c) => { if (c.op === 'connection') setupSubs() })
watch(isConnected, v => { if (v) setupSubs() }); if (isConnected()) setupSubs()

// ═══════════════════════ 快照操作 ═══════════════════════

function buildSnapshot() {
  const msg = latestMsg.value; if (!msg) return null
  const p = msg.cartesian_position_xyz || {}; const o = msg.cartesian_position?.orientation || {}
  return { timestamp: new Date().toISOString(), mode: mode.value, joints: JOINT_NAMES.map((n, i) => ({ name: n, position_rad: (msg.joint_position_rad || [])[i] ?? null })) }
}
function recordSnapshot() { const s = buildSnapshot(); if (s) { records.value.push(s); showToast('已记录') } }
function copyJson() { const s = buildSnapshot(); if (s) { navigator.clipboard.writeText(JSON.stringify(s, null, 2)); showToast('已复制') } }
function exportJson() { if (!records.value.length) return showToast('无记录'); const b = new Blob([JSON.stringify(records.value, null, 2)], { type: 'application/json' }); const a = document.createElement('a'); a.href = URL.createObjectURL(b); a.download = `ivg_monitor_${new Date().toISOString().slice(0, 10)}.json`; a.click() }
</script>

<template>
  <div class="max-w-6xl mx-auto px-4 py-6">
    <div class="flex items-center gap-4 mb-6">
      <h1 class="text-2xl font-bold text-slate-900">监控面板</h1>
      <el-tag :type="mode === 'real' ? 'success' : mode === 'simulation' ? 'warning' : 'info'" size="small" effect="dark" round>
        {{ mode === 'real' ? '真实机械臂' : mode === 'simulation' ? '仿真模式' : '检测中…' }}
      </el-tag>
    </div>

    <!-- 状态概览 -->
    <div class="flex flex-wrap gap-2 mb-6">
      <el-tag :type="statusOk.is_online ? 'success' : 'danger'" size="small">在线: {{ statusOk.is_online ? '✓' : '✗' }}</el-tag>
      <el-tag :type="statusOk.enable ? 'success' : 'danger'" size="small">使能: {{ statusOk.enable ? '✓' : '✗' }}</el-tag>
      <el-tag :type="statusOk.in_motion ? 'warning' : 'info'" size="small">运动: {{ statusOk.in_motion ? '运动中' : '静止' }}</el-tag>
      <el-tag :type="statusOk.planning_status === 'error' ? 'danger' : statusOk.planning_status === 'idle' ? 'success' : 'warning'" size="small">规划: {{ statusOk.planning_status }}</el-tag>
    </div>

    <!-- 位姿 + 关节 -->
    <div class="grid grid-cols-1 md:grid-cols-2 gap-4 mb-6">
      <div class="bg-white rounded-lg border border-slate-200 p-4">
        <h2 class="text-sm font-bold text-slate-500 uppercase mb-3">末端位姿</h2>
        <table class="w-full text-sm"><tr v-for="(v, k) in [['X',pose.x,'QX',pose.qx],['Y',pose.y,'QY',pose.qy],['Z',pose.z,'QZ',pose.qz],['Roll',pose.roll,'QW',pose.qw],['Pitch',pose.pitch,'',''],['Yaw',pose.yaw,'','']]" :key="String(k)"><td class="text-slate-400 w-16 py-1">{{ v[0] }}</td><td class="font-mono">{{ v[1] }}</td><td class="text-slate-400 w-16">{{ v[2] }}</td><td class="font-mono">{{ v[3] }}</td></tr></table>
      </div>
      <div class="bg-white rounded-lg border border-slate-200 p-4">
        <h2 class="text-sm font-bold text-slate-500 uppercase mb-3">关节角</h2>
        <table class="w-full text-sm"><thead><tr class="text-slate-400 text-left border-b border-slate-100"><th class="py-1 font-medium">关节</th><th class="py-1 font-medium">角度 (°)</th><th class="py-1 font-medium">弧度</th></tr></thead><tr v-for="j in joints" :key="j.name" class="border-b border-slate-50"><td class="py-1.5 text-slate-600">{{ j.name }}</td><td class="py-1.5 font-mono">{{ j.deg }}</td><td class="py-1.5 font-mono text-slate-500 text-xs">{{ j.rad }}</td></tr></table>
      </div>
    </div>

    <!-- 操作 -->
    <div class="flex flex-wrap gap-2 mb-4">
      <el-button size="small" @click="recordSnapshot">记录当前快照</el-button>
      <el-button size="small" @click="copyJson">复制 JSON</el-button>
      <el-button size="small" @click="exportJson" :disabled="!records.length">导出 JSON ({{ records.length }})</el-button>
      <el-button size="small" @click="records = []">清除记录</el-button>
    </div>

    <div v-if="toast" class="fixed bottom-6 left-1/2 -translate-x-1/2 bg-slate-800 text-white px-5 py-2.5 rounded-lg text-sm z-50 shadow-lg">{{ toast }}</div>
  </div>
</template>
