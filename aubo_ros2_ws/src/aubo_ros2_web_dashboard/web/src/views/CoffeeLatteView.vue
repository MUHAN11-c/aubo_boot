<script setup lang="ts">
/**
 * CoffeeLatteView — 咖啡拉花面板
 *
 * 替代旧版: coffee_latte_panel.html + coffee_latte_io.js (115行)
 *
 * 功能:
 *   - DO 开关 (咖啡 DO4 / 打花 DO2) — 通过 /set_latte_do2, /set_latte_do4 服务
 *   - DI 反馈灯 (DI2/DI3/DI4) — 通过 /latte_di_status 话题
 *   - 流程步骤示意
 *
 * 数据流:
 *   DI: rosbridge ← /latte_di_status (JSON/CSV) → parseDi → 信号灯状态
 *   DO: 点击开关 → setLatteDo(id, on) → /set_latte_doX 服务
 */
import { useRos } from '@/composables/useRos'
import { useRosService } from '@/composables/useRosService'
import { LATTE_DI_STATUS_TOPIC, LATTE_DI_STATUS_TYPE, ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, JOINT_STATES_TOPIC, JOINT_STATES_TYPE, TF_TOPIC, TF_STATIC_TOPIC, TF_TYPE } from '@/constants/ros'

const { isConnected, subscribe, onRosJson, onControlJson } = useRos()
const { setLatteDo } = useRosService()

// ═══════════════════════ DI 信号灯 ═══════════════════════

const di2 = ref(false); const di3 = ref(false); const di4 = ref(false)

/** 解析 DI 消息 — 支持 JSON {"di2":1,"di3":0,"di4":1} 和 CSV "1,0,1" 两种格式 */
function parseDi(msg: any) {
  let text = typeof msg === 'string' ? msg : msg?.data ? String(msg.data) : ''
  if (!text) return
  try { const obj = JSON.parse(text); if (obj && typeof obj === 'object') { di2.value = !!obj.di2; di3.value = !!obj.di3; di4.value = !!obj.di4; return } } catch { /* 尝试 CSV */ }
  const parts = text.split(',').map((s: string) => s.trim())
  if (parts.length >= 3) { di2.value = parts[0] === '1'; di3.value = parts[1] === '1'; di4.value = parts[2] === '1' }
}

// ═══════════════════════ DO 开关 ═══════════════════════

const do2On = ref(false); const do4On = ref(false)

async function toggleDo(id: 2 | 4) {
  const cur = id === 2 ? do2On.value : do4On.value
  try { await setLatteDo(id, !cur); if (id === 2) do2On.value = !cur; else do4On.value = !cur }
  catch { /* 服务不可用时静默失败 */ }
}

// ═══════════════════════ 订阅 ═══════════════════════

onRosJson(LATTE_DI_STATUS_TOPIC, parseDi)

function setupSubs() {
  if (!isConnected()) return
  subscribe(LATTE_DI_STATUS_TOPIC, LATTE_DI_STATUS_TYPE, 10)
  subscribe(ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, 10)
  subscribe(JOINT_STATES_TOPIC, JOINT_STATES_TYPE, 30)
  subscribe(TF_TOPIC, TF_TYPE, 30)
  subscribe(TF_STATIC_TOPIC, TF_TYPE, 1)
}
onControlJson((c) => { if (c.op === 'connection') setupSubs() })
watch(isConnected, v => { if (v) setupSubs() })
if (isConnected()) setupSubs()
</script>

<template>
  <div class="max-w-6xl mx-auto px-4 py-6">
    <div class="flex items-center gap-4 mb-6">
      <h1 class="text-2xl font-bold text-slate-900">机械臂咖啡拉花</h1>
      <el-tag size="small" type="warning" round>演示</el-tag>
    </div>

    <div class="grid grid-cols-1 md:grid-cols-2 gap-4">
      <!-- 工序开关 DO -->
      <div class="bg-white rounded-lg border border-slate-200 p-4">
        <h2 class="text-sm font-bold text-slate-500 uppercase mb-4">工序开关 (DO)</h2>
        <div class="space-y-3">
          <div class="flex items-center justify-between">
            <div>
              <span class="text-sm font-medium text-slate-700">咖啡开关</span>
              <el-tag size="small" class="ml-2">DO4</el-tag>
            </div>
            <el-switch v-model="do4On" @change="toggleDo(4)" />
          </div>
          <div class="flex items-center justify-between">
            <div>
              <span class="text-sm font-medium text-slate-700">打花开关</span>
              <el-tag size="small" class="ml-2">DO2</el-tag>
            </div>
            <el-switch v-model="do2On" @change="toggleDo(2)" />
          </div>
        </div>
      </div>

      <!-- 反馈信号灯 DI -->
      <div class="bg-white rounded-lg border border-slate-200 p-4">
        <h2 class="text-sm font-bold text-slate-500 uppercase mb-4">反馈信号灯 (DI)</h2>
        <div class="space-y-3">
          <div class="flex items-center gap-3">
            <span class="w-3 h-3 rounded-full transition-colors" :class="di2 ? 'bg-green-500 shadow-[0_0_6px] shadow-green-500' : 'bg-slate-300'" />
            <span class="text-sm text-slate-600">咖啡反馈</span>
            <el-tag size="small">DI2</el-tag>
          </div>
          <div class="flex items-center gap-3">
            <span class="w-3 h-3 rounded-full transition-colors" :class="di3 ? 'bg-green-500 shadow-[0_0_6px] shadow-green-500' : 'bg-slate-300'" />
            <span class="text-sm text-slate-600">打花反馈</span>
            <el-tag size="small">DI3</el-tag>
          </div>
          <div class="flex items-center gap-3">
            <span class="w-3 h-3 rounded-full transition-colors" :class="di4 ? 'bg-amber-500 shadow-[0_0_6px] shadow-amber-500' : 'bg-slate-300'" />
            <span class="text-sm text-slate-600">警告反馈</span>
            <el-tag size="small">DI4</el-tag>
          </div>
        </div>
      </div>
    </div>

    <!-- 流程示意 -->
    <div class="bg-white rounded-lg border border-slate-200 p-4 mt-4">
      <h2 class="text-sm font-bold text-slate-500 uppercase mb-3">咖啡流程示意</h2>
      <div class="flex items-center gap-3 text-sm">
        <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">1</span>出杯/定位</span>
        <span class="text-slate-300">→</span>
        <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">2</span>萃取与奶泡</span>
        <span class="text-slate-300">→</span>
        <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">3</span>拉花执行</span>
      </div>
    </div>
  </div>
</template>
