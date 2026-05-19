<script setup lang="ts">
/**
 * LatteCupConfigPanel — 杯子参数面板 (仅在参数化生成模式显示)
 */
defineProps<{
  visible: boolean
  centerX: number
  centerY: number
  surfaceZ: number
  radius: number
}>()

const emit = defineEmits<{
  'update:centerX': [v: number]
  'update:centerY': [v: number]
  'update:surfaceZ': [v: number]
  'update:radius': [v: number]
}>()
</script>

<template>
  <div v-if="visible" class="mb-3 p-3 bg-blue-50/50 rounded border border-blue-100">
    <h3 class="text-xs font-bold text-slate-600 mb-2">杯子参数 (m, base_link)</h3>
    <div class="grid grid-cols-2 gap-2">
      <div>
        <label class="text-[11px] text-slate-400">中心 X</label>
        <input :value="centerX" type="number" step="0.01"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:centerX', Number(($event.target as HTMLInputElement).value))" />
      </div>
      <div>
        <label class="text-[11px] text-slate-400">中心 Y</label>
        <input :value="centerY" type="number" step="0.01"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:centerY', Number(($event.target as HTMLInputElement).value))" />
      </div>
      <div>
        <label class="text-[11px] text-slate-400">液面 Z</label>
        <input :value="surfaceZ" type="number" step="0.01"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:surfaceZ', Number(($event.target as HTMLInputElement).value))" />
      </div>
      <div>
        <label class="text-[11px] text-slate-400">杯半径</label>
        <input :value="radius" type="number" step="0.005"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:radius', Number(($event.target as HTMLInputElement).value))" />
      </div>
    </div>
  </div>
</template>
