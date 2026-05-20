<script setup lang="ts">
/**
 * LattePourConfigPanel — 倾倒参数面板 (可折叠, 仅在参数化生成模式显示)
 */
defineProps<{
  visible: boolean
  advancedOpen: boolean
  mixHeight: number; drawHeight: number; finishHeight: number
  wiggleAmp: number; wiggleFreq: number
  maxVel: number; maxAcc: number; maxJerk: number
  antiSlosh: boolean
}>()

const emit = defineEmits<{
  'toggle': []
  'update:mixHeight': [v: number]; 'update:drawHeight': [v: number]; 'update:finishHeight': [v: number]
  'update:wiggleAmp': [v: number]; 'update:wiggleFreq': [v: number]
  'update:maxVel': [v: number]; 'update:maxAcc': [v: number]; 'update:maxJerk': [v: number]
  'update:antiSlosh': [v: boolean]
}>()

function num(e: Event) { return Number((e.target as HTMLInputElement).value) }
</script>

<template>
  <div v-if="visible" class="mb-3">
    <button class="w-full text-xs text-left text-slate-500 hover:text-slate-700 py-1 flex items-center gap-1"
            @click="emit('toggle')">
      <span class="inline-block transition-transform" :class="advancedOpen ? 'rotate-90' : ''">&#9654;</span>
      倾倒参数 (高级)
    </button>
    <div v-show="advancedOpen" class="grid grid-cols-2 gap-2 p-2 bg-slate-50 rounded border border-slate-100">
      <div>
        <label for="pour-mix-height" class="text-[11px] text-slate-400">融合高度 (m)</label>
        <input id="pour-mix-height" :value="mixHeight" type="number" step="0.001" min="0.0" max="0.2" class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:mixHeight', num($event))" />
      </div>
      <div>
        <label for="pour-draw-height" class="text-[11px] text-slate-400">成形高度 (m)</label>
        <input id="pour-draw-height" :value="drawHeight" type="number" step="0.001" min="0.0" max="0.2" class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:drawHeight', num($event))" />
      </div>
      <div>
        <label for="pour-finish-height" class="text-[11px] text-slate-400">收尾高度 (m)</label>
        <input id="pour-finish-height" :value="finishHeight" type="number" step="0.001" min="0.0" max="0.2" class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:finishHeight', num($event))" />
      </div>
      <div>
        <label for="pour-wiggle-amp" class="text-[11px] text-slate-400">摆动振幅 (m)</label>
        <input id="pour-wiggle-amp" :value="wiggleAmp" type="number" step="0.001" min="0.0" max="0.05" class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:wiggleAmp', num($event))" />
      </div>
      <div>
        <label for="pour-wiggle-freq" class="text-[11px] text-slate-400">摆动频率 (Hz)</label>
        <input id="pour-wiggle-freq" :value="wiggleFreq" type="number" step="0.5" min="0.5" max="20" class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @input="emit('update:wiggleFreq', num($event))" />
      </div>
      <div class="flex items-center gap-2">
        <label for="pour-anti-slosh" class="text-[11px] text-slate-400">抗晃荡</label>
        <input id="pour-anti-slosh" type="checkbox" :checked="antiSlosh" @change="emit('update:antiSlosh', ($event.target as HTMLInputElement).checked)" />
      </div>
    </div>
  </div>
</template>
