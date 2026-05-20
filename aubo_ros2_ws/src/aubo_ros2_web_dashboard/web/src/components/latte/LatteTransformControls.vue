<script setup lang="ts">
/**
 * LatteTransformControls — RPY + Speed 控制
 */
defineProps<{
  roll: number; pitch: number; yaw: number
  speedScale: number; toolId: string
}>()

const emit = defineEmits<{
  'update:roll': [v: number]; 'update:pitch': [v: number]; 'update:yaw': [v: number]
  'update:speedScale': [v: number]; 'update:toolId': [v: string]
  'saveRpy': []
}>()

const rpySaved = ref(false)
let saveTimer: ReturnType<typeof setTimeout> | null = null
function onRpySave() {
  emit('saveRpy')
  rpySaved.value = true
  if (saveTimer) clearTimeout(saveTimer)
  saveTimer = setTimeout(() => { rpySaved.value = false }, 1200)
}

function num(e: Event) { return Number((e.target as HTMLInputElement).value) }
</script>

<template>
  <div>
    <!-- RPY -->
    <div class="flex items-center justify-between mb-1.5">
      <span class="text-[11px] text-slate-400">姿态微调</span>
      <span v-if="rpySaved" class="text-[10px] text-green-500 transition-opacity">已保存 ✓</span>
    </div>
    <div class="grid grid-cols-3 gap-2 mb-3">
      <div>
        <label for="rpy-roll" class="text-[11px] text-slate-400">Roll°</label>
        <input id="rpy-roll" :value="roll" type="number" step="1" min="-180" max="180"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @change="onRpySave()" @input="emit('update:roll', num($event))" />
      </div>
      <div>
        <label for="rpy-pitch" class="text-[11px] text-slate-400">Pitch°</label>
        <input id="rpy-pitch" :value="pitch" type="number" step="1" min="-180" max="180"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @change="onRpySave()" @input="emit('update:pitch', num($event))" />
      </div>
      <div>
        <label for="rpy-yaw" class="text-[11px] text-slate-400">Yaw°</label>
        <input id="rpy-yaw" :value="yaw" type="number" step="1" min="-180" max="180"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @change="onRpySave()" @input="emit('update:yaw', num($event))" />
      </div>
    </div>

    <!-- Tool (只读显示) -->
    <div class="mb-2 flex items-center gap-1 text-[11px] text-slate-400">
      <span>工具偏移:</span>
      <span class="font-mono text-slate-600">{{ toolId }}</span>
    </div>

    <!-- Speed -->
    <div class="mb-3">
      <label class="text-xs text-slate-500">速度倍率 <span class="font-mono text-slate-700">{{ speedScale.toFixed(2) }}x</span></label>
      <input :value="speedScale" type="range" :min="0.01" :max="10.0" step="0.01" class="w-full h-1.5 mt-1"
             @input="emit('update:speedScale', num($event))" />
    </div>
  </div>
</template>
