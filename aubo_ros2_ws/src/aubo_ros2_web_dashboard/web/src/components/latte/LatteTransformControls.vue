<script setup lang="ts">
/**
 * LatteTransformControls — RPY + Speed + Tool 控制 (从 CoffeeLatteView 提取)
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

function num(e: Event) { return Number((e.target as HTMLInputElement).value) }
</script>

<template>
  <div>
    <!-- RPY -->
    <div class="grid grid-cols-3 gap-2 mb-3">
      <div>
        <label class="text-[11px] text-slate-400">Roll°</label>
        <input :value="roll" type="number" step="1"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @change="emit('saveRpy')" @input="emit('update:roll', num($event))" />
      </div>
      <div>
        <label class="text-[11px] text-slate-400">Pitch°</label>
        <input :value="pitch" type="number" step="1"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @change="emit('saveRpy')" @input="emit('update:pitch', num($event))" />
      </div>
      <div>
        <label class="text-[11px] text-slate-400">Yaw°</label>
        <input :value="yaw" type="number" step="1"
               class="w-full text-xs border border-slate-200 rounded px-1 py-0.5 mt-0.5"
               @change="emit('saveRpy')" @input="emit('update:yaw', num($event))" />
      </div>
    </div>

    <!-- Speed -->
    <div class="mb-3">
      <label class="text-xs text-slate-500">速度倍率 <span class="font-mono text-slate-700">{{ speedScale.toFixed(2) }}x</span></label>
      <input :value="speedScale" type="range" :min="0.01" :max="10.0" step="0.01" class="w-full h-1.5 mt-1"
             @input="emit('update:speedScale', num($event))" />
    </div>
  </div>
</template>
