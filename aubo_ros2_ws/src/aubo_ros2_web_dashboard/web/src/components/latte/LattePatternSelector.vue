<script setup lang="ts">
/**
 * LattePatternSelector — 拉花轨迹源选择 (当前仅心形)
 */
import { PATTERN_TYPES } from '@/constants/latte'

const props = defineProps<{
  patternType: string
  episodeIdx: number
  tulipLayers: number
}>()

const emit = defineEmits<{
  'update:patternType': [v: string]
  'update:episodeIdx': [v: number]
  'update:tulipLayers': [v: number]
}>()

function onPatternChange(e: Event) {
  emit('update:patternType', (e.target as HTMLSelectElement).value)
}
</script>

<template>
  <div class="mb-3">
    <label for="latte-pattern-type" class="text-xs text-slate-500 mb-1 block">轨迹源</label>
    <select id="latte-pattern-type"
      :value="patternType"
      class="w-full text-xs border border-slate-200 rounded px-2 py-1.5 bg-white"
      @change="onPatternChange"
    >
      <option v-for="pt in PATTERN_TYPES" :key="pt.value" :value="pt.value">{{ pt.label }}</option>
    </select>
  </div>
</template>
