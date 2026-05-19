<script setup lang="ts">
/**
 * LattePatternSelector — 拉花轨迹源选择
 * 录制回放 (Episode 滑块) / 参数化生成 (heart | rosetta | tulip | swan)
 */
import { PATTERN_TYPES, MAX_EPISODE } from '@/constants/latte'

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

const isGenerated = computed(() => props.patternType !== '')

function onPatternChange(e: Event) {
  emit('update:patternType', (e.target as HTMLSelectElement).value)
}

function onEpisodeInput(e: Event) {
  emit('update:episodeIdx', Number((e.target as HTMLInputElement).value))
}

function onLayersInput(e: Event) {
  emit('update:tulipLayers', Number((e.target as HTMLInputElement).value))
}
</script>

<template>
  <div class="mb-3">
    <label class="text-xs text-slate-500 mb-1 block">轨迹源</label>
    <select
      :value="patternType"
      class="w-full text-xs border border-slate-200 rounded px-2 py-1.5 bg-white"
      @change="onPatternChange"
    >
      <option v-for="pt in PATTERN_TYPES" :key="pt.value" :value="pt.value">{{ pt.label }}</option>
    </select>

    <!-- Episode 滑块 (仅录制回放) -->
    <div v-if="!isGenerated" class="mt-2">
      <label class="text-xs text-slate-400">Episode <span class="font-mono text-slate-700">{{ episodeIdx }}</span></label>
      <input :value="episodeIdx" type="range" :min="0" :max="MAX_EPISODE" step="1"
             class="w-full h-1.5 mt-1" @input="onEpisodeInput" />
    </div>

    <!-- 郁金香层数 (仅 tulip) -->
    <div v-if="patternType === 'tulip'" class="mt-2">
      <label class="text-xs text-slate-400">层数 <span class="font-mono text-slate-700">{{ tulipLayers }}</span></label>
      <input :value="tulipLayers" type="range" :min="1" :max="6" step="1"
             class="w-full h-1.5 mt-1" @input="onLayersInput" />
    </div>
  </div>
</template>
