<script setup lang="ts">
/**
 * PoseCard — 末端位姿卡片
 *
 * 显示 XYZ 位置 + RPY 欧拉角
 * 响应式更新，数据从父组件传入
 */
defineProps<{
  x?: string; y?: string; z?: string
  roll?: string; pitch?: string; yaw?: string
}>()
</script>

<template>
  <div class="bg-white rounded-lg border border-slate-200 p-4">
    <h3 class="text-xs font-bold text-slate-500 uppercase mb-2">末端位姿</h3>
    <div class="grid grid-cols-3 gap-2 text-sm">
      <div><span class="text-slate-400 text-xs">X</span><br/><span class="font-mono">{{ x || '—' }}</span></div>
      <div><span class="text-slate-400 text-xs">Y</span><br/><span class="font-mono">{{ y || '—' }}</span></div>
      <div><span class="text-slate-400 text-xs">Z</span><br/><span class="font-mono">{{ z || '—' }}</span></div>
      <div><span class="text-slate-400 text-xs">Roll</span><br/><span class="font-mono">{{ roll || '—' }}</span></div>
      <div><span class="text-slate-400 text-xs">Pitch</span><br/><span class="font-mono">{{ pitch || '—' }}</span></div>
      <div><span class="text-slate-400 text-xs">Yaw</span><br/><span class="font-mono">{{ yaw || '—' }}</span></div>
    </div>
  </div>
</template>
