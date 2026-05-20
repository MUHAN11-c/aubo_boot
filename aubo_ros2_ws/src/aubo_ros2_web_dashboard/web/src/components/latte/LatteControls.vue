<script setup lang="ts">
/**
 * LatteControls — 预览/执行按钮 + 结果面板
 */
defineProps<{
  previewLoading: boolean
  execExecuting: boolean
  rosConnected: boolean
  message: string
  success: boolean | null
  numFrames: number
  pathLength: number
  tcpWaypoints: number
  spoutWaypoints: number
}>()

const emit = defineEmits<{
  'preview': []
  'execute': []
}>()
</script>

<template>
  <div class="flex gap-2 mb-2">
    <el-button size="small" :loading="previewLoading" @click="emit('preview')">预览</el-button>
    <el-button size="small" type="warning" :loading="execExecuting"
               :disabled="!rosConnected" :title="!rosConnected ? '请先连接 ROS (检查 rosbridge 状态)' : ''"
               @click="emit('execute')">执行</el-button>
  </div>

  <!-- 消息 -->
  <div v-if="message" role="alert" class="text-xs p-2 rounded mb-1"
       :class="success === true ? 'bg-green-50 text-green-700' : success === false ? 'bg-amber-50 text-amber-700' : 'bg-blue-50 text-blue-600'">
    {{ message }}
  </div>

  <!-- 统计 -->
  <div v-if="numFrames > 0" class="text-[11px] text-slate-500 space-y-0.5">
    <div>帧数: <span class="font-mono">{{ numFrames }}</span> · 路径: <span class="font-mono">{{ pathLength?.toFixed(2) }}m</span></div>
    <div>TCP waypoints: <span class="font-mono">{{ tcpWaypoints }}</span> · Spout: <span class="font-mono">{{ spoutWaypoints }}</span></div>
  </div>
</template>
