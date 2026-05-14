<script setup lang="ts">
/**
 * ToolSwapBar — 末端夹爪快换控制
 *
 * 显示当前工具状态，提供快换按钮
 * 监听 /tool_changer_status 话题自动更新当前工具
 */
import { useRos } from '@/composables/useRos'
import { useRosService } from '@/composables/useRosService'
import { TOOL_CHANGER_STATUS_TOPIC, TOOL_CHANGER_STATUS_TYPE } from '@/constants/ros'
import { TOOL_LIST } from '@/constants/grasp'

const emit = defineEmits<{ log: [msg: string] }>()
function log(msg: string) { emit('log', msg) }

const { isConnected, subscribe, onRosJson, onControlJson } = useRos()
const { changeTool } = useRosService()

/** 当前工具状态 */
const currentToolId = ref('')
const currentToolName = ref('')

// 监听工具状态变化
onRosJson(TOOL_CHANGER_STATUS_TOPIC, (msg: any) => {
  if (msg?.tool_id) { currentToolId.value = msg.tool_id; currentToolName.value = msg.tool_name || msg.tool_id }
})

// 连接后订阅
function setup() { if (isConnected()) subscribe(TOOL_CHANGER_STATUS_TOPIC, TOOL_CHANGER_STATUS_TYPE) }
onControlJson((c) => { if (c.op === 'connection') setup() })
watch(isConnected, v => { if (v) setup() })
if (isConnected()) setup()

/** 执行工具快换 */
async function doChange(toolId: string) {
  log(`快换 → ${toolId}…`)
  try { const r = await changeTool(toolId); log(r.success ? `✓ ${r.message || '成功'}` : `✗ ${r.message || '失败'}`) }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}
</script>

<template>
  <div class="bg-white rounded-lg border border-slate-200 p-4">
    <h3 class="text-sm font-bold text-slate-700 mb-2">末端夹爪快换</h3>

    <!-- 当前工具指示 -->
    <div class="flex items-center gap-1.5 mb-2">
      <span class="w-2 h-2 rounded-full" :class="currentToolId ? 'bg-green-500' : 'bg-slate-300'" />
      <span class="text-xs text-slate-500">
        {{ currentToolId ? `当前: ${currentToolName || currentToolId}` : '当前: 无工具' }}
      </span>
    </div>

    <!-- 快换按钮 -->
    <div class="flex flex-wrap gap-1.5">
      <el-button
        v-for="tool in TOOL_LIST" :key="tool.id"
        size="small"
        :type="currentToolId === tool.id ? 'primary' : 'default'"
        :disabled="currentToolId === tool.id"
        @click="doChange(tool.id)"
      >
        {{ tool.label }}
      </el-button>
    </div>
  </div>
</template>
