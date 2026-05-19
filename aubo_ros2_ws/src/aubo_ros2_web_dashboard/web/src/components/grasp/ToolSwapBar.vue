<script setup lang="ts">
/**
 * ToolSwapBar — 末端夹爪快换控制
 *
 * 显示当前工具状态，提供快换按钮。
 * 因末端夹爪类型无硬件反馈，系统启动时通过以下机制确定初始工具喵~：
 *
 *   1. 调用 /get_current_tool 服务（立即获取后端状态）
 *   2. 订阅 /tool_changer_status 话题（持续更新，后端每 5 秒周期性发布）
 *   3. 若以上均返回 is_connected=false（或超时未收到），显示手动选择器让用户指定
 *
 * 手动选择持久化到 localStorage，后端真实状态到达时自动覆盖喵~
 */
import { useRos } from '@/composables/ros/useRos'
import { useRosService } from '@/composables/ros/useRosService'
import { useDashboardSettings } from '@/composables/settings/useDashboardSettings'
import { canonicalRosTopic } from '@/lib/utils'
import { TOOL_CHANGER_STATUS_TOPIC, TOOL_CHANGER_STATUS_TYPE } from '@/constants/ros'
import { TOOL_LIST } from '@/constants/grasp'

const emit = defineEmits<{ log: [msg: string] }>()
function log(msg: string) { emit('log', msg) }

const { isConnected, subscribe, onRosJson, onControlJson, callService } = useRos()
const { call } = useRosService()
const settings = useDashboardSettings()
const toolStatusTopic = computed(() => settings.rosName('topic-tool-status', TOOL_CHANGER_STATUS_TOPIC))
const gripperSwapService = computed(() => settings.rosName('svc-gripper-swap', '/run_gripper_swap'))

const MANUAL_STORAGE_KEY = 'ivg_tool_manual_id'

// ═══════════════════════ 状态 ═══════════════════════

/** 是否收到过后端状态反馈 */
const toolStatusReceived = ref(false)
/** 工具是否已连接（来自后端消息或用户手动设置） */
const isToolConnected = ref(false)
/** 当前工具 ID */
const currentToolId = ref('')
/** 当前工具显示名 */
const currentToolName = ref('')
/** 用户是否完成了手动选择（用于区分「等待中」和「已确认无工具」） */
const manualSelectionDone = ref(false)

/** 是否应显示手动选择器 */
const showManualSelector = computed(() =>
  toolStatusReceived.value && !isToolConnected.value
)

/** 是否处于初始等待状态（连接了但还没收到任何反馈） */
const isWaitingForStatus = computed(() =>
  isConnected() && !toolStatusReceived.value
)

// ═══════════════════════ 从后端消息更新状态 ═══════════════════════

function applyBackendStatus(msg: { is_connected?: boolean; tool_id?: string; tool_name?: string }): void {
  toolStatusReceived.value = true
  const connected = !!msg?.is_connected
  if (connected && msg?.tool_id) {
    // 后端确认已连接 → 权威数据，完全覆盖（包括清除手动选择）喵~
    isToolConnected.value = true
    currentToolId.value = msg.tool_id
    currentToolName.value = msg.tool_name || msg.tool_id
    manualSelectionDone.value = false
  } else if (!manualSelectionDone.value) {
    // 后端确认未连接 且 用户未手动选择 → 应用后端状态喵~
    isToolConnected.value = false
    currentToolId.value = ''
    currentToolName.value = ''
  }
  // else: 后端确认未连接 但 用户已手动选择 → 保留手动状态，不被定时器覆盖喵~
}

// ═══════════════════════ 手动选择 ═══════════════════════

function setManualTool(toolId: string): void {
  manualSelectionDone.value = true
  if (toolId) {
    currentToolId.value = toolId
    currentToolName.value = TOOL_LIST.find(t => t.id === toolId)?.label || toolId
    isToolConnected.value = true
  } else {
    currentToolId.value = ''
    currentToolName.value = ''
    isToolConnected.value = false
  }
  try { localStorage.setItem(MANUAL_STORAGE_KEY, toolId) } catch { /* */ }
  log(`手动设定当前工具: ${toolId || '无工具'}`)
}

function clearManualSelection(): void {
  manualSelectionDone.value = false
  currentToolId.value = ''
  currentToolName.value = ''
  isToolConnected.value = false
  try { localStorage.removeItem(MANUAL_STORAGE_KEY) } catch { /* */ }
}

// ═══════════════════════ 话题消息处理 ═══════════════════════

onRosJson(null, (msg: any, topic: string) => {
  if (canonicalRosTopic(topic) !== canonicalRosTopic(toolStatusTopic.value)) return
  applyBackendStatus(msg)
})

// ═══════════════════════ 连接后初始化 ═══════════════════════

async function tryFetchCurrentTool(): Promise<void> {
  // 调用 /get_current_tool 服务立即获取当前状态（避免等待话题周期性推送）喵~
  try {
    const r: any = await callService('/get_current_tool', 'ivg_interfaces/srv/GetCurrentTool', {}, 3000)
    if (r) {
      applyBackendStatus({ is_connected: r.is_connected, tool_id: r.tool_id, tool_name: r.tool_name })
    }
  } catch {
    // 服务调用失败是预期情况（节点可能尚未完全就绪），等待话题推送即可喵~
    if (!toolStatusReceived.value) {
      // 若 8 秒内话题也未到达，显示手动选择器喵~
      setTimeout(() => {
        if (!toolStatusReceived.value) {
          toolStatusReceived.value = true // 触发手动选择器
        }
      }, 8000)
    }
  }
}

function setup(): void {
  if (!isConnected()) return
  subscribe(toolStatusTopic.value, settings.topicType('topic-tool-status', TOOL_CHANGER_STATUS_TYPE))
  tryFetchCurrentTool()
}

onControlJson((c) => { if (c.op === 'connection') setup() })
watch(isConnected, v => { if (v) setup() })
watch(() => settings.version.value, () => { if (isConnected()) setup() })
if (isConnected()) setup()

// ═══════════════════════ 快换执行 ═══════════════════════

/** 执行工具快换 */
async function doChange(targetId: string): Promise<void> {
  // 构建 direction 字符串喵~
  //   - 后端已知当前工具: "gripper0_to_gripper2" → 后端先释放当前再取目标
  //   - 手动选择/无当前工具: "gripper2"           → 后端跳过释放，直接取目标
  //     （手动选择时前端不声称知道当前工具，由后端的 current_tool_ 权威决策）喵~
  let direction: string
  if (!manualSelectionDone.value && currentToolId.value && currentToolId.value !== targetId) {
    direction = `${currentToolId.value}_to_${targetId}`
  } else if (currentToolId.value === targetId) {
    return // 同一工具，不操作
  } else {
    // 无当前工具 或 手动选择模式 → 直接指定目标喵~
    direction = targetId
  }

  log(`快换 ${direction}…`)
  try {
    const r = await call(
      gripperSwapService.value,
      settings.serviceType('svc-gripper-swap', 'ivg_interfaces/srv/RunGripperSwap'),
      { direction }
    )
    log(r.success ? `✓ ${r.message || '成功'}` : `✗ ${r.message || '失败'}`)
  } catch (e: any) {
    log(`✗ 错误: ${e}`)
  }
}

// ═══════════════════════ 挂载时恢复上次手动选择 ═══════════════════════

onMounted(() => {
  try {
    const saved = localStorage.getItem(MANUAL_STORAGE_KEY)
    if (saved !== null) {
      // 只恢复记忆，不立即生效 — 等后端状态确认后再决定是否显示
      // 如果后端确认 is_connected=false，此值会作为默认选项提示用户
    }
  } catch { /* */ }
})
</script>

<template>
  <div class="bg-white rounded-lg border border-slate-200 p-4">
    <h3 class="text-sm font-bold text-slate-700 mb-2">末端夹爪快换</h3>

    <!-- 当前工具指示 -->
    <div class="flex items-center gap-1.5 mb-2">
      <span
        class="w-2 h-2 rounded-full"
        :class="isToolConnected && currentToolId ? 'bg-green-500' : isWaitingForStatus ? 'bg-amber-400 animate-pulse' : 'bg-slate-300'"
      />
      <span class="text-xs text-slate-500">
        <template v-if="isWaitingForStatus">等待工具状态反馈…</template>
        <template v-else-if="isToolConnected && currentToolId">
          当前: {{ currentToolName || currentToolId }}
          <span v-if="manualSelectionDone" class="text-amber-500">(手动)</span>
        </template>
        <template v-else>当前: 无工具</template>
      </span>
    </div>

    <!-- 手动工具选择器（后端确认无工具时显示） -->
    <div
      v-if="showManualSelector"
      class="bg-amber-50 border border-amber-200 rounded-lg p-3 mb-3"
    >
      <p class="text-xs text-amber-700 mb-2 leading-relaxed">
        ⚠ 未检测到已安装的末端工具（末端夹爪类型无硬件反馈）。
        请手动选择当前末端实际安装的工具喵~
      </p>
      <div class="flex flex-wrap gap-1.5">
        <el-button
          v-for="tool in TOOL_LIST"
          :key="tool.id"
          size="small"
          :type="currentToolId === tool.id ? 'primary' : 'default'"
          @click="setManualTool(tool.id)"
        >
          {{ tool.label }}
        </el-button>
        <el-button
          size="small"
          :type="!currentToolId && manualSelectionDone ? 'primary' : 'default'"
          @click="setManualTool('')"
        >
          无工具
        </el-button>
      </div>
    </div>

    <!-- 快换按钮 -->
    <div class="flex flex-wrap gap-1.5">
      <el-button
        v-for="tool in TOOL_LIST"
        :key="tool.id"
        size="small"
        :type="currentToolId === tool.id ? 'primary' : 'default'"
        :disabled="currentToolId === tool.id || isWaitingForStatus"
        @click="doChange(tool.id)"
      >
        {{ tool.label }}
      </el-button>
    </div>
  </div>
</template>
