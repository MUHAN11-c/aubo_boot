<script setup lang="ts">
/**
 * GraspControls — 抓取控制面板
 *
 * 根据当前模式 (工件/AI) 显示不同的控制按钮组
 * 工件模式: 单次抓取 + 循环抓取 + 工件编号输入
 * AI 模式:  采集开关 + 循环发布 + 抓取位姿显示
 */
import { useRosService } from '@/composables/ros/useRosService'
import { useDashboardSettings } from '@/composables/settings/useDashboardSettings'

const props = defineProps<{
  mode: 'workpiece' | 'graspnet'
  objectId: string
  graspPoseHtml?: string
}>()

const emit = defineEmits<{
  'update:objectId': [value: string]
  log: [msg: string]
}>()

const { call, callSetBool } = useRosService()
const settings = useDashboardSettings()
const loopGraspService = computed(() => settings.rosName('svc-loop-grasp-control', '/loop_grasp_control'))
const graspnetCaptureService = computed(() => settings.rosName('svc-graspnet-capture', '/graspnet_capture_control'))
const publishGraspsLoopService = computed(() => settings.rosName('svc-publish-grasps-loop', '/publish_grasps_worker_loop_control'))
const executeGraspService = computed(() => settings.rosName('svc-execute-single-grasp', '/execute_single_grasp'))

function log(msg: string) { emit('log', msg) }

// ── 工件模式操作 ──

async function doSingleGrasp() {
  log('执行单次抓取…')
  try {
    const r = await call(executeGraspService.value, settings.serviceType('svc-execute-single-grasp', 'ivg_interfaces/srv/ExecuteGraspPose'), { object_id: props.objectId, use_visual_estimation: true })
    log(`✓ ${r.success ? '成功' : '失败'} ${r.message || ''}`)
  }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}

async function doLoopGrasp(start: boolean) {
  log(start ? '启动循环抓取…' : '停止循环')
  try { await callSetBool(loopGraspService.value, start); log(start ? '✓ 循环已启动' : '✓ 已停止') }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}

// ── AI 模式操作 ──

async function doCapture(start: boolean) {
  try { await callSetBool(graspnetCaptureService.value, start); log(start ? '✓ 采集开始' : '✓ 采集停止') }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}

async function doGraspnetLoop(start: boolean) {
  try { await callSetBool(publishGraspsLoopService.value, start); log(start ? '✓ 循环开启' : '✓ 循环关闭') }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}
</script>

<template>
  <div class="bg-white rounded-lg border border-slate-200 p-4">
    <!-- 工件模式 -->
    <template v-if="mode === 'workpiece'">
      <h3 class="text-sm font-bold text-slate-700 mb-3">工件抓取</h3>
      <div class="mb-2">
        <label class="text-xs text-slate-500">目标工件编号</label>
        <input
          :value="objectId"
          class="w-full text-sm border border-slate-300 rounded px-2 py-1.5 mt-1"
          placeholder="与现场 object_id 一致"
          @input="emit('update:objectId', ($event.target as HTMLInputElement).value)"
        />
      </div>
      <div class="flex flex-wrap gap-1.5">
        <el-button size="small" type="primary" @click="doSingleGrasp">执行单次抓取</el-button>
        <el-button size="small" type="success" @click="doLoopGrasp(true)">启动循环</el-button>
        <el-button size="small" type="danger" @click="doLoopGrasp(false)">停循环</el-button>
      </div>
    </template>

    <!-- AI 大模型模式 -->
    <template v-else>
      <h3 class="text-sm font-bold text-slate-700 mb-3">AI 大模型抓取</h3>
      <div class="flex flex-wrap gap-1.5">
        <el-button size="small" type="primary" @click="doCapture(true)">开始采集</el-button>
        <el-button size="small" type="danger" @click="doCapture(false)">停止采集</el-button>
        <el-button size="small" type="success" @click="doGraspnetLoop(true)">循环抓取开</el-button>
        <el-button size="small" type="warning" @click="doGraspnetLoop(false)">循环抓取关</el-button>
      </div>
      <!-- AI 抓取位姿显示 -->
      <div v-if="graspPoseHtml" class="mt-3 p-2 bg-slate-50 rounded text-xs" v-html="graspPoseHtml" />
    </template>
  </div>
</template>
