<script setup lang="ts">
/**
 * SettingsView — 系统设置页
 *
 * 替代旧版: settings_panel.html + settings_panel.js (214行)
 *
 * 功能:
 *   - 网关状态显示 (从 /api/v1/runtime BFF 获取)
 *   - 三分类标签页 (公共参数 / 视觉抓取 / 咖啡拉花)
 *   - 动态表单 (根据 BFF 返回的 settings_categories 自动生成)
 *   - 保存: localStorage + POST /api/v1/settings (写入 YAML)
 *   - 导出/导入 JSON
 *
 * 数据流:
 *   GET /api/v1/runtime → settings_categories → 生成表单
 *   localStorage ←→ 表单值 (离线缓存)
 *   POST /api/v1/settings → YAML 持久化
 */
const STORAGE_KEY = 'ivg_vision_grasp_topics_v3'

interface SettingItem { id: string; label: string; default: string; allow_empty?: boolean; msg_type?: string; srv_type?: string }
interface CategoryItems { topics?: SettingItem[]; tf_topics?: SettingItem[]; services?: SettingItem[] }

const gatewayInfo = ref<Record<string, unknown>>({})
const gatewayOk = ref(false)
const categoryDefs = ref<Record<string, CategoryItems>>({})
const formValues = ref<Record<string, string>>({})
const activeTab = ref('common')
const message = ref(''); const messageOk = ref(true)

/** 页面加载: 从 BFF 获取配置定义 + 从 localStorage 恢复已保存值 */
onMounted(async () => {
  try { const r = await fetch('/api/v1/runtime', { credentials: 'same-origin' }); if (r.ok) { const data = await r.json(); gatewayInfo.value = data; gatewayOk.value = true; if (data.settings_categories) categoryDefs.value = data.settings_categories } } catch { gatewayOk.value = false }
  try { const raw = localStorage.getItem(STORAGE_KEY); if (raw) formValues.value = JSON.parse(raw) } catch { /* ignore */ }
})

function showMsg(text: string, ok: boolean) { message.value = text; messageOk.value = ok; if (ok) setTimeout(() => { message.value = '' }, 4000) }

/** 获取当前标签页的表单分区 (topics / tf_topics / services) */
function getItems(tab: string): Array<{ items: SettingItem[]; typeLabel: string }> {
  const cat = categoryDefs.value[tab]; if (!cat) return []
  const result: Array<{ items: SettingItem[]; typeLabel: string }> = []
  if (cat.topics?.length) result.push({ items: cat.topics, typeLabel: 'ROS 话题' })
  if (cat.tf_topics?.length) result.push({ items: cat.tf_topics, typeLabel: 'TF 话题' })
  if (cat.services?.length) result.push({ items: cat.services, typeLabel: 'ROS 服务' })
  return result
}

function fieldDefault(id: string): string { for (const tab of ['common', 'vision', 'latte']) { for (const s of getItems(tab)) { const item = s.items.find(i => i.id === id); if (item) return item.default } }; return '' }
function isChanged(id: string): boolean { const v = formValues.value[id] ?? ''; return v !== '' && v !== fieldDefault(id) }
function readAll(): Record<string, string> { return { ...formValues.value } }

async function handleSave() {
  const values = readAll(); try { localStorage.setItem(STORAGE_KEY, JSON.stringify(values)) } catch { /* */ }
  try { const r = await fetch('/api/v1/settings', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(values), credentials: 'same-origin' }); showMsg(r.ok ? '已保存到 YAML 配置（重启网关后仍有效）。' : '已保存到浏览器本地（网关不可达，未写入 YAML）。', r.ok) }
  catch { showMsg('已保存到浏览器本地（网关不可达）。', false) }
}
function handleReset() { formValues.value = {}; showMsg('已恢复默认值，请点击保存以持久化。', false) }
function handleClear() { try { localStorage.removeItem(STORAGE_KEY) } catch { /* */ }; formValues.value = {}; showMsg('已清除。', true) }
function handleExport() { const json = JSON.stringify(readAll(), null, 2); const b = new Blob([json], { type: 'application/json' }); const a = document.createElement('a'); a.href = URL.createObjectURL(b); a.download = `ivg_settings_${new Date().toISOString().slice(0, 10)}.json`; a.click(); URL.revokeObjectURL(a.href); showMsg('已导出。', true) }
function handleImport() { const inp = document.createElement('input'); inp.type = 'file'; inp.accept = '.json'; inp.onchange = () => { const f = inp.files?.[0]; if (!f) return; const r = new FileReader(); r.onload = () => { try { const obj = JSON.parse(r.result as string); if (!obj || typeof obj !== 'object') throw new Error('格式错误'); formValues.value = obj; showMsg(`已导入 ${Object.keys(obj).length} 项，请点击保存。`, false) } catch (e: any) { showMsg('导入失败: ' + e.message, false) } }; r.readAsText(f) }; inp.click() }

const tabs = ['common', 'vision', 'latte'] as const
const tabLabels: Record<string, string> = { common: '公共参数', vision: '视觉抓取', latte: '咖啡拉花' }
</script>

<template>
  <div class="max-w-4xl mx-auto px-4 py-8">
    <h1 class="text-2xl font-bold text-slate-900 mb-1">IVG 系统设置</h1>
    <p class="text-sm text-slate-500 mb-6">配置修改后保存即生效（浏览器本地存储）。</p>

    <!-- 网关状态 -->
    <div class="bg-white rounded-lg border border-slate-200 p-4 mb-6">
      <div class="flex items-center gap-2 mb-2">
        <span class="text-sm font-medium text-slate-700">网关</span>
        <el-tag :type="gatewayOk ? 'success' : 'danger'" size="small">{{ gatewayOk ? '已连接' : '不可达' }}</el-tag>
      </div>
      <div v-if="gatewayOk" class="text-xs text-slate-500 flex flex-wrap gap-x-4 gap-y-1">
        <span>版本: <code>{{ gatewayInfo.version }}</code></span>
        <span>rosbridge: <code>{{ gatewayInfo.rosbridge_port }}</code></span>
        <span>web_video: <code>{{ gatewayInfo.web_video_port }}</code></span>
      </div>
    </div>

    <!-- 分类标签 -->
    <div class="flex gap-1 mb-4">
      <button v-for="tab in tabs" :key="tab" class="px-4 py-2 text-sm rounded-md transition-colors border" :class="activeTab === tab ? 'bg-blue-600 text-white border-blue-600' : 'bg-white text-slate-600 border-slate-200 hover:bg-slate-50'" @click="activeTab = tab">{{ tabLabels[tab] }}</button>
    </div>

    <!-- 表单区域 -->
    <div v-for="tab in tabs" :key="tab" v-show="activeTab === tab">
      <div v-for="(section, si) in getItems(tab)" :key="si" class="mb-6">
        <h3 class="text-sm font-bold text-slate-700 mb-3">{{ section.typeLabel }}</h3>
        <div class="grid grid-cols-1 md:grid-cols-2 gap-3">
          <div v-for="item in section.items" :key="item.id" class="flex flex-col gap-1 p-3 bg-white rounded-md border" :class="isChanged(item.id) ? 'border-amber-300 bg-amber-50/30' : 'border-slate-200'">
            <label :for="'cfg-' + item.id" class="text-xs text-slate-600 flex items-center gap-2">
              {{ item.label || item.id }}
              <el-tag v-if="isChanged(item.id)" size="small" type="warning" round>已修改</el-tag>
              <span v-if="item.msg_type" class="text-slate-400 text-xs">({{ item.msg_type }})</span>
              <span v-if="item.srv_type" class="text-slate-400 text-xs">({{ item.srv_type }})</span>
            </label>
            <input :id="'cfg-' + item.id" type="text" :value="formValues[item.id] ?? ''" :placeholder="item.allow_empty ? '留空禁用' : item.default" class="text-sm border border-slate-300 rounded px-2 py-1.5 focus:outline-none focus:border-blue-400" @input="formValues = { ...formValues, [item.id]: ($event.target as HTMLInputElement).value }" />
          </div>
        </div>
      </div>
      <p v-if="getItems(tab).length === 0" class="text-sm text-slate-400">暂无配置项（等待网关数据…）</p>
    </div>

    <!-- 操作 -->
    <div class="flex flex-wrap gap-2 mt-6"><el-button type="primary" @click="handleSave">保存设置</el-button><el-button @click="handleReset">恢复默认</el-button><el-button @click="handleClear">清除存储</el-button><el-button @click="handleExport">导出 JSON</el-button><el-button @click="handleImport">导入 JSON</el-button></div>
    <p v-if="message" class="mt-2 text-sm" :class="messageOk ? 'text-green-600' : 'text-red-500'">{{ message }}</p>
  </div>
</template>
