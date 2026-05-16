/**
 * useDashboardSettings — Dashboard ROS 话题/服务设置读取层
 *
 * 旧版页面通过 getSetting() 从 DOM/localStorage 读取实际订阅目标。
 * Vue 迁移后由本 composable 统一合并 runtime YAML 默认值与浏览器覆盖值。
 */
import { useRuntime } from './useRuntime'
import { VISION_SETTING_DEFS, FIXED_SERVICE_TYPES } from '@/constants/grasp'

const STORAGE_KEY = 'ivg_vision_grasp_topics_v3'

interface RuntimeSettingItem {
  id: string
  default?: string
  msg_type?: string
  srv_type?: string
}

interface RuntimeCategory {
  topics?: RuntimeSettingItem[]
  tf_topics?: RuntimeSettingItem[]
  services?: RuntimeSettingItem[]
  fixed_service_types?: Record<string, string>
}

const defaults = ref<Record<string, string>>(Object.fromEntries(VISION_SETTING_DEFS.map(d => [d.id, d.defaultValue])))
const topicTypes = ref<Record<string, string>>(Object.fromEntries(VISION_SETTING_DEFS.filter(d => d.msgType).map(d => [d.id, d.msgType!])))
const serviceTypes = ref<Record<string, string>>({
  ...Object.fromEntries(VISION_SETTING_DEFS.filter(d => d.serviceType).map(d => [d.id, d.serviceType!])),
  ...FIXED_SERVICE_TYPES,
})
const overrides = ref<Record<string, string>>({})
const loaded = ref(false)
const version = ref(0)
let loadPromise: Promise<void> | null = null

function normalizeRosName(raw: string, fallback = ''): string {
  const val = String(raw || fallback || '').trim()
  if (!val) return ''
  if (val.includes('__ivg_disabled')) return fallback
  return val.startsWith('/') ? val : `/${val}`
}

function readOverrides(): Record<string, string> {
  try {
    const raw = localStorage.getItem(STORAGE_KEY)
    if (!raw) return {}
    const obj = JSON.parse(raw)
    return obj && typeof obj === 'object' ? obj as Record<string, string> : {}
  } catch {
    return {}
  }
}

function absorbItem(item: RuntimeSettingItem): void {
  if (!item.id) return
  if (item.default != null) defaults.value[item.id] = String(item.default)
  if (item.msg_type) topicTypes.value[item.id] = String(item.msg_type)
  if (item.srv_type) serviceTypes.value[item.id] = String(item.srv_type)
}

function absorbRuntimeCategories(categories: Record<string, RuntimeCategory> | undefined): void {
  if (!categories) return
  for (const cat of Object.values(categories)) {
    for (const item of cat.topics ?? []) absorbItem(item)
    for (const item of cat.tf_topics ?? []) absorbItem(item)
    for (const item of cat.services ?? []) absorbItem(item)
    Object.assign(serviceTypes.value, cat.fixed_service_types ?? {})
  }
}

async function loadSettings(): Promise<void> {
  if (loadPromise) return loadPromise
  const { load, config } = useRuntime()
  loadPromise = (async () => {
    const runtime = await load()
    absorbRuntimeCategories((runtime as any)?.settings_categories ?? (config.value as any)?.settings_categories)
    overrides.value = readOverrides()
    loaded.value = true
    version.value++
  })().finally(() => { loadPromise = null })
  return loadPromise
}

if (typeof window !== 'undefined') {
  window.addEventListener('storage', (event) => {
    if (event.key !== STORAGE_KEY) return
    overrides.value = readOverrides()
    version.value++
  })
  window.addEventListener('ivg-dashboard-settings-changed', () => {
    overrides.value = readOverrides()
    version.value++
  })
}

export function useDashboardSettings() {
  if (!loaded.value && !loadPromise) void loadSettings()

  function raw(id: string, fallback = ''): string {
    const override = overrides.value[id]
    if (override != null && String(override).trim() !== '') return String(override)
    return defaults.value[id] ?? fallback
  }

  function rosName(id: string, fallback = ''): string {
    return normalizeRosName(raw(id, fallback), defaults.value[id] ?? fallback)
  }

  function optionalRosName(id: string, fallback = ''): string {
    const val = raw(id, fallback).trim()
    return val ? normalizeRosName(val, fallback) : ''
  }

  function topicType(id: string, fallback = ''): string {
    return topicTypes.value[id] ?? fallback
  }

  function serviceType(id: string, fallback = ''): string {
    return serviceTypes.value[id] ?? fallback
  }

  return {
    loaded: readonly(loaded),
    version: readonly(version),
    loadSettings,
    raw,
    rosName,
    optionalRosName,
    topicType,
    serviceType,
  }
}
