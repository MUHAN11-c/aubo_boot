<script setup lang="ts">
/**
 * SiteNav — 全局导航栏
 *
 * 替代旧版: ivg_site_nav.js (122行 Web Component)
 *
 * 功能:
 *   - 6 个页面链接，当前页高亮
 *   - 品牌 logo "灵视 IVG" 链接到首页
 *
 * 数据流:
 *   useRoute().path → currentPage() → 匹配 navLinks → 高亮样式
 */
const route = useRoute()
const isFullscreen = ref(false)

/** 导航链接定义 — 与路由表保持一致 */
const navLinks = [
  { to: '/',         page: 'index',    label: '门户' },
  { to: '/vision',   page: 'vision',   label: '视觉抓取' },
  { to: '/latte',    page: 'latte',    label: '咖啡拉花' },
  { to: '/monitor',  page: 'monitor',  label: '监控面板' },
  { to: '/log',      page: 'log',      label: '日志' },
  { to: '/settings', page: 'settings', label: '设置' },
]

/** 从 URL 路径提取页面标识 (用于高亮匹配) */
function currentPage(path: string): string {
  if (path === '/') return 'index'
  return path.replace(/^\//, '').split('/')[0]
}

function fullscreenElement(): Element | null {
  const doc = document as Document & { webkitFullscreenElement?: Element | null }
  return document.fullscreenElement || doc.webkitFullscreenElement || null
}

async function toggleFullscreen(): Promise<void> {
  try {
    if (fullscreenElement()) {
      await document.exitFullscreen?.()
    } else {
      await document.documentElement.requestFullscreen?.()
    }
  } catch (e) {
    console.warn('[ivg-nav] 全屏:', e)
  }
}

function syncFullscreen(): void {
  isFullscreen.value = !!fullscreenElement()
}

onMounted(() => {
  document.addEventListener('fullscreenchange', syncFullscreen)
  document.addEventListener('webkitfullscreenchange', syncFullscreen)
  syncFullscreen()
})

onUnmounted(() => {
  document.removeEventListener('fullscreenchange', syncFullscreen)
  document.removeEventListener('webkitfullscreenchange', syncFullscreen)
})
</script>

<template>
  <nav class="bg-white border-b border-slate-200 shadow-sm" aria-label="灵视IVG 站内导航">
    <div class="w-full max-w-none px-3 flex items-center h-12 overflow-x-auto">
      <!-- 品牌 (点击回首页) -->
      <RouterLink to="/" class="text-base font-bold text-slate-900 mr-6 shrink-0">
        灵视 <span class="text-blue-600">IVG</span>
      </RouterLink>

      <!-- 页面链接 -->
      <ul class="flex items-center gap-0.5 shrink-0">
        <li v-for="link in navLinks" :key="link.page">
          <RouterLink
            :to="link.to"
            class="px-3 py-2 min-h-11 text-sm rounded-md transition-colors inline-flex items-center"
            :class="currentPage(route.path) === link.page
              ? 'bg-blue-50 text-blue-700 font-medium'
              : 'text-slate-600 hover:text-slate-900 hover:bg-slate-100'"
          >
            {{ link.label }}
          </RouterLink>
        </li>
      </ul>
      <button
        type="button"
        class="ml-auto px-3 py-2 min-h-11 text-sm rounded-md border border-slate-200 text-slate-600 hover:text-slate-900 hover:bg-slate-100 shrink-0"
        :aria-pressed="isFullscreen ? 'true' : 'false'"
        :aria-label="isFullscreen ? '退出全屏' : '进入全屏'"
        :title="isFullscreen ? '退出全屏' : '全屏显示页面（iPad：若无效请用 Safari 添加到主屏幕）'"
        @click="toggleFullscreen"
      >
        {{ isFullscreen ? '退出全屏' : '全屏' }}
      </button>
    </div>
  </nav>
</template>
