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
</script>

<template>
  <nav class="bg-white border-b border-slate-200 shadow-sm" aria-label="灵视IVG 站内导航">
    <div class="max-w-7xl mx-auto px-4 flex items-center h-12">
      <!-- 品牌 (点击回首页) -->
      <RouterLink to="/" class="text-base font-bold text-slate-900 mr-6 shrink-0">
        灵视 <span class="text-blue-600">IVG</span>
      </RouterLink>

      <!-- 页面链接 -->
      <ul class="flex items-center gap-0.5">
        <li v-for="link in navLinks" :key="link.page">
          <RouterLink
            :to="link.to"
            class="px-3 py-1.5 text-sm rounded-md transition-colors"
            :class="currentPage(route.path) === link.page
              ? 'bg-blue-50 text-blue-700 font-medium'
              : 'text-slate-600 hover:text-slate-900 hover:bg-slate-100'"
          >
            {{ link.label }}
          </RouterLink>
        </li>
      </ul>
    </div>
  </nav>
</template>
