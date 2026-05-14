/**
 * Vue Router 路由表 — 6 条路由，懒加载
 *
 * 路由映射:
 *   /           → DashboardView   (门户首页)
 *   /vision     → VisionGraspView (视觉抓取)
 *   /latte      → CoffeeLatteView (咖啡拉花)
 *   /monitor    → TfMonitorView   (TF 监控)
 *   /log        → LogView         (系统日志)
 *   /settings   → SettingsView    (设置)
 *
 * 设计决策: 使用 createWebHistory (干净的 URL 无 #)
 *   — 需要 FastAPI 网关在所有未匹配路径返回 index.html (SPA fallback)
 *   — scrollBehavior 确保页面切换时滚动到顶部
 */
import { createRouter, createWebHistory } from 'vue-router'

const routes = [
  { path: '/',         name: 'home',     component: () => import('@/views/DashboardView.vue') },
  { path: '/vision',   name: 'vision',   component: () => import('@/views/VisionGraspView.vue') },
  { path: '/latte',    name: 'latte',    component: () => import('@/views/CoffeeLatteView.vue') },
  { path: '/monitor',  name: 'monitor',  component: () => import('@/views/TfMonitorView.vue') },
  { path: '/log',      name: 'log',      component: () => import('@/views/LogView.vue') },
  { path: '/settings', name: 'settings', component: () => import('@/views/SettingsView.vue') },
]

const router = createRouter({
  history: createWebHistory(),
  routes,
  scrollBehavior() { return { top: 0 } },
})

export default router
