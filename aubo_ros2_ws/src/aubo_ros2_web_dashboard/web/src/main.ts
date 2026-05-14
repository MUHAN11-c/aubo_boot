/**
 * main.ts — Vue 3 应用入口
 *
 * 初始化顺序:
 *   1. createApp(App)     — 创建根组件
 *   2. createPinia()      — 全局状态管理 (备用)
 *   3. router             — Vue Router (6 条路由)
 *   4. Element Plus CSS   — UI 组件库样式
 *   5. Tailwind v4 base   — 原子 CSS 基础层
 *   6. mount('#app')      — 挂载到 index.html 的 <div id="app">
 */
import { createApp } from 'vue'
import { createPinia } from 'pinia'
import App from './App.vue'
import router from './router'
import 'element-plus/dist/index.css'
import './styles/base.css'

const app = createApp(App)
app.use(createPinia())
app.use(router)
app.mount('#app')
