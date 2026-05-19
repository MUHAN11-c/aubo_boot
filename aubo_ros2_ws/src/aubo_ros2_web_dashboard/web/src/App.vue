<script setup lang="ts">
/**
 * App.vue — 应用根布局
 *
 * 组件树:
 *   App
 *   ├── SiteNav           ← 全局导航栏 (6个页面链接)
 *   ├── RouterView        ← 当前路由的页面内容
 *   └── RobotStatusBar    ← 底部机械臂状态栏 (固定定位)
 *
 * 布局说明:
 *   - 顶部: SiteNav (h-12 = 3rem)
 *   - 中间: RouterView (min-height = 100vh - 8rem, 留出上下空间)
 *   - 底部: RobotStatusBar (h-8 = 2rem, fixed)
 *
 * 全局 ROS 连接: App 挂载时自动建立 rosbridge 连接（单例共享），
 * 所有子页面通过 useRos() 共享同一条连接。
 */
import { useRos } from '@/composables/ros/useRos'

const { connect, isConnected } = useRos()

onMounted(() => {
  if (!isConnected()) connect().catch(() => { /* 首次连接失败由 useRos 内部自动重连 */ })
})
</script>

<template>
  <SiteNav />
  <main class="flex-1 min-h-0 pb-[calc(2rem+env(safe-area-inset-bottom,0px))] overflow-x-hidden">
    <RouterView />
  </main>
  <RobotStatusBar />
</template>
