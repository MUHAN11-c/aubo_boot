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
 */
</script>

<template>
  <SiteNav />
  <main class="min-h-[calc(100vh-8rem)]">
    <RouterView />
  </main>
  <RobotStatusBar />
</template>
