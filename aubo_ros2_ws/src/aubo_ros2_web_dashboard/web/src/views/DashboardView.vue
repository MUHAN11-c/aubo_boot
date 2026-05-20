<script setup lang="ts">
/**
 * DashboardView — 门户首页
 *
 * 替代旧版: index.html (88行)
 *
 * 页面结构:
 *   - Hero 区域: 标题 + 简介 + 标签
 *   - 快速入口: 两个主按钮 → 视觉抓取 / 咖啡拉花
 *   - 功能卡片: 视觉抓取说明 + 咖啡拉花说明
 *   - 页脚
 *
 * 外部链接 (VPE + 手眼标定) 使用当前主机名动态构建
 */
import { VPE_WEB_PORT, HAND_EYE_WEB_PORT } from '@/constants/ros'

const hostname = window.location.hostname || '127.0.0.1'
</script>

<template>
  <div class="max-w-3xl mx-auto px-4 py-12">
    <!-- ═══ Hero ═══ -->
    <header class="text-center mb-12">
      <p class="text-sm text-blue-600 font-medium mb-2">灵视 IVG</p>
      <h1 class="text-3xl font-bold text-slate-900 mb-3">机械臂操作门户</h1>
      <p class="text-slate-500 max-w-md mx-auto">
        用浏览器即可查看相机、抓取演示与运行数据。点下面按钮或顶部菜单进入，无需安装软件。
      </p>
      <div class="flex justify-center gap-2 mt-4">
        <el-tag size="small" round>浏览器即用</el-tag>
        <el-tag size="small" round>机械臂与相机</el-tag>
        <el-tag size="small" round type="warning">灵视IVG</el-tag>
      </div>
    </header>

    <!-- ═══ 快速入口 ═══ -->
    <nav class="flex justify-center gap-3 mb-12" aria-label="快速入口">
      <RouterLink to="/vision" class="inline-flex items-center px-5 py-2.5 bg-blue-600 text-white text-sm font-medium rounded-lg hover:bg-blue-700 transition-colors no-underline">
        视觉抓取面板
      </RouterLink>
      <RouterLink to="/latte" class="inline-flex items-center px-5 py-2.5 bg-blue-600 text-white text-sm font-medium rounded-lg hover:bg-blue-700 transition-colors no-underline">
        咖啡拉花
      </RouterLink>
    </nav>

    <!-- ═══ 功能说明 ═══ -->
    <div class="space-y-4">
      <p class="text-sm text-slate-400 text-center mb-4">以下为各功能说明，与上方「快速入口」对应。</p>

      <!-- 视觉抓取卡片 -->
      <section class="bg-white rounded-xl border border-slate-200 p-6">
        <h2 class="text-lg font-bold text-slate-900 mb-2">视觉抓取</h2>
        <p class="text-sm text-slate-500 mb-3">实时查看相机画面，配合机械臂完成抓取相关演示，并查看运行状态。</p>
        <ul class="text-sm text-slate-600 space-y-1 mb-4">
          <li><span class="font-medium text-slate-700">面板</span> 抓取流程与图像、曲线等集中在一页操作。</li>
          <li><span class="font-medium text-slate-700">视觉位姿</span> 独立网页工具，用于位姿相关配置与查看。</li>
          <li><span class="font-medium text-slate-700">手眼标定</span> 独立网页工具，用于相机与机械臂相对关系标定。</li>
        </ul>
        <div class="flex gap-2 flex-wrap">
          <RouterLink to="/vision" class="inline-flex items-center px-4 py-1.5 bg-blue-600 text-white text-xs font-medium rounded-md hover:bg-blue-700 transition-colors no-underline">视觉抓取面板</RouterLink>
          <a :href="`//${hostname}:${VPE_WEB_PORT}/`" target="_blank" rel="noopener" class="inline-flex items-center px-4 py-1.5 bg-blue-600 text-white text-xs font-medium rounded-md hover:bg-blue-700 transition-colors no-underline">打开视觉位姿网页</a>
          <a :href="`//${hostname}:${HAND_EYE_WEB_PORT}/`" target="_blank" rel="noopener" class="inline-flex items-center px-4 py-1.5 bg-white border border-slate-300 text-slate-700 text-xs font-medium rounded-md hover:bg-slate-50 transition-colors no-underline">打开手眼标定网页</a>
        </div>
      </section>

      <!-- 咖啡拉花卡片 -->
      <section class="bg-white rounded-xl border border-slate-200 p-6">
        <h2 class="text-lg font-bold text-slate-900 mb-2">
          咖啡拉花 <el-tag size="small" round type="warning">演示</el-tag>
        </h2>
        <p class="text-sm text-slate-500 mb-3">面向咖啡拉花场景的界面与流程示意：杯位、动作与预览等区域，便于后续接入真实设备与调度。</p>
        <RouterLink to="/latte" class="inline-flex items-center px-4 py-1.5 bg-blue-600 text-white text-xs font-medium rounded-md hover:bg-blue-700 transition-colors no-underline">打开咖啡拉花面板</RouterLink>
      </section>
    </div>

    <!-- ═══ 页脚 ═══ -->
    <footer class="text-center mt-12 text-xs text-slate-400">
      灵视 IVG · 机械臂操作门户
    </footer>
  </div>
</template>
