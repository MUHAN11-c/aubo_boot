import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import tailwindcss from '@tailwindcss/vite'
import AutoImport from 'unplugin-auto-import/vite'
import Components from 'unplugin-vue-components/vite'
import { ElementPlusResolver } from 'unplugin-vue-components/resolvers'
import { fileURLToPath, URL } from 'node:url'

export default defineConfig({
  plugins: [
    vue(),
    tailwindcss(),
    AutoImport({
      imports: ['vue', 'vue-router', '@vueuse/core'],
      resolvers: [ElementPlusResolver()],
      dts: 'src/types/auto-imports.d.ts',
    }),
    Components({
      resolvers: [ElementPlusResolver()],
      dts: 'src/types/components.d.ts',
    }),
  ],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
      // robotwebtools 本地源码编译产物 — 方便随时修改源码+重新编译
      roslib: fileURLToPath(new URL(
        '../../robotwebtools/roslibjs/packages/roslib/dist/RosLib.js',
        import.meta.url
      )),
      ros3d: fileURLToPath(new URL(
        '../../robotwebtools/ros3djs/build/ros3d.esm.js',
        import.meta.url
      )),
    },
  },
  server: {
    port: 5173,
    watch: {
      // 监视 robotwebtools 编译产物 — 修改 ros3d/roslib 源码后 rebuild，Vite 自动热更新
      ignored: ['!**/robotwebtools/**/build/**', '!**/robotwebtools/**/dist/**'],
    },
    proxy: {
      '/api': 'http://127.0.0.1:8090',
      '/ws': {
        target: 'ws://127.0.0.1:8090',
        ws: true,
      },
    },
  },
})
