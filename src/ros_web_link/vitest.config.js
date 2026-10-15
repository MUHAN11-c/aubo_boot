// vitest.config.js — 浏览器通信层纯核单测（零 ROS 依赖，离线跑）
// 复用 aubo_ros2_web_dashboard 的 node_modules（本包不重复装依赖）：
//   ../ivg/aubo_ros2_web_dashboard/node_modules/.bin/vitest run --root .
import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    environment: 'node',
    include: ['web/public/js/**/*.test.js'],
  },
});
