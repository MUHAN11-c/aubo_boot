// vitest.config.js — 浏览器通信层纯核单测（零 ROS 依赖，离线跑）
// 本包自包含 devDependencies（vitest 4.1.7）：npm install 后 `npm test`。
// 独立包边界：不再通过 symlink 复用 aubo_ros2_web_dashboard 的 node_modules。
import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    environment: 'node',
    include: ['web/public/js/**/*.test.js'],
  },
});
