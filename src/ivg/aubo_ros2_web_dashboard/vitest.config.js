import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['web/public/js/**/*.test.js'],
    environment: 'node',
  },
});
