#!/usr/bin/env node
/**
 * Vendor @foxglove 官方 CDR 包 → 浏览器兼容 ESM bundle
 *
 * 用法:
 *   node scripts/build_vendor.mjs
 *   npm run build:vendor
 *
 * 输出:
 *   web/public/js/transport/foxglove/vendor/foxglove_cdr.js
 *
 * 打包内容:
 *   @foxglove/rosmsg              → parse()         (ros2msg schema 解析)
 *   @foxglove/rosmsg2-serialization → MessageReader / MessageWriter  (CDR 编解码)
 *   @foxglove/cdr                 → CdrReader       (CDR 二进制读写)
 *
 * 为什么需要打包:
 *   @foxglove 包是 CommonJS 格式 (require())，浏览器不支持。
 *   esbuild 将 CJS 转换为 ESM 并打包成单一文件，浏览器可直接 import。
 *
 * 依赖更新:
 *   npm update @foxglove/rosmsg @foxglove/rosmsg2-serialization @foxglove/cdr
 *   npm run build:vendor
 */

import * as esbuild from "esbuild";
import { fileURLToPath } from "node:url";
import path from "node:path";
import fs from "node:fs";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const rootDir = path.dirname(__dirname);
const outDir = path.join(rootDir, "web", "public", "js", "transport", "foxglove", "vendor");
const entryFile = path.join(__dirname, "vendor_entry.mjs");
const outFile = path.join(outDir, "foxglove_cdr.js");

// 确保输出目录存在
fs.mkdirSync(outDir, { recursive: true });

const t0 = performance.now();

await esbuild.build({
  entryPoints: [entryFile],
  bundle: true,
  format: "esm",
  platform: "browser",
  target: "es2020",
  outfile: outFile,
  banner: {
    js: [
      "// Foxglove CDR vendor bundle",
      "// 内容: @foxglove/rosmsg + @foxglove/rosmsg2-serialization + @foxglove/cdr",
      "// 自动生成 — 重新生成: node scripts/build_vendor.mjs",
      "",
    ].join("\n"),
  },
  external: [],
  minify: false,
  sourcemap: false,
});

const elapsed = (performance.now() - t0).toFixed(0);
const sizeKB = (fs.statSync(outFile).size / 1024).toFixed(0);

console.log(`✓ Vendor bundle built → ${path.relative(rootDir, outFile)} (${sizeKB}KB, ${elapsed}ms)`);
