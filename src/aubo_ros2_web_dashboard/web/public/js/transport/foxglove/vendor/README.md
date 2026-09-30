# Foxglove CDR Vendor Bundle

浏览器端使用的 `@foxglove` 官方 CDR 包，通过 esbuild 打包为 ES module。

## 依赖包

| npm 包 | 用途 |
|--------|------|
| `@foxglove/rosmsg` | ros2msg schema 文本 → `MessageDefinition[]` |
| `@foxglove/rosmsg2-serialization` | `MessageReader` / `MessageWriter` — CDR 二进制 ↔ JS 对象 |
| `@foxglove/cdr` | `CdrReader` — CDR 二进制底层读取 |

## 构建

```bash
cd aubo_ros2_web_dashboard

# 首次安装
npm install

# 生成 vendor bundle
npm run build:vendor
```

输出: `foxglove_cdr.js`（约 170KB）

## 为什么需要打包

`@foxglove` 官方包仅提供 CommonJS 格式（`require()`），浏览器不支持。esbuild 将 CJS 转换为 ESM 并打包成单一文件。

## 版本更新

```bash
npm update @foxglove/rosmsg @foxglove/rosmsg2-serialization @foxglove/cdr
npm run build:vendor
```

## 前端导入

```js
// 直接相对路径导入, 无需 importmap
import { parse, MessageReader, MessageWriter } from './vendor/foxglove_cdr.js';
import { CdrReader } from './vendor/foxglove_cdr.js';
```
