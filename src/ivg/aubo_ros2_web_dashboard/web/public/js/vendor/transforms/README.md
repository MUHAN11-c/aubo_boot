# transforms vendor — 浏览器端 TF 树（L4）

自 Lichtblick（BMW，MPL-2.0）移植的 TransformTree/CoordinateFrame/Transform/geometry/time，
含其依赖 den/collection（ArrayMap/ObjectPool）与 gl-matrix 3.4.3（MIT）。

## 文件

- `tf_tree_bundle.mjs` — esbuild 打包的单文件 ESM（浏览器直接 import，零构建架构）
- `src/transforms/`、`src/den/` — 上游 TypeScript 源（**MPL-2.0 文件级版权头原样保留**；
  唯一改动：`@lichtblick/den/collection*` import 重映射到本地 `../den/*`，见各文件头注释）
- `entry.ts` / `package.json` / `do_build.sh` — 打包工程（node + esbuild 0.24）

## 来源锚点

- 上游仓库：github.com/lichtblick-suite/lichtblick（Foxglove Studio 开源延续 fork）
- 路径：`packages/suite-base/src/panels/ThreeDeeRender/transforms/`、`packages/den/collection/`
- 取样：main 分支浅克隆（2026-10-10）；许可 MPL-2.0（原文见各源文件头）

## 重建

```bash
bash do_build.sh   # 在本目录外任意处执行亦可（脚本内固定 /tmp/transforms_build）
```
上游演进后：替换 src/ 下对应文件 → 重跑 do_build.sh → 同轮更新本 README 取样日期。

## 能力（浏览器端 tf2 等价物）

按 frame 时间有序变换数组 + 二分查找 + 区间插值；latest/exact 双查询模式 + maxDelta 容差；
环检测；换父清史；容量上限 + 摊销驱逐。接口细节见 `src/transforms/TransformTree.ts` 注释。
