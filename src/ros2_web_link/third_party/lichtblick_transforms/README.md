# Lichtblick transforms 源码

本目录直接来自 [Lichtblick Suite v1.30.0](https://github.com/lichtblick-suite/lichtblick/tree/v1.30.0)，固定 commit `fcfc7d559d8e76db36c9730e78049a60f976c2d2`。

- `TransformTree.ts`、`CoordinateFrame.ts`、`Transform.ts`、`geometry.ts`、`time.ts`：上游 `packages/suite-base/src/panels/ThreeDeeRender/transforms/`。
- `den/ArrayMap.ts`、`den/ObjectPool.ts`：上游 `packages/den/collection/`，为上述 TF 实现实际使用的运行时依赖。
- 唯一源码修改是把 workspace alias 和相对导入改为本目录的 `.js` 相对路径；变换树、插值和几何算法均保持上游实现。未复制测试、整个 den 导出入口或未使用模块。

所有 TypeScript 源文件保留上游 MPL-2.0 及版权文件头；`LICENSE` 为该版本上游许可证全文。本目录仍按 MPL-2.0 提供源码，并随本包发布；本包的 Apache-2.0 声明不覆盖这些文件。`gl-matrix` 由 npm 固定版本提供。

上游默认允许超出历史时间范围的查询夹到首末样本，静态 TF 也没有独立类型。`scene/tf.ts` 只负责 ROS 消息/QoS、容量、静态标记、显式 `/clock` 回退和严格时间范围检查；数值缓存、变换组合及插值仍由这里的上游实现负责。
