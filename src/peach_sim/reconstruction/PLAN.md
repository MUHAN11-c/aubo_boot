# 数据驱动 Blender 重建执行记录

依据：2026-09-24 用户批准的从真实数据重建方案；旧外观、统计与布局不作为输入。

目标：可编辑 Blender 场景，真实 RGB-D 支撑的局部枝果几何、完整树行环境、渲染及检测分割核验。
约束：离线；驱动只读；numpy 1.26.4；不启动真机；运行结束清进程。
技术：独立 Blender 4.5.14 Python/bpy；项目 venv 的 Pillow/numpy/Ultralytics YOLO+MobileSAM。

- [ ] 采样 RGB/Depth/IR/VOC，记录源、深度有效性、相机近似；以原模型分割恢复局部尺度。
- [ ] 从零建纸袋、独立叶片、连通枝条，保存局部及整园 Blender 场景。
- [ ] 渲染多机位 RGB/深度/实例 ID；检测分割对照；检查连接、穿插和轮廓。
- [ ] 保存证据与命令，更新包入口说明，清理进程。

关键校验：深度零洞不填为背景；公开 FOV 不是逐机标定；遮挡实例不使用整框背景深度；纸袋闭合、果实在内；叶片和挂果枝必须连接父枝；轴向深度与射线距离不可混用。

裁定：当前有大量用户未提交修改，新增 reconstruction 作为全新 Blender 入口，不覆盖旧生成物；包 README 改为新入口并明确旧入口未验收。无需重新请求已经批准的离线建模权限。
来源：https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset （RGB 对齐、90°×59°）；https://eorganic.org/node/25727 （袋口绕枝、纸袋收拢固定）；https://extension.uga.edu/publications/detail.html?number=C1087 （开心形骨架）；Blender 4.5 bpy/Passes 文档；项目 inference.py 的 Ultralytics SAM 用法。
