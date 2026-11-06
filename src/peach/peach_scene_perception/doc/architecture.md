# peach_scene_perception 架构

## 职责与入口

`ScenePerceptionNode` 的图名和 console script 均为 `peach_scene_perception_node`。独立入口是 `launch/scene_perception.launch.py`；整栈也可由 `peach_harvester` 的 brain 进程装配同一节点类。只有 Active 时处理 RGB-D 和受理 BeginScene；configure 创建 ROS 实体并预热模型，cleanup/shutdown 停止 worker 后释放实体。诊断 Updater 在进程内复用。

本节点产出观测与几何初值，不选择下一颗执行目标、不控制机械臂。产品构造 `PerceptionPipeline(..., enable_fruit=False)`，在分割前移除 class_id=1 的裸果检测；裸果算法类仍存在，不能据此声称默认产品会处理裸果。

## 源码地图

- `scene_perception_node.py`：生命周期、同步订阅、图像解码、TF、发布、BeginScene 和事件落盘编排。
- `image_codec.py`：BGR8/16UC1 的布局检查与 NumPy 视图；其它编码在节点通过 cv_bridge 转换。`motion_gate.py` 判断当前速度是否停稳。
- `pipeline.py`：推理、位姿、身份与结果组装门面；不持有 Node，但会构造 ROS 消息值对象。`inference.py` 包含 YOLO、FastSAM/SAM、框去重和候选估计。
- `pose_pipelines.py`：袋圆柱/关键点与裸果球拟合管线；`contracts.py` 是内部观测、几何、匹配和锁窗值对象。`image_gates.py` 处理投影、分割框与深度前景。
- `identity.py`：χ² 门、匈牙利分配、注册表 EMA、确认、收集锁窗、优先级与记忆锚点；`plan_updater.py` 将一帧记录推进为观测组装材料。
- `msg_builders.py` 构造 ROS 候选、拟合诊断、点云与 Marker；`debug_draw.py` 绘制像素叠加；`stream_metrics.py` 统计帧率、耗时、光照质量和自适应窗口。
- `params.py` 包装 GPL listener 并补充跨字段校验；`config/scene_perception_parameters.yaml` 是声明 schema，`scene_perception_parameters.py` 由 setup 生成。部署值、Orin 覆盖与 percipio 年龄门分别在 `scene_perception.yaml`、`scene_perception_orin.yaml`、`percipio_frontend.yaml`。
- `setup.py`/`package.xml` 定义安装、依赖和入口；`resource/`、`setup.cfg` 是 ament 索引与脚本位置配置。`model/` 保存检测、默认分割和显式回退权重。

## 一帧如何处理

1. RGB、深度和 CameraInfo 近似同步。入口先做帧龄检查；原图预览可绕过运动推理门，推理在当前关节速度判停稳时提交给容量 1、满时替换旧帧的 worker。尚无速度消息时先允许检测，此门不能代替重建的曝光静止证据。
2. 解码并归一化深度为 uint16 毫米，检查深度非零时间戳、布局、RGB-D/标定尺寸及内参。uint16 按 `depth_scale_unit` 换算；浮点深度按米处理。
3. 按深度 stamp 查询输出系 TF。感知可为显示回退 latest TF，但将状态标为 stale；只有 `tf_status=='ok'` 才更新权威身份。
4. YOLO 检测、置信门、跨类框去重，然后做默认 FastSAM-s 实例分割。`MobileSam` 是兼容类名，实际后端取决于权重；非 FastSAM 权重使用 Ultralytics SAM。掩膜面积必须严格大于 `sam_min_area`。缺掩膜时保留失败诊断，不把深度连通域降级成可用候选。
5. 袋位姿、端点、入口旋转与方向来自同一曝光。退化袋轴可按当前点云条件化，但保留原同曝光的更严格资格和保守直径；细节见 [视觉共核](../../peach_vision_core/README.md)。
6. 同类 χ² 门与匈牙利一对一分配形成稳定 ID；位置、轴和直径 EMA 只更新注册表及记忆锚点，不覆写当帧候选几何。确认记录进入收集窗口，稳定或超时后锁定；`target_N` 是注册顺序，`priority` 才是抓取顺序。
7. 发布初值、逐目标观测、拟合与可视化。锁定后位姿分割可只针对锁定集/当前目标；打开调试图时，其余检测框仍可额外分割供显示，不能据此推断分割耗时必然下降。

BeginScene 每次增加 scene_epoch 并重置锁窗、使在途帧世代失效。已有非空 scene_key 改变时清注册表并从 target_0 重新编号；同场景重开窗口保留身份。无观测的记忆回填明确标记 `anchor_from_memory`、REOBSERVE，不能代替新观测或模型许可。

## 参数与验证边界

完整接线和配置覆盖顺序见 [io.md](io.md)。模型、位姿管线、身份配置等在构造时捕获；实时 GPL 参数值改变不代表已有算法对象被重建。`sync_slop_s` 启动只读且必须大于零。

`test/` 当前除 ament flake8/pep257 包装外，还已有 `test_image_codec.py` 的四项 Image 视图测试；这些只覆盖布局、共享内存、异常输入和端序。离线回放、mock 和现场证据口径见 [测试指导](../../peach_system_tests/doc/testing.md)，lint 与视图测试均不证明感知精度或采摘成功。
