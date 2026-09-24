# peach_sim — Blender 数据驱动果园重建

当前外观建模入口是 **`reconstruction/`**（2026-09-24 重建）。旧 Blender 资产、程序化贴图、随机摆袋和历史外观结论不作为本次输入；旧 Gazebo 入口仍在仓库中，但不是本次重建场景，也不代表通过真实感验收。

## 打开结果

- `reconstruction/output/bagged_peach_orchard.blend`：完整可编辑场景，米制单位，三台固定相机；材质为 Blender 节点，无外链纹理依赖。
- `reconstruction/output/reference.png`：与 PeachDataSet/Peach_bag/RGB/1200.png 对照的机位。
- `reconstruction/output/detail.png`：局部斜视，检查连接和立体形状。
- `reconstruction/output/orchard.png`：整树与树行环境。
- `reconstruction/output/perception_validation.json`：实际渲染实例 GT 对 YOLO／MobileSAM，以及原始深度的对照。
- `reconstruction/output/geometry_validation.json`：袋网格闭合、有限坐标及推断果实包容检查。

打开 Blender 文件后，在相机列表选择 `Reference 1200 / approximate 90deg`、`Fruit branch / oblique` 或 `Orchard / aisle overview`。参考相机为启动视图；NumPad 0 进入相机。场景对象以 `Reference/`、`TreeXX/` 命名，可直接编辑。

## 数据依据与可信边界

`/home/mu/Downloads/PeachDataSet`：2,050 组套袋桃、577 组裸桃、1,000 组幼桃。`audit_sources.py` 对每类分层抽取 24 组 RGB／Depth／Infrared；`analyze_reference.py` 用项目的 `best.pt` 和 `mobile_sam.pt` 分析 6 帧套袋桃。原始图只用于分析，不作为带水印纹理贴到模型上。

局部 1200 帧：9 个去重后的袋实例，SAM 轮廓与行带有效深度约束可见表面。零深度不当成表面；少量缺失行带插值并记录有效行数，五行滑动平均抑制波动。画面截断的袋端向画外推断延伸，不把图像边界当成袋口。

相机使用作者公开的 **RGB 90° 水平视场**与方形像素近似（fx=fy=640，1280×720）。不是逐机标定，也未恢复相机真实重力姿态，不能宣称毫米级实景复刻。深度采用 Azure Kinect 毫米惯例，原文件不附独立标定。枝条遮挡部分、叶片方向、袋背厚度、整园树形和树距都是明确的建模推断。参考袋内果实不可见，因此不虚构其表面；扩展果园中内果才是合成几何。

树行实例的袋宽高从高深度支持、未截断的测量样本抽取，每个实例记录尺寸来源。树形采用开心形主枝、侧枝、结果枝和独立叶片；挂果接在枝上。室内历史数据 `src/peach_stereo/test/data/{percipio2,hh4}_frames.jsonl` 独立统计，不混入户外尺度分布。

外部依据：

- [数据集作者说明](https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset)：模态对齐、分辨率与相机视场。
- [eOrganic 套袋流程及实拍](https://eorganic.org/node/25727)：袋口跨枝、收拢、扎丝。
- [UGA 果树修剪](https://extension.uga.edu/publications/detail.html?number=C1087)：开心形树冠与主枝结构。
- [Blender 4.5 渲染通道](https://docs.blender.org/manual/en/4.5/render/layers/passes.html)：深度与实例通道；本项目另外输出 RGB 编码的世界 Position，避免把射线距离误当光轴深度。

## 重现

在工作区根目录执行，使用独立 Blender 4.5.14，不向项目 venv 安装 Blender，也不改变 numpy 1.26.4：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

aubo_py3.12/bin/python src/peach_sim/reconstruction/audit_sources.py
aubo_py3.12/bin/python src/peach_sim/reconstruction/analyze_reference.py
aubo_py3.12/bin/python src/peach_sim/reconstruction/test_measurement.py

_tools/blender-4.5.14-linux-x64/blender -b -t 8 --python-exit-code 1 \
  -P src/peach_sim/reconstruction/build_scene.py -- --view all --samples 48 --width 1280

_tools/blender-4.5.14-linux-x64/blender -b \
  src/peach_sim/reconstruction/output/bagged_peach_orchard.blend -t 4 \
  --python-exit-code 1 -P src/peach_sim/reconstruction/validate_geometry.py

aubo_py3.12/bin/python src/peach_sim/reconstruction/validate_perception.py
```

快速看外观可以 `--samples 16 --width 960`；原图深度对照必须重新生成 1280×720。只改相机和采样可用 `render_saved.py -- --view detail --samples 48 --width 1280`，无需重建全部几何。

全部任务为离线前台程序，完成即退出，不运行 ROS 节点、不动真机。保存的 `.blend` 是建模源产物；尚未替换 Gazebo SDF／碰撞体、未接 ROS 相机桥。

## 验收口径

检测与分割仅是辅助证据，不是照片级真实性证明。YOLO 用可见实例包围框进行**一对一**匹配；SAM 单独使用 GT 框提示，与真实渲染实例掩膜比较，不能冒充端到端分割召回。远景的像素阈值和全部检出均落在 JSON 中。

几何门检查袋面闭合及推断果实是否包容，不代表全部枝叶/袋间无碰撞，也不是机器人接触或安全验收。详细实跑结论见 `reconstruction/RESULTS.md`。

## 旧 ROS 场景

原 `peach_sim.params/scene/cli`、`config/orchard.yaml`、`worlds/peach_orchard.sdf` 仍是旧 Gazebo 管线；新建模不消费它们，二者不是同一份 GT。保留既有包接口避免本轮改变 ROS 运行栈；切换仿真资产需另做坐标与碰撞对账。

## 许可

包源码 BSD-3-Clause，见 LICENSE。真实数据集与模型权重保持各自原许可，未重新分发。
