# PS800-E1 帧率/立体评测工具集（2026-09-16）

围绕「为什么开深度帧率只有 2.43 fps」与「主机侧单图案立体可行性」两个问题的
一次性评测工具与结论复现入口。完整数据与结论见 `docs/testing-log.md` 09-16
五个条目。**不进 colcon 构建**，独立编译：

```bash
bash build.sh
export LD_LIBRARY_PATH=../../camport4/lib/linux/lib_x64:$LD_LIBRARY_PATH
```

## 工具一览

| 工具 | 用途 |
|------|------|
| `feature_dump` | 枚举各组件 0x1000-0x4FFF 特征 ID，读回名称/可写性/当前值（75 项全量） |
| `temp_probe` | 只读温度探针（不开流、不点激光）：三路探测 SDK 温度可读性——Device 全特征扫描名字含 Temp 项 / `TY_ENUM_TEMPERATURE_ID` 选测点 + `TY_STRUCT_TEMPERATURE` / struct 尺寸兜底扫描（09-20 激光热管理可行性验证用） |
| `write_test` | 空闲态写 image number(0x1610) 并读回，验证寄存器接受度 |
| `laser_check [reset]` | 查询/复位 laser 设置（auto ctrl / power）。**laser 跨连接自动复位；曝光值跨连接残留**，改完设备参数务必核验 |
| `raw_ir_test <exposure> [plain/laser/flood/dual]` | 裸 SDK 独立 IR 采集测试。`laser`/`dual` 模式先解锁投射器 |
| `stereo_grab <ir/depth> <count> <outdir>` | 采集：`ir`=解锁激光双目对+标定落盘；`depth`=设备端 18 图案深度 |
| `stereo_live` | **实时彩色深度演示**：单窗口 JET 伪彩、自动色阶、帧率叠加；`a` 切 k 帧融合(1/2/4/8)，`q/ESC` 退出 |
| `sgbm_eval.py` | 离线评测：极线验证(ORB 比值筛选)、时域噪声 k=1/5/10、有效率、与设备深度一致性 |

## 关键事实（测出来的，勿重蹈）

1. **深度 2.43 fps 是设计**：18 幅散斑图案×~23ms；官方 0.8fps@全分辨率；
   仓库可及的所有参数（image number/SGPM/分辨率/曝光/frame_rate）都不改深度帧率。
2. **独立 IR 真实散斑的解锁序列**（否则输出非光信号底噪）：
   `TYSetBool(LASER, TY_BOOL_LASER_AUTO_CTRL, false)` +
   `TYSetInt(LASER, TY_INT_LASER_POWER, 100)`，再开 IR 流。
   L+R 同帧硬件同步 ≈14.5 对/秒。
3. **单图案立体精度够套袋**：噪声 1.49mm@800mm（设备 18 图案 1.13mm），
   k=5 融合 0.96mm 反超；实时 14.7fps（3WAY 半分辨率处理仅 ~13ms）。
4. **标定结构体是 float32**：`cv::Mat(..., CV_64F, ptr)` 直接包装 = 位型错读
   → stereoRectify 输出 NaN → 全图无效。必须
   `Mat(..., CV_32F, ptr).convertTo(K, CV_64F)`。
5. 极线验证必须 ORB + 比值筛选（散斑图裸 BFMatcher 全是错配）。
6. 右相机外参即相对左 IR（头文件注释明示），基线 62.2mm。

## 安全注意

- `laser`/`dual`/`stereo_live`/`stereo_grab ir` 会让投射器**满功率持续点亮**，
  看完即关；laser 设置在连接关闭后自动复位（auto=1/power=50）。
- 这些工具独占相机连接：先停 ROS 相机栈再用；ROS 栈在跑时工具 open 会失败。
- 相机 IP 硬编码 169.254.10.110（测试台架），换机改源码。
