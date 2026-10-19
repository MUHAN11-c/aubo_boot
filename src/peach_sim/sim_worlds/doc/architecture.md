# `sim_worlds` 架构（src/peach_sim/ 仿真子工作空间·场景包，活文档）

> 2026-10-13 P1 重组：原顶层单包拆成五包（`sim_worlds` 场景 / `sim_description` 机体 / `sim_bringup` 入口 / `sim_perception` 感知复制（P3）/ `sim_system_tests` 验收门（P2））。2026-10-19 组目录改回 `src/peach_sim/`；包名、图名、话题名、世界产物不变。独立栈：不进 `harvest_system` / lifecycle / 运行 launch；系统级架构在 [src/peach/peach_bringup/doc/architecture.md](../../../peach/peach_bringup/doc/architecture.md)。

> 2026-09-30 套袋专项 v10：纠正 v9 厚盒底和撑满侧壁，改为薄封边、内果局部鼓起和下部余纸，独立三袋模型含原尺寸内果/果梗/扎丝；新增正侧面检查。仅套袋资产更新，既有整园/field/五光照仍属 v9 历史结果，不是 v10 验收。命令与证据见 [sim_worlds README](../README.md)。

> 2026-09-24 果园外观重建：当前 Blender 入口为 `src/peach_sim/sim_worlds/reconstruction/`，不复用旧模型/贴图/随机布局。RGB-D 约束局部可见袋面；整树、遮挡部分与树行是标注清楚的推断。旧 Gazebo SDF/GT 仍属旧管线，尚未接入此次新场景；无 ROS IDL、运动或驱动变更。数据口径、离线命令与验收边界见 [sim_worlds README](../README.md)。根目录 `blender_orchard/` 观感预览链已于 2026-09-28 并入 `src/peach_sim/sim_worlds/reconstruction/`（袋面折痕、程序贴图、PeachDataSet 先验统计迁入；数据口径以 priors 分位为准），果园外观建模收敛为单管线；光照/叶遮挡/枝干扰为受控变量，多视角感知验证矩阵与基线回归门见 README。

## 包职责与文件树

sim_worlds/                 # Gazebo Harmonic 果园场景包（场景生成+巡园+离线 Blender 线；src/peach_sim/ 组）
  sim_worlds/{scene,cli,preview,params,textures,nav_map,tour_core,tour_node,xmlsafe}.py   # scene=确定性果园纯核；nav_map/tour_core=地图/巡园纯核，tour_node=NavigateToPose 节点
  入口（setup.py console_scripts）：generate_orchard（场景生成）/ scene_preview（正交预览 PNG）/ orchard_tour（巡园）
  config/orchard.yaml      # 场景 schema（几何/摆位唯一事实源）
  worlds/ maps/            # 程序化 SDF+manifest+贴图（生成器产物入库）/ 确定性栅格图
  blender/ reconstruction/ # 离线 Blender 4.5 资产与重建管线（不随安装分发）
  test/                    # 纯核测试（params/scene/textures/nav_map/tour_core + lint）
兄弟包：sim_description（urdf/ 机体 xacro + meshes/）、sim_bringup（launch/{orchard_sim,orchard_nav}.launch.py + config/{orchard_bridge,nav2_params,gz_gui_ogre1}）、sim_perception（P3 感知复制）、sim_system_tests（P2 launch_testing 验收门）

## 关节状态现状（区别于真机臂栈）

本包固定工位模式包含完整臂链：六关节由 URDF `hold_joint`（harvester_robot.urdf.xacro，shoulder/upperArm/foreArm/wrist1-3）保持拍照位，gz 侧 `gz-sim-joint-state-publisher-system` 发布 `/peach_sim/joint_states`，经 `ros_gz_bridge` 出 `/joint_states`（**不是** `joint_state_broadcaster`，`gz_ros2_control` 未接）。移动模式只有底盘、不含臂链；履带 P4b 起为 fixed 挂接（gz VelocityControl 直驱），无可发布的履带关节。消费侧按名字对齐（带臂驾驶以 sim.launch 统一机体 JSB 为准）。

**里程计 TF 单一来源**：gz-sim 8 `OdometryPublisher` 认的子帧元素是 `<robot_base_frame>`，缺省 `{模型名}/base_footprint`。机体 xacro 写成 `platform_base_link` 后，插件把 `odom → platform_base_link` 发到 gz `/tf`，`ros_gz_bridge` 按 Pose 头的 `child_frame_id` 原样桥进 ROS。`map → odom` 只由 AMCL 或 SLAM 发布。不再另挂广播器。

## 固定工位查看外观

`orchard_sim` 与 xacro 默认 `tool_profile:=bite_shear_v1`：使用 peach 既有的
球形环抱咬合式剪切手 CAD 网格及 `tcp_bite_shear_v1.xacro` 帧链。
TCP 保持腕部 `(0, 0.03024, 0.1655) m`、`Rx(-90°)`，不另造普通球体或
仿真 TCP。三种真实剪切手仍可显式选择，真机工具档案未修改。

`mobile:=false` 使用 BUNKER Pro 2 着色网格 `meshes/bunker_chassis.glb`，
替代原箱体/圆柱视觉。来源为 AgileX `ugv_gazebo_sim` 固定提交
`27633a956c845903ee630538afeb17fe70afdd84`；来源、许可声明及重导出命令见
`meshes/BUNKER_SOURCE.md`。`blender/color_bunker.py` 将原 STL 归一化到当前
平台名义包络，导出黑色橡胶、金属灰、深灰与橙色 PBR 常量材质；此适配并非
厂商实机尺寸复刻。碰撞代理、关节与驱动插件独立保留，移动模式仍用原低模。

`field_no_grass.sdf` 自 P2（2026-10-13）带不可见地面碰撞
（`ground_collision` 平面，动态底盘可落地驾驶），自 P4b（2026-10-14）带
`Sensors`（ogre2）渲染系统——腕载 rgbd/激光可成像；它是
`harvest.launch.py`（field 预设）自主采摘的正式世界，不再是纯视觉查看件。
其 `<physics type="ignored">` 仍不关闭物理插件。太阳强度为 1，避免原强度
4 将彩色底盘与金属臂的亮部过曝。

## Gazebo 模型不可见：定位与解决方法

### 先判断失败发生在哪一层

按“启动参数 → 描述展开 → 实体创建 → 实时位姿 → 网格解析 → 相机/材质”
检查。`ros_gz_sim create` 的 `Entity creation successful` 只证明创建请求成功，
不证明模型仍在视野内。也不要看到多个 `robot_state_publisher` 就认定描述争用：
先核对它们的 `ROS_DOMAIN_ID`、话题与各自 launch，独立 domain 的 mock 栈
不能直接视为本仿真的污染源。

本包 Gazebo 实体名是 `sim_worlds.scene.PLATFORM_MODEL = 'harvester'`；
URDF 的 `<robot name="peach_harvester">` 不是最终生成的实体名。
定位/设位服务使用 `harvester`，用 `peach_harvester` 会找不到实体。

工作区根目录先 source Jazzy 与工作区，再做有限时长的只读探针：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
pgrep -af 'ros2 launch|gz sim|robot_state_publisher|parameter_bridge'

# 确认世界和服务确实存在；以下路径适用于当前无草查看世界。
timeout 6 gz service -l
timeout 6 gz service -s /world/field_no_grass/scene/graph \
  --reqtype gz.msgs.Empty --reptype gz.msgs.StringMsg --timeout 3000 --req ''
timeout 6 gz service -s /world/field_no_grass/scene/info \
  --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req ''

# 查当前高度，间隔数秒再取一次。scene/info 主要核对实体与网格，
# 不能用它返回的初始 pose 代替下面的实时位姿。
timeout 6 gz topic -e -t /world/field_no_grass/pose/info -n 1
```

`scene/graph` 中没有 `harvester`：检查 create 日志、`robot_description`
发布及世界的 UserCommands 插件。实体存在但 `pose/info` 的 z 持续变负：
检查重力、动态/固定模式和地面 `collision`。位姿正常而不可见：再查网格
路径、导出坐标轴、GUI 加载错误与相机。这个顺序避免用视角或资源路径
修补实际的物理下落问题。

### 视觉地面、物理与动态底盘

`field_no_grass.sdf` 的果园 GLB 模型只有 `<visual>`；地面承托由独立
`ground_collision` 平面碰撞承担（P2 起补齐，动态底盘可落地驾驶——
`harvest.launch.py` 自主采摘即在该世界行驶）。世界显式加载 Physics 系统；
`<physics type="ignored">` 不关闭物理，实际仍可加载 DART——早于 P2 的
无碰撞版本曾因此穿过可见地面自由落体（实体仍在、创建日志成功，高度降到
百万米以下，调相机不能恢复），该排查顺序保留作历史参照。

只看完整采摘机器人时，仍可用 `mobile:=false`：展开完整臂链，并由
`world_joint` 固定整车。当前臂座高度 = 车体顶面 z=0.54 m
（`body_bottom+body_z`，2026-10-13 起臂直接落底盘、无垫台），履带底面
靠近 z=0。
`mobile:=true` 是导航底盘模式，明确不包含机械臂；在重建世界驾驶/采摘走
`harvest.launch.py`（field 预设，generate_field 站位同源 spawn）。

```bash
# 先在原 launch 终端 Ctrl+C，确认其 gz server/gui 与桥接节点退出。
# 再启动固定工位；不同时叠起第二套同名世界。
ros2 launch sim_bringup orchard_sim.launch.py \
  world:=install/sim_worlds/share/sim_worlds/worlds/field_no_grass.sdf \
  mobile:=false gui:=true tool_profile:=bite_shear_v1
```

### 实体正常时，再解决取景与网格问题

`GZ_SIM_RESOURCE_PATH` 需要包含 `aubo_description`、`sim_description` 的 share
父目录及本包 share；本包 `_resource_path()` 已配置。直接调用 `gz sim`
或独立探针时需自行核对路径。Gazebo 返回的 `model://...` URI 应能在这些
目录中解析到实际文件；查看 GUI 日志中的 `Unable to find`/加载错误。
本机 GLB 查看链采用 Z-up 烘焙、`export_yup=False`，不要根据文件后缀
假定方向正确，必须在实际 Gazebo 画面中核验。

先等大型果园网格加载完成，再让 GUI 相机定位到模型：

```bash
timeout 6 gz service -s /gui/move_to \
  --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'data: "harvester"'
```

返回 `true` 只表示请求被接受，仍要检查画面。自动取景偏离、距离过远或
工具被腕部遮挡时，手动换到工具正面；也可先用
`gz service -i -s /gui/move_to/pose` 核对类型，再用 `gz.msgs.GUICamera`
显式传相机位姿。此操作只移动查看相机，不改变机器人模型位置。
SDF 的旧式 `<gui><camera>` 在本机 Harmonic 会报
`<gui><camera> can't be converted yet`，不能仅凭该配置认定初始视角生效。

截图是实际显示的验收证据；保存到现有目录，并在请求后确认文件生成：

```bash
mkdir -p /tmp/peach_sim_view
timeout 6 gz service -s /gui/screenshot \
  --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'data: "/tmp/peach_sim_view"'
```

### 末端与底盘外观的适配思路

“球形末端”在本场景指 peach 的球形环抱咬合式剪切手 `bite_shear_v1`，
不是普通球体代理。先对照三种既有 CAD 网格、工具 yaml 与 wrapper，选定
现有 `tool_profile`，让外形、collision、快换和 TCP 一起展开。不要另加
第四套工具几何或把 TCP 改成随意取的球心。原档案与网格在
`aubo_description`，本包仅选择它们，真机档案不受仿真外观调整影响。

底盘参考优先下载厂商公开的模型资产，而不引入其旧发行版控制代码。
固定上游提交，保留原网格、哈希及许可声明，再适配到当前平台包络。
本包 BUNKER 的来源与重导出入口集中在
[BUNKER_SOURCE.md](../meshes/BUNKER_SOURCE.md)。详细 GLB 只负责固定工位
视觉；原简单碰撞体与导航驱动仍独立保留。STL 不含颜色，着色后用 GLB
保存 Principled PBR 的 Base Color、Metallic 和 Roughness 常量，避免
程序化材质在导出时丢失。颜色核验须同时看材质数据和实际截图；过曝时先
调整光强，不把材质改色当作光照问题的替代。当前太阳强度为 1。

### 构建、验证与收尾

新 xacro/网格必须出现在 `setup.py` 的 `data_files` 安装范围，修改源码后
重新构建并 source 工作区。当前安装目录采用普通安装：

```bash
colcon build --packages-select sim_worlds sim_description sim_bringup
source install/setup.bash
colcon test --packages-select sim_worlds sim_description sim_bringup \
  --pytest-args -q -k 'urdf_mobile or nav_config'
colcon test-result --test-result-base build/sim_worlds --verbose
```

sim 组五包全量验收门（2026-10-15 标准化轮起）：`sim_perception` 已移植
peach_harvester 纯核行为测试（基线=fork 点，见其 README）；`sim_system_tests`
launch_test 依赖 `test_depend` 声明被启动的包（否则隔离环境解析不到
`sim_bringup`），同机多栈时以 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 绕开机器级
SHM 端口互扰（详见其 README 隔离段）。

混用普通安装与 `--symlink-install` 时，现有 colcon/setuptools 的数据安装
可能在新文件处报 `No such file or directory`，或在已有普通文件处报
`File exists`。先查构建日志与 install 文件/链接类型，采用一致的安装方式；
本包普通构建可完成安装，不需要修改模型代码来绕过安装错误。删除不用的
源文件后，另核对并清理它在本包 build/install 中的旧副本，避免旧档案
继续被加载。

验收分别核对：描述展开使用真实 `bite_shear_v1.stl` 和原 TCP、Gazebo
实体包含臂链与 BUNKER 网格、两次实时高度不下落、实际画面显示正确外形和
颜色。结构/配置测试通过只证明接线与模型结构，不能替代截图。
探针与离线导出结束即退出；查看结束在启动终端 Ctrl+C，并检查本次
launch 的 server、GUI、RSP、bridge 都已退出。清理按确认过的 PID/进程树，
不误停其他项目或独立 domain 的栈。

参考：[Gazebo Harmonic 建模示例](https://gazebosim.org/docs/harmonic/building_robot/)
区分 visual/collision，并在 `type="ignored"` 下显式加载 Physics；
[Spawn URDF](https://gazebosim.org/docs/harmonic/spawn_urdf/)
说明 URDF 经 create 服务转换并生成到 Gazebo。
