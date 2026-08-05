# PeachPose 零基础逐行教程：从启动命令到输出内容

> 适用读者：完全没读过这个包的人。跟着本文从 `ros2 launch` 命令出发，
> 逐层逐行走完整个 `peach_pose_ros2` 包，直到看懂每一个输出话题。
> 行号对应 2026-08-04 版本；代码若有改动，以代码为准。
> 包的定位与速查用法见 `README.md`，本文负责"讲透"。

## 0. 预备知识（零基础最低门槛）

- **ROS 2 话题**：节点之间用"话题"传消息。相机驱动发图片，本节点订阅图片、
  发布桃子的位姿结果。
- **launch 文件**：一键拉起一组节点的 Python 脚本。
- **坐标系（TF）**：相机看到的点首先在"相机坐标系"里，要变到机器人
  `base_link` 坐标系机械臂才用得上，这个变换由 TF 树提供。
- 本包**只发"参考位姿"，不发运动指令**——它告诉机械臂"桃子在哪、从哪个
  方向套袋子"，但不驱动电机。

## 1. 全景图

```
ros2 launch peach_pose_ros2 peach_pose.launch.py
  └─ launch 文件（launch/peach_pose.launch.py）
      └─ console_scripts 启动器（构建期由 setup.py 生成，
        shebang 指向 aubo_py3.12 venv 的 python）
          └─ peach_pose_ros2.peach_pose_node:main → PeachPoseNode
                  订阅: /camera/color/image_raw      (RGB 彩图)
                        /camera/depth/image_raw      (深度图)
                        /camera/color/camera_info    (相机内参)
                  算法: YOLO 检测 → MobileSAM 分割 → 几何拟合管线
                  发布: ~/grasp_candidates      ← 主输出（3D 抓取参考）
                        ~/grasp_candidates_2d   ← 图像上的关键点
                        ~/fitting               ← 拟合诊断指标
                        ~/detections ~/masks ~/markers ~/debug_image ~/detection_cloud
```

"~"是节点命名空间缩写，实际话题名形如
`/peach_pose_node/grasp_candidates`。

## 2. 第一层入口：launch 文件逐行

文件：`src/peach_pose_ros2/launch/peach_pose.launch.py`（25 行）

```python
"""
PeachPose 感知节点启动.

节点为标准 console_scripts 入口，其 shebang 在构建期由 setup.py 的
options.build_scripts.executable 指向 aubo_py3.12 venv 解释器
（保证 numpy 1.26 + cv_bridge + torch）。无相机冒烟用的数据集回放工具
为 tools/peach_dataset_replayer.py（不随 colcon 构建），需另开终端手动运行。
"""
```

模块 docstring 说清了两件事：① 节点是**标准 console_scripts 入口**，
只是启动器的 shebang 在构建期被指到项目 venv（机制见第 3 章）——因为
YOLO/SAM 需要 torch，而 torch 只装在 venv 里；② 没相机时用
`tools/peach_dataset_replayer.py` 回放数据集冒烟（第 9 章有命令）。

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
```

`ament_index_python`：按包名找"安装后"的目录（`install/.../share/
peach_pose_ros2`）——launch 读的是安装空间，不是源码目录。

```python
def generate_launch_description():
    """生成 PeachPose 感知 launch 描述."""
    share = get_package_share_directory('peach_pose_ros2')
    config = os.path.join(share, 'config', 'peach_pose.yaml')

    return LaunchDescription([
        Node(
            package='peach_pose_ros2',
            executable='peach_pose_node',
            name='peach_pose_node',
            parameters=[config],
            output='screen',
        )
    ])
```

法定入口函数（名字固定），全部内容就是启动一个节点。逐参数看：

- `package`/`executable`：找 `peach_pose_ros2` 包 `lib/peach_pose_ros2/`
  下名为 `peach_pose_node` 的可执行文件——即构建期由 `entry_points.
  console_scripts` 生成的启动器（第 3 章）。
- `name='peach_pose_node'`：节点名。参数文件 yaml 的顶层键必须也是
  `peach_pose_node`，两者对不上参数就静默不生效。
- `parameters=[config]`：把第 4 章的 yaml 灌给节点。
- `output='screen'`：日志打到终端（默认只进日志文件，调试时看不到）。

整个 launch 没有任何解释器/环境变量处理——**解释器的选择在构建期就已
固化在启动器 shebang 里**（下一章讲它是怎么做到的），这正是"标准写法"
想要的：launch 只管节点和参数。

## 3. 第二层入口：console_scripts 与 venv shebang

> 历史说明：2026-08-05 之前本包用的是"bash 包装脚本"方案（scripts/ 下
> 23 行脚本在运行时选解释器），已迁移为 ROS 2 官方文档 *Using Python
> Packages with ROS 2* 推荐的标准 console_scripts 做法，包装脚本已删除
> （历史见 git）。

### 3.1 console_scripts 是什么

ament_python 包在 `setup.py` 里声明 `entry_points={'console_scripts':
['名字 = 包.模块:main']}`，构建时 setuptools 会为每个入口**生成**一个小
启动器（装在 `lib/<pkg>/` 下），内容是"用 shebang 指定的解释器 import
模块并调用 main()"。看本包实物（`install/peach_pose_ros2/lib/
peach_pose_ros2/peach_pose_node` 前三行）：

```python
#!/home/mu/Desktop/aubo_e5_jazzy_ws/aubo_py3.12/bin/python
# EASY-INSTALL-ENTRY-SCRIPT: 'peach-pose-ros2==0.1.0','console_scripts','peach_pose_node'
import re
```

第一行 shebang 指向 venv 解释器（shebang 的机制：内核执行文件时按第一行
`#!` 后的路径选解释器）。`ros2 run`/`Node()` 执行的就是这个生成文件。

### 3.2 shebang 为什么指向 venv：setup.py 逐段

文件：`src/peach_pose_ros2/setup.py`

```python
def _resolve_python():
    ws_venv = Path(__file__).resolve().parents[2] / 'aubo_py3.12' / 'bin' / 'python'
    if ws_venv.exists():
        return str(ws_venv)
    return sys.executable
```

构建期解析解释器路径：**工作区 `aubo_py3.12/bin/python` 存在则用之，
否则回退当前（构建）解释器**。`Path(__file__).parents[2]`
从 `src/peach_pose_ros2/setup.py` 上溯到工作区根——**源码不写死绝对
路径**，工作区迁移/换机后重新构建，shebang 自动指向新位置的 venv。

```python
    entry_points={
        'console_scripts': [
            'peach_pose_node = peach_pose_ros2.peach_pose_node:main',
        ],
    },
    options={
        'build_scripts': {'executable': _resolve_python()},
    },
```

- `entry_points`：声明标准入口 `peach_pose_node`，映射到
  `peach_pose_node.py` 的 `main()`（第 5 章讲的节点就从这里进）。
- `options.build_scripts.executable`：关键后门。setuptools 的
  `install_scripts` 命令在生成 console_scripts 启动器时，会从
  `build_scripts` 命令取 `executable` 属性写 shebang（上游源码级事实，
  不设置则回退为跑构建的解释器——apt 版 colcon 固定是
  `/usr/bin/python3`，系统 python 没有 torch，入口就是坏的）。

### 3.3 常见问题

> **为什么不直接在 .py 开头写 venv 的 shebang？**
> 被执行的不是你的 .py，而是生成的 console_scripts 启动器；能控制它
> shebang 的唯一官方通道就是上面的 `build_scripts.executable`。

> **构建只是复制，跟执行时用什么解释器有关吗？**
> Python 源码是解释器无关的文本，复制无关；但 console_scripts 启动器是
> 构建期**生成**的（文件头有 `EASY-INSTALL-ENTRY-SCRIPT` 标记），shebang
> 是生成时写入的——耦合点只有这一处，本包用 `options` 把它钉到 venv。

> **还要手动 `source venv` 吗？**
> 不需要。启动器 shebang 已指向 venv，`ros2 run`/`ros2 launch` 直接进
> venv 环境（venv 带 `--system-site-packages`，cv2/cv_bridge 走系统侧
> 也可见）。万一要换解释器：建好新 venv 后改 `_resolve_python()` 的
> 路径或约定重建即可（本包没有为此保留专门的环境变量——用不到的机制
> 不留）。

> **和旧 bash 包装脚本方案比？**
> 两者都把节点送进 venv；新方案入口回到 ROS 2 标准（无额外机制、launch
> 零特殊处理），代价是解释器在构建期固化、换解释器要重建（包装脚本可
> 运行时切换）。2026-08-05 起本项目采用新方案。

## 4. 参数文件 peach_pose.yaml 逐行

文件：`src/peach_pose_ros2/config/peach_pose.yaml`（36 行）。yaml 里的值
会**覆盖**节点代码里 `declare_parameter` 的默认值，两者一一对应（项目约定：
改默认值必须两边同步）。

```yaml
# PeachPose 桃姿节点参数（本机 Percipio + 手眼）
peach_pose_node:
  ros__parameters:
```

顶层键 = 节点名（与 launch 的 `name=` 一致），`ros__parameters` 是固定写法。

```yaml
    color_topic: /camera/color/image_raw
    depth_topic: /camera/depth/image_raw
    camera_info_topic: /camera/color/camera_info
```

三个输入话题：RGB 彩图（bgr8）、深度图（uint16，已与彩图对齐）、相机内参。
默认话题名对应 Percipio 相机驱动的输出。

```yaml
    # 手眼标定光学系；空串则用深度图 header.frame_id
    camera_optical_frame: camera_color_optical_frame
    # 输出坐标系：经 TF（含 extrinsics_publisher）变到此帧；空串=保持相机系
    output_frame: base_link
    tf_timeout_sec: 0.5
```

坐标系三件套：相机光学系名（手眼外参挂在它上面）；**输出坐标系
`base_link`**——所有 3D 结果都会乘 TF 变到机械臂基座系，机械臂直接用；
查 TF 最多等 0.5 秒。

```yaml
    # Percipio DepthScaleUnit（常见 0.25）：管线按 uint16「毫米」/1000→米，
    # 故先 raw *= depth_scale_unit。数据集回放(真毫米)请设 1.0
    depth_scale_unit: 0.25
```

深度单位换算系数：Percipio 原始深度值 ×0.25 才是毫米。算法管线内部统一按
"毫米"处理，所以收到原始值先乘这个系数。回放数据集（已经是毫米）时设 1.0。

```yaml
    sync_slop_s: 0.05
    min_detection_conf: 0.5
    yolo_conf: 0.3
```

- `sync_slop_s`：RGB/深度/内参三路消息的时间戳允许差 50 ms 以内算"同一帧"
  （近似时间同步，第 5.4 节细讲）。
- `yolo_conf`：YOLO 推理时的置信度阈值（0.3，宽进）。
- `min_detection_conf`：检测结果**进入几何管线**的阈值（0.5，严出）。
  两级阈值是刻意设计：YOLO 多给候选，管线只处理高置信的。

```yaml
    publish_debug_image: true
    publish_masks: true
    # 检测框内深度反投影彩色点云（~/detection_cloud）；stride>1 降采样减轻 RViz
    publish_detection_cloud: true
    detection_cloud_stride: 2
```

可视化开关：debug 叠加图、SAM 掩膜图、检测框点云（隔 2 像素取一个点，
减轻 RViz 渲染压力）。只看主输出的话三个都可以关。

```yaml
    yolo_model_path: ""
    sam_model_path: ""
    model_version: "yolo:6981750db67a726e|mobile_sam:6dbb90523a35330f"
    # 内参: percipio color_camera_info.yaml；外参: hand_eye/active.yaml
    calibration_version: "percipio-640x480-chessboard|hand_eye:import_humble_20260128T114006"
    gravity_hint_xyz: ""
```

- 模型路径留空 = 用包内自带的 `model/best.pt`（YOLO）和
  `model/mobile_sam.pt`（MobileSAM）。
- `model_version`/`calibration_version` 只是**随结果发布的版本标识字符串**，
  便于追溯"这个结果是谁算的"（换模型/换手眼标定后记得改）。
- `gravity_hint_xyz`：相机系下的重力方向提示（"x,y,z"），空串 = 管线默认
  相机系 +Y 朝下（Percipio 正装）。重力方向用来给"袋底→袋颈"定方向。

```yaml
    tool.D_inner: 0.104
    tool.L_insert: 0.200
    tool.L_blade: 0.025
    tool.entry_d_tool: 0.030
    tool.entry_d_s: 0.040
    tool.clearance_min: 0.005
    tool.margin_neck: 0.015
    tool.version: "1.1"
```

套袋工具的 8 个几何参数（单位：米），组成 `ToolGeometry`：

| 参数 | 值 | 含义 |
|---|---|---|
| `D_inner` | 0.104 | 工具内径（袋子要比它小才能套进） |
| `L_insert` | 0.200 | 最大插入行程 |
| `L_blade` | 0.025 | 刃口长度 |
| `entry_d_tool` | 0.030 | 接近时工具端面预停距离 |
| `entry_d_s` | 0.040 | 附加安全距离（与上者相加为总预停距离） |
| `clearance_min` | 0.005 | 最小径向净空（安全门控用） |
| `margin_neck` | 0.015 | 行程预留的颈部余量 |
| `version` | "1.1" | 工具版本标识 |

这些值直接决定安全门控的判定（第 7 章"误差预算与门控"），换工具必改。

## 5. 第三层入口：peach_pose_node.py

文件：`src/peach_pose_ros2/peach_pose_ros2/peach_pose_node.py`（826 行）。
它是 ROS 与算法之间的"翻译层"：把 ROS 消息翻译成 numpy 数组交给纯算法包
`peach_pose/`，再把算法结果翻译回 ROS 消息发出去。算法包本身**不 import
rclpy**，可以脱离 ROS 离线跑测试——这是刻意分层。

### 5.1 模块头与 8 个工具函数（1–221 行）

```python
class PeachPoseNode(Node):   # 第 224 行
```

之前是 8 个纯函数工具，全部带中文 docstring，先有个印象即可（用到时回查）：

| 函数 | 行 | 干什么 |
|---|---|---|
| `_transform_msg_to_matrix` | 47 | TF 的 Transform 消息 → 4×4 齐次矩阵 |
| `_apply_T_to_grasp3d` | 67 | 把抓取几何（点/方向/姿态）从相机系变到输出系 |
| `_rotation_to_quat` | 95 | 3×3 旋转矩阵 → 四元数（Shepperd 法，数值稳健） |
| `_point` / `_px` | 121/130 | numpy → Point 消息；`_px` 专塞像素坐标 |
| `_metric` | 139 | 从指标字典取 float，缺了填 -1（诊断消息约定） |
| `_status_color` | 150 | ACCEPT 绿 / REOBSERVE 黄 / REJECT 红 |
| `_pack_rgb_bgr` / `_bbox_cloud_xyzrgb` / `_xyzrgb_to_cloud` | 159–221 | 检测框内深度反投影成彩色 PointCloud2 |

第 91 行有个重要常量：

```python
STATUS_MAP = {'ACCEPT': 0, 'REOBSERVE': 1, 'REJECT': 2}
```

算法内部用字符串状态，ROS 消息用整数枚举，这里做映射。三态语义：
**ACCEPT=可以放心套袋；REOBSERVE=信息不足建议换个角度看；
REJECT=不要动作**。

### 5.2 构造函数七件事（224–288 行）

```python
    def __init__(self):
        super().__init__('peach_pose_node')
        self.bridge = CvBridge()
        self._declare_params()
        self._load_params()
```

① 节点名注册为 `peach_pose_node`（对上了 yaml 顶层键）；② `CvBridge` 是
ROS 图像消息 ↔ OpenCV 数组的翻译官；③④ 声明并读入全部参数（第 4 章
那些）。

```python
        share = Path(get_package_share_directory('peach_pose_ros2'))
        yolo = self.yolo_model_path or str(share / 'model' / 'best.pt')
        sam = self.sam_model_path or str(share / 'model' / 'mobile_sam.pt')
```

⑤ 模型路径：参数为空串就用包内自带权重。

```python
        self.engine = InferenceEngine(
            yolo_model=yolo, sam_model=sam, yolo_conf=self.yolo_conf)
```

⑥ 建推理引擎。**此时不加载权重**——`InferenceEngine` 是懒加载，第一次
调用 detect/segment 时才真正 `from ultralytics import YOLO` 并读 `.pt`
文件（首次推理会慢几秒，属正常）。

```python
        from peach_pose_ros2.peach_pose.pipeline import RobustBagPosePipeline
        self.estimator = CandidateEstimator(
            pipeline=RobustBagPosePipeline(tool=self.tool))
```

⑦ 建几何估计器。这里有个不明显的细节：节点只显式构造了**袋装桃管线**
（`RobustBagPosePipeline`，圆柱拟合定轴）；**裸果管线**（球拟合+梗洼定轴）
由 `CandidateEstimator.__init__` 内部用同一份工具几何自动建好
（candidates.py:60），按 YOLO 的类别自动路由（class 0=袋装桃走袋线，
class 1=裸果走果线）。

然后是 8 个发布者（246–258 行，全部队列深度 10）：

| 话题 | 消息类型 | 内容 |
|---|---|---|
| `~/grasp_candidates` | `peach_pose_msgs/BagGraspCandidateArray` | **主输出**：3D 抓取参考 |
| `~/grasp_candidates_2d` | `peach_pose_msgs/BagGrasp2DArray` | 图像平面关键点 |
| `~/fitting` | `peach_pose_msgs/BagFittingArray` | 拟合诊断指标 |
| `~/detections` | `vision_msgs/Detection2DArray` | YOLO 检测框 |
| `~/masks` | `sensor_msgs/Image`（mono8） | SAM 掩膜画布 |
| `~/markers` | `visualization_msgs/MarkerArray` | RViz 可视化 |
| `~/debug_image` | `sensor_msgs/Image`（bgr8） | debug 叠加图 |
| `~/detection_cloud` | `sensor_msgs/PointCloud2` | 检测框内彩色点云 |

### 5.3 订阅与三路时间同步（260–275 行）

```python
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
```

QoS 显式 RELIABLE——行内注释（261 行）说明了原因："与数据集回放 / 相机
驱动对齐：RELIABLE，避免 Best Effort 对不上"。ROS 2 里发布/订阅双方的
reliability 不匹配会**静默收不到消息**，这是高频踩坑点。

```python
        sub_rgb = message_filters.Subscriber(
            self, Image, self.color_topic, qos_profile=qos)
        sub_depth = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos)
        sub_info = message_filters.Subscriber(
            self, CameraInfo, self.camera_info_topic, qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=10, slop=self.sync_slop_s)
        self.sync.registerCallback(self._on_rgbd)
```

为什么用 `message_filters` 而不是直接 `create_subscription`：相机每帧产生
**三条**消息（彩图、深度、内参），它们时间戳不完全相同。普通订阅会各自
乱序到达；`ApproximateTimeSynchronizer` 把三路里时间戳差在 `slop`
（50 ms）内的凑成一组，一次性回调 `_on_rgbd(rgb_msg, depth_msg,
info_msg)`——回调里拿到的三张图保证是"同一时刻"的。

### 5.4 TF 监听（277–280 行）

```python
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
```

TF 缓存+监听器。手眼标定的静态 TF（wrist3_Link→camera_link）由独立的
extrinsics_publisher 节点发布，这里只是**查**，不广播。本节点没有任何
服务、action 或 TF 广播——纯"订阅→计算→发布"。

## 6. 主回调 _on_rgbd 逐段（433–586 行）

这是整个节点的心脏，每来一组同步好的 RGB-D 帧就执行一次。分六段看。

### ① 消息转 OpenCV + 校验（437–465）

```python
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
```

RGB 统一成 OpenCV 的 bgr8 三通道；深度 `passthrough`=保持原格式（uint16
毫米数）。

```python
        if depth.dtype != np.uint16:
            ...return   # 深度不是 uint16 直接丢帧
        if self.depth_scale_unit != 1.0:
            depth = np.clip(np.round(depth.astype(np.float32) * self.depth_scale_unit), ...)
```

深度原始值乘 `depth_scale_unit`（0.25）变成真毫米（451 行注释有公式：
`z_m = raw × depth_scale_unit / 1000`）。随后校验 RGB/深度尺寸一致、
CameraInfo 分辨率与深度一致——任一不过就 warn 并丢整帧，**不产出错误结果**。

### ② 内参 + 坐标系（467–494）

```python
        K = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)
```

相机内参矩阵 K 从 CameraInfo 消息取（467 行注释："始终用本机
CameraInfo，勿回退 FOV 推导"——内参只能信标定值）。

接着确定两个坐标系并查 TF：

```python
        T_out_cam = self._lookup_T_out_cam(rgb_msg.header.stamp, out_frame, cam_frame)
```

`_lookup_T_out_cam`（409–431）：先按帧时间戳查 TF；查不到退而查"最新"
TF 并一次性告警；再失败返回 None。488 行注释记录了设计决策：TF 查不到且
设了 `output_frame` 时，**退回相机系发布**（`out_frame = cam_frame`），
"避免静默用错坐标系"——下游拿到相机系的结果至少能发现并处理，比拿一个
自以为在 base_link 实际错误的值安全。

### ③ YOLO 检测 + 发检测话题 + 检测点云（496–519）

```python
        dets = self.engine.detect(rgb)
        ...
        for d in dets:
            if float(d.get('conf', 0.0)) < self.min_detection_conf:
                continue
            kept.append(d)
```

`engine.detect`（inference.py:92）跑 YOLO：输入 BGR 图，输出按置信度降序
的 `[{class_id, class_name, bbox:(x1,y1,x2,y2), conf}]`。两级过滤中的第二级
（≥0.5）在这里发生。过的检测框装进 `Detection2DArray` 发 `~/detections`；
若开了 `publish_detection_cloud`，把每个框内的深度像素反投影成彩色点云、
乘 TF 变到输出系，发 `~/detection_cloud`（RViz 里能直接看到"检测到了哪块
空间"）。

### ④ 逐目标处理（537–570，核心循环）

每个保留下来的检测框走一遍完整算法：

```python
        for i, det in enumerate(kept):
            bbox = tuple(det['bbox'])
            sam_mask = None
            segs = self.engine.segment(rgb, [bbox])
            if segs:
                sam_mask = segs[0][0]
                mask_canvas[sam_mask > 0] = np.uint8((i % 250) + 1)
```

以 YOLO 框为 box prompt 调 MobileSAM（inference.py:142），得到该目标的
像素级掩膜；掩膜画到全图画布上（第 i 个目标灰度值 = i%250+1，RViz 里
不同目标颜色不同）。

```python
            obs = BagObservation(
                rgb=rgb, depth=depth, camera_K=K, frame_id=cam_frame,
                gravity_hint=self.gravity_hint,
                detections=[det],
                metadata={...},
            )
            results = self.estimator.estimate_modes(obs, tid, bbox, sam_mask)
            result = results['hybrid_dilated']
```

把这一帧的一切打包成 `BagObservation`（数据合约，contracts.py:86），交给
`CandidateEstimator.estimate_modes`（candidates.py:71）。它内部：把 SAM
掩膜与"深度连通域膨胀 5px"取交构造前景（`hybrid_dilated`，candidates.py:
109）→ 按 class_id 路由到袋线或果线管线 → 返回该目标的
`TargetPoseResult`。SAM 掩膜缺失或与深度相交不足 50 像素时，**显式**标
`mask_unavailable` 给 REOBSERVE（candidates.py:90 注释："禁止静默回退"）。

```python
            if T_out_cam is not None and out_frame != cam_frame:
                _apply_T_to_grasp3d(result.grasp_3d, T_out_cam)
```

3D 结果从相机系变到 `base_link`（点做 R·p+t，方向只乘 R 再归一化）。
随后四个 `_to_*` 函数把结果分别装进 3D 候选、2D 关键点、诊断、Marker
四类消息（字段来源见第 8 章），debug 图上画框/掩膜/箭头/状态字。

### ⑤ 批量发布（572–586）

循环结束后一次性发四个主话题；`~/masks` 和 `~/debug_image` 按参数开关发。
注意设计：即使一个目标都没有，空数组也照发——下游靠"收到空数组"知道
"这帧没看到桃子"，而不是"节点死了"。

### ⑥ Marker 细节（690–784，选读）

每个目标占 Marker id 段 `i*20 .. i*20+19`：袋轴线段（+0）、行程箭头（+1）、
刀具圆柱（+2，半透明，直径=D_inner）、果球（+3，仅裸果）、RGB 三轴架
（+4/5/6）、状态文字（+10）。每帧先发一个 DELETEALL 清空——530 行注释
记了个坑：DELETEALL 不能设 namespace/id，否则与第一个 ADD 冲突。
709 行注释：ColorRGBA 必须喂 Python float（numpy 类型序列化会炸）。

## 7. 算法管线（pipeline.py）科普

文件：`peach_pose/pipeline.py`（626 行）。两条并列管线共享安全门控：
**袋线**（袋装桃，class 0，圆柱拟合定轴）和**果线**（裸果，class 1，
球拟合+梗洼定轴）。袋线主入口 `RobustBagPosePipeline.estimate()`
（58 行）的 12 个阶段：

1. **bbox 裁剪门**：检测框裁到图内，<8px 直接拒（`invalid_bbox`）。
2. **有效深度掩膜**：0.3~2.5 m 内的深度才可用。
3. **前景掩膜**：优先 SAM∩有效深度（≥50 像素）；SAM 不可用则显式降级为
   "中心区域深度带 + 最大连通域"，并打 `depth_fallback` 标记。
4. **反投影点云**：pinhole 模型把前景像素变成 3D 点，剔深度离群点；
   不足 100 点拒（`insufficient_measured_points`）。
5. **重力向量**：参数提示或默认相机系 +Y，归一化。
6. **法线估计**（fitting.py:36）：深度图邻域叉积，逐点标有效。
7. **轴估计三级降级**：法线够 200 点 → 圆柱 RANSAC 定轴（inlier≥0.35 才
   采纳）；不够 → 用重力先验（置信度封顶 0.4）；最后统一符号定向
   （袋底→袋颈 = 逆重力），与重力夹角太大就标 `orientation_uncertain`。
   88–90 行中文注释讲了为什么不用 PCA（近回转体退化）。
8. **底/颈/袋径**：点云沿轴投影，P10/P90 分位定底/颈位置，横向 P95×2
   为直径上界。
9. **2D 交叉校验**：掩膜 PCA 主轴与 3D 轴投影夹角，夹角 >45° 打
   `axis_2d_mismatch`。
10. **entry/travel/抓取架**：`entry_start = 袋底 − (d_tool+d_s)·轴`
    （contracts.py:179）；行程 = 到颈部减刃长减余量（contracts.py:201）；
    构造 Z 轴=袋轴的右手抓取架。
11. **误差预算与安全门控**：`预算 = (预停距离+行程)·sin(轴角误差)` 必须
    ≤ `径向净空 = D_inner/2 − 袋半径 − clearance_min`，否则
    `tool_clearance_failed`。**状态规则：无 flag→ACCEPT；
    有 tool_clearance_failed→REJECT；其余→REOBSERVE**（159–183 行）。
12. **2D 关键点投影 + 输出组装**：底/颈/抓取点/行程终点投回像素，
    17 项指标装满，返回 `TargetPoseResult`。

**果线差异**（385–626 行）：几何换成球拟合（半径夹紧 25–45 mm）+ 梗洼
检测定轴（Fibonacci 球面扫描找径向下陷最深的 20° 方向帽），另有一步
**重力极性校正**（梗朝下的洼其实是萼洼——457 行注释引用 Kok 2024 的
对极点混淆问题，翻轴并降置信）；entry/travel/门控公式与袋线完全相同。

所有失败路径统一走 `_failed`：status=REJECT + 单一原因 flag——**宁可拒，
不出错结果**是这个包的总体设计哲学。

## 8. 推理引擎（inference.py）

`InferenceEngine`（37 行）封装 ultralytics 的 YOLO 与 MobileSAM：

- **懒加载**：首次调用才 import 并读权重（107/166 行），节点启动快；
  首帧推理慢几秒是模型加载+CUDA warmup，属正常。
- **设备**（79 行）：有 N 卡用 `cuda:0`，否则回退 CPU（慢，仅应急）。
- **线程安全**（76 行）：所有推理经一把 `threading.Lock` 串行化——
  模块 docstring（11–28 行中文）详解了 CUDA 竞态的原因。
- `detect(rgb)`（92 行）：YOLO 前向，`conf≥0.3`、`iou=0.5`，输出框按
  置信度降序。
- `segment(rgb, bboxes)`（142 行）：MobileSAM 以检测框为 box prompt，
  掩膜概率 >0.5 二值化，面积 <100 像素丢弃，最多处理 8 个框。

## 9. 输出内容详解：8 个话题逐个看

以下假设默认命名空间，实际话题名以 `ros2 topic list | grep peach` 为准。

### 9.1 `~/grasp_candidates`（主输出，`BagGraspCandidateArray`）

每帧一条，含 0~N 个候选。单个候选字段（msg/BagGraspCandidate.msg）：

| 字段 | 含义 |
|---|---|
| `header` | frame_id=`base_link`（或回退相机系），stamp=RGB 帧时刻 |
| `target_id` | `target_0`、`target_1`…（帧内序号，2D/3D/诊断按它对齐） |
| `entry_pose` | **核心**：工具接近的预停位姿（袋底后退预停距离，Z 轴=袋轴） |
| `bag_bottom` / `bag_neck` | 袋底/袋颈 3D 点 |
| `translation_direction` | 套袋平移方向（单位向量，底→颈） |
| `bag_diameter_upper_m` | 袋径上界（遮挡时为观测下界） |
| `suggested_travel_m` | 建议平移行程 |
| `confidence` | 0~1 综合置信度 |
| `status` | **0=ACCEPT / 1=REOBSERVE / 2=REJECT** |
| `diagnostic_flags` | 字符串数组，REOBSERVE/REJECT 的原因清单 |
| `strategy_id` | 掩膜策略（`hybrid_dilated`） |
| `model_version` / `calibration_version` / `tool_version` | 版本追溯三件套 |

看一眼实际内容：

```bash
ros2 topic echo --once /peach_pose_node/grasp_candidates
```

### 9.2 `~/grasp_candidates_2d`（`BagGrasp2DArray`）

同一批目标在图像上的表达：检测框 `bbox_x/y/w/h`、底/颈/抓取点/行程终点
的像素坐标（各有 `has_*` 布尔表示是否可投影）+ confidence/status/flags。
画 UI 叠加层用它，不用自己反投影。

### 9.3 `~/fitting`（`BagFittingArray`）

拟合诊断（21 个标量 + 若干来源字符串），做算法调参/验收用：轴来源
（`axis_source`）、轴置信度、2D/3D 轴夹角、误差预算与径向净空、有效深度
比例、点数、袋长袋径行程、圆柱/球拟合 RMS 与内点率、梗洼下陷深度、
极性校正标志。**无效标量一律填 -1**（_metric 的约定）。

### 9.4 可视化四件套

- `~/detections`：YOLO 框（`vision_msgs/Detection2DArray`）。
- `~/masks`：mono8 掩膜画布，第 i 个目标灰度 = i%250+1。
- `~/debug_image`：bgr8 叠加图（框+掩膜+箭头+状态字）——
  `ros2 run rqt_image_view rqt_image_view /peach_pose_node/debug_image`
  是最快的"看一眼对不对"。
- `~/detection_cloud`：检测框内彩色点云（PointCloud2，已在输出系）。
- `~/markers`：RViz MarkerArray，加 MarkerArray display 订阅
  `/peach_pose_node/markers` 即可看到轴线/行程/刀具圆柱/状态字。

## 10. 跑起来看效果

```bash
# 真机链路（相机 + 手眼 TF 需已在运行；见仓库根 AGENTS.md 第 5 节）
ros2 launch peach_pose_ros2 peach_pose.launch.py

# 无相机冒烟：另开终端回放数据集
aubo_py3.12/bin/python tools/peach_dataset_replayer.py --dataset <数据集根目录>

# 观察输出
ros2 topic echo --once /peach_pose_node/grasp_candidates
ros2 run rqt_image_view rqt_image_view /peach_pose_node/debug_image
```

## 11. 常见问题与代码坑位索引

- **收不到消息**：先查 QoS——相机驱动/回放器与本节点必须都是 RELIABLE
  （本节点已是，见 5.3）。
- **首帧慢**：模型懒加载 + CUDA warmup，等几秒；之后每帧耗时主要看
  SAM（可用 `~/fitting` 配合日志里的 `last_timings_ms` 观察）。
- **结果坐标系不对**：看 `~/grasp_candidates` 的 `header.frame_id`——
  若是相机系而不是 base_link，说明 TF 没查到（手眼外参节点没跑），
  节点是刻意回退而非出错（6.②）。
- **全是 REOBSERVE/REJECT**：读 `diagnostic_flags` 字符串，对照第 7 章
  门控规则；`~/fitting` 里有全部量化指标。
- **代码里的中文注释坑位**：节点 179（深度单位）、260（QoS）、451（深度
  换算）、467（内参勿推导）、488（TF 回退）、530（DELETEALL 坑）、
  709（ColorRGBA 必须 float）；pipeline.py 88–90（为何不用 PCA）、
  437–445（为何固定球半径）、457（重力极性校正）。
- **已过时提示**：`peach_pose/__init__.py` docstring 提到的
  `depth.py`/`visualization.py`/`inspector/main_window`/`run.sh`
  已不存在（精简时移除），以本文结构图为准。
