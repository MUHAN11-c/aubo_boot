# GraspNet 抓取位姿 TF 使用说明

## 概述

GraspNet Demo 节点现在会发布抓取位姿的 TF 变换，使得抓取位姿可以在 TF 树中被跟踪和查询。

---

## ROS2 TF 命令速查

| 用途 | 命令 |
|------|------|
| 查看两帧之间的变换（数值） | `ros2 run tf2_ros tf2_echo <源帧> <目标帧>` |
| 监控两帧之间的变换与延迟 | `ros2 run tf2_ros tf2_monitor <帧1> <帧2>` |
| 生成 TF 树 PDF 图 | `ros2 run tf2_tools view_frames`（需安装 `graphviz`，当前目录生成 `frames_*.pdf`） |
| GUI 查看 TF 树 | `ros2 run rqt_tf_tree rqt_tf_tree` |
| 查看 /tf 话题 | `ros2 topic echo /tf` |
| 查看静态 TF | `ros2 topic echo /tf_static` |

### 本项目常用诊断命令（TF 链）

在 `graspnet_demo.launch.py` 下，TF 链为：`base_link` → … → `wrist3_Link` → `camera_frame` → `grasp_pose_0`。可按顺序检查每一段：

```bash
# 机械臂基座到末端
ros2 run tf2_ros tf2_echo base_link wrist3_Link

# 手眼标定：末端 -> 相机（由 hand_eye_static_tf_node 发布）
ros2 run tf2_ros tf2_echo wrist3_Link camera_frame

# 抓取位姿（需先触发 /publish_grasps 后才有）
ros2 run tf2_ros tf2_echo camera_frame grasp_pose_0

# 抓取位姿在基座下的表示（publish_grasps_client 会查此变换）
ros2 run tf2_ros tf2_echo base_link grasp_pose_0
```

若使用 `graspnet_demo_with_tf.launch.py` 且 RViz 固定帧为 `world`，可再加：

```bash
ros2 run tf2_ros tf2_echo world grasp_pose_0
```

---

## TF 变换结构

### 发布的 TF

每个抓取位姿都会发布一个 TF 变换：

```
camera_frame -> grasp_pose_0
camera_frame -> grasp_pose_1
camera_frame -> grasp_pose_2
...
```

### 完整的 TF 树

```
world
  └─ base_link
      └─ ... (机械臂关节)
          └─ wrist3_Link (末端执行器)
              └─ camera_frame (相机)
                  ├─ grasp_pose_0 (第1个抓取位姿)
                  ├─ grasp_pose_1 (第2个抓取位姿)
                  └─ grasp_pose_X (第X个抓取位姿)
```

---

## 查看 TF

### 方法1：使用 tf2_echo（实时查看）

查看特定抓取位姿相对于 camera_frame 的变换：

```bash
ros2 run tf2_ros tf2_echo camera_frame grasp_pose_0
```

查看抓取位姿相对于机械臂基座的变换：

```bash
ros2 run tf2_ros tf2_echo base_link grasp_pose_0
```

查看抓取位姿相对于世界坐标系的变换：

```bash
ros2 run tf2_ros tf2_echo world grasp_pose_0
```

### 方法2：生成 TF 树图（可视化）

```bash
# 生成 TF 树 PDF（需安装 graphviz，在当前目录生成 frames_<时间戳>.pdf）
ros2 run tf2_tools view_frames

# 查看生成的 PDF（文件名以 frames_ 开头）
evince frames_*.pdf
# 或指定节点/话题：ros2 run tf2_tools view_frames --node=NODE_NAME
```

### 方法3：在 RViz2 中可视化

1. 启动 RViz2
2. 添加 TF 显示：Add -> TF
3. 可以看到所有的坐标系，包括 `grasp_pose_X`
4. 调整 TF 显示选项：
   - Marker Scale: 调整坐标轴大小
   - Show Names: 显示坐标系名称
   - Show Axes: 显示坐标轴

---

## 在代码中使用 TF

### Python 示例：查询抓取位姿

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class GraspTFListener(Node):
    def __init__(self):
        super().__init__('grasp_tf_listener')
        
        # 创建 TF buffer 和 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建定时器查询 TF
        self.timer = self.create_timer(1.0, self.query_grasp_pose)
    
    def query_grasp_pose(self):
        try:
            # 查询抓取位姿相对于 base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',           # 目标坐标系
                'grasp_pose_0',        # 源坐标系
                rclpy.time.Time()      # 最新时间
            )
            
            # 获取位置
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 获取姿态（四元数）
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            self.get_logger().info(
                f'Grasp pose in base_link: '
                f'pos=[{x:.3f}, {y:.3f}, {z:.3f}], '
                f'quat=[{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]'
            )
            
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {str(e)}')

def main():
    rclpy.init()
    node = GraspTFListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ 示例：查询抓取位姿

```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class GraspTFListener : public rclcpp::Node {
public:
    GraspTFListener() : Node("grasp_tf_listener") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GraspTFListener::queryGraspPose, this)
        );
    }
    
private:
    void queryGraspPose() {
        try {
            auto transform = tf_buffer_->lookupTransform(
                "base_link", "grasp_pose_0", tf2::TimePointZero
            );
            
            RCLCPP_INFO(this->get_logger(),
                "Grasp pose in base_link: pos=[%.3f, %.3f, %.3f]",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraspTFListener>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 用于运动规划

### 示例：MoveIt 使用抓取位姿

```python
import rclpy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

# 初始化 MoveIt
move_group = MoveGroupCommander("manipulator")
planning_scene = PlanningSceneInterface()

# 创建目标位姿（使用 TF）
target_pose = PoseStamped()
target_pose.header.frame_id = "grasp_pose_0"  # 使用抓取位姿作为参考
target_pose.pose.position.x = 0.0  # 相对于抓取位姿的偏移
target_pose.pose.position.y = 0.0
target_pose.pose.position.z = -0.1  # 抓取位姿上方10cm
target_pose.pose.orientation.w = 1.0

# 设置目标并规划
move_group.set_pose_target(target_pose)
plan = move_group.plan()

# 执行运动
if plan[0]:
    move_group.execute(plan[1], wait=True)
```

---

## 常见用例

### 1. 查看抓取位姿相关变换

```bash
# 监控 base_link 到 grasp_pose_0 的变换（发布频率、延迟等）
ros2 run tf2_ros tf2_monitor base_link grasp_pose_0

# 或监控 camera_frame 到 grasp_pose_0
ros2 run tf2_ros tf2_monitor camera_frame grasp_pose_0
```

### 2. 选择最佳抓取

```python
# 查询所有抓取位姿并评估可达性
for i in range(num_grasps):
    try:
        transform = tf_buffer.lookup_transform(
            'base_link', f'grasp_pose_{i}', rclpy.time.Time()
        )
        # 评估是否在机械臂工作空间内
        if is_reachable(transform):
            execute_grasp(i)
            break
    except:
        continue
```

### 3. 调试抓取位姿

```bash
# 查看抓取位姿与末端执行器的相对位置
ros2 run tf2_ros tf2_echo wrist3_Link grasp_pose_0

# 查看抓取位姿与相机的距离
ros2 run tf2_ros tf2_echo camera_frame grasp_pose_0
```

---

## TF 坐标系说明

### camera_frame
- 相机坐标系
- 原点：相机光学中心
- 坐标约定：X-右，Y-下，Z-前（相机朝向）

### grasp_pose_X
- 抓取位姿坐标系
- 原点：夹爪中心点
- 方向：
  - X 轴：接近方向（approach direction）
  - Y 轴：夹爪宽度方向
  - Z 轴：夹爪高度方向

---

## 注意事项

1. **TF 只发布一次**
   - 与 MarkerArray 和点云相同，TF 只在计算时发布一次
   - 如果需要重新发布，调用 `/publish_grasps` 服务

2. **坐标系命名**
   - 抓取位姿从 0 开始编号：`grasp_pose_0`, `grasp_pose_1`, ...
   - 编号对应抓取质量排序（0 = 最佳抓取）

3. **时间戳**
   - TF 使用当前时间戳
   - 静态 TF 不会自动更新

4. **性能考虑**
   - 大量 TF 变换可能影响性能
   - 建议只发布少量最佳抓取（使用 `max_grasps_num` 参数）

---

## 故障排除

### TF 查询失败

**问题：** `lookup_transform` 返回 "Frame does not exist"

**解决方案：**
```bash
# 1. 检查 TF 是否发布
ros2 run tf2_ros tf2_monitor | grep grasp_pose

# 2. 检查节点是否运行
ros2 node list | grep graspnet

# 3. 重新触发发布
ros2 service call /publish_grasps std_srvs/srv/Trigger
```

### TF 延迟

**问题：** TF 查询返回旧数据

**解决方案：**
- 使用 `rclpy.time.Time()` 查询最新 TF
- 检查 TF buffer 大小（默认10秒）

---

## 总结

✅ **TF 发布功能的优势：**
- 标准化的位姿表示
- 方便与其他 ROS2 节点集成
- 支持坐标系转换
- 便于运动规划
- 易于调试和可视化

📖 **相关文档：**
- [TF2 教程](http://wiki.ros.org/tf2/Tutorials)
- [MoveIt 文档](https://moveit.ros.org/)
- [RViz TF 显示](http://wiki.ros.org/rviz/DisplayTypes/TF)
