# 工作空间限制脚本使用说明

## 功能说明

`limit_workspace.py` 脚本通过添加边界墙（碰撞对象）来限制 MoveIt2 的工作空间范围。机械臂将无法规划超出这些边界的路径。

## 快速开始

### 1. 启动 MoveIt2

```bash
cd ~/IVG/aubo_ros2_ws
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py
```

### 2. 运行工作空间限制脚本

在另一个终端中：

```bash
cd ~/IVG/aubo_ros2_ws
source install/setup.bash
python3 src/aubo_ros2_driver/aubo_moveit_config/scripts/limit_workspace.py
```

**注意**：脚本会自动使用同目录下的 `workspace_limits.yaml` 配置文件（如果存在）。如果该文件不存在，则使用默认值。

### 3. 在 RViz2 中查看

- 打开 RViz2（如果还没有打开）
- 在 Motion Planning 插件的 "Scene Geometry" 部分应该能看到边界墙
- 边界墙会显示为半透明的盒子

## 自定义工作空间边界

有三种方式可以自定义工作空间边界（按优先级从高到低）：

### 方法1：命令行参数（最灵活，推荐）

```bash
# 使用命令行参数指定边界
python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8

# 只修改部分参数（其他使用默认值）
python3 limit_workspace.py --x-min -0.6 --x-max 0.6

# 修改墙厚度
python3 limit_workspace.py --wall-thickness 0.1
```

**所有命令行参数：**
- `--x-min`: X轴最小边界（米）
- `--x-max`: X轴最大边界（米）
- `--y-min`: Y轴最小边界（米）
- `--y-max`: Y轴最大边界（米）
- `--z-min`: Z轴最小边界（米）
- `--z-max`: Z轴最大边界（米）
- `--wall-thickness`: 边界墙厚度（米，默认0.05）
- `--config` / `-c`: 指定配置文件路径
- `--remove` / `-r`: 移除所有边界墙

### 方法2：YAML配置文件（便于保存和重复使用）

1. 编辑配置文件 `workspace_limits.yaml`：

```yaml
# X轴边界（米）
x_min: -0.5
x_max: 0.5

# Y轴边界（米）
y_min: -0.3
y_max: 0.3

# Z轴边界（米）
z_min: 0.1
z_max: 0.8

# 边界墙厚度（米）
wall_thickness: 0.05

# 启用/禁用边界墙（可选）
enabled_walls:
  left: true      # 左墙（X轴负方向）
  right: true     # 右墙（X轴正方向）
  front: true     # 前墙（Y轴正方向）
  back: true      # 后墙（Y轴负方向）
  top: true       # 顶墙（Z轴正方向）
  bottom: false   # 底墙（Z轴负方向）
```

2. 使用配置文件运行：

```bash
# 方式1：直接运行（自动使用 workspace_limits.yaml）
python3 limit_workspace.py

# 方式2：指定配置文件路径
python3 limit_workspace.py --config workspace_limits.yaml

# 方式3：使用其他配置文件
python3 limit_workspace.py --config my_custom_config.yaml
```

**默认行为**：如果不指定 `--config` 参数，脚本会自动尝试加载同目录下的 `workspace_limits.yaml` 文件。

**控制边界墙显示：**

在YAML配置文件中，可以通过 `enabled_walls` 选项控制哪些边界墙需要启用：

```yaml
# 示例：只启用左右两面墙（限制X轴方向）
enabled_walls:
  left: true
  right: true
  front: false
  back: false
  top: false
  bottom: false

# 示例：只启用前后两面墙（限制Y轴方向）
enabled_walls:
  left: false
  right: false
  front: true
  back: true
  top: false
  bottom: false

# 示例：只启用顶墙（限制Z轴上方）
enabled_walls:
  left: false
  right: false
  front: false
  back: false
  top: true
  bottom: false
```

### 方法3：修改代码中的默认值

编辑 `limit_workspace.py` 文件中的默认值（不推荐，除非需要永久修改默认值）。

## 移除边界墙

使用命令行参数移除：

```bash
python3 limit_workspace.py --remove
```

或者使用简写：

```bash
python3 limit_workspace.py -r
```

## 工作原理

脚本可以创建最多 6 面边界墙（根据配置启用）：
1. **左墙**：限制 X 轴负方向
2. **右墙**：限制 X 轴正方向
3. **前墙**：限制 Y 轴正方向
4. **后墙**：限制 Y 轴负方向
5. **顶墙**：限制 Z 轴正方向
6. **底墙**：限制 Z 轴负方向（默认禁用）

这些墙作为碰撞对象添加到 MoveIt2 的规划场景中，MoveIt2 在规划路径时会自动避开这些碰撞对象。

**默认配置**：默认启用前5面墙（左、右、前、后、顶），底墙默认禁用。

## 注意事项

1. **坐标系**：脚本使用 "world" 坐标系，确保与你的机器人配置一致
2. **启动顺序**：必须在 MoveIt2 启动后再运行此脚本
3. **墙的厚度**：建议使用 0.05 米，太薄可能影响碰撞检测，太厚会占用过多空间
4. **性能**：添加边界墙后，规划时间可能会略微增加

## 故障排除

### 边界墙没有显示

1. 检查 MoveIt2 是否正在运行：`ros2 node list | grep move_group`
2. 检查话题是否存在：`ros2 topic list | grep planning_scene`
3. 在 RViz2 中检查 "Scene Geometry" 是否启用

### 规划仍然超出边界

1. 检查坐标系是否正确（应该是 "world"）
2. 确保边界墙的位置和尺寸正确
3. 尝试增加墙的厚度

### 脚本运行错误

1. 确保已 source ROS2 环境：`source install/setup.bash`
2. 确保已安装必要的 ROS2 包：`moveit_msgs`, `shape_msgs`, `geometry_msgs`
3. 检查 Python 版本：`python3 --version`（需要 Python 3.6+）

## 使用示例

### 示例1：使用命令行参数限制小工作空间

```bash
# 限制工作空间为：X:[-0.5, 0.5], Y:[-0.3, 0.3], Z:[0.1, 0.8]
python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8
```

### 示例2：使用配置文件

```bash
# 1. 编辑 workspace_limits.yaml
# 2. 运行脚本
python3 limit_workspace.py --config workspace_limits.yaml
```

### 示例3：只修改部分参数

```bash
# 只修改X轴边界，其他使用默认值
python3 limit_workspace.py --x-min -0.6 --x-max 0.6
```

### 示例4：查看帮助信息

```bash
python3 limit_workspace.py --help
```

### 示例5：移除边界墙

```bash
python3 limit_workspace.py --remove
```

### 示例6：使用配置文件控制边界墙显示

创建配置文件 `my_config.yaml`：

```yaml
x_min: -0.6
x_max: 0.6
y_min: -0.4
y_max: 0.4
z_min: 0.0
z_max: 1.0
wall_thickness: 0.05

# 只启用左右两面墙
enabled_walls:
  left: true
  right: true
  front: false
  back: false
  top: false
  bottom: false
```

运行：

```bash
python3 limit_workspace.py --config my_config.yaml
```

## 与其他限制方法对比

| 方法 | 优点 | 缺点 |
|------|------|------|
| 关节限制（joint_limits.yaml） | 简单、全局生效 | 只能限制关节角度 |
| 路径约束（代码中） | 精确控制姿态 | 需要修改代码 |
| **碰撞对象（本脚本）** | **可视化、灵活** | **需要额外脚本** |
