# 快速使用指南

## 三种配置方式

### 1. 命令行参数（最方便）

```bash
# 完整配置
python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8

# 只修改部分参数
python3 limit_workspace.py --x-min -0.6 --x-max 0.6

# 查看帮助
python3 limit_workspace.py --help
```

### 2. YAML配置文件（推荐用于保存配置）

```bash
# 方式1：直接运行（自动使用 workspace_limits.yaml）
python3 limit_workspace.py

# 方式2：指定配置文件路径
python3 limit_workspace.py --config workspace_limits.yaml
```

**默认行为**：脚本会自动使用同目录下的 `workspace_limits.yaml` 配置文件（如果存在）。

**控制边界墙显示**（在YAML配置文件中）：

```yaml
# 只启用左右两面墙
enabled_walls:
  left: true
  right: true
  front: false
  back: false
  top: false
  bottom: false
```

### 3. 默认值（无需配置）

```bash
python3 limit_workspace.py
```

## 移除边界墙

```bash
python3 limit_workspace.py --remove
```

## 配置优先级

**命令行参数 > 配置文件 > 默认值**

例如：
- 配置文件设置 `x_max: 0.8`
- 命令行参数 `--x-max 0.6`
- 最终结果：`x_max = 0.6`（命令行参数优先）

## 常用示例

```bash
# 小工作空间
python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8

# 大工作空间
python3 limit_workspace.py --x-min -1.0 --x-max 1.0 --y-min -1.0 --y-max 1.0 --z-min 0.0 --z-max 1.5

# 只限制X和Y轴
python3 limit_workspace.py --x-min -0.6 --x-max 0.6 --y-min -0.6 --y-max 0.6
```

## 控制边界墙显示

在YAML配置文件中，可以控制哪些边界墙需要启用：

```yaml
# 示例1：只启用左右两面墙（限制X轴方向）
enabled_walls:
  left: true
  right: true
  front: false
  back: false
  top: false
  bottom: false

# 示例2：只启用前后两面墙（限制Y轴方向）
enabled_walls:
  left: false
  right: false
  front: true
  back: true
  top: false
  bottom: false

# 示例3：只启用顶墙（限制Z轴上方）
enabled_walls:
  left: false
  right: false
  front: false
  back: false
  top: true
  bottom: false
```
